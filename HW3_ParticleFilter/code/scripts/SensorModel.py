import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
from scipy.spatial import distance
from scipy.stats import expon
import pdb

from MapReader import MapReader

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):
        """
        Initialize Sensor Model parameters here
        """
        self.map = occupancy_map

        # Weuight
        self.zh = 10000
        self.zs = 0.01
        self.zm = 0.01
        self.zr = 100000

        # 
        self.Z_MAX = 8183
        self.hit_sigma = 200
        self.sl = 0.01
        
        # sensor position offset (cm)
        self.laser_offset = 25
        self.step_size = 10

    # 1. Local measurement noise - Gaussian distribution
    def p_hit(self, z_tk, z_tk_star):
   
        p_hit_return = 0.0
        if (0 <= z_tk)and(z_tk <= self.Z_MAX):
            var = float(self.hit_sigma)**2
            denom = (2*math.pi*var)**.5
            num = math.exp(-(float(z_tk)-float(z_tk_star))**2/(2*var))
            p_hit_return =  num/denom

        return p_hit_return

    # 2. Unexpected objects - Exponential distribution
    def p_short(self, z_tk, z_tk_star):

        p_short_return = 0.0
        if (0 <= z_tk)and(z_tk <= z_tk_star):
            n = 1 / (1 - math.exp(- z_tk_star * self.sl))
            exp_value = math.exp(-self.sl * z_tk)
            p_short_return = n * self.sl * exp_value
       
        return p_short_return

    # 3. Failures - Max-range measurements - Uniform distribution pmax
    def p_max(self, z_tk):

        p_max_return = 0.0
        if (z_tk == self.Z_MAX):
            p_max_return = 1.0
      
        return p_max_return

    # 4. Random measurements - unexplained measurements - Uniform distribution prand
    def p_rand(self, z_tk):

        p_rand_return = 0.0
        if (0 <= z_tk)and(z_tk < self.Z_MAX):
            p_rand_return =  1.0 / self.Z_MAX
        
        return p_rand_return

    # Ray casting
    def ray_casting(self, x_t1):

        step = 2
        x_t1[0] = int(x_t1[0]/10)
        x_t1[1] = int(x_t1[1]/10) 

        x = x_t1[0]
        y = x_t1[1]
        angle = x_t1[2]

        condition = True
        while condition:
            # In map boundaries
            cond1 = (0 < x < self.map.shape[1]) and (0 < y < self.map.shape[0])
            # While it is unoccupied
            cond2 = (abs(self.map[int(round(y)), int(round(x))]) < 0.0001)
            if cond1 and cond2:
                condition = True
            else: condition = False

            x += step * np.cos(angle)
            y += step * np.sin(angle)

        d = distance.euclidean( (x, y), (x_t1[0], x_t1[1]) )*10

        return d

 
    # Algorithm beam range finder model(zt, xt, m):
    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """

        """
        TODO : Add your code here
        """
        # Particle state belief at time t 
        pos_x, pos_y, pos_theta = x_t1

        # Calculate laser position
        laser_x = pos_x + (self.laser_offset * math.cos(pos_theta))
        laser_y = pos_y + (self.laser_offset * math.sin(pos_theta))

        q = 1.0

        for deg in range (0, 180, self.step_size):

            z_t1_el = z_t1_arr[deg]
            z_t1_rc = self.ray_casting([laser_x, laser_y, pos_theta + math.radians(deg-90)])

            # find weighted average of 4 distributions
            p = ( self.zh * self.p_hit(z_t1_el, z_t1_rc) +
                + self.zs * self.p_short(z_t1_el, z_t1_rc) +
                + self.zm * self.p_max(z_t1_el) +
                + self.zr * self.p_rand(z_t1_el) )

            q = q*p

        return q

if __name__=='__main__':
    pass