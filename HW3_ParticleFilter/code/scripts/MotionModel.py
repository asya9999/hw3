import sys
import numpy as np
import math
from numpy import linalg as LA

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):

        """
        TODO : Initialize Motion Model parameters here
        """
        self.a1 = 0.0001
        self.a2 = 0.0001
        self.a3 = 0.01
        self.a4 = 0.01


    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        """
        TODO : Add your code here
        """
        # Calculate differences between parameters x, y, theta
        dif_u0 = u_t0[0] - u_t1[0]
        dif_u1 = u_t0[1] - u_t1[1]
        dif_u2 = u_t0[2] - u_t1[2]

        # Calculate relative motion parameters δrot1, δtrans, δrot2 from the odometry readings
        rot1 = math.atan2(-dif_u1, -dif_u0) - u_t0[2]
        trans = math.sqrt(dif_u0**2 + dif_u1**2)
        rot2 = -dif_u2 - rot1

        # Calculate sigmas for removing independent noise
        sqrt_sigma1 = np.sqrt(rot1**2 * self.a1 + trans**2 * self.a2) 
        sqrt_sigma2 = np.sqrt(trans**2 * self.a3 + (rot1**2 + rot2**2) * self.a4)
        sqrt_sigma3 = np.sqrt(rot2**2 * self.a1 + trans**2 * self.a2)
        

        # Calculate and add to the final array updates of translation and rotation
        x_t1 = []
        
        x_t1.append(x_t0[0] + (trans - np.random.normal(0, sqrt_sigma2))
                    * math.cos(x_t0[2] + rot1 - np.random.normal(0, sqrt_sigma1)))
        x_t1.append(x_t0[1] + (trans - np.random.normal(0, sqrt_sigma2))
                    * math.sin(x_t0[2] + rot1 - np.random.normal(0, sqrt_sigma1)))
        x_t1.append(x_t0[2] + rot1 - np.random.normal(0, sqrt_sigma1)
                    + rot2 - np.random.normal(0, sqrt_sigma3))
        
        x_t1 = np.array(x_t1)

        return x_t1

if __name__=="__main__":
    pass