import numpy as np
import pdb
import random
class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """
    def low_variance_sampler(self, X_bar):
        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """
        # Principle of the low variance resampling procedure starts here
        X_bar = np.array(X_bar)
        X_bar_resampled = []
        # This we are drawing from unifrom distribution which points to exactly one particle
        r = random.uniform(0, 1/len(X_bar)) 
        wt = np.array(X_bar[:,3])/sum(X_bar[:,3]) #sample weight
        const, i, m = wt[0], 0, 0
        while m < len(X_bar):
            # We choose a random number r and then select those particles
            u = r + m/len(X_bar)
            # Сheck whether i is the index of the first particle such that the corresponding sum of weights exceeds u
            while const<u: 
                i +=1
                # Сompute the sum in the righthand side of the eguation
                const +=wt[i]
            X_bar_resampled.append(X_bar[i])
            m +=1
        return np.array(X_bar_resampled)

if __name__ == "__main__":
    pass