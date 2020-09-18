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
        #Principle of the low variance resampling procedure starts herer
        X_bar_resampled = []
        r = random.uniform(0, 1/len(X_bar)) #this we are drawing from unifrom distribution which points to exactly one particle
        wt = X_bar[:][3]
        wt /= np.array(wt)/sum(wt) #sample weight
        const, i, m = wt[0], 0, 0
        for m in range(len(X_bar)):
            u = r + m/len(X_bar)  # We choose a random number r and then select those particles
            while const<u: #check whether i is the index of the first particle such that the corresponding sum of weights exceeds u
                i +=1
                const +=wt[i] # compute the sum in the righthand side of the eguation
            X_bar_resampled.append(X_bar[i])
        return np.asarray(X_bar_resampled)

if __name__ == "__main__":
    pass
