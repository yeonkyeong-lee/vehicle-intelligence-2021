import numpy as np
from helpers import distance

class ParticleFilter:
    def __init__(self, num_particles):
        self.initialized = False
        self.num_particles = num_particles

    # Set the number of particles.
    # Initialize all the particles to the initial position
    #   (based on esimates of x, y, theta and their uncertainties from GPS)
    #   and all weights to 1.0.
    # Add Gaussian noise to each particle.
    def initialize(self, x, y, theta, std_x, std_y, std_theta):
        self.particles = []
        for i in range(self.num_particles):
            self.particles.append({
                'x': np.random.normal(x, std_x),
                'y': np.random.normal(y, std_y),
                't': np.random.normal(theta, std_theta),
                'w': 1.0,
                'assoc': [],
            })
        self.initialized = True

    # Add measurements to each particle and add random Gaussian noise.
    def predict(self, dt, velocity, yawrate, std_x, std_y, std_theta):
        # Be careful not to divide by zero.
        v_yr = velocity / yawrate if yawrate else 0
        yr_dt = yawrate * dt
        for p in self.particles:
            # We have to take care of very small yaw rates;
            #   apply formula for constant yaw.
            if np.fabs(yawrate) < 0.0001:
                xf = p['x'] + velocity * dt * np.cos(p['t'])
                yf = p['y'] + velocity * dt * np.sin(p['t'])
                tf = p['t']
            # Nonzero yaw rate - apply integrated formula.
            else:
                xf = p['x'] + v_yr * (np.sin(p['t'] + yr_dt) - np.sin(p['t']))
                yf = p['y'] + v_yr * (np.cos(p['t']) - np.cos(p['t'] + yr_dt))
                tf = p['t'] + yr_dt
            p['x'] = np.random.normal(xf, std_x)
            p['y'] = np.random.normal(yf, std_y)
            p['t'] = np.random.normal(tf, std_theta)

    # Find the predicted measurement that is closest to each observed
    #   measurement and assign the observed measurement to this
    #   particular landmark.
    def associate(self, predicted, observations):
        associations = []
        # For each observation, find the nearest landmark and associate it.
        #   You might want to devise and implement a more efficient algorithm.
        for o in observations:
            min_dist = -1.0
            for p in predicted:
                dist = distance(o, p)
                if min_dist < 0.0 or dist < min_dist:
                    min_dist = dist
                    min_id = p['id']
                    min_x = p['x']
                    min_y = p['y']
            association = {
                'id': min_id,
                'x': min_x,
                'y': min_y,
            }
            associations.append(association)
        # Return a list of associated landmarks that corresponds to
        #   the list of (coordinates transformed) predictions.
        return associations

    # Update the weights of each particle using a multi-variate
    #   Gaussian distribution.
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):
        # TODO: For each particle, do the following:
        for particle in self.particles :

        # 1. Select the set of landmarks that are visible
        #    (within the sensor range).
            visible_landmarks = []
            for idx in map_landmarks : 
                dist = distance(map_landmarks[idx], particle)
                if dist < sensor_range : 
                    visible_landmarks.append({'id' : idx,
                                               'x' : map_landmarks[idx]['x'],
                                               'y' : map_landmarks[idx]['y']
                                            })

        # 2. Transform each observed landmark's coordinates from the
        #    particle's coordinate system to the map's coordinates.
            observations_coord = []
            for landmark in observations :
                xl, yl = landmark['x'], landmark['y']
                xp, yp, theta = particle['x'], particle['y'], particle['t']
                observations_coord.append({
                    'x' : xp + xl * np.cos(theta) - yl * np.sin(theta),
                    'y' : yp + xl * np.sin(theta) + yl * np.cos(theta)
                })
                # landmark['x'] = xp + xl * np.cos(theta) - yl * np.sin(theta)
                # landmark['y'] = yp + xl * np.sin(theta) + yl * np.cos(theta)

        # 3. Associate each transformed observation to one of the
        #    predicted (selected in Step 1) landmark positions.
        #    Use self.associate() for this purpose - it receives
        #    the predicted landmarks and observations; and returns
        #    the list of landmarks by implementing the nearest-neighbour
        #    association algorithm.
            if len(visible_landmarks) == 0 : 
                continue

            associations = self.associate(visible_landmarks, observations_coord)
            particle['assoc'] = [asso['id'] for asso in associations]

        # 4. Calculate probability of this set of observations based on
        #    a multi-variate Gaussian distribution (two variables being
        #    the x and y positions with means from associated positions
        #    and variances from std_landmark_x and std_landmark_y).
        #    The resulting probability is the product of probabilities
        #    for all the observations.

            std = np.array([[std_landmark_x, 0], [0, std_landmark_y]])**2

            weight = 1
            for idx, observation in enumerate(observations_coord) :
                mean = [associations[idx]['x'], associations[idx]['y']]
                x = np.array([observation['x'], observation['y']])
                weight *= self.multivariate_normal_distribution(x, mean, std)
            
        # 5. Update the particle's weight by the calculated probability.
            particle['w'] = weight

    # Resample particles with replacement with probability proportional to
    #   their weights.
    def resample(self):
        # TODO: Select (possibly with duplicates) the set of particles
        #       that captures the posteior belief distribution, by
        # 1. Drawing particle samples according to their weights.
        # 2. Make a copy of the particle; otherwise the duplicate particles
        #    will not behave independently from each other - they are
        #    references to mutable objects in Python.
        # Finally, self.particles shall contain the newly drawn set of
        #   particles.
        weights = [p['w'] for p in self.particles]
        weights_sum = np.sum(weights)

        if weights_sum < 1e-8 :
            weights = np.ones_like(weights) / len(weights)
        else : 
            weights = weights / weights_sum

        self.particles = np.random.choice(self.particles, size=len(weights), p=weights)
        for i in range(len(self.particles)) :
            self.particles[i] = self.particles[i].copy()

    # Choose the particle with the highest weight (probability)
    def get_best_particle(self):
        highest_weight = -1.0
        for p in self.particles:
            if p['w'] > highest_weight:
                highest_weight = p['w']
                best_particle = p
        return best_particle
    
    def multivariate_normal_distribution(self, x, mean, std) :
        
        return (np.linalg.det(2 * np.pi * std)**-0.5) * (np.exp(-0.5 * (x - mean).T @ np.linalg.inv(std) @ (x - mean)))