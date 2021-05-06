# Week 4 - Motion Model & Particle Filters

## Report

### update_weights

```python
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
```
- 4번 과정을 위해 아래 함수를 구현했다.
```python
def multivariate_normal_distribution(self, x, mean, std) :
    
    return (np.linalg.det(2 * np.pi * std)**-0.5) * \ 
        (np.exp(-0.5 * (x - mean).T @ np.linalg.inv(std) @ (x - mean)))
```

### resample

```python
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
```
- numpy의 random.choice를 사용하여 resample을 구현하였다. 

---

[//]: # (Image References)
[empty-update]: ./empty-update.gif
[example]: ./example.gif

## Assignment

You will complete the implementation of a simple particle filter by writing the following two methods of `class ParticleFilter` defined in `particle_filter.py`:

* `update_weights()`: For each particle in the sample set, calculate the probability of the set of observations based on a multi-variate Gaussian distribution.
* `resample()`: Reconstruct the set of particles that capture the posterior belief distribution by drawing samples according to the weights.

To run the program (which generates a 2D plot), execute the following command:

```
$ python run.py
```

Without any modification to the code, you will see a resulting plot like the one below:

![Particle Filter without Proper Update & Resample][empty-update]

while a reasonable implementation of the above mentioned methods (assignments) will give you something like

![Particle Filter Example][example]

Carefully read comments in the two method bodies and write Python code that does the job.
