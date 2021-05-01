# Week 2 - Markov Localization

---

[//]: # (Image References)
[plot]: ./markov.gif

## Assignment

You will complete the implementation of a simple Markov localizer by writing the following two functions in `markov_localizer.py`:

* `motion_model()`: For each possible prior positions, calculate the probability that the vehicle will move to the position specified by `position` given as input.
* `observation_model()`: Given the `observations`, calculate the probability of this measurement being observed using `pseudo_ranges`.

The algorithm is presented and explained in class.

All the other source files (`main.py` and `helper.py`) should be left as they are.

If you correctly implement the above functions, you expect to see a plot similar to the following:

![Expected Result of Markov Localization][plot]

If you run the program (`main.py`) without any modification to the code, it will generate only the frame of the above plot because all probabilities returned by `motion_model()` are zero by default.

# Report
- motion model
```
for i in range(map_size) : 
    G = norm_pdf(position-i, mov, stdev)
    position_prob += G * priors[i]
```

- observation model
```
if len(observations) < 1 or len(observations) > len(pseudo_ranges) :
    return 0.0
for i in range(len(observations)) :
    distance_prob *= norm_pdf(observations[i], pseudo_ranges[i], stdev)
```
