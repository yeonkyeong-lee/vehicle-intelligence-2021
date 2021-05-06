# Week 5 - Path Planning & the A* Algorithm

## Report

### value function 과 policy function 계산하기
```python
def optimum_policy_2D(grid, init, goal, cost):
    # ... 
        for y, x, t in p:
            # Mark the final state with a special value that we will
            # use in generating the final path policy.
            if (y, x) == goal and value[(t, y, x)] > 0:
                # TODO: implement code.
                value[(t, y, x)] = 0
                policy2D[(y, x)] = '*'
                change = True
            # Try to use simple arithmetic to capture state transitions.
            elif grid[(y, x)] == 0:
                # TODO: implement code.
                for c, a in zip(list(cost), action) : 

                    t2 = (t + a) % 4 
                    y2, x2 = y + forward[t2][0], x + forward[t2][1]

                    if 0 <= y2 < grid.shape[0] \
                        and 0 <= x2 < grid.shape[1] \
                        and grid[(y2, x2)] == 0 : 

                        v2 = value[(t2, y2, x2)] + c

                        if v2 < value[(t, y, x)] : 
                            change = True
                            value[(t, y, x)] = v2
                            policy[(t, y, x)] = a
```
- 각 action과 그에 따른 cost를 이용해 value, policy 업데이트

### policy 를 이용하여 최적의 경로 찾기

```python
def optimum_policy_2D(grid, init, goal, cost) : 
    # Now navigate through the policy table to generate a
    # sequence of actions to take to follow the optimal path.
    # TODO: implement code.
    
    t, y, x = init[2], init[0], init[1]

    while (y, x) != goal : 
        a = policy[(t, y, x)]
        t2 = (t + a)  % 4
        y2, x2 = y + forward[t2][0], x + forward[t2][1]

        if 0 <= y2 < grid.shape[0] and 0 <= x2 < grid.shape[1] \
            and grid[(y2, x2)] == 0 :

            policy2D[(y, x)] = action_name[a+1]
            t, y, x = t2, y2, x2

    # Return the optimum policy generated above.
    return policy2D
```
- init position에서 출발하여 policy를 따라 경로 생성

---

## Examples

We have four small working examples for demonstration of basic path planning algorithms:

* `search.py`: simple shortest path search algorithm based on BFS (breadth first search) - only calculating the cost.
* `path.py`: built on top of the above, generating an optimum (in terms of action steps) path plan.
* `astar.py`: basic implementation of the A* algorithm that employs a heuristic function in expanding the search.
* `policy.py`: computation of the shortest path based on a dynamic programming technique.

These sample source can be run as they are. Explanation and test results are given in the lecture notes.

## Assignment

You will complete the implementation of a simple path planning algorithm based on the dynamic programming technique demonstrated in `policy.py`. A template code is given by `assignment.py`.

The assignmemt extends `policy.py` in two aspects:

* State space: since we now consider not only the position of the vehicle but also its orientation, the state is now represented in 3D instead of 2D.
* Cost function: we define different cost for different actions. They are:
	- Turn right and move forward
	- Move forward
	- Turn left and move forward

This example is intended to illustrate the algorithm's capability of generating an alternative (detouring) path when left turns are penalized by a higher cost than the other two possible actions. When run with the settings defined in `assignment.py` without modification, a successful implementation shall generate the following output,

```
[[' ', ' ', ' ', 'R', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', '#'],
 ['*', '#', '#', '#', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', ' '],
 [' ', ' ', ' ', '#', ' ', ' ']]
```

because of the prohibitively high cost associated with a left turn.

You are highly encouraged to experiment with different (more complex) maps and different cost settings for each type of action.
