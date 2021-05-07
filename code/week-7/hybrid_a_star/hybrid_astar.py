import numpy as np
from math import fmod

class HybridAStar:
    # Determine how many grid cells to have for theta-axis.
    NUM_THETA_CELLS = 360

    # Define min, max, and resolution of steering angles
    omega_min = -35
    omega_max = 35
    omega_step = 5

    # A very simple bicycle model
    speed = 0.5
    length = 0.5

    # Initialize the search structure.
    def __init__(self, dim):
        self.dim = dim
        self.closed = np.zeros(self.dim, dtype=np.int)
        self.came_from = np.full(self.dim, None)

    # Expand from a given state by enumerating reachable states.
    def expand(self, current, goal):
        g = current['g']
        x, y, theta = current['x'], current['y'], current['t']

        # The g value of a newly expanded cell increases by 1 from the
        # previously expanded cell.
        g2 = g + 1
        next_states = []

        # Consider a discrete selection of steering angles.
        for delta_t in range(self.omega_min, self.omega_max, self.omega_step):
            # TODO: implement the trajectory generation based on
            # a simple bicycle model.
            # Let theta2 be the vehicle's heading (in radian)
            # between 0 and 2 * PI.
            # Check validity and then add to the next_states list.
            omega = self.speed / self.length * np.tan(np.radians(delta_t))
            next_x = x + self.speed * np.cos(theta)
            next_y = y + self.speed * np.sin(theta)
            next_theta = self.normalize_angle(theta + omega)
            next_f = g2 + self.heuristic(next_x, next_y, goal)
            s = {
                'x' : next_x,
                'y' : next_y,
                't' : next_theta,
                'f' : next_f, 
                'g' : g2
            }
            next_states.append(s)

        return next_states

    # Perform a breadth-first search based on the Hybrid A* algorithm.
    def search(self, grid, start, goal):
        # Initial heading of the vehicle is given in the
        # last component of the tuple start.
        theta = start[-1]
        # Determine the cell to contain the initial state, as well as
        # the state itself.
        stack = self.theta_to_stack_num(theta)
        g = 0
        s = {
            'f': self.heuristic(start[0], start[1], goal),
            'g': g,
            'x': start[0],
            'y': start[1],
            't': theta,
        }
        self.final = s
        # Close the initial cell and record the starting state for
        # the sake of path reconstruction.
        self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
        self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
        total_closed = 1
        opened = [s]
        # Examine the open list, according to the order dictated by
        # the heuristic function.
        while len(opened) > 0:
            # TODO: implement prioritized breadth-first search
            # for the hybrid A* algorithm.
            opened.sort(key=lambda s : s['f'], reverse=True)
            curr = opened.pop()
            x, y = curr['x'], curr['y']
            if (self.idx(x), self.idx(y)) == goal:
                self.final = curr
                found = True
                break

            # Compute reachable new states and process each of them.
            next_states = self.expand(curr, goal)
            for n in next_states:
                # and add to opened list. 
                # Adjust closed list.
                n_x, n_y, n_theta = n['x'], n['y'], n['t']
                stack = self.theta_to_stack_num(n_theta)

                # Check validity of the state
                idx_x, idx_y = self.idx(n_x), self.idx(n_y)
                if 0 <= idx_x < grid.shape[0] and 0 <= idx_y < grid.shape[1] and \
                    self.closed[stack][idx_x][idx_y] == 0 and grid[(idx_x, idx_y)] == 0: 
                    
                    # Check grid validity 
                    if self.validity(x, y, n_x, n_y, grid):

                        self.came_from[stack][idx_x][idx_y] = curr
                        self.closed[stack][idx_x][idx_y] = 1 # mark as closed
                        opened.append(n) # add to opened list
                        total_closed += 1

        else:
            # We weren't able to find a valid path; this does not necessarily
            # mean there is no feasible trajectory to reach the goal.
            # In other words, the hybrid A* algorithm is not complete.
            found = False

        return found, total_closed

    # Calculate the stack index of a state based on the vehicle's heading.
    def theta_to_stack_num(self, theta):
        # TODO: implement a function that calculate the stack number
        # given theta represented in radian. Note that the calculation
        # should partition 360 degrees (2 * PI rad) into different
        # cells whose number is given by NUM_THETA_CELLS.
        interval = np.radians(360 / self.NUM_THETA_CELLS)
        return int(theta // interval)

    # Calculate the index of the grid cell based on the vehicle's position.
    def idx(self, pos):
        # We simply assume that each of the grid cell is the size 1 X 1.
        return int(np.floor(pos))

    # Implement a heuristic function to be used in the hybrid A* algorithm.
    def heuristic(self, x, y, goal):
        # TODO: implement a heuristic function.
        goal_x, goal_y = goal
        ret = np.sqrt((x - goal_x)**2 + (y - goal_y)**2)

        return ret

    # Reconstruct the path taken by the hybrid A* algorithm.
    def reconstruct_path(self, start, goal):
        # Start from the final state, and follow the link to the
        # previous state using the came_from matrix.
        curr = self.final
        x, y = curr['x'], curr['y']
        path = []
        while x != start[0] and y != start[1]:
            path.append(curr)
            stack = self.theta_to_stack_num(curr['t'])
            x, y = curr['x'], curr['y']
            curr = self.came_from[stack][self.idx(x)][self.idx(y)]
        # Reverse the path so that it begins at the starting state
        # and ends at the final state.
        path.reverse()
        return path
    
    def validity(self, x, y, n_x, n_y, grid): 
        dx = n_x - x
        dy = n_y - y
        step_size = 0.01

        if (abs(dx) > abs(dy)) : 
            rang = np.arange(x, n_x, step_size)
            for new_x in rang : 
                slope = dy / dx
                new_y = slope * (new_x - x) + y
                id_x, id_y = self.idx(new_x), self.idx(new_y)
                if 0 <= id_x < grid.shape[0] and 0 <= id_y < grid.shape[1] and \
                    grid[(id_x, id_y)] == 0 : 
                    continue 
                else : return False
        else : 
            rang = np.arange(y, n_y, step_size)
            for new_y in rang : 
                slope = dx / dy
                new_x = slope * (new_y - y) + x
                id_x, id_y = self.idx(new_x), self.idx(new_y)
                if 0 <= id_x < grid.shape[0] and 0 <= id_y < grid.shape[1] and \
                    grid[(id_x, id_y)] == 0 : 
                    continue 
                else : return False

        return True
    
    def normalize_angle(self, angle):
        """
        Wrap the angle between 0 and 2 * pi.

        Args:
            angle (float): angle to wrap.

        Returns:
            The wrapped angle.
        """
        pi_2 = 2. * np.pi

        return fmod(fmod(angle, pi_2) + pi_2, pi_2) 
