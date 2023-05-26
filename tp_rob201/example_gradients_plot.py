""" A set of robotics control functions """

import random

import numpy as np
# Just for displaying gradient, see if necessary
from matplotlib import pyplot as plt

# Global var for goal position in potential_field_control(), 
# taken in the simulation environment (1113, 750)
# TODO: dynamically get walls position to throw dice again if collision
goal_pos = [random.randint(1, 500), random.randint(1, 300)]

## Version 1 : Display a grid with static point goal and obstacle
## TODO: Version 2 : with an simulation compatible goal and a single obstacle, computed as
## the shortest distance to an obstacle of the lidar scan
## TODO: Version 3 : Same with several obstacle and iterative RANSAC call
# creating two evenly spaced array in map ranges with steps of 5
x = np.arange(0, 500, 5)
y = np.arange(0, 300, 5)

# Retrieve goal pos
# global goal_pos

# Obstacle is randomly picked for the moment, just avoid borders
n_obstacles = 5
obstacles_pos = np.zeros((n_obstacles, 2))
for k in range(n_obstacles):
    obstacles_pos[k] = [random.randint(30, 470), random.randint(30, 270)]

# Creating the meshGrid for display
X, Y = np.meshgrid(x, y, indexing='ij')

# Vectors are plotted according to the gradient value
grad_x = (np.zeros_like(X)).astype('float64')
grad_y = (np.zeros_like(Y)).astype('float64')

# Goal potential parameters
d_lim = 50  # Near goal radius to be influenced with a quadratic potential instead of linear
min_r = 10  # Near goal radius to be considered as reached
K_goal = 1  # Attractive coefficient (= set max attractive gradient)
# Obstacle potential parameters
d_safe = 150  # Obstacle radius beyond which the agent is no longer repelled
K_obs = 10000  # Repulsive coefficient
for i in range(len(x)):
    for j in range(len(y)):
        # Let d_goal be the distance between current agent (point of the grid) and goal
        # potential is computed for each cell of the grid
        d_goal = np.sqrt((goal_pos[0] - X[i][j]) ** 2 + (goal_pos[1] - Y[i][j]) ** 2)

        ## Goal attractive potential
        # If agent is in obstacle radius, then the gradient is null
        if d_goal < min_r:
            grad_x[i][j] = 0
            grad_y[i][j] = 0
        # If agent lies between goal radius and security area, then we use a quadratic potential
        elif d_goal < d_lim:  # and d_obstacle > min_r:
            grad_x[i][j] = (K_goal / d_lim) * (goal_pos[0] - X[i][j])
            grad_y[i][j] = (K_goal / d_lim) * (goal_pos[1] - Y[i][j])
        # Beyond d_lim, we prefer linear potential for constant gradient
        elif d_goal > d_lim:
            grad_x[i][j] = (K_goal / d_goal) * (goal_pos[0] - X[i][j])
            grad_y[i][j] = (K_goal / d_goal) * (goal_pos[1] - Y[i][j])

        ## Obstacles repulsive potentials
        # Warning: Remember potentiel are cumulative and repulsive obstacle gradient is *added*
        # to attractive goal gradient

        for k in range(n_obstacles):
            # Let d_obstacle be the distance between current agent and current obstacle
            d_obstacle = np.sqrt((obstacles_pos[k][0] - X[i][j]) ** 2 + (obstacles_pos[k][1] - Y[i][j]) ** 2)
            if d_obstacle > d_safe:
                grad_x[i][j] += 0
                grad_y[i][j] += 0
            elif d_obstacle == 0:
                # Simulation only consideration, as robot can't fuse with obstacle. Stop the robot in this case (don't consider goal anymore)
                grad_x[i][j] = 0
                grad_y[i][j] = 0
            else:  # = if d_obstacle < d_safe
                grad_x[i][j] -= (K_obs / d_obstacle ** 3) * (1 / d_obstacle - 1 / d_safe) * (
                        obstacles_pos[k][0] - X[i][j])
                grad_y[i][j] -= (K_obs / d_obstacle ** 3) * (1 / d_obstacle - 1 / d_safe) * (
                        obstacles_pos[k][1] - Y[i][j])

        # As we use unitary command for robot, we add a normalization of gradient after the sum
        norm = np.sqrt(grad_x[i][j] ** 2 + grad_y[i][j] ** 2)
        if norm > 1.0:
            grad_x[i][j] = grad_x[i][j] / norm
            grad_y[i][j] = grad_y[i][j] / norm

fig, ax = plt.subplots(figsize=(len(x), len(y)))
ax.quiver(X, Y, grad_x, grad_y)
ax.add_patch(plt.Circle(goal_pos, min_r, color='y'))
ax.annotate("Goal", xy=goal_pos, fontsize=10, ha="center")

for k in range(n_obstacles):
    ax.add_patch(plt.Circle(obstacles_pos[k], min_r, color='m'))
    ax.annotate("Obstacle", xy=obstacles_pos[k], fontsize=8, ha="center")

ax.set_title('Combined Potential when Goal and Obstacle are different ')

plt.show()
