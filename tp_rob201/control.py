""" A set of robotics control functions """

import random
import numpy as np
import math


def reactive_obst_avoid(self, lidar, rotation):
    """
    Simple reactive obstacle avoidance.

    This function uses a naive strategy:
      - If currently rotating == 0, check front distance:
          • If clear (>80), move forward at 0.2 m/s.
          • Else, rotate randomly left or right.
      - If currently rotating != 0, check sides at ±30°:
          • If both clear (>100), move forward at 0.3 m/s and reset rotation.
          • Else, keep rotating in the same direction.

    Args:
        self:           Robot instance (provides odometry, etc.).
        lidar:          Lidar sensor object (must implement get_sensor_values()).
        rotation (float): Current rotation command.

    Returns:
        command (dict): { "forward": <linear speed>, "rotation": <angular speed> }.
    """
    # Read all laser distances
    laser_dist = lidar.get_sensor_values()

    # No rotation requested: look straight ahead
    if rotation == 0:
        # If path ahead is clear, go forward
        if laser_dist[180] > 80:
            speed = 0.2
            rotation_speed = 0.0
            rotation = 0
        else:
            # Otherwise pick a random turn direction
            rotation = random.choice([-0.2, 0.2])
            speed = 0.0
            rotation_speed = rotation
    else:
        # Currently turning: check ±30° side distances
        if laser_dist[150] > 100 and laser_dist[210] > 100:
            # If both sides are clear, resume forward motion
            speed = 0.3
            rotation_speed = 0.0
            rotation = 0
        else:
            # Otherwise continue rotating in same direction
            speed = 0.0
            rotation_speed = rotation

    command = {
        "forward": speed,
        "rotation": rotation_speed
    }
    return command


def potential_field_control(lidar, current_pose, goal_pose):
    """
    Potential field–based motion controller.

    Combines attractive force toward goal and repulsive force from the
    nearest obstacle.

    Attractive force:
      F_attr = K * (delta_pos / max(d, d_lim))

    Repulsive force (only if obstacle is within d_safe and closer than 20 m):
      F_rep = K_obs / r^3 * (1/r - 1/d_safe) * obs_vector

    Resulting command:
      - Linear speed ∝ kspeed * ‖F_total‖, clipped to [-0.5, 0.5].
      - Angular speed ∝ Kw * heading_error, clipped to [-1, 1].
      - Arrival if distance to goal < 15.

    Args:
        lidar:         Lidar sensor object (get_sensor_values, get_ray_angles).
        current_pose:  [x, y, theta] in world frame.
        goal_pose:     [x, y, theta] target in world frame.

    Returns:
        command (dict), reached (bool):
          - command: { "forward": <m/s>, "rotation": <rad/s> }
          - reached: True if within 15 m of goal, else False.
    """
    # Tunable parameters
    K      = 0.5   # Attractive gain
    kspeed = 0.2   # Base speed gain
    d_lim  = 50   # Distance threshold for limiting attraction
    K_obs  = -2500 # Repulsive gain (negative for repulsion)
    Kw     = 0.3   # Rotational gain
    d_safe = 100     # Safety distance for obstacle influence

    # Compute attractive force toward goal
    current_pos = np.array(current_pose[:2])
    goal_pos    = np.array(goal_pose[:2])
    delta_pos   = goal_pos - current_pos
    d           = np.linalg.norm(delta_pos)
    if d > d_lim:
        F_attr = (K * delta_pos) / d
    else:
        F_attr = (K * delta_pos) / d_lim

    # Find the closest obstacle
    lidar_values = lidar.get_sensor_values()
    angles       = lidar.get_ray_angles()
    min_index    = np.argmin(lidar_values)
    min_dist     = lidar_values[min_index]
    angle_global = angles[min_index] + current_pose[2]

    obs_vector = np.array([
        math.cos(angle_global) * min_dist,
        math.sin(angle_global) * min_dist
    ])

    # Compute repulsive force if obstacle is within safety radius and very close
    force_rep = np.array([0.0, 0.0])
    if min_dist < d_safe and min_dist < 20:
        # Inverse-cube falloff * (1/r - 1/d_safe) term
        force_rep = (K_obs / min_dist**3) * (1/min_dist - 1/d_safe) * obs_vector

    # Sum total force
    f_total = F_attr + force_rep

    # Compute desired heading
    theta_desired = math.atan2(f_total[1], f_total[0])
    theta_error   = theta_desired - current_pose[2]
    # Wrap to [-π, π]
    theta_error   = math.atan2(math.sin(theta_error), math.cos(theta_error))

    # Compute forward speed proportional to force magnitude,
    # but reduce when turning sharply
    magnitude = np.linalg.norm(f_total)
    if abs(theta_error) < (np.pi / 2):
        speed = kspeed * magnitude
    else:
        speed = kspeed * magnitude * ((np.pi) / abs(theta_error))
    speed = np.clip(speed, -0.4, 0.4)

    # Compute rotation command
    rot_cmd = np.clip(Kw * theta_error, -1, 1)

    # Check arrival condition
    if d < 15:
        return {"forward": 0.0, "rotation": 0.0}, True

    return {"forward": speed, "rotation": rot_cmd}, False
