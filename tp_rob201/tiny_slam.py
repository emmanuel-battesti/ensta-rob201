""" A simple robotics navigation code including SLAM, exploration, planning """

import cv2
import numpy as np
from occupancy_grid import OccupancyGrid


class TinySlam:
    """Simple occupancy grid SLAM."""

    def __init__(self, occupancy_grid: OccupancyGrid):
        # Store reference to the occupancy grid for map updates
        self.grid = occupancy_grid
        # Current offset from odometry frame to map frame
        self.odom_pose_ref = np.array([0, 0, 0])

    def _score(self, lidar, pose):
        """
        Compute alignment score between a LIDAR scan and the occupancy map.

        Projects each valid laser hit into the map frame using 'pose',
        converts world coordinates to map indices, and sums occupancy values.

        Args:
            lidar:  Lidar sensor providing get_sensor_values() and get_ray_angles().
            pose:   Pose [x, y, theta] in the map frame.

        Returns:
            score:  Sum of occupancy_map at projected scan endpoints.
        """
        distance = lidar.get_sensor_values()
        angle    = lidar.get_ray_angles()
        # Ignore points beyond 95% of maximum range
        max_range = lidar.max_range * 0.95
        valid     = (distance > 0) & (distance < max_range)
        # Filter valid measurements
        distance = distance[valid]
        angle    = angle[valid]

        # Compute world coordinates of each valid beam endpoint
        x_world = pose[0] + distance * np.cos(pose[2] + angle)
        y_world = pose[1] + distance * np.sin(pose[2] + angle)
        # Convert to map indices
        xi, yi = self.grid.conv_world_to_map(x_world, y_world)

        # Keep only indices within map bounds
        mask = (xi < self.grid.x_max_map) & (yi < self.grid.y_max_map)
        xi, yi = xi[mask], yi[mask]

        # Sum occupancy values at those indices to get the score
        score = np.sum(self.grid.occupancy_map[xi, yi])
        return score

    def get_corrected_pose(self, odom_pose, odom_pose_ref=None):
        """
        Convert raw odometry pose into map frame pose using current odom_pose_ref.

        Args:
            odom_pose:      [xO, yO, thetaO] raw odometry pose.
            odom_pose_ref:  Reference odometry origin in map frame (optional).

        Returns:
            corrected_pose: [x, y, theta] in map frame.
        """
        if odom_pose_ref is None:
            odom_pose_ref = self.odom_pose_ref

        xO, yO, thetaO       = odom_pose
        xOref, yOref, thetaOref = self.odom_pose_ref

        # Compute combined orientation
        theta = thetaOref + thetaO

        # Compute distance and bearing in odometry frame
        dO     = np.hypot(xO, yO)
        alphaO = np.arctan2(yO, xO)

        # Transform into map frame
        x = xOref + dO * np.cos(thetaOref + alphaO)
        y = yOref + dO * np.sin(thetaOref + alphaO)

        return np.array([x, y, theta])

    def localise(self, lidar, raw_odom_pose):
        """
        Perform scan-matching by randomly perturbing odom_pose_ref to maximize alignment.

        Uses a simple hill-climbing approach:
          - Evaluate initial alignment score.
          - Propose random small offsets; accept if score improves.
          - Stop after N consecutive non-improving proposals.

        Args:
            lidar:          Lidar sensor object.
            raw_odom_pose:  Current odometry pose [x, y, theta].

        Returns:
            best_score:     Highest alignment score found.
        """
        # Start from current reference
        best_pose_ref = self.odom_pose_ref.copy()
        best_score    = self._score(lidar, self.get_corrected_pose(raw_odom_pose, best_pose_ref))

        N                  = 200    # Max non-improving iterations
        sigma              = 0.4    # Stddev for random offset proposals
        no_improvement_cnt = 0

        # Hill-climb loop
        while no_improvement_cnt < N:
            # Propose a small random offset in x, y, theta
            offset = np.random.normal(0, sigma, size=3)
            test_ref = best_pose_ref + offset

            # Compute score at the new reference
            score = self._score(lidar, self.get_corrected_pose(raw_odom_pose, test_ref))

            if score > best_score:
                # Accept better reference
                best_score    = score
                best_pose_ref = test_ref
                no_improvement_cnt = 0
            else:
                no_improvement_cnt += 1

        # Update stored reference and return best score
        self.odom_pose_ref = best_pose_ref
        return best_score

    def update_map(self, lidar, pose):
        """
        Update the occupancy grid with a new LIDAR observation.

        For each beam:
          - Carve out free space along the ray (negative updates).
          - Mark the endpoint as occupied (positive update).
        Then clamp values to a fixed range.

        Args:
            lidar:  Lidar sensor object.
            pose:   Corrected robot pose [x, y, theta] in map frame.
        """
        # Get all beam distances
        distances = lidar.get_sensor_values()
        angles    = lidar.get_ray_angles() + pose[2]

        # Compute world endpoints for each beam
        xi = pose[0] + distances * np.cos(angles)
        yi = pose[1] + distances * np.sin(angles)

        # Mark free space along each ray
        for x_end, y_end in zip(xi, yi):
            # Subtract 0.5 from each cell along the line
            self.grid.add_value_along_line(pose[0], pose[1], x_end, y_end, -0.5)

        # Mark obstacles at the endpoints
        self.grid.add_map_points(xi, yi, 4)

        # Clamp map values between -10 and +40 to prevent runaway beliefs
        self.grid.occupancy_map = np.clip(self.grid.occupancy_map, -10, 40)

    def compute(self):
        """
        Dummy compute function for profiling exercises (unused in TP2).

        Demonstrates a slow polar-to-Cartesian conversion using Python loops.
        """
        ranges     = np.random.rand(3600)
        ray_angles = np.arange(-np.pi, np.pi, np.pi / 1800)

        points = []
        for i in range(3600):
            x = ranges[i] * np.cos(ray_angles[i])
            y = ranges[i] * np.sin(ray_angles[i])
            points.append([x, y])
        return np.array(points).T
