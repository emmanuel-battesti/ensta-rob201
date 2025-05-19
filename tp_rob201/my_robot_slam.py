"""
Robot controller definition
SLAM-based reactive navigation to two sequential goals via potential fields
"""
import numpy as np
import random
from place_bot.entities.robot_abstract import RobotAbstract
from place_bot.entities.odometer import OdometerParams
from place_bot.entities.lidar import LidarParams
from planner import Planner

from tiny_slam import TinySlam
from control import potential_field_control
from occupancy_grid import OccupancyGrid


class MyRobotSlam(RobotAbstract):
    """A robot controller with SLAM and reactive potential-field navigation."""

    def __init__(
        self,
        lidar_params: LidarParams = LidarParams(),
        odometer_params: OdometerParams = OdometerParams()
    ):
        # Initialize the base robot with LIDAR and odometry settings
        super().__init__(should_display_lidar=False,
                         lidar_params=lidar_params,
                         odometer_params=odometer_params)

        # Set up the occupancy grid based on a predefined area and robot start position
        size_area = (1400, 1000)
        robot_position = (439.0, 195)
        self.occupancy_grid = OccupancyGrid(
            x_min=-(size_area[0] / 2 + robot_position[0]),
            x_max=(size_area[0] / 2 - robot_position[0]),
            y_min=-(size_area[1] / 2 + robot_position[1]),
            y_max=(size_area[1] / 2 - robot_position[1]),
            resolution=2
        )

        # Initialize SLAM and path planner
        self.tiny_slam = TinySlam(self.occupancy_grid)
        self.planner = Planner(self.occupancy_grid)

        # Define the list of goals the robot will visit sequentially
        self.goals = [
            (-200, 25, 0),
            (-400, 0, 0),
            (-700, -10, 0),
            (-900, -280, 0)
        ]

        # Internal state variables
        self.current_goal_idx = 0  # index of the next goal to reach
        self.iteration = 0         # number of SLAM iterations performed
        self.rotation = 0.0        # current rotation command
        self.trag_index = 0        # index along the return trajectory
        self.x = False             # flag indicating return phase has started
        self.traj = []             # full return trajectory
        self.indice = 0            # alias for current_goal_idx
        self.correct = np.array([0.0, 0.0, 0.0])  # corrected pose from SLAM
        self.return_goal_pose = None  # store return destination pose
        self.return_traj_np = None    # store return trajectory for display

    def control(self):
        """Main control loop called each simulation step."""
        return self.control_tp2()

    def control_tp1(self):
        """
        TP1 control: basic SLAM map building + random obstacle avoidance.
        """
        self.tiny_slam.compute()  # dummy compute for profiler exercise
        command = self.reactive_obst_avoid(self.lidar(), self.rotation)
        self.rotation = command["rotation"]
        return command

    def control_tp2(self):
        """
        TP2 control:
        1) Build map and localize via SLAM.
        2) Navigate through predefined goals with potential fields.
        3) After goals, plan and follow a return path back to origin.
        """
        command = {"forward": 0.0, "rotation": 0.0}

        # 1) SLAM localization update
        raw_odom_pose = self.odometer_values()
        best_score = self.tiny_slam.localise(self.lidar(), raw_odom_pose)
        self.iteration += 1
        # Correct the odometry pose to map frame
        self.correct = self.tiny_slam.get_corrected_pose(raw_odom_pose)

        # 2) Update map during initial iterations or while exploring goals
        if (best_score > 5000 or self.iteration < 150) and self.indice < len(self.goals):
            self.tiny_slam.update_map(self.lidar(), raw_odom_pose)
            print("Iteration:", self.iteration)

        # Always display the occupancy grid based on raw odometry for mapping
        self.occupancy_grid.display_cv(raw_odom_pose)

        # 3) Exploration phase: navigate through each goal
        if self.iteration > 150 and self.indice < len(self.goals):
            print("Indice:", self.indice, "Best score:", best_score)
            goal = self.goals[self.indice]
            command, reached = potential_field_control(self.lidar(), self.correct, goal)
            # If close enough to the goal, advance to the next one
            if reached:
                self.indice += 1

        # 4) Return phase: after all goals reached, plan a path back to (0,0,0)
        elif self.indice >= len(self.goals):
            # Initialize return trajectory only once
            if not self.x:
                self.x = True
                self.trag_index = 0

                # Plan return path using A* planner
                goal_pose = np.array([0.0, 0.0, 0.0])
                full_traj = self.planner.plan(self.correct, goal_pose)
                if full_traj is None:
                    print("Unable to plan return!")
                    return {"forward": 0.0, "rotation": 0.0}

                print("Raw trajectory length:", len(full_traj))

                # Subsample trajectory for fewer waypoints
                N = max(1, len(full_traj) // 20)
                self.traj = full_traj[::N] + [full_traj[-1]]
                print(f"Subsampled trajectory: {len(self.traj)} points (step = {N})")

                # Store for continuous display
                self.return_goal_pose = goal_pose
                self.return_traj_np = np.array(self.traj).T

            # Continuously display the return path overlay
            self.occupancy_grid.display_cv(
                self.correct,
                self.return_goal_pose,
                self.return_traj_np
            )

            # Follow each waypoint in the subsampled trajectory
            if self.trag_index < len(self.traj):
                next_goal = self.traj[self.trag_index]
                dist = np.linalg.norm(self.correct[:2] - next_goal[:2])
                print(f"[Return] target #{self.trag_index} = {next_goal[:2]}, dist = {dist:.1f}")
                cmd, reached = potential_field_control(self.lidar(), self.correct, next_goal)
                if reached:
                    self.trag_index += 1
                return cmd

            # End of return path: stop the robot
            print("End of return trajectory")
            return {"forward": 0.0, "rotation": 0.0}

        # Default: no movement
        return command
