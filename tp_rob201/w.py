 #     if self.current_goal_idx >= len(self.goals):
        #         if not self.x:
        #             self.x = True
        #             start_pose = corected_pose
        #             goal_pose = np.array([0.0, 0.0, 0.0])
        #             self.traj = self.planner.plan(raw_pose, goal_pose)
        #             traj_np = np.array(self.traj).T
        #             self.occupancy_grid.display_cv(start_pose, goal_pose, traj_np)
        #         if self.trag_index < len(self.traj):
        #             next_goal = self.traj[self.trag_index]
        #             self.trag_index += 1
        #             cmd, _ = potential_field_control(self.lidar(), corected_pose, next_goal, 0)
        #             return cmd
        #         else:
        #             print("End of trajectory")
        #             return {"forward": 0.0, "rotation": 0.0} 
        #     '''
