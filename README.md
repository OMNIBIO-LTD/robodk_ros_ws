to send goal to ee pose: ros2 run goal_pose_cmd goal_pose_cmd_node

to read end-effector pose: ros2 run goal_pose_cmd read_tcp_pose_node then ros2 topic echo /tcp_pose

to bridge isaacsim with robodk: ros2 run goal_pose_cmd joint_state_streamer (for this  you should import robot in isaacsim, and create a joint state subscriber)

 