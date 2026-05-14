
**Workspace Setup:**

Before running any commands, build the workspace and source the setup script:

```bash
colcon build
source install/setup.bash
```

## Usage Instructions:

### Open Scene in IsaacSim:
```bash
src/usd_scenes/stackpro_democell.usd
```

- **Send goal to end-effector pose:**
	```bash
	ros2 run goal_pose_cmd goal_pose_cmd_node
	```

- **Read end-effector pose:**

NOTE: robot should already be imported in RoboDK and its frame should be set as active frame

	```bash
	ros2 run goal_pose_cmd read_tcp_pose_node
	ros2 topic echo /tcp_pose
	```

- **Bridge Isaac Sim with RoboDK:**
	```bash
	ros2 run goal_pose_cmd joint_state_streamer
	```
	*Note: Import the robot in Isaac Sim and create a joint state subscriber using Action Graphs with the topic name `/joint_states`.*

- **Sample URDF robot for Isaac Sim:**
	```bash
	~/robodk_ros_ws/src/kuka_robot_descriptions/kuka_quantec_support/urdf/kr240_r2900_2.urdf
	```