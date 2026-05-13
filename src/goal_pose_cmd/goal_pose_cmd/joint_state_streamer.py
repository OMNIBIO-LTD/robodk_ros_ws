#!/usr/bin/env python3
"""
ROS2 node that reads joint positions from a robot in RoboDK and publishes
them as sensor_msgs/JointState.

Parameters
----------
publish_rate  : float  – publishing frequency in Hz (default 50.0)
topic         : str    – output topic name (default '/joint_states')
joint_names   : str    – comma-separated joint names
                         (default 'joint_1,joint_2,joint_3,joint_4,joint_5,joint_6')
joint_signs   : str    – comma-separated signs (+1 or -1) per joint, used to
                         correct axis direction mismatches between RoboDK and
                         the target simulator (default '1,1,1,1,1,1')
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT


class JointStateStreamer(Node):
    def __init__(self):
        super().__init__('joint_state_streamer')

        # --- Parameters ---
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('topic', '/joint_states')
        self.declare_parameter(
            'joint_names',
            'joint_1,joint_2,joint_3,joint_4,joint_5,joint_6'
        )
        self.declare_parameter('joint_signs', '1,1,1,1,1,1')
        self.declare_parameter('gripper_joint_name', 'gripper_slider')
        self.declare_parameter('gripper_command_topic', '/gripper_command')
        self.declare_parameter('gripper_initial_value', -25.0)

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        names_str = self.get_parameter('joint_names').get_parameter_value().string_value
        signs_str = self.get_parameter('joint_signs').get_parameter_value().string_value
        self.joint_names = [n.strip() for n in names_str.split(',')]
        self.joint_signs = [float(s.strip()) for s in signs_str.split(',')]

        self.gripper_joint_name = (
            self.get_parameter('gripper_joint_name').get_parameter_value().string_value.strip()
        )
        gripper_cmd_topic = (
            self.get_parameter('gripper_command_topic').get_parameter_value().string_value
        )
        self.gripper_value = (
            self.get_parameter('gripper_initial_value').get_parameter_value().double_value
        )

        # --- RoboDK connection ---
        self.get_logger().info('Connecting to RoboDK...')
        self.RDK = Robolink()

        self.robot = self.RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
        if not self.robot.Valid():
            self.get_logger().fatal('No robot selected or available in RoboDK')
            raise RuntimeError('No robot selected or available in RoboDK')

        self.get_logger().info(f'Connected to robot: {self.robot.Name()}')

        # Validate DOF vs joint name / sign count
        dof = len(self.robot.Joints().list())
        if dof != len(self.joint_names):
            self.get_logger().warning(
                f'Robot has {dof} DOF but {len(self.joint_names)} joint names were given. '
                f'Auto-generating names: joint_1 ... joint_{dof}'
            )
            self.joint_names = [f'joint_{i + 1}' for i in range(dof)]
        if dof != len(self.joint_signs):
            self.get_logger().warning(
                f'joint_signs length ({len(self.joint_signs)}) != DOF ({dof}). '
                f'Defaulting all signs to +1.'
            )
            self.joint_signs = [1.0] * dof

        # --- Publisher ---
        self.pub = self.create_publisher(JointState, topic, 10)

        # --- Gripper command subscription ---
        if self.gripper_joint_name:
            self.create_subscription(
                Float64, gripper_cmd_topic, self._gripper_cb, 10
            )

        # --- Timer ---
        self.create_timer(1.0 / rate, self._publish)

        published_joints = list(self.joint_names)
        if self.gripper_joint_name:
            published_joints.append(self.gripper_joint_name)
        self.get_logger().info(
            f'Publishing joint states on "{topic}" at {rate} Hz '
            f'| joints: {published_joints}'
        )
        if self.gripper_joint_name:
            self.get_logger().info(
                f'Listening for gripper commands on "{gripper_cmd_topic}" '
                f'(initial value {self.gripper_value})'
            )

    def _gripper_cb(self, msg: Float64):
        self.gripper_value = float(msg.data)

    def _publish(self):
        try:
            # robot.Joints() returns a Mat; .list() gives a flat Python list in degrees
            joints_deg = self.robot.Joints().list()
        except Exception as e:
            self.get_logger().error(f'Failed to read joints from RoboDK: {e}')
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        # RoboDK stores angles in degrees; ROS uses radians.
        # Apply per-joint sign correction for axis direction mismatches.
        positions = [
            s * math.radians(j)
            for j, s in zip(joints_deg, self.joint_signs)
        ]
        names = list(self.joint_names)

        if self.gripper_joint_name:
            names.append(self.gripper_joint_name)
            positions.append(self.gripper_value)

        msg.name = names
        msg.position = positions

        self.pub.publish(msg)

        self.get_logger().debug(
            'joints (deg): ' + '  '.join(f'{j:.2f}' for j in joints_deg)
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointStateStreamer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down joint_state_streamer...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
