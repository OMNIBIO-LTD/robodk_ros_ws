#!/usr/bin/env python3
"""
ROS2 node that reads joint positions from a robot in RoboDK and publishes
them as sensor_msgs/JointState on /joint_states.

Parameters
----------
publish_rate         : float  – Hz (default 50.0)
topic                : str    – output topic (default '/joint_states')
joint_names          : str    – comma-separated joint names
                                (default 'joint_1,joint_2,joint_3,joint_4,joint_5,joint_6')
joint_signs          : str    – comma-separated ±1 per joint (default '1,1,1,1,1,1')
robodk_host          : str    – RoboDK API host (default 'localhost')
robodk_port          : int    – RoboDK API port (default 20500)
gripper_joint_name   : str    – extra joint appended to message for gripper slider
gripper_command_topic: str    – Float64 topic to read gripper value from
gripper_initial_value: float  – initial gripper position published (default -25.0)
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
        self.declare_parameter('joint_names',
                               'joint_1,joint_2,joint_3,joint_4,joint_5,joint_6')
        self.declare_parameter('joint_signs', '1,1,1,1,1,1')
        self.declare_parameter('robodk_host', 'localhost')
        self.declare_parameter('robodk_port', 20500)
        self.declare_parameter('gripper_joint_name', 'gripper_slider')
        self.declare_parameter('gripper_command_topic', '/gripper_command')
        self.declare_parameter('gripper_initial_value', -25.0)

        rate         = self.get_parameter('publish_rate').value
        topic        = self.get_parameter('topic').value
        names_str    = self.get_parameter('joint_names').value
        signs_str    = self.get_parameter('joint_signs').value
        rdk_host     = self.get_parameter('robodk_host').value
        rdk_port     = self.get_parameter('robodk_port').value
        self.gripper_joint_name = self.get_parameter('gripper_joint_name').value.strip()
        gripper_cmd_topic       = self.get_parameter('gripper_command_topic').value
        self.gripper_value      = self.get_parameter('gripper_initial_value').value

        self.joint_names = [n.strip() for n in names_str.split(',')]
        self.joint_signs = [float(s.strip()) for s in signs_str.split(',')]

        # --- RoboDK connection (no popup — auto-select first robot) ---
        self.get_logger().info(f'Connecting to RoboDK at {rdk_host}:{rdk_port} ...')
        self.RDK = Robolink(robodk_ip=rdk_host, port=rdk_port)

        # Try to get the first available robot without showing a dialog
        self.robot = None
        robots = self.RDK.ItemList(ITEM_TYPE_ROBOT)
        if robots:
            self.robot = robots[0]
            self.get_logger().info(f'Auto-selected robot: {self.robot.Name()}')
        else:
            self.get_logger().fatal('No robot found in RoboDK station')
            raise RuntimeError('No robot found in RoboDK station')

        # Validate DOF vs joint name / sign count
        dof = len(self.robot.Joints().list())
        if dof != len(self.joint_names):
            self.get_logger().warning(
                f'Robot has {dof} DOF but {len(self.joint_names)} joint names provided — '
                f'auto-generating: joint_1 ... joint_{dof}'
            )
            self.joint_names = [f'joint_{i + 1}' for i in range(dof)]
        if dof != len(self.joint_signs):
            self.joint_signs = [1.0] * dof

        # --- Publisher ---
        self.pub = self.create_publisher(JointState, topic, 10)

        # --- Gripper command subscription ---
        if self.gripper_joint_name:
            self.create_subscription(Float64, gripper_cmd_topic, self._gripper_cb, 10)

        self._rdk_host = rdk_host
        self._rdk_port = rdk_port

        # --- Timer ---
        self.create_timer(1.0 / rate, self._publish)

        published = list(self.joint_names)
        if self.gripper_joint_name:
            published.append(self.gripper_joint_name)
        self.get_logger().info(
            f'Publishing on "{topic}" at {rate} Hz | joints: {published}'
        )

    def _gripper_cb(self, msg: Float64):
        self.gripper_value = float(msg.data)

    def _reconnect(self):
        try:
            self.RDK = Robolink(robodk_ip=self._rdk_host, port=self._rdk_port)
            robots = self.RDK.ItemList(ITEM_TYPE_ROBOT)
            if robots:
                self.robot = robots[0]
                self.get_logger().info(f'Reconnected to RoboDK — robot: {self.robot.Name()}')
                return True
        except Exception as e:
            self.get_logger().warning(f'Reconnect failed: {e}')
        return False

    def _publish(self):
        try:
            joints_deg = self.robot.Joints().list()
        except Exception as e:
            self.get_logger().warning(f'RoboDK connection lost ({e}) — reconnecting...')
            if not self._reconnect():
                return
            try:
                joints_deg = self.robot.Joints().list()
            except Exception:
                return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
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
        node.get_logger().info('Shutting down joint_state_streamer')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
