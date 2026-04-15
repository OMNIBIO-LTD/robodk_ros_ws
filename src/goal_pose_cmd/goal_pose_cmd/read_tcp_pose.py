#!/usr/bin/env python3
"""
ROS2 node that reads the current end-effector (TCP) pose from a robot
in RoboDK and publishes it as geometry_msgs/PoseStamped.

It operates in two modes:
  1. Continuous: publishes the TCP pose at a fixed rate (default 10 Hz)
  2. On-demand: subscribes to a trigger topic; when any message arrives,
     it reads and publishes the current TCP pose

Both modes run simultaneously.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME
from robodk.robomath import Pose_2_TxyzRxyz


def euler_to_quaternion(rx, ry, rz):
    """
    Convert Euler angles (in radians, XYZ convention as returned
    by Pose_2_TxyzRxyz) to quaternion (x, y, z, w).
    """
    cx = math.cos(rx / 2.0)
    sx = math.sin(rx / 2.0)
    cy = math.cos(ry / 2.0)
    sy = math.sin(ry / 2.0)
    cz = math.cos(rz / 2.0)
    sz = math.sin(rz / 2.0)

    w = cx * cy * cz - sx * sy * sz
    x = sx * cy * cz + cx * sy * sz
    y = cx * sy * cz - sx * cy * sz
    z = cx * cy * sz + sx * sy * cz

    return x, y, z, w


class RoboDKPosePublisher(Node):
    def __init__(self):
        super().__init__('robodk_pose_publisher')

        # --- Parameters ---
        self.declare_parameter('publish_rate', 10.0)        # Hz
        self.declare_parameter('pub_topic', '/tcp_pose')
        self.declare_parameter('trigger_topic', '/get_tcp_pose')

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        trigger_topic = self.get_parameter('trigger_topic').get_parameter_value().string_value

        # --- RoboDK setup (same pattern as the reference script) ---
        self.get_logger().info('Connecting to RoboDK...')
        self.RDK = Robolink()

        self.robot = self.RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
        if not self.robot.Valid():
            self.get_logger().fatal('No robot selected or available')
            raise RuntimeError('No robot selected or available')

        self.reference_frame = self.robot.getLink(ITEM_TYPE_FRAME)
        if not self.reference_frame.Valid():
            self.get_logger().fatal('Robot has no valid reference frame')
            raise RuntimeError('Robot has no valid reference frame')

        # Set frames explicitly so Pose() returns TCP w.r.t. reference frame
        self.robot.setPoseFrame(self.reference_frame)
        self.robot.setPoseTool(self.robot.PoseTool())

        self.get_logger().info(f'Connected to robot: {self.robot.Name()}')

        # --- Publisher: current TCP pose ---
        self.pose_pub = self.create_publisher(PoseStamped, pub_topic, 10)

        # --- Subscriber: on-demand trigger ---
        self.trigger_sub = self.create_subscription(
            Empty,
            trigger_topic,
            self.trigger_callback,
            10
        )

        # --- Timer: continuous publishing ---
        period = 1.0 / rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Publishing TCP pose on "{pub_topic}" at {rate} Hz'
        )
        self.get_logger().info(
            f'Trigger a single read by publishing to "{trigger_topic}"'
        )

    def read_and_publish(self):
        """Read the current TCP pose from RoboDK and publish it."""
        try:
            pose = self.robot.Pose()
        except Exception as e:
            self.get_logger().error(f'Failed to read robot pose: {e}')
            return

        # Pose_2_TxyzRxyz always works — returns [X, Y, Z, Rx, Ry, Rz]
        # X,Y,Z in mm; Rx,Ry,Rz in radians
        xyzrxyz = Pose_2_TxyzRxyz(pose)

        # Convert Euler angles to quaternion
        qx, qy, qz, qw = euler_to_quaternion(
            xyzrxyz[3], xyzrxyz[4], xyzrxyz[5]
        )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.reference_frame.Name()

        # mm -> meters
        msg.pose.position.x = xyzrxyz[0] / 1000.0
        msg.pose.position.y = xyzrxyz[1] / 1000.0
        msg.pose.position.z = xyzrxyz[2] / 1000.0

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_pub.publish(msg)

        self.get_logger().debug(
            f'TCP pose (mm): X={xyzrxyz[0]:.1f} Y={xyzrxyz[1]:.1f} '
            f'Z={xyzrxyz[2]:.1f} | '
            f'Rx={math.degrees(xyzrxyz[3]):.1f} '
            f'Ry={math.degrees(xyzrxyz[4]):.1f} '
            f'Rz={math.degrees(xyzrxyz[5]):.1f}'
        )

    def timer_callback(self):
        """Continuous publishing at the configured rate."""
        self.read_and_publish()

    def trigger_callback(self, msg: Empty):
        """On-demand: publish once when triggered."""
        self.get_logger().info('Trigger received — reading TCP pose')
        self.read_and_publish()


def main(args=None):
    rclpy.init(args=args)
    node = RoboDKPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()