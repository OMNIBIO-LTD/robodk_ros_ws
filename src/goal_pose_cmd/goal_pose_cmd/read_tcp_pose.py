#!/usr/bin/env python3
"""
ROS2 node that reads the current end-effector (TCP) pose from a robot
in RoboDK and publishes it as geometry_msgs/PoseStamped on /tcp_pose.

Publishes continuously at publish_rate Hz and also on demand when
any message arrives on /get_tcp_pose.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
from robodk.robomath import Pose_2_TxyzRxyz


def euler_to_quaternion(rx, ry, rz):
    cx = math.cos(rx / 2.0)
    sx = math.sin(rx / 2.0)
    cy = math.cos(ry / 2.0)
    sy = math.sin(ry / 2.0)
    cz = math.cos(rz / 2.0)
    sz = math.sin(rz / 2.0)
    return (
        sx * cy * cz + cx * sy * sz,   # x
        cx * sy * cz - sx * cy * sz,   # y
        cx * cy * sz + sx * sy * cz,   # z
        cx * cy * cz - sx * sy * sz,   # w
    )


class RoboDKPosePublisher(Node):
    def __init__(self):
        super().__init__('robodk_pose_publisher')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('pub_topic', '/tcp_pose')
        self.declare_parameter('trigger_topic', '/get_tcp_pose')
        self.declare_parameter('robodk_host', 'localhost')
        self.declare_parameter('robodk_port', 20500)

        rate       = self.get_parameter('publish_rate').get_parameter_value().double_value
        pub_topic  = self.get_parameter('pub_topic').get_parameter_value().string_value
        trig_topic = self.get_parameter('trigger_topic').get_parameter_value().string_value
        rdk_host   = self.get_parameter('robodk_host').get_parameter_value().string_value
        rdk_port   = self.get_parameter('robodk_port').get_parameter_value().integer_value

        self.get_logger().info(f'Connecting to RoboDK at {rdk_host}:{rdk_port}...')
        self.RDK = Robolink(robodk_ip=rdk_host, port=rdk_port)

        robots = self.RDK.ItemList(ITEM_TYPE_ROBOT)
        if not robots:
            self.get_logger().fatal('No robot found in RoboDK station')
            raise RuntimeError('No robot found in RoboDK station')
        self.robot = robots[0]
        self.get_logger().info(f'Connected to robot: {self.robot.Name()}')

        self.pose_pub = self.create_publisher(PoseStamped, pub_topic, 10)
        self.trigger_sub = self.create_subscription(Empty, trig_topic, self.trigger_callback, 10)
        self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(f'Publishing TCP pose on "{pub_topic}" at {rate} Hz')

    def read_and_publish(self):
        try:
            xyzrxyz = Pose_2_TxyzRxyz(self.robot.Pose())
        except Exception as e:
            self.get_logger().error(f'Failed to read robot pose: {e}')
            return

        # X/Y/Z in mm → metres for ROS convention
        qx, qy, qz, qw = euler_to_quaternion(xyzrxyz[3], xyzrxyz[4], xyzrxyz[5])

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.pose.position.x = xyzrxyz[0] / 1000.0
        msg.pose.position.y = xyzrxyz[1] / 1000.0
        msg.pose.position.z = xyzrxyz[2] / 1000.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

        self.get_logger().debug(
            f'TCP (mm): X={xyzrxyz[0]:.1f} Y={xyzrxyz[1]:.1f} Z={xyzrxyz[2]:.1f} | '
            f'Rx={math.degrees(xyzrxyz[3]):.1f} Ry={math.degrees(xyzrxyz[4]):.1f} '
            f'Rz={math.degrees(xyzrxyz[5]):.1f}'
        )

    def timer_callback(self):
        self.read_and_publish()

    def trigger_callback(self, msg: Empty):
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
