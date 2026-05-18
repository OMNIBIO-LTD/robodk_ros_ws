#!/usr/bin/env python3
"""Headless gripper publisher — publishes at 10 Hz to both topics:
  /gripper_command  (Float64) — for joint_state_streamer compatibility
  /joint_states     (JointState, gripper_slider only) — drives Isaac directly

Usage: python3 gripper_pub.py <value>
  value: -25.0 = open, 0.0 = stop, 25.0 = close
"""
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def main():
    value = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    rclpy.init()
    node = Node('gripper_pub')

    cmd_pub = node.create_publisher(Float64, '/gripper_command', 10)
    js_pub = node.create_publisher(JointState, '/joint_states', 10)

    cmd_msg = Float64()
    cmd_msg.data = value

    def _publish():
        cmd_pub.publish(cmd_msg)
        js = JointState()
        js.header.stamp = node.get_clock().now().to_msg()
        js.name = ['gripper_slider']
        js.position = [value]
        js_pub.publish(js)

    node.create_timer(0.1, _publish)
    node.get_logger().info(f'gripper_pub: value={value} → /gripper_command + /joint_states at 10 Hz')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
