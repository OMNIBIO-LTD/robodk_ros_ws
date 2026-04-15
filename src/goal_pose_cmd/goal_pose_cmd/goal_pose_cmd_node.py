#!/usr/bin/env python3
"""
ROS2 subscriber that receives goal poses (geometry_msgs/PoseStamped)
and commands a robot in RoboDK to move to those poses.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import math
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME
from robodk.robomath import TxyzRxyz_2_Pose


def quaternion_to_euler(q):
    """Convert quaternion (x, y, z, w) to roll, pitch, yaw in radians."""
    # Roll (X)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (Z)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class RoboDKGoalSubscriber(Node):
    def __init__(self):
        super().__init__('robodk_goal_subscriber')

        # --- RoboDK setup (mirrors your reference script) ---
        self.get_logger().info('Connecting to RoboDK...')
        self.RDK = Robolink()

        # Select a robot
        self.robot = self.RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
        if not self.robot.Valid():
            self.get_logger().fatal('No robot selected or available')
            raise RuntimeError('No robot selected or available')

        # Get the reference frame
        self.reference_frame = self.robot.getLink(ITEM_TYPE_FRAME)
        if not self.reference_frame.Valid():
            self.get_logger().fatal('Robot has no valid reference frame')
            raise RuntimeError('Robot has no valid reference frame')

        # Configure robot defaults
        self.robot.setPoseFrame(self.reference_frame)
        self.robot.setPoseTool(self.robot.PoseTool())
        self.robot.setRounding(10)
        self.robot.setSpeed(200)  # mm/s

        self.get_logger().info(
            f'Connected to robot: {self.robot.Name()}'
        )

        # Declare parameters for movement type and speed
        self.declare_parameter('move_type', 'joint')   # 'linear' or 'joint'
        self.declare_parameter('speed_mm_s', 200.0)
        self.declare_parameter('topic', '/goal_pose')

        topic = self.get_parameter('topic').get_parameter_value().string_value

        # --- ROS2 subscription ---
        self.subscription = self.create_subscription(
            PoseStamped,
            topic,
            self.goal_pose_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {topic}')

        self.target_count = 0

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f'Received goal pose on frame: {msg.header.frame_id}'
        )

        # Extract position (ROS uses meters, RoboDK uses mm)
        tx = msg.pose.position.x * 1000.0
        ty = msg.pose.position.y * 1000.0
        tz = msg.pose.position.z * 1000.0

        # Convert quaternion to Euler angles (radians)
        rx, ry, rz = quaternion_to_euler(msg.pose.orientation)

        # Convert to degrees for RoboDK's TxyzRxyz format
        rx_deg = math.degrees(rx)
        ry_deg = math.degrees(ry)
        rz_deg = math.degrees(rz)

        self.get_logger().info(
            f'Target pose: x={tx:.1f} y={ty:.1f} z={tz:.1f} '
            f'rx={rx_deg:.1f} ry={ry_deg:.1f} rz={rz_deg:.1f}'
        )

        # Build the 4x4 pose matrix (same approach as your reference)
        pose = TxyzRxyz_2_Pose([tx, ty, tz, rx, ry, rz])

        # Create a named target in RoboDK
        target_name = f'GoalPose_{self.target_count}'
        new_target = self.RDK.AddTarget(target_name, self.reference_frame)
        new_target.setPose(pose)
        self.get_logger().info(f'Created RoboDK target: {target_name}')

        # Read movement parameters
        move_type = (
            self.get_parameter('move_type')
            .get_parameter_value()
            .string_value
        )
        speed = (
            self.get_parameter('speed_mm_s')
            .get_parameter_value()
            .double_value
        )
        self.robot.setSpeed(speed)

        # Move the robot
        try:
            if move_type == 'joint':
                self.robot.MoveJ(pose)
                self.get_logger().info(f'MoveJ to {target_name} complete')
            else:
                self.robot.MoveL(pose)
                self.get_logger().info(f'MoveL to {target_name} complete')
        except Exception as e:
            self.get_logger().error(f'Movement failed: {e}')

        self.target_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = RoboDKGoalSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()