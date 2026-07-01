#!/usr/bin/env python3
"""
ROS2 node that traverses every pallet cell in sequence.

Loads the cell poses produced by ``utilities/save_cell_poses.py``
(``pallet_cell_poses.json``) and publishes each one as a
``geometry_msgs/PoseStamped`` on the goal-pose topic (default ``/goal_pose``),
waiting ``delay_sec`` seconds between cells so the robot (driven by
``goal_pose_cmd_node``) has time to move.

When every cell has been sent, it logs the full list of cells traversed.

Parameters
----------
poses_file : str
    Path to the pallet_cell_poses.json file.
topic : str
    Topic to publish PoseStamped goals on (must match goal_pose_cmd_node).
delay_sec : float
    Delay between commanding one cell and the next.
snake : bool
    If True, alternate column direction per row (boustrophedon) to
    minimise travel; if False, traverse in plain ascending (row, col) order.
frame_id : str
    If non-empty, overrides the frame_id stored in the JSON header.
"""

import json
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def load_cell_poses(filepath):
    """Load the {"row,col": pose_stamped_dict} mapping from JSON."""
    with open(filepath, "r") as f:
        return json.load(f)


def ordered_cells(poses, snake=True):
    """
    Return a list of (row, col, pose_dict) in traversal order.

    Sorted by row, then column. When ``snake`` is True the column order
    reverses on every other row so the path zig-zags instead of jumping
    back to column 1 at the start of each row.
    """
    parsed = []
    for key, pose in poses.items():
        row_str, col_str = key.split(",")
        parsed.append((int(row_str), int(col_str), pose))

    parsed.sort(key=lambda item: (item[0], item[1]))

    if not snake:
        return parsed

    ordered = []
    rows = sorted({row for row, _, _ in parsed})
    for i, row in enumerate(rows):
        row_cells = [c for c in parsed if c[0] == row]
        row_cells.sort(key=lambda item: item[1])
        if i % 2 == 1:  # reverse every other row
            row_cells.reverse()
        ordered.extend(row_cells)
    return ordered


def dict_to_pose_stamped(node, pose_dict, frame_id_override=""):
    """Convert a PoseStamped-shaped dict into a PoseStamped message."""
    msg = PoseStamped()

    header = pose_dict.get("header", {})
    msg.header.frame_id = frame_id_override or header.get("frame_id", "map")
    # Stamp at publish time (JSON stamp is a placeholder).
    msg.header.stamp = node.get_clock().now().to_msg()

    pose = pose_dict["pose"]
    pos = pose["position"]
    ori = pose["orientation"]

    msg.pose.position.x = float(pos["x"])
    msg.pose.position.y = float(pos["y"])
    msg.pose.position.z = float(pos["z"])
    msg.pose.orientation.x = float(ori["x"])
    msg.pose.orientation.y = float(ori["y"])
    msg.pose.orientation.z = float(ori["z"])
    msg.pose.orientation.w = float(ori["w"])

    return msg


class CellTraverser(Node):
    def __init__(self):
        super().__init__('cell_traverser')

        default_file = os.path.join(os.getcwd(), 'pallet_cell_poses.json')
        self.declare_parameter('poses_file', default_file)
        self.declare_parameter('topic', '/goal_pose')
        self.declare_parameter('delay_sec', 3.0)
        self.declare_parameter('snake', True)
        self.declare_parameter('frame_id', '')

        self.poses_file = self.get_parameter('poses_file').value
        self.topic = self.get_parameter('topic').value
        self.delay_sec = float(self.get_parameter('delay_sec').value)
        self.snake = bool(self.get_parameter('snake').value)
        self.frame_id = self.get_parameter('frame_id').value

        if not os.path.isfile(self.poses_file):
            self.get_logger().fatal(f'Poses file not found: {self.poses_file}')
            raise FileNotFoundError(self.poses_file)

        poses = load_cell_poses(self.poses_file)
        self.cells = ordered_cells(poses, snake=self.snake)
        self.get_logger().info(
            f'Loaded {len(self.cells)} cells from {self.poses_file}'
        )

        self.publisher = self.create_publisher(PoseStamped, self.topic, 10)
        self.get_logger().info(f'Publishing goals on {self.topic}')

        self.traversed = []  # list of (row, col) actually commanded

    def wait_for_subscriber(self, timeout_sec=10.0):
        """Give goal_pose_cmd_node time to connect before we start."""
        self.get_logger().info('Waiting for a subscriber on the goal topic...')
        waited = 0.0
        step = 0.2
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=step)
            waited += step
            if waited >= timeout_sec:
                self.get_logger().warning(
                    'No subscriber connected within timeout — publishing '
                    'anyway (make sure goal_pose_cmd_node is running).'
                )
                return
        self.get_logger().info('Subscriber connected.')

    def _sleep(self, seconds):
        """Sleep while keeping the node spinning."""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        self.wait_for_subscriber()

        total = len(self.cells)
        for idx, (row, col, pose_dict) in enumerate(self.cells, start=1):
            if not rclpy.ok():
                break

            msg = dict_to_pose_stamped(self, pose_dict, self.frame_id)
            self.publisher.publish(msg)
            self.traversed.append((row, col))

            pos = msg.pose.position
            self.get_logger().info(
                f'[{idx}/{total}] Commanded cell ({row},{col}) -> '
                f'x={pos.x:.3f} y={pos.y:.3f} z={pos.z:.3f}'
            )

            # Delay before the next cell (skip after the last one).
            if idx < total:
                self._sleep(self.delay_sec)

        self._log_summary()

    def _log_summary(self):
        cells_str = ', '.join(f'({r},{c})' for r, c in self.traversed)
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            f'Traversal complete. {len(self.traversed)} cells traversed:'
        )
        self.get_logger().info(cells_str)
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = CellTraverser()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — cells traversed so far:')
        node._log_summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
