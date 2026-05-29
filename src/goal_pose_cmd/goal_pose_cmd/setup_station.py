#!/usr/bin/env python3
"""
Set up a RoboDK station matching the stackpro_democell.usd scene.

Loads the KUKA KR16-2 robot, attaches a gripper tool,
places four Lipton-box-sized frame markers on a pickup table, and
creates a RoboDK program that picks and stacks them.

Usage (standalone):
    python3 setup_station.py [--host localhost] [--port 20500] [--clear]

Usage (as ROS2 node / via API):
    ros2 run goal_pose_cmd setup_station
"""

import argparse

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME, ITEM_TYPE_TOOL
from robodk.robomath import TxyzRxyz_2_Pose, eye


# ---------------------------------------------------------------------------
# Station geometry  (all units: mm, radians)
# ---------------------------------------------------------------------------

ROBOT_LIB = "/home/user/RoboDK/Library/KUKA-KR-16-2.robot"

# Gripper TCP offset: 160 mm along Z from robot flange
TOOL_POSE = TxyzRxyz_2_Pose([0, 0, 160, 0, 0, 0])

# Box dimensions (Lipton box approximation, mm)
BOX_W, BOX_D, BOX_H = 80, 60, 120

# Four pickup positions — row of boxes 650 mm in front of robot, on a table at Z=200
PICKUP_TABLE_Z = 200
PICKUP_Y       = 650
PICKUP_X_START = -165   # centres four boxes around X=0

PICKUP_POSITIONS = [
    [PICKUP_X_START + i * (BOX_W + 30), PICKUP_Y, PICKUP_TABLE_Z + BOX_H]
    for i in range(4)
]

# Stack drop column: 350 mm to the right, 400 mm forward
STACK_X, STACK_Y = 350, 400
STACK_Z_BASE     = PICKUP_TABLE_Z + BOX_H

# Vertical clearance above grasp / place point
APPROACH_LIFT = 150

# Gripper pointing straight down (180° around X)
RX_DOWN = 3.14159


# ---------------------------------------------------------------------------
def _add_robot(RDK):
    robot = RDK.Item("KUKA KR 16-2", ITEM_TYPE_ROBOT)
    if robot.Valid():
        print("[setup] reusing existing robot:", robot.Name())
        return robot
    print("[setup] loading robot from library …")
    robot = RDK.AddFile(ROBOT_LIB)
    if not robot.Valid():
        raise RuntimeError(f"Failed to load robot from: {ROBOT_LIB}")
    robot.setPose(eye(4))
    print("[setup] robot loaded:", robot.Name())
    return robot


def _add_tool(RDK, robot):
    tool = RDK.Item("Gripper", ITEM_TYPE_TOOL)
    if tool.Valid():
        print("[setup] reusing existing tool:", tool.Name())
        robot.setPoseTool(TOOL_POSE)
        return tool
    tool = robot.AddTool(TOOL_POSE, "Gripper")
    robot.setPoseTool(TOOL_POSE)
    print("[setup] gripper tool added (TCP +160 mm Z)")
    return tool


def _add_box_frames(RDK):
    """Add a named reference frame for each box pickup position."""
    frames = []
    for i, (x, y, z) in enumerate(PICKUP_POSITIONS):
        name = f"LiptonBox_{i + 1}"
        f = RDK.Item(name, ITEM_TYPE_FRAME)
        if not f.Valid():
            f = RDK.AddFrame(name)
            f.setPoseAbs(TxyzRxyz_2_Pose([x, y, z - BOX_H / 2, 0, 0, 0]))
        frames.append(f)
        print(f"[setup] box frame: {name}")
    return frames


def _add_targets(RDK, robot, ref_frame):
    targets = {}

    def _get_or_create(name, x, y, z):
        t = RDK.Item(name, ITEM_TYPE_FRAME)
        if t.Valid():
            targets[name] = t
            return t
        t = RDK.AddTarget(name, ref_frame, robot)
        t.setAsCartesianTarget()
        t.setPose(TxyzRxyz_2_Pose([x, y, z, RX_DOWN, 0, 0]))
        targets[name] = t
        return t

    # Home
    _get_or_create("Home", 0, 500, 800)

    # Pick approach + grasp for each box
    for i, (x, y, z) in enumerate(PICKUP_POSITIONS):
        _get_or_create(f"Pick{i+1}_Approach", x, y, z + APPROACH_LIFT)
        _get_or_create(f"Pick{i+1}_Grasp",    x, y, z)

    # Place approach + drop (stack builds up)
    for i in range(4):
        z_drop = STACK_Z_BASE + i * BOX_H
        _get_or_create(f"Place{i+1}_Approach", STACK_X, STACK_Y, z_drop + APPROACH_LIFT)
        _get_or_create(f"Place{i+1}",          STACK_X, STACK_Y, z_drop)

    print(f"[setup] {len(targets)} targets ready")
    return targets


def _build_program(RDK, robot, targets):
    prog_name = "StackPro_PickPlace"
    existing = RDK.Item(prog_name)
    if existing.Valid():
        print(f"[setup] program '{prog_name}' already exists — skipping")
        return existing

    prog = RDK.AddProgram(prog_name, robot)

    prog.addMoveJ(targets["Home"])

    for i in range(4):
        prog.addMoveJ(targets[f"Pick{i+1}_Approach"])
        prog.addMoveL(targets[f"Pick{i+1}_Grasp"])
        # Gripper close → wire to /gripper_command via ROS node
        prog.customInstruction("GripperClose", "python3", "gripper_close.py", True)
        prog.addMoveL(targets[f"Pick{i+1}_Approach"])
        prog.addMoveJ(targets[f"Place{i+1}_Approach"])
        prog.addMoveL(targets[f"Place{i+1}"])
        prog.customInstruction("GripperOpen", "python3", "gripper_open.py", True)
        prog.addMoveL(targets[f"Place{i+1}_Approach"])
        prog.addMoveJ(targets["Home"])

    print(f"[setup] program '{prog_name}' created")
    return prog


def setup_station(host: str = "localhost", port: int = 20500, clear: bool = False) -> dict:
    RDK = Robolink(robodk_ip=host, port=port)

    if clear:
        print("[setup] clearing station …")
        RDK.CloseStation()

    robot     = _add_robot(RDK)
    _add_tool(RDK, robot)

    ref_frame = robot.getLink(ITEM_TYPE_FRAME)
    if not ref_frame.Valid():
        ref_frame = RDK.AddFrame("World")
        robot.setPoseFrame(ref_frame)

    _add_box_frames(RDK)
    targets = _add_targets(RDK, robot, ref_frame)
    prog    = _build_program(RDK, robot, targets)

    RDK.Render()

    return {
        "robot":   robot.Name(),
        "targets": list(targets.keys()),
        "program": prog.Name() if prog.Valid() else None,
    }


def main(args=None):
    parser = argparse.ArgumentParser(description="Setup RoboDK StackPro demo station")
    parser.add_argument("--host",  default="localhost")
    parser.add_argument("--port",  type=int, default=20500)
    parser.add_argument("--clear", action="store_true")
    parsed = parser.parse_args(args)

    result = setup_station(parsed.host, parsed.port, parsed.clear)
    print("[setup] done:", result)


if __name__ == "__main__":
    main()
