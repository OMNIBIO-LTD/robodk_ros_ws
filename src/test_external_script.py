# -*- coding: utf-8 -*-
"""
Created on Mon Feb  2 12:07:24 2026

@author: gunsto
"""

# This macro shows an example to draw a polygon of radius R and n_sides vertices using the RoboDK API for Python
# More information about the RoboDK API here:
# https://robodk.com/doc/en/RoboDK-API.html
from robodk.robolink import *  # API to communicate with RoboDK
from robodk.robomath import *  # Basic matrix operations (e.g., rotations)

# Any interaction with RoboDK must be done through RDK:
RDK = Robolink()

# Select a robot (popup is displayed if more than one robot is available)
robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)

if not robot.Valid():
    raise Exception('No robot selected or available')

# Get the reference frame of the robot (parent frame for targets)
reference_frame = robot.getLink(ITEM_TYPE_FRAME)
if not reference_frame.Valid():
    raise Exception('The robot does not have a valid reference frame.')

# Get the current position of the TCP with respect to the reference frame:
# (4x4 matrix representing position and orientation)
target_ref = robot.Pose()
print("Drawing a polygon around the target: ")
print(Pose_2_TxyzRxyz(target_ref))

# Move the robot to the first point:
robot.MoveJ(target_ref)

# It is important to provide the reference frame and the tool frames when generating programs offline
robot.setPoseFrame(reference_frame)  # Set the reference frame explicitly
robot.setPoseTool(robot.PoseTool())  # Keep the current tool
robot.setRounding(10)  # Set the rounding parameter
robot.setSpeed(200)  # Set linear speed in mm/s

# Set the number of sides of the polygon:
steps = 6
R = 150

# Copy the original target as a base
target_i = Mat(target_ref)

# Make a hexagon around the reference target:
for i in range(steps):  # +1 to close the polygon
    # -----------------------------
    # Movement relative to the reference frame
    # Create a copy of the target
    target_i = Mat(target_i)

    # Modify the position
    pos_i = target_i.Pos()
    pos_i[0] = pos_i[0] + 50  # Move along X-axis
    target_i.setPos(pos_i)

    # Rotate the pose (incrementally rotate around Z-axis)
    rotation_increment = rotz(0.2 * i)  # Rotate 5 degrees per step
    target_i = target_i * rotation_increment  # Apply the rotation

    # Create a new target in RoboDK for the pose
    new_target_name = f"Target_{i}"
    new_target = RDK.AddTarget(new_target_name, reference_frame)  # Use reference_frame here
    new_target.setPose(target_i)

    print(f"New target created: {new_target_name}")

    # Move the robot to the new target
    robot.MoveL(new_target)

# Move back to the center, then home:
robot.MoveL(target_ref)

print('Done')