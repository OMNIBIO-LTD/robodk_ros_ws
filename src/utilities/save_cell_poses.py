#!/usr/bin/env python3
"""
Compute and save the center poses of pallet cells in a grid,
formatted as geometry_msgs/msg/PoseStamped (position + orientation + header),
ready to be iterated and published (e.g. in ROS 2).

Pallet initial pose (bottom-left corner):
    x_init = 0.9278964843749999
    y_init = 0.5959041137695313

Pallet dimensions:
    width  (W) = 1.0   # spans the +x-axis
    length (L) = 1.2   # spans the -y-axis (length grows in negative y)

Grid:
    n = 5 rows    (along the length / -y)
    m = 5 columns (along the width  / +x)

Default z / orientation (applied to every cell):
    z: 1.497
    orientation:
        x: -6.123233995736766e-17
        y: 1.0
        z: 6.123233995736766e-17
        w: 3.749399456654644e-33
"""

import json


# --- Default z and orientation applied to every cell pose ---
DEFAULT_Z = 1.28
DEFAULT_ORIENTATION = {
    "x": -6.123233995736766e-17,
    "y": 1.0,
    "z": 6.123233995736766e-17,
    "w": 3.749399456654644e-33,
}


def compute_cell_centers(L, W, n, m, x_init=0.0, y_init=0.0):
    """
    Divide a pallet into an n x m grid of cells and compute each cell's center.

    Axis convention:
        width  (W) spans the +x-axis and is divided into m columns.
        length (L) spans the -y-axis and is divided into n rows,
        i.e. rows advance in the negative-y direction from y_init.

    Parameters
    ----------
    L : float        # pallet length (spans -y-axis, divided across the rows)
    W : float        # pallet width  (spans +x-axis, divided across the columns)
    n : int          # number of rows (along L/-y)
    m : int          # number of columns (along W/+x)
    x_init : float   # x-coordinate of pallet's start corner
    y_init : float   # y-coordinate of pallet's start corner

    Returns
    -------
    dict[(int, int), (float, float)]
        Keys are (row, col), 1-indexed. col=1 is nearest x_init (grows +x);
        row=1 is nearest y_init (grows -y). Values are (center_x, center_y).
    """
    cell_width = W / m        # cell size along +x (pallet width)
    cell_length = L / n       # cell size along -y (pallet length)

    centers = {}
    for row in range(n):
        for col in range(m):
            cx = x_init + col * cell_width + cell_width / 2
            cy = y_init - (row * cell_length + cell_length / 2)
            centers[(row + 1, col + 1)] = (cx, cy)

    return centers


def centers_to_pose_stamped_dict(centers, frame_id="map", z=DEFAULT_Z,
                                  orientation=None):
    """
    Convert {(row, col): (x, y)} into a dict of PoseStamped-formatted
    entries, keyed by "row,col" strings (JSON-safe).

    Each value follows the geometry_msgs/msg/PoseStamped structure:
        {
            "header": {
                "frame_id": "...",
                "stamp": {"sec": 0, "nanosec": 0}
            },
            "pose": {
                "position": {"x": ..., "y": ..., "z": ...},
                "orientation": {"x": ..., "y": ..., "z": ..., "w": ...}
            }
        }

    Note: stamp is left at {"sec": 0, "nanosec": 0} as a placeholder;
    fill it in at publish time (e.g. self.get_clock().now().to_msg() in ROS 2).
    """
    if orientation is None:
        orientation = DEFAULT_ORIENTATION

    poses = {}
    for (row, col), (cx, cy) in centers.items():
        poses[f"{row},{col}"] = {
            "header": {
                "frame_id": frame_id,
                "stamp": {"sec": 0, "nanosec": 0},
            },
            "pose": {
                "position": {"x": cx, "y": cy, "z": z},
                "orientation": dict(orientation),
            },
        }
    return poses


def save_poses_to_file(poses, filepath):
    """Save the poses dict to a JSON file."""
    with open(filepath, "w") as f:
        json.dump(poses, f, indent=2)


def load_poses_from_file(filepath):
    """Load a poses dict back from a JSON file."""
    with open(filepath, "r") as f:
        return json.load(f)


def iterate_poses(poses):
    """
    Generator that yields (row, col, pose_stamped_dict) tuples,
    convenient for publishing each pose in sequence.
    """
    for key, pose_stamped in poses.items():
        row_str, col_str = key.split(",")
        yield int(row_str), int(col_str), pose_stamped


def main():
    # --- Pallet initial pose ---
    x_init = 0.9278964843749999
    y_init = 0.5959041137695313

    # --- Pallet dimensions ---
    L = 1.2   # length
    W = 0.8   # width

    # --- Grid size ---
    n = 5     # rows
    m = 5     # columns

    frame_id = "map"  # change to your desired reference frame
    output_path = "pallet_cell_poses.json"

    # 1. Compute cell centers
    centers = compute_cell_centers(L, W, n, m, x_init, y_init)

    # 2. Convert to PoseStamped-formatted dict
    poses = centers_to_pose_stamped_dict(centers, frame_id=frame_id)

    # 3. Save to file
    save_poses_to_file(poses, output_path)

    print(f"Saved {len(poses)} cell poses to '{output_path}'")
    for row, col, pose_stamped in iterate_poses(poses):
        pos = pose_stamped["pose"]["position"]
        print(f"  cell ({row},{col}): x={pos['x']:.6f}, "
              f"y={pos['y']:.6f}, z={pos['z']:.6f}")


if __name__ == "__main__":
    main()