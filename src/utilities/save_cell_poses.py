#!/usr/bin/env python3
"""
Compute and save the center poses of pallet cells in a grid,
formatted as geometry_msgs/msg/PoseStamped (position + orientation + header),
ready to be iterated and published (e.g. in ROS 2).

Pallet initial pose (bottom-left corner):
    x_init = 0.9278964843749999
    y_init = 0.5959041137695313

Pallet dimensions:
    length (L) = 1.2   # along x-axis
    width  (W) = 1.0   # along y-axis

Grid:
    n = 5 rows (along width/y)
    m = 5 columns (along length/x)

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
DEFAULT_Z = 1.497
DEFAULT_ORIENTATION = {
    "x": -6.123233995736766e-17,
    "y": 1.0,
    "z": 6.123233995736766e-17,
    "w": 3.749399456654644e-33,
}


def compute_cell_centers(L, W, n, m, x_init=0.0, y_init=0.0):
    """
    Divide a pallet into an n x m grid of cells and compute each cell's center.

    Parameters
    ----------
    L : float        # pallet length (along x-axis, spans the columns)
    W : float        # pallet width (along y-axis, spans the rows)
    n : int          # number of rows (along W/y)
    m : int          # number of columns (along L/x)
    x_init : float   # x-coordinate of pallet's bottom-left corner
    y_init : float   # y-coordinate of pallet's bottom-left corner

    Returns
    -------
    dict[(int, int), (float, float)]
        Keys are (row, col), 1-indexed, row=1 is bottom row, col=1 is
        leftmost column. Values are (center_x, center_y) of that cell.
    """
    cell_width = L / m
    cell_height = W / n

    centers = {}
    for row in range(n):
        for col in range(m):
            cx = x_init + col * cell_width + cell_width / 2
            cy = y_init + row * cell_height + cell_height / 2
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
    W = 1.0   # width

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