"""Build Path and Formation objects from ROS2 node parameters.

Paths are anchored to the leader robot's initial odometry position so no
pre-measurement of the room coordinate frame is required.
"""

import json
import math

import numpy as np

# Project root is added to sys.path by formation_node.py before this is imported.
from formation import Formation
from path import StraightLinePath, WaypointPath


def build_path(node, start_pos: np.ndarray, start_heading: float):
    """Return a Path anchored at start_pos, oriented along start_heading.

    Parameters (declared in formation_node):
      path_type     - 'straight' | 'waypoint'
      path_length   - metres, for straight paths
      waypoints_json- JSON array of [fwd, lat] offsets from start, for waypoint paths
                      e.g. '[[0,0],[1,0],[2,0.5],[3,0]]'
    """
    path_type = node.get_parameter('path_type').value
    c, s = math.cos(start_heading), math.sin(start_heading)
    R = np.array([[c, -s], [s, c]])  # rotation from leader-relative to world frame

    if path_type == 'straight':
        length = node.get_parameter('path_length').value
        end_pos = start_pos + length * np.array([c, s])
        return StraightLinePath(start_pos.copy(), end_pos)

    if path_type == 'waypoint':
        raw = json.loads(node.get_parameter('waypoints_json').value)
        # Each entry is [forward_offset, lateral_offset] relative to leader start
        pts = np.array([start_pos + R @ np.array(pt, dtype=float) for pt in raw])
        return WaypointPath(pts)

    raise ValueError(f"Unknown path_type '{path_type}'. Use 'straight' or 'waypoint'.")


def build_formation(node) -> Formation:
    """Return a Formation from node parameters.

    Parameters:
      formation_type    - 'line' | 'triangle' | 'diamond'
      formation_spacing - metres between robots
      n_robots          - number of robots (used only for 'line')
    """
    ftype = node.get_parameter('formation_type').value
    spacing = node.get_parameter('formation_spacing').value
    n = node.get_parameter('n_robots').value

    if ftype == 'line':
        return Formation.line(n, spacing)
    if ftype == 'triangle':
        return Formation.triangle(spacing)
    if ftype == 'diamond':
        return Formation.diamond(spacing)

    raise ValueError(f"Unknown formation_type '{ftype}'. Use 'line', 'triangle', or 'diamond'.")
