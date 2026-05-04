import json
import math

import numpy as np

# project root is added to sys.path by formation_node.py before this is imported.
from formation import Formation
from path import StraightLinePath, WaypointPath


# returns a Path anchored at start_pos, oriented along start_heading.
# path_type: 'straight' | 'waypoint'; path_length in metres; waypoints_json is
# a JSON array of [fwd, lat] offsets from start (e.g. '[[0,0],[1,0],[2,0]]').
def build_path(node, start_pos: np.ndarray, start_heading: float):
    path_type = node.get_parameter('path_type').value
    c, s = math.cos(start_heading), math.sin(start_heading)
    R = np.array([[c, -s], [s, c]])  # rotation from leader-relative to world frame

    if path_type == 'straight':
        length = node.get_parameter('path_length').value
        end_pos = start_pos + length * np.array([c, s])
        return StraightLinePath(start_pos.copy(), end_pos)

    if path_type == 'waypoint':
        raw = json.loads(node.get_parameter('waypoints_json').value)
        # each entry is [forward_offset, lateral_offset] relative to leader start
        pts = np.array([start_pos + R @ np.array(pt, dtype=float) for pt in raw])
        return WaypointPath(pts)

    raise ValueError(f"Unknown path_type '{path_type}'. Use 'straight' or 'waypoint'.")


# returns a Formation from node parameters (formation_type, formation_spacing, n_robots).
def build_formation(node) -> Formation:
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
