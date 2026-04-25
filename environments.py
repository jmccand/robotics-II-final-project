from dataclasses import dataclass, field

import numpy as np

from obstacles import Circle
from path import FunctionPath, Path, StraightLinePath, WaypointPath


@dataclass
class Environment:
    name: str
    path: Path
    obstacles: list = field(default_factory=list)
    boundary: float = 50.0
    path_speed: float = 0.05
    default_frames: int = 2000


# 1. Straight line from (0, 0) to (0, 20) — northward vertical path
STRAIGHT_LINE = Environment(
    name="straight_line",
    path=StraightLinePath(np.array([0.0, 0.0]), np.array([0.0, 20.0])),
    obstacles=[],
    boundary=25.0,
    path_speed=0.1,
    default_frames=1000,
)

# 2. Quadratic y = -x^2 - 5x + 5 from x=-8 to x=3
#    Vertex at x=-2.5, y=11.25; endpoints at y=-19.
QUADRATIC = Environment(
    name="quadratic",
    path=FunctionPath(
        f=lambda x: -x**2 - 5.0 * x + 5.0,
        df=lambda x: -2.0 * x - 5.0,
        x_start=3.0,
        x_end=-8.0,
    ),
    obstacles=[],
    boundary=25.0,
    path_speed=0.05,
)

# 3. Obstacle environment — two circles blocking a straight-line path
START = np.array([-30.0, -40.0])
GOAL = np.array([20.0, 40.0])

OBSTACLES = [
    Circle(center=np.array([-10.0, -10.0]), radius=10.0),
    Circle(center=np.array([10.0, 15.0]), radius=15.0),
]


def _obstacle_path(clearance: float = 1.0) -> WaypointPath:
    # Strictly geometric path: START -> line -> arc(C1) -> line (outer tangent) -> arc(C2) -> line -> GOAL
    c1, r1 = OBSTACLES[0].center, OBSTACLES[0].radius + clearance
    c2, r2 = OBSTACLES[1].center, OBSTACLES[1].radius + clearance

    def _tangent_pt(P: np.ndarray, C: np.ndarray, r: float, side: str) -> np.ndarray:
        # Angle from C to P
        d = P - C
        phi = np.arctan2(d[1], d[0])
        # Angle from (C-P) line to tangent point
        alpha = np.arccos(r / np.linalg.norm(d))
        angle = phi + alpha if side == "left" else phi - alpha
        return C + r * np.array([np.cos(angle), np.sin(angle)])

    def _outer_tangent_pts(C1: np.ndarray, r1: float, C2: np.ndarray, r2: float, side: str) -> tuple[np.ndarray, np.ndarray]:
        d = C2 - C1
        dist = np.linalg.norm(d)
        phi = np.arctan2(d[1], d[0])
        # Angle of the normal to the tangent line
        alpha = np.arccos((r1 - r2) / dist)
        angle = phi + alpha if side == "left" else phi - alpha
        p1 = C1 + r1 * np.array([np.cos(angle), np.sin(angle)])
        p2 = C2 + r2 * np.array([np.cos(angle), np.sin(angle)])
        return p1, p2

    def _arc(C: np.ndarray, r: float, p_start: np.ndarray, p_end: np.ndarray, n: int = 15) -> list[np.ndarray]:
        a0 = np.arctan2(p_start[1] - C[1], p_start[0] - C[0])
        a1 = np.arctan2(p_end[1] - C[1], p_end[0] - C[0])
        # Shortest arc
        diff = (a1 - a0 + np.pi) % (2.0 * np.pi) - np.pi
        return [C + r * np.array([np.cos(a), np.sin(a)]) for a in np.linspace(a0, a0 + diff, n)]

    # We bypass both on the "left" side relative to the direction of travel
    # 'left' tangent point from C1's perspective towards START is the one for a 'left' bypass
    entry1 = _tangent_pt(START, c1, r1, "right")
    exit1, entry2 = _outer_tangent_pts(c1, r1, c2, r2, "left")
    exit2 = _tangent_pt(GOAL, c2, r2, "left")

    arc1 = _arc(c1, r1, entry1, exit1)
    arc2 = _arc(c2, r2, entry2, exit2)

    waypoints = np.array([START, *arc1, *arc2, GOAL])
    return WaypointPath(waypoints)


OBSTACLE_ENV = Environment(
    name="obstacles",
    path=_obstacle_path(),
    obstacles=OBSTACLES,
    boundary=45.0,
    path_speed=0.033,
    default_frames=3000,
)

# 4. Convergence test — stationary target, robots start in a line
CONVERGENCE = Environment(
    name="convergence",
    path=StraightLinePath(np.array([0.0, 0.0]), np.array([1.0, 0.0])),
    obstacles=[],
    boundary=10.0,
    path_speed=0.0,
    default_frames=500,
)

ENVIRONMENTS: dict[str, Environment] = {
    "straight_line": STRAIGHT_LINE,
    "quadratic": QUADRATIC,
    "obstacles": OBSTACLE_ENV,
    "convergence": CONVERGENCE,
}
