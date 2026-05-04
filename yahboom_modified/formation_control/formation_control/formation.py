import numpy as np


# 2D body-frame offsets per robot; x = along path, y = lateral (left of path)
class Formation:
    def __init__(self, offsets: np.ndarray):
        self.offsets = np.asarray(offsets, dtype=float)

    @property
    def n(self) -> int:
        return len(self.offsets)

    def desired_positions(self, centroid: np.ndarray, heading: float) -> np.ndarray:
        c, s = np.cos(heading), np.sin(heading)
        R = np.array([[c, -s], [s, c]])
        return centroid + (R @ self.offsets.T).T

    @staticmethod
    def column(n: int, spacing: float = 1.0) -> "Formation":
        # n robots in single file along the path direction, leader at front
        offsets = np.column_stack([-np.arange(n) * spacing, np.zeros(n)])
        return Formation(offsets)

    @staticmethod
    def line(n: int, spacing: float = 1.0) -> "Formation":
        lateral = np.linspace(-(n - 1) / 2.0, (n - 1) / 2.0, n) * spacing
        offsets = np.column_stack([np.zeros(n), lateral])
        return Formation(offsets)

    @staticmethod
    def triangle(spacing: float = 1.0) -> "Formation":
        offsets = np.array([
            [0.0,  0.6],
            [-0.5, -0.3],
            [0.5, -0.3],
        ]) * spacing
        return Formation(offsets)

    @staticmethod
    def diamond(spacing: float = 1.0) -> "Formation":
        offsets = np.array([
            [ 1,  0.0],
            [ 0.0,  1],
            [ 0.0, -1],
            [-1,  0.0],
        ]) * spacing
        return Formation(offsets)
