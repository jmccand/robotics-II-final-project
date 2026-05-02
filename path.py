from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable

import numpy as np
from scipy.optimize import minimize_scalar


# abstract base for parametric paths, t in [0, 1]
class Path(ABC):
    @abstractmethod
    def point(self, t: float) -> np.ndarray: ...

    @abstractmethod
    def tangent(self, t: float) -> np.ndarray: ...

    def normal(self, t: float) -> np.ndarray:
        # 90 deg CCW rotation of the tangent
        tx, ty = self.tangent(t)
        return np.array([-ty, tx])

    def closest_t(self, pos: np.ndarray) -> float:
        pos = np.asarray(pos, dtype=float)
        ts = np.linspace(0.0, 1.0, 200)
        dists = [np.linalg.norm(self.point(t) - pos) for t in ts]
        t0 = float(ts[np.argmin(dists)])
        step = 1.0 / 200
        result = minimize_scalar(
            lambda t: np.linalg.norm(self.point(t) - pos),
            bounds=(max(0.0, t0 - step), min(1.0, t0 + step)),
            method="bounded",
        )
        return float(np.clip(result.x, 0.0, 1.0))


# straight line from start to end
class StraightLinePath(Path):
    def __init__(self, start: np.ndarray, end: np.ndarray):
        self._start = np.asarray(start, dtype=float)
        self._disp = np.asarray(end, dtype=float) - self._start
        self._unit = self._disp / np.linalg.norm(self._disp)

    def point(self, t: float) -> np.ndarray:
        return self._start + t * self._disp

    def tangent(self, t: float) -> np.ndarray:
        return self._unit.copy()

    def closest_t(self, pos: np.ndarray) -> float:
        pos = np.asarray(pos, dtype=float)
        t = float(np.dot(pos - self._start, self._disp) / np.dot(self._disp, self._disp))
        return float(np.clip(t, 0.0, 1.0))


# piecewise linear path through waypoints, arc-length parametrized with t in [0, 1]
class WaypointPath(Path):
    def __init__(self, waypoints: np.ndarray):
        pts = np.asarray(waypoints, dtype=float)
        segs = np.diff(pts, axis=0)
        lens = np.linalg.norm(segs, axis=1)
        self._pts = pts
        self._dirs = segs / lens[:, None]
        self._cum = np.concatenate([[0.0], np.cumsum(lens)])
        self._total = self._cum[-1]

    def _locate(self, t: float) -> tuple[int, float]:
        s = float(np.clip(t, 0.0, 1.0)) * self._total
        idx = int(np.clip(np.searchsorted(self._cum, s, side="right") - 1, 0, len(self._dirs) - 1))
        return idx, s - self._cum[idx]

    def point(self, t: float) -> np.ndarray:
        idx, ds = self._locate(t)
        return self._pts[idx] + ds * self._dirs[idx]

    def tangent(self, t: float) -> np.ndarray:
        idx, _ = self._locate(t)
        return self._dirs[idx].copy()


# path defined as y = f(x) over [x_start, x_end], arc-length parametrized
class FunctionPath(Path):
    def __init__(
        self,
        f: Callable[[float], float],
        df: Callable[[float], float],
        x_start: float,
        x_end: float,
        n_samples: int = 500,
    ):
        self._f = f
        self._df = df
        self.x_start = float(x_start)
        self.x_end = float(x_end)

        # build arc-length table so t in [0,1] maps uniformly to arc length
        xs = np.linspace(x_start, x_end, n_samples)
        pts = np.column_stack([xs, [f(x) for x in xs]])
        seg_lens = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
        self._total = cum[-1]
        self._t_table = cum / self._total  # arc-length fractions
        self._x_table = xs

    def _t_to_x(self, t: float) -> float:
        return float(np.interp(np.clip(t, 0.0, 1.0), self._t_table, self._x_table))

    def point(self, t: float) -> np.ndarray:
        x = self._t_to_x(t)
        return np.array([x, self._f(x)])

    def tangent(self, t: float) -> np.ndarray:
        x = self._t_to_x(t)
        sign = 1.0 if self.x_end >= self.x_start else -1.0
        dy_dx = self._df(x)
        vec = np.array([sign, sign * dy_dx])
        return vec / np.linalg.norm(vec)
