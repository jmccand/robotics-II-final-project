from abc import ABC, abstractmethod

import numpy as np

from formation import Formation
from path import Path


# abstract base for formation control laws
class FormationController(ABC):
    @abstractmethod
    def compute_control(
        self,
        robot_idx: int,
        states: np.ndarray,
        path: Path,
        t: float,
        formation: Formation,
        obstacles: list,
    ) -> np.ndarray: ...

    def _repulse_obstacles(
        self, pos: np.ndarray, obstacles: list, radius: float, gain: float
    ) -> np.ndarray:
        f = np.zeros(2)
        for obs in obstacles:
            dist = obs.distance(pos)
            if 0.0 < dist < radius:
                diff = pos - obs.closest_point(pos)
                d = np.linalg.norm(diff)
                if d > 0:
                    f += gain * (diff / d) * (1.0 / dist)
        return f

    def desired_positions(
        self,
        states: np.ndarray,
        formation: Formation,
        path: Path,
        t: float,
    ) -> np.ndarray:
        # default: slots aligned to the path point at t
        tang = path.tangent(t)
        heading = np.arctan2(tang[1], tang[0])
        return formation.desired_positions(path.point(t), heading)

    def should_terminate(self, states: np.ndarray, path: Path) -> bool:
        return False

    def ideal_positions(
        self,
        states: np.ndarray,
        formation: Formation,
        path: Path,
        t: float,
    ) -> np.ndarray:
        # default: ideal matches desired (e.g. for behavior and VS)
        return self.desired_positions(states, formation, path, t)
