from __future__ import annotations

import numpy as np

from formation import Formation
from formation_controller import FormationController
from path import Path


# fits the virtual structure to robot positions via Procrustes, then drives the
# fitted centroid and heading along the path using rigid-body velocity translation
class VirtualStructureController(FormationController):
    def __init__(
        self,
        k_slot: float = 8.0,
        slot_weight: float = 0.7,
        w_avoid: float = 10.0,
        avoid_radius: float = 0.75,
        k_obs: float = 5.0,
        obs_avoid_radius: float = 8.0,
    ):
        self.k_slot = k_slot
        self.slot_weight = slot_weight
        self.w_avoid = w_avoid
        self.avoid_radius = avoid_radius
        self.k_obs = k_obs
        self.obs_avoid_radius = obs_avoid_radius
        self._cache_t: float | None = None
        self._cached_fit: tuple | None = None

    def _fit(
        self, states: np.ndarray, formation: Formation
    ) -> tuple[np.ndarray, float, np.ndarray]:
        # orthogonal Procrustes: find (centroid, heading, R) minimising
        # sum_i || x_i - (centroid + R @ o_i) ||^2
        X = states[:, :2]
        O = formation.offsets

        x_bar = X.mean(axis=0)
        o_bar = O.mean(axis=0)

        M = (X - x_bar).T @ (O - o_bar)

        U, _, Vt = np.linalg.svd(M)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt

        psi = np.arctan2(R[1, 0], R[0, 0])
        centroid = x_bar - R @ o_bar

        return centroid, psi, R

    def _get_fit(
        self, states: np.ndarray, formation: Formation, t: float
    ) -> tuple[np.ndarray, float, np.ndarray]:
        if self._cache_t != t:
            self._cached_fit = self._fit(states, formation)
            self._cache_t = t
        return self._cached_fit  # type: ignore[return-value]

    def compute_control(
        self,
        robot_idx: int,
        states: np.ndarray,
        path: Path,
        t: float,
        formation: Formation,
        obstacles: list,
    ) -> np.ndarray:
        pos = states[robot_idx, :2]

        # step 1: fit the virtual structure to current robot positions
        centroid_fit, _, R_fit = self._get_fit(states, formation, t)

        # step 2: rigid-body transform from fitted pose to ideal pose on path
        tang = path.tangent(t)
        path_heading = np.arctan2(tang[1], tang[0])
        c, s = np.cos(path_heading), np.sin(path_heading)
        R_ideal = np.array([[c, -s], [s, c]])
        R_delta = R_ideal @ R_fit.T
        c_ideal = path.point(t)

        # step 3: apply the formation-level transform to this robot's position
        des_slot = R_delta @ (pos - centroid_fit) + c_ideal
        ideal_slot = formation.desired_positions(c_ideal, path_heading)[robot_idx]

        f_avoid_robot = np.zeros(2)
        for j, other in enumerate(states):
            if j == robot_idx:
                continue
            diff = pos - other[:2]
            dist = np.linalg.norm(diff)
            if 0.0 < dist < self.avoid_radius:
                f_avoid_robot += diff / dist

        step = self.slot_weight * (des_slot - pos) + (1 - self.slot_weight) * (ideal_slot - pos)
        v_cmd = self.k_slot * step
        v_cmd += self.w_avoid * f_avoid_robot
        v_cmd += self._repulse_obstacles(pos, obstacles, self.obs_avoid_radius, self.k_obs)
        return v_cmd

