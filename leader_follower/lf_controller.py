import numpy as np

from formation import Formation
from formation_controller import FormationController
from path import Path


# robot 0 follows the path; robot i follows robot i-1 via relative formation offset
class LeaderFollowerController(FormationController):
    def __init__(
        self,
        k_leader: float = 8.0,
        k_tangent: float = 1.5,
        w_align: float = 0.5,
        k_follow: float = 8.0,
        w_avoid: float = 0.25,
        avoid_radius: float = 1.0,
        k_obs: float = 3.0,
        obs_avoid_radius: float = 3.0,
    ):
        self.k_leader = k_leader
        self.k_tangent = k_tangent  # forward drive along path tangent for leader
        self.w_align = w_align
        self.k_follow = k_follow
        self.w_avoid = w_avoid
        self.avoid_radius = avoid_radius
        self.k_obs = k_obs
        self.obs_avoid_radius = obs_avoid_radius

    def _leader_path_heading(self, states: np.ndarray, path: Path) -> float:
        t_proj = path.closest_t(states[0, :2])
        tang = path.tangent(t_proj)
        return float(np.arctan2(tang[1], tang[0]))

    def _robot_repulsion(self, robot_idx: int, states: np.ndarray) -> np.ndarray:
        pos = states[robot_idx, :2]
        f = np.zeros(2)
        for j, other in enumerate(states):
            if j == robot_idx:
                continue
            diff = pos - other[:2]
            dist = np.linalg.norm(diff)
            if 0.0 < dist < self.avoid_radius:
                f += diff / dist
        return f

    def _chain_slot(self, robot_idx: int, states: np.ndarray, formation: Formation, path: Path, t: float) -> np.ndarray:
        parent = robot_idx - 1
        heading = self._leader_path_heading(states, path)
        c, s = np.cos(heading), np.sin(heading)
        R = np.array([[c, -s], [s, c]])
        rel_offset = formation.offsets[robot_idx] - formation.offsets[parent]
        return states[parent, :2] + R @ rel_offset

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
        vel = states[robot_idx, 2:]
        tang = path.tangent(t)
        path_heading = np.arctan2(tang[1], tang[0])

        f_avoid_robot = self.w_avoid * self._robot_repulsion(robot_idx, states)
        f_avoid_obs = self._repulse_obstacles(pos, obstacles, self.obs_avoid_radius, self.k_obs)

        if robot_idx == 0:
            t_proj = path.closest_t(pos)
            tang = path.tangent(t_proj)
            path_heading = np.arctan2(tang[1], tang[0])
            slot = formation.desired_positions(path.point(t_proj), path_heading)[0]
            f_path = self.k_leader * (slot - pos)
            f_along = self.k_tangent * tang  # constant forward drive along path tangent
            f_align = -self.w_align * vel
            return f_path + f_along + f_align + f_avoid_robot + f_avoid_obs
        else:
            parent = robot_idx - 1
            slot = self._chain_slot(robot_idx, states, formation, path, t)
            f_follow = self.k_follow * (slot - pos)
            f_align = self.w_align * (states[parent, 2:] - vel)
            return f_follow + f_align + f_avoid_robot + f_avoid_obs

    def desired_positions(
        self,
        states: np.ndarray,
        formation: Formation,
        path: Path,
        t: float,
    ) -> np.ndarray:
        t_ref = path.closest_t(states[0, :2])
        tang = path.tangent(t_ref)
        path_heading = np.arctan2(tang[1], tang[0])
        desired = np.zeros((formation.n, 2))
        # leader targets the path slot at its projection
        desired[0] = formation.desired_positions(path.point(t_ref), path_heading)[0]
        # followers target slots relative to their parents
        for i in range(1, formation.n):
            desired[i] = self._chain_slot(i, states, formation, path, t)
        return desired

    def ideal_positions(
        self,
        states: np.ndarray,
        formation: Formation,
        path: Path,
        t: float,  # noqa: ARG002
    ) -> np.ndarray:
        # project the estimated formation centroid (robot 0 minus its own offset)
        # so that at t=0 the centroid is exactly path.point(0) → zero initial error,
        # and thereafter the ideal tracks the leader's actual progress
        heading = self._leader_path_heading(states, path)
        c, s = np.cos(heading), np.sin(heading)
        R = np.array([[c, -s], [s, c]])
        centroid_est = states[0, :2] - R @ formation.offsets[0]
        t_ref = path.closest_t(centroid_est)
        tang = path.tangent(t_ref)
        path_heading = np.arctan2(tang[1], tang[0])
        return formation.desired_positions(path.point(t_ref), path_heading)
