import numpy as np

from formation import Formation
from formation_controller import FormationController
from path import Path


# weighted sum of formation-attraction, path-alignment, and repulsion forces
class BehaviorController(FormationController):
    def __init__(
        self,
        w_formation: float = 1.5,
        w_align: float = 0.15,
        w_avoid: float = 1.25,
        w_avoid_obs: float = 3.0,
        u_scale: float = 1.0,
        avoid_radius: float = 1.0,
        obs_avoid_radius: float = 3.0,
    ):
        self.w_formation = w_formation
        self.w_align = w_align
        self.w_avoid = w_avoid
        self.w_avoid_obs = w_avoid_obs
        self.u_scale = u_scale
        self.avoid_radius = avoid_radius
        self.obs_avoid_radius = obs_avoid_radius

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
        heading = np.arctan2(tang[1], tang[0])
        slot = formation.desired_positions(path.point(t), heading)[robot_idx]

        f_formation = slot - pos
        f_align = tang - vel

        f_avoid_robot = np.zeros(2)
        for j, other in enumerate(states):
            if j == robot_idx:
                continue
            diff = pos - other[:2]
            dist = np.linalg.norm(diff)
            if 0.0 < dist < self.avoid_radius:
                f_avoid_robot += diff / dist

        f_avoid_obs = self._repulse_obstacles(pos, obstacles, self.obs_avoid_radius, 1.0)

        u = self.u_scale * (
            self.w_formation * f_formation
            + self.w_align * f_align
            + self.w_avoid * f_avoid_robot
            + self.w_avoid_obs * f_avoid_obs
        )
        return u
