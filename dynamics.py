import numpy as np


# velocity-controlled robot dynamics, state: [x, y, vx, vy]
class Dynamics:
    def __init__(self, dt: float = 0.01, max_speed: float = 5.0):
        self.dt = dt
        self.max_speed = max_speed

    def step(self, state: np.ndarray, v_cmd: np.ndarray) -> np.ndarray:
        speed = np.linalg.norm(v_cmd)
        if speed > self.max_speed:
            v_cmd = v_cmd * (self.max_speed / speed)
        new = state.copy()
        new[:2] += v_cmd * self.dt
        new[2:] = v_cmd
        return new
