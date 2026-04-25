import argparse

import matplotlib.pyplot as plt
import numpy as np

from virtual_structure.vs_controller import VirtualStructureController
from dynamics import Dynamics
from formation import Formation
from path import StraightLinePath
from plot import Plot


def simulate(frames: int = 1000, save: str | None = None):
    path = StraightLinePath(np.array([-4.0, 0.0]), np.array([4.0, 0.0]))
    formation = Formation.triangle(spacing=1.0)
    controller = VirtualStructureController(k_p=10.0)
    dynamics = Dynamics()
    plotter = Plot(path, formation)

    path_speed = 5.0
    t = 0.0

    tang0 = path.tangent(t)
    heading0 = np.arctan2(tang0[1], tang0[0])
    desired0 = formation.desired_positions(path.point(t), heading0)

    states = np.zeros((formation.n, 4))
    for i in range(formation.n):
        states[i, :2] = desired0[i]

    for _ in range(frames):
        desired = controller.desired_positions(states, formation, path, t)
        ideal = controller.ideal_positions(states, formation, path, t)

        plotter.record(states, t, desired, ideal_positions=ideal)

        new_states = np.zeros_like(states)
        for i in range(formation.n):
            v_cmd = controller.compute_control(i, states, path, t, formation, [])
            new_states[i] = dynamics.step(states[i], v_cmd)
        states = new_states
        t = min(t + path_speed * dynamics.dt, 1.0)

    plotter.animate()
    if save:
        plotter.save(save)
    else:
        plt.show()

    plotter.plot_deviations()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", default=None, help="Output filename (.mp4 or .gif)")
    parser.add_argument("--frames", type=int, default=1000)
    args = parser.parse_args()
    simulate(frames=args.frames, save=args.save)
