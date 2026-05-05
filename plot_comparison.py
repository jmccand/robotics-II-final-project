"""
Generates one combined figure with two 2×2 blocks side by side:
  left  — straight-line path, all 4 controllers
  right — robot-2 dropout, all 4 controllers
Output: plots/comparison.png
"""
from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from dynamics import Dynamics
from environments import ENVIRONMENTS
from formation import Formation
from behavior.behavior_controller import BehaviorController
from virtual_structure.vs_controller import VirtualStructureController
from leader_follower.lf_controller import LeaderFollowerController
from leader_follower.dlf_controller import DirectLeaderFollowerController

CONTROLLERS = {
    "Behavior": BehaviorController(),
    "Virtual Structure": VirtualStructureController(),
    "Leader-Follower\n(chain)": LeaderFollowerController(),
    "Leader-Follower\n(direct)": DirectLeaderFollowerController(),
}

FORMATION = Formation.diamond(1.0)


def run(env, controller, dropout_robot=None, dropout_frame=0):
    dyn = Dynamics()
    t = 0.0
    tang0 = env.path.tangent(t)
    h0 = np.arctan2(tang0[1], tang0[0])
    states = np.zeros((FORMATION.n, 4))
    for i in range(FORMATION.n):
        states[i, :2] = FORMATION.desired_positions(env.path.point(t), h0)[i]

    times, devs = [], []
    for frame_idx in range(env.default_frames):
        ideal = controller.ideal_positions(states, FORMATION, env.path, t)
        times.append(frame_idx * dyn.dt)
        devs.append([np.linalg.norm(states[i, :2] - ideal[i]) for i in range(FORMATION.n)])

        new_states = np.zeros_like(states)
        for i in range(FORMATION.n):
            v = controller.compute_control(i, states, env.path, t, FORMATION, env.obstacles)
            new_states[i] = dyn.step(states[i], v)

        if dropout_robot is not None and frame_idx >= dropout_frame:
            new_states[dropout_robot] = states[dropout_robot]
            new_states[dropout_robot, 2:] = 0.0

        states = new_states
        t = min(t + env.path_speed * dyn.dt, 1.0)
        if env.path_speed > 0 and (t >= 1.0 or controller.should_terminate(states, env.path)):
            break

    return np.array(times), np.array(devs)


if __name__ == "__main__":
    env_sl = ENVIRONMENTS["straight_line"]
    env_do = ENVIRONMENTS["straight_line"]

    configs = [
        (env_sl, dict(), "Straight-Line Path"),
        (env_do, dict(dropout_robot=2, dropout_frame=300), "Robot 3 Dropout"),
    ]

    ctrl_names = list(CONTROLLERS.keys())
    ctrl_list = list(CONTROLLERS.values())

    # 2 rows × 4 cols: left two cols = straight-line, right two cols = dropout
    fig, axes = plt.subplots(2, 4, figsize=(26, 10))

    for block, (env, run_kwargs, block_title) in enumerate(configs):
        col_offset = block * 2

        # block title spanning two columns
        col_left = axes[0, col_offset].get_position().x0
        col_right = axes[0, col_offset + 1].get_position().x1
        fig.text(
            (col_left + col_right) / 2, 0.98,
            block_title,
            ha="center", va="top", fontsize=22, fontweight="bold",
        )

        for idx, (ctrl_name, ctrl) in enumerate(CONTROLLERS.items()):
            row, col = divmod(idx, 2)
            ax = axes[row, col_offset + col]
            time_s, devs = run(env, ctrl, **run_kwargs)

            for i in range(devs.shape[1]):
                ax.plot(time_s, devs[:, i], label=f"Robot {i + 1}", linewidth=2.5)
            ax.set_title(ctrl_name, fontsize=18)
            ax.set_xlabel("Time (s)", fontsize=16)
            ax.set_ylabel("Deviation (m)", fontsize=16)
            ax.tick_params(labelsize=15)
            ax.legend(fontsize=14)
            ax.grid(True, alpha=0.3)

    # vertical divider between the two blocks
    fig.add_artist(plt.Line2D(
        [0.5, 0.5], [0.02, 0.97],
        transform=fig.transFigure,
        color="gray", linewidth=1.5, linestyle="--",
    ))

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    os.makedirs("plots", exist_ok=True)
    out = "plots/comparison.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out}")
