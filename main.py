from __future__ import annotations

# formation control experiment runner
# usage: python main.py --env <env> --controller <ctrl> [--formation <f>] [--ic <f>] [--n N] [--spacing S] [--frames N] [--save]
# example: python main.py --env obstacles --controller virtual_structure --formation diamond --save
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np

from dynamics import Dynamics
from environments import ENVIRONMENTS, Environment
from formation import Formation
from formation_controller import FormationController
from plot import Plot
from behavior.behavior_controller import BehaviorController
from virtual_structure.vs_controller import VirtualStructureController
from leader_follower.lf_controller import LeaderFollowerController


CONTROLLERS: dict[str, type[FormationController]] = {
    "behavior": BehaviorController,
    "virtual_structure": VirtualStructureController,
    "leader_follower": LeaderFollowerController,
}

FORMATION_NAMES = ["line", "triangle", "diamond"]


def make_formation(name: str, n: int, spacing: float) -> Formation:
    if name == "line":
        return Formation.line(n, spacing)
    if name == "triangle":
        return Formation.triangle(spacing)
    if name == "diamond":
        return Formation.diamond(spacing)
    raise ValueError(f"Unknown formation: {name!r}")


def simulate(
    env: Environment,
    controller: FormationController,
    formation: Formation,
    ic_formation: Formation,
    frames: int,
    video_path: str | None = None,
    plot_path: str | None = None,
    dropout_robot: int | None = None,
    dropout_frame: int = 0,
) -> None:
    dynamics = Dynamics()
    plotter = Plot(
        env.path, formation,
        obstacles=env.obstacles, boundary=env.boundary,
        dropout_robot=dropout_robot, dropout_frame=dropout_frame,
    )

    t = 0.0
    tang0 = env.path.tangent(t)
    heading0 = np.arctan2(tang0[1], tang0[0])

    ic_positions = ic_formation.desired_positions(env.path.point(t), heading0)
    states = np.zeros((formation.n, 4))
    for i in range(formation.n):
        states[i, :2] = ic_positions[i]

    for frame_idx in range(frames):
        desired = controller.desired_positions(states, formation, env.path, t)
        ideal = controller.ideal_positions(states, formation, env.path, t)

        plotter.record(states, t, desired, ideal_positions=ideal)

        new_states = np.zeros_like(states)
        for i in range(formation.n):
            v_cmd = controller.compute_control(
                i, states, env.path, t, formation, env.obstacles
            )
            new_states[i] = dynamics.step(states[i], v_cmd)

        if dropout_robot is not None and frame_idx >= dropout_frame:
            new_states[dropout_robot] = states[dropout_robot]
            new_states[dropout_robot, 2:] = 0.0

        states = new_states
        t = min(t + env.path_speed * dynamics.dt, 1.0)

    plotter.animate()
    if video_path:
        plotter.save(video_path)
        plt.close("all")
    else:
        plt.show()

    fig = plotter.plot_deviations()
    if plot_path:
        fig.savefig(plot_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved plot to {plot_path}")
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Formation control experiment runner")
    parser.add_argument(
        "--env",
        choices=list(ENVIRONMENTS),
        required=True,
        help="Environment to run in",
    )
    parser.add_argument(
        "--controller",
        choices=list(CONTROLLERS),
        required=True,
        help="Formation controller to use",
    )
    parser.add_argument(
        "--formation",
        choices=FORMATION_NAMES,
        default="triangle",
        help="Target formation shape (default: triangle)",
    )
    parser.add_argument(
        "--ic",
        choices=FORMATION_NAMES,
        default=None,
        metavar="FORMATION",
        help="Initial formation (default: same as --formation, giving zero initial error)",
    )
    parser.add_argument(
        "--n",
        type=int,
        default=5,
        help="Number of robots for line formation (default: 5)",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=1.0,
        help="Formation slot spacing in world units (default: 1.0)",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=None,
        help="Number of simulation frames (default: per-environment)",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        default=False,
        help="Auto-save video to videos/ and deviation plot to plots/",
    )
    parser.add_argument(
        "--dropout",
        type=int,
        default=None,
        metavar="I",
        help="Robot index that drops out (stops moving)",
    )
    parser.add_argument(
        "--dropout-frame",
        type=int,
        default=0,
        metavar="F",
        help="Frame at which the dropout occurs (default: 0)",
    )
    args = parser.parse_args()

    if args.dropout is None and args.dropout_frame != 0:
        parser.error("--dropout-frame requires --dropout")

    formation = make_formation(args.formation, args.n, args.spacing)
    ic_name = args.ic if args.ic is not None else args.formation
    ic_formation = make_formation(ic_name, args.n, args.spacing)

    if ic_formation.n != formation.n:
        parser.error(
            f"--ic {ic_name} has {ic_formation.n} robots but "
            f"--formation {args.formation} has {formation.n}. Robot counts must match."
        )

    env = ENVIRONMENTS[args.env]
    controller = CONTROLLERS[args.controller]()
    frames = args.frames if args.frames is not None else env.default_frames

    env_part = "dropout" if args.dropout is not None else args.env
    stem = f"{args.controller}_{env_part}"
    video_path: str | None = None
    plot_path: str | None = None
    if args.save:
        os.makedirs("videos", exist_ok=True)
        os.makedirs("plots", exist_ok=True)
        video_path = f"videos/{stem}.mp4"
        plot_path = f"plots/{stem}.png"

    print(f"Environment : {env.name}")
    print(f"Controller  : {args.controller}")
    print(f"Formation   : {args.formation}  n={formation.n}  spacing={args.spacing}")
    print(f"IC          : {ic_name}")
    print(f"Frames      : {frames}")
    if args.dropout is not None:
        print(f"Dropout     : robot {args.dropout} at frame {args.dropout_frame}")
    if args.save:
        print(f"Saving to   : {video_path}, {plot_path}")

    simulate(
        env, controller, formation, ic_formation, frames,
        video_path=video_path,
        plot_path=plot_path,
        dropout_robot=args.dropout,
        dropout_frame=args.dropout_frame,
    )


if __name__ == "__main__":
    main()
