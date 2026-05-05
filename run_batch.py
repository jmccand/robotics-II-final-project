import argparse
import subprocess
import sys

PYTHON = sys.executable

ALL_CONTROLLERS = ["behavior", "virtual_structure", "leader_follower", "direct_leader_follower"]

# (key shown in --filter, --env passed to main.py, extra args)
ALL_CONFIGS = [
    ("straight_line", "straight_line", []),
    ("quadratic",     "quadratic",     []),
    ("obstacles",     "obstacles",     []),
    ("dropout",       "straight_line", ["--dropout", "2", "--dropout-frame", "300"]),
    ("convergence",   "convergence",   ["--ic", "line", "--n", "4"]),
]

parser = argparse.ArgumentParser(
    description="Run a batch of formation control experiments. "
                "With no filters, all controllers × all environments are run. "
                "Pass any combination of controller/environment flags to restrict the batch.",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog="Examples:\n"
           "  python run_batch.py                            # run everything\n"
           "  python run_batch.py --leader_follower          # all envs, one controller\n"
           "  python run_batch.py --straight_line --dropout  # all controllers, two envs\n"
           "  python run_batch.py --virtual_structure --obstacles  # one each",
)

for ctrl in ALL_CONTROLLERS:
    parser.add_argument(f"--{ctrl}", action="store_true", default=False,
                        help=f"Include the {ctrl} controller")

for key, _, _ in ALL_CONFIGS:
    parser.add_argument(f"--{key}", action="store_true", default=False,
                        help=f"Include the {key} environment")

args = parser.parse_args()

selected_controllers = [c for c in ALL_CONTROLLERS if getattr(args, c)] or ALL_CONTROLLERS
selected_env_keys = [k for k, _, _ in ALL_CONFIGS if getattr(args, k)] or [k for k, _, _ in ALL_CONFIGS]

for ctrl in selected_controllers:
    for key, env, extra in ALL_CONFIGS:
        if key not in selected_env_keys:
            continue
        cmd = [
            PYTHON, "main.py",
            "--env", env,
            "--controller", ctrl,
            "--formation", "diamond",
            "--save",
            *extra,
        ]
        print(f"\n>>> {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
