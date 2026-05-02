# robotics-II-final-project
Our final projet for RPI's Robotics II course. In the project, we compare different approaches to formation control, including leader-follower, virtual structure, and behavior-based approaches.

# Instructions

## Simulation

Install dependencies: `pip install numpy scipy matplotlib` plus `ffmpeg` for MP4 export.

```bash
# Single experiment
python main.py --env straight_line --controller leader_follower --formation line --save

# --env:        straight_line | quadratic | obstacles | convergence
# --controller: behavior | virtual_structure | leader_follower
# --formation:  line | triangle | diamond
# --save        write MP4 to videos/ and PNG to plots/

# Full batch (all 3 controllers × 5 environments)
python run_batch.py
```

## Hardware (ROSMASTER-X3)

**Step 1 — Bring up each robot** (run once per robot, on the Jetson):
```bash
ros2 launch yahboomcar_multi X3_bringup_multi.launch.xml robot_name:=robot1
ros2 launch yahboomcar_multi X3_bringup_multi.launch.xml robot_name:=robot2
```

**Step 2 — Build the formation controller** (first time or after changes):
```bash
cd /path/to/robotics-II-final-project
colcon build
source install/setup.bash
```

**Step 3 — Launch formation control:**
```bash
ros2 launch formation_control hardware.launch.py

# Key arguments (all optional):
# controller:=leader_follower     behavior | virtual_structure | leader_follower
# formation_type:=line            line | triangle | diamond
# formation_spacing:=0.5          metres between robots
# path_length:=2.0                length of straight path in metres
# max_hw_speed:=0.3               speed cap sent to robots (m/s)
```

The node waits for odometry from both robots, then anchors the path relative to robot 1's starting position and heading.
