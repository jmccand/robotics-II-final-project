# robotics-II-final-project

RPI Robotics II final project comparing four multi-robot formation control approaches: behavior-based, virtual structure, leader-follower (chain), and leader-follower (direct). The project includes a pure Python simulation and a ROS2 hardware layer for ROSMASTER-X3 robots with A1 lidar.

---

## Simulation

Install dependencies: `pip install numpy scipy matplotlib`.

```bash
# Single experiment
python main.py --env straight_line --controller leader_follower --formation diamond --save

# --env:        straight_line | quadratic | obstacles | convergence
# --controller: behavior | virtual_structure | leader_follower | direct_leader_follower
# --formation:  line | triangle | diamond
# --ic          initial formation (convergence tests only, e.g. --ic line --n 4)
# --save        write MP4 to videos/ and PNG to plots/

# Full batch (all 4 controllers × 5 environments)
python run_batch.py

# Comparison figures (straight-line and dropout side by side)
python plot_comparison.py
```

---

## Hardware (ROSMASTER-X3 + A1 lidar, ROS2 Foxy)

`yahboom_modified/` is a self-contained colcon workspace with 7 ROS2 packages:

```
yahboom_modified/
├── formation_control/      ← new: hardware bridge node + launch
├── yahboomcar_base_node/   ← patched: last_vel_time_ epoch-0 bug fix
├── yahboomcar_bringup/     ← Yahboom motor driver
├── yahboomcar_ctrl/        ← keyboard / joystick teleop
├── yahboomcar_description/ ← URDF + generic multi-robot description launch
├── yahboomcar_msgs/        ← Yahboom custom message types
└── yahboomcar_multi/       ← patched: multi-robot bringup, EKF params, SLAM launch
```

Everything runs on the robots' Jetsons — no external laptop required. All commands below are run over SSH or in a terminal on the relevant Jetson.

### Prerequisites

On **both** Jetsons:
- ROS2 Foxy installed and sourced (pre-installed on the Yahboom image)
- `Rosmaster_Lib` Python package installed (pre-installed on the Yahboom image)
- Both Jetsons on the same network with the same `ROS_DOMAIN_ID` (default 0)
- The following ROS2 packages installed:
  ```bash
  sudo apt install \
    ros-foxy-slam-toolbox \
    ros-foxy-robot-localization \
    ros-foxy-imu-filter-madgwick \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-joint-state-publisher \
    ros-foxy-robot-state-publisher \
    ros-foxy-nav2-amcl \
    ros-foxy-nav2-lifecycle-manager
  ```

---

### Step 0 — Clone and build (once per Jetson)

On **each** Jetson:

```bash
cd ~/codes/
git clone <repo-url>
cd robotics-II-final-project/yahboom_modified
colcon build --symlink-install
echo "source ~/codes/robotics-II-final-project/yahboom_modified/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

This overlays the existing Yahboom workspace. The last-sourced workspace wins, so all commands will use the packages from `yahboom_modified`. **Every new terminal must have the workspace sourced** (the `echo` line above handles that permanently).

---

### Step 1 — Bring up each robot

On **robot1's Jetson**:

```bash
ros2 launch yahboomcar_multi X3_A1_bringup_multi.launch.xml robot_name:=robot1
```

On **robot2's Jetson**:

```bash
ros2 launch yahboomcar_multi X3_A1_bringup_multi.launch.xml robot_name:=robot2
```

This starts, under each robot's namespace:
- `Mcnamu_driver_X3` — hardware serial interface, subscribes to `/<robot_name>/cmd_vel`
- `base_node_X3` — wheel-encoder odometry, publishes `/<robot_name>/odom_raw`
- `imu_filter_madgwick_node` — fuses IMU data
- `ekf_filter_node` — fuses odometry + IMU, publishes `/<robot_name>/odom`
- `robot_state_publisher` — broadcasts TF tree under `/<robot_name>/` frames
- `sllidar_node` — A1 lidar driver, publishes `/<robot_name>/scan`
- static transform `/<robot_name>/base_link → /<robot_name>/laser` (required for SLAM)

---

### Step 2 — Start SLAM on robot1

Run on **robot1's Jetson** only.

```bash
ros2 launch yahboomcar_multi X3_slam_robot1_launch.py
```

This launches `slam_toolbox` with `yahboomcar_multi/param/X3_slam_robot1.yaml`, which sets `odom_frame: robot1/odom`, `base_frame: robot1/base_footprint`, and `scan_topic: /robot1/scan`. SLAM publishes the TF chain `map → robot1/odom → robot1/base_footprint`, placing robot1 in the shared world frame.

**Verify SLAM is working:**

```bash
# Confirm the map topic is publishing (~0.1–1 Hz)
ros2 topic hz /map

# Confirm the TF chain — should stream position/orientation values
ros2 run tf2_ros tf2_echo map robot1/base_footprint
```

If `tf2_echo` prints `Waiting for transform...` indefinitely, check that the bringup is running and that `/robot1/scan` has data (`ros2 topic hz /robot1/scan`).

---

### Step 3 — Drive the robots to build the map

Use `tmux` or two SSH sessions. `Ctrl+C` when mapping is done.

On **robot1's Jetson**:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/robot1/cmd_vel
```

On **robot2's Jetson**:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/robot2/cmd_vel
```

---

### Step 4 — Localize robot2 in the map (AMCL)

Robot1 is already localized by SLAM. Robot2 needs AMCL to join the same map frame.

On **robot2's Jetson**:

```bash
ros2 launch yahboomcar_multi X3_amcl_robot2_launch.py
```

**Physical setup:** place robot2 **0.5 m directly behind robot1**, both facing the same direction. SLAM sets the map origin at robot1's starting position with +x forward, so robot2's starting map coordinates are `(-0.5, 0)`.

Set the initial pose from **robot1's Jetson**:

```bash
ros2 topic pub --once /robot2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: -0.5, y: 0.0, z: 0.0},
  orientation: {w: 1.0}}, covariance: [0.5,0,0,0,0,0, 0,0.5,0,0,0,0,
  0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.1]}}'
```

Verify before proceeding:

```bash
ros2 run tf2_ros tf2_echo map robot2/base_footprint
```

Once that prints a steady stream of transforms, both robots are in the shared map frame.

---

### Step 5 — Launch formation control

Run on **robot1's Jetson**. Steps 1, 2, and 4 must already be running.

```bash
ros2 launch formation_control hardware.launch.py
```

The node waits until TF resolves `map → robotN/base_footprint` for all robots, initializes a straight-line path starting at robot1's map-frame position, then commands all robots at 20 Hz.

**Environment presets** (`env:=<name>` sets a group of parameters):

| `env` | Description | Overrides |
|---|---|---|
| `default` | 2 m path along map +x axis | — |
| `forward` | 3 m straight ahead in the leader's facing direction at launch | `path_length=3.0`, `use_robot_heading=true` |

```bash
ros2 launch formation_control hardware.launch.py
ros2 launch formation_control hardware.launch.py controller:=behavior
ros2 launch formation_control hardware.launch.py controller:=virtual_structure
```

**All arguments:**

| Argument | Default | Options |
|---|---|---|
| `env` | `forward` | `default` \| `forward` |
| `controller` | `leader_follower` | `behavior` \| `virtual_structure` \| `leader_follower` |
| `formation_type` | `column` | `column` \| `line` \| `triangle` \| `diamond` |
| `formation_spacing` | `0.5` | spacing in metres |
| `num_robots` | `2` | number of robots |
| `path_length` | `2.0` | trajectory length in metres (ignored by `forward` env) |
| `path_angle_deg` | `0.0` | path heading in degrees (ignored when `env:=forward`) |
| `max_hw_speed` | `0.15` | velocity cap sent to robots (m/s) |
| `path_speed` | `0.05` | rate of path parameter advance (s⁻¹) |
| `map_frame` | `map` | TF root frame shared by both robots |

`Ctrl+C` stops the node and sends a zero-velocity command to all robots.

---

### Topic map

```
        robot1 Jetson                              robot2 Jetson
  ┌──────────────────────────┐             ┌──────────────────────┐
  │  bringup (robot_name=robot1)           │  bringup (robot_name=robot2)
  │  ekf → robot1/odom       │             │  ekf → robot2/odom   │
  │         robot1/base_fprint             │       robot2/base_fprint
  │  driver ← /robot1/cmd_vel             │  driver ← /robot2/cmd_vel
  │  lidar  → /robot1/scan   │             │  lidar  → /robot2/scan│
  │                          │             │                      │
  │  slam_toolbox:           │             │  amcl:               │
  │   map→robot1/odom (TF)   │             │   map→robot2/odom(TF)│
  │   publishes /map         │◄── /map ───►│   subscribes /map    │
  │                          │             └──────────────────────┘
  │  formation_hw_node       │
  │   TF lookup:             │◄─ map→robot2/base_footprint (via network TF)
  │    map→robot1/base_fprint│
  │    map→robot2/base_fprint│──► /robot1/cmd_vel, /robot2/cmd_vel
  └──────────────────────────┘
```

---

### Key constraints

- Each robot **must** have a unique `robot_name` — never reuse a name across two bringup launches.
- Run SLAM Toolbox **exactly once** — a second instance will produce a conflicting `/map`.