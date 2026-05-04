# robotics-II-final-project

RPI Robotics II final project comparing three multi-robot formation control approaches: leader-follower, virtual structure, and behavior-based. The project includes a pure Python simulation and a ROS2 hardware layer for ROSMASTER-X3 robots.

---

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

---

## Hardware (ROSMASTER-X3, ROS2 Foxy)

`yahboom_modified/` is a self-contained colcon workspace with 7 ROS2 packages:

```
yahboom_modified/
├── formation_control/      ← new: hardware bridge node + launch
├── yahboomcar_base_node/   ← patched: removed unused turtlesim dependency
├── yahboomcar_bringup/     ← Yahboom motor driver
├── yahboomcar_ctrl/        ← keyboard / joystick teleop
├── yahboomcar_description/ ← URDF + patched: added generic multi-robot description launch
├── yahboomcar_msgs/        ← Yahboom custom message types
└── yahboomcar_multi/       ← patched: multi-robot bringup launch + EKF params
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

On **each** Jetson, SSH into the robot and open a terminal in a docker container. Then:

```bash
cd ~/codes/
git clone <repo-url>
cd robotics-II-final-project/yahboom_modified
colcon build --symlink-install
echo "source ~/codes/robotics-II-final-project/yahboom_modified/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

This overlays the existing Yahboom workspace — no renaming needed. The last-sourced workspace wins, so all commands will use the packages from `yahboom_modified`. **Every new terminal must have the workspace sourced** (the `echo` line above handles that permanently).

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

If the lidar node exits immediately with a serial error, your robot has the S2 lidar — replace `X3_A1` with `X3_S2` in both commands (the only difference is baud rate).

This starts, under each robot's namespace:
- `Mcnamu_driver_X3` — hardware serial interface, subscribes to `/<robot_name>/cmd_vel`
- `base_node_X3` — wheel-encoder odometry, publishes `/<robot_name>/odom_raw`
- `imu_filter_madgwick_node` — fuses IMU data
- `ekf_filter_node` — fuses odometry + IMU, publishes `/<robot_name>/odom`
- `robot_state_publisher` — broadcasts TF tree under `/<robot_name>/` frames
- `sllidar_node` — lidar driver, publishes `/<robot_name>/scan` on `/dev/rplidar`
- static transform `/<robot_name>/base_link → /<robot_name>/laser` (required for SLAM)

---

### Step 2 — Start SLAM on robot1 (builds the shared map)

Run on **robot1's Jetson** only.

```bash
ros2 launch yahboomcar_multi X3_slam_robot1_launch.py
```

This launches slam_toolbox with `yahboomcar_multi/param/X3_slam_robot1.yaml`, which sets `odom_frame: robot1/odom`, `base_frame: robot1/base_footprint`, and `scan_topic: /robot1/scan`. SLAM will publish the TF chain `map → robot1/odom → robot1/base_footprint`, placing robot1 in the shared world frame.

**Verify SLAM is working** (run these on robot1's Jetson in a second terminal):

```bash
# 1. Confirm the map topic is publishing (should show ~0.1–1 Hz)
ros2 topic hz /map

# 2. Confirm the critical TF chain exists — this is what formation_control reads.
#    If SLAM is working you will see a stream of position/orientation values.
#    Ctrl+C to stop.
ros2 run tf2_ros tf2_echo map robot1/base_footprint
```

If `tf2_echo` prints `Waiting for transform...` indefinitely, SLAM is not publishing the `map → robot1/odom` link — check that the bringup (Step 1) is running and that `/robot1/scan` has data (`ros2 topic hz /robot1/scan`).

---

### Step 3 — Drive the robots to build the map

SSH two terminals (or use `tmux`). `Ctrl+C` when mapping is done.

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
AMCL subscribes to `/map` directly from SLAM Toolbox's live output — no saved map file needed.

On **robot2's Jetson**:

```bash
ros2 launch yahboomcar_multi X3_amcl_robot2_launch.py
```

This runs AMCL with `odom_frame: robot2/odom`, `base_frame: robot2/base_footprint`, and `global_frame: map`, publishing `map → robot2/odom`. AMCL subscribes to `/map` (published by SLAM on robot1) and `/robot2/scan`.

AMCL starts with a random particle distribution and needs an initial pose estimate to converge quickly. From **robot1's Jetson**, set robot2's starting position in the map:

```bash
ros2 topic pub --once /robot2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 0.5, y: 0.0, z: 0.0},
  orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0,
  0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'
```

Adjust `x`, `y`, and the orientation to match where robot2 actually started. Once the TF chain `map → robot2/odom → robot2/base_footprint` appears in `ros2 run tf2_tools view_frames`, both robots are in the shared frame and formation control can start.

---

### Step 5 — Launch formation control

Run on **robot1's Jetson**. Steps 1, 2, and 4 must already be running.

```bash
ros2 launch formation_control hardware.launch.py
```

The node waits until TF can resolve `map → robotN/base_footprint` for all robots, initializes a straight-line path starting at robot1's map-frame position, then commands all robots at 20 Hz.

**Optional arguments** (append as `key:=value`):

| Argument | Default | Options |
|---|---|---|
| `controller` | `leader_follower` | `behavior` \| `virtual_structure` \| `leader_follower` |
| `formation_type` | `line` | `line` \| `triangle` \| `diamond` |
| `formation_spacing` | `0.5` | spacing in metres |
| `num_robots` | `2` | number of robots |
| `path_length` | `2.0` | trajectory length in metres |
| `path_angle_deg` | `0.0` | path heading in degrees (0 = +x axis) |
| `max_hw_speed` | `0.3` | velocity cap sent to robots (m/s) |
| `path_speed` | `0.05` | rate of path parameter advance (s⁻¹) |
| `map_frame` | `map` | TF root frame shared by both robots |

Example:

```bash
ros2 launch formation_control hardware.launch.py \
  controller:=behavior formation_type:=triangle formation_spacing:=0.6 max_hw_speed:=0.2
```

`Ctrl+C` stops the node and sends a zero-velocity command to all robots.

---

### Topic map

```
        robot1 Jetson                              robot2 Jetson
  ┌──────────────────────────┐             ┌──────────────────────┐
  │  bringup (robot_name=robot1)           │  bringup (robot_name=robot2)
  │  ekf → robot1/odom→      │             │  ekf → robot2/odom→  │
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
- `max_hw_speed` should not exceed `0.5 m/s` indoors; the robots have no brake and will overshoot.
