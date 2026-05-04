# Contributions to the Yahboomcar ROS2 Library

This document records every change we made to the Yahboom ROSMASTER-X3 ROS2 source code to produce the `yahboom_modified/` workspace used in this project.

---

## Testing Status

We conducted hardware testing on two ROSMASTER-X3 robots running ROS2 Foxy on NVIDIA Jetson hardware. Our testing confirmed that both robots can be brought up with correct namespaced topics and that their relative positions can be resolved consistently through the shared TF tree (`map → robot1/base_footprint` and `map → robot2/base_footprint`).

We were not able to validate formation control end-to-end on hardware (`ros2 launch formation_control hardware.launch.py`). See **Next Steps** below.

---

## New Package: `formation_control`

We introduce a new ROS2 Python package that bridges the project's Python simulation controllers to the real robots.

### `formation_control/formation_control/formation_hw_node.py`

The main hardware node. Our key design decisions:

- **Position from TF** (`map → robotN/base_footprint`): Both robots are localized in a shared world frame via SLAM (robot1) and AMCL (robot2). We use TF to obtain consistent map-frame positions for both robots simultaneously. Using `/robotN/odom` directly would give positions in each robot's own local odometry frame with a different origin, making relative position computation incorrect.

- **Velocity from TF position differentiation**: We estimate world-frame velocity by finite-differencing consecutive TF positions at the 20 Hz control rate. This is a known limitation: AMCL makes discrete position corrections at scan rate (~5–10 Hz) that appear as velocity spikes at 20 Hz, which can corrupt the velocity-matching control term. We identify replacing this with EKF odometry velocity as a next step.

- **Path initialized from robot1's heading** (`use_robot_heading=True` in the `forward` env): Rather than driving along the map's fixed +x axis (which is determined by the direction robot1 faced when SLAM started), we initialize the path in the direction robot1 is actually facing at launch time. This makes the behavior consistent regardless of map orientation.

- **World-frame → robot-frame conversion**: Our formation controllers compute velocity commands in the map frame. Before publishing to `cmd_vel`, we rotate commands into each robot's body frame using its current TF heading, since the holonomic drive interprets `linear.x/y` as forward/lateral in the robot's own frame.

Parameters:

| Parameter | Default | Description |
|---|---|---|
| `env` | `forward` | Named preset — `default` (base params only) or `forward` (path_length=3.0, max_hw_speed=0.15, use_robot_heading=True) |
| `controller` | `leader_follower` | `leader_follower`, `behavior`, or `virtual_structure` |
| `formation_type` | `column` | `column`, `line`, `triangle`, or `diamond` |
| `formation_spacing` | `0.5` | Inter-robot spacing (m) |
| `num_robots` | `2` | Number of robots |
| `path_length` | `2.0` | Trajectory length (m) |
| `path_angle_deg` | `0.0` | Path direction in map frame (unused when `use_robot_heading=True`) |
| `use_robot_heading` | `False` | Align path with robot1's heading at initialization |
| `max_hw_speed` | `0.15` | Velocity cap sent to robots (m/s) |
| `path_speed` | `0.05` | Rate of path parameter advance (s⁻¹) |
| `map_frame` | `map` | Shared TF root frame |

### `formation_control/launch/hardware.launch.py`

We implement a launch file with named environment presets, using `OpaqueFunction` to apply preset parameter overrides at launch time. This is required for conditional parameters in ROS2 Foxy. The `forward` preset sets `path_length=3.0`, `max_hw_speed=0.15`, and `use_robot_heading=True`.

---

## New Files in Existing Packages

### `yahboomcar_multi/launch/X3_slam_robot1_launch.py`

We add a dedicated launch file that starts `slam_toolbox` (`async_slam_toolbox_node`) with parameters loaded from `X3_slam_robot1.yaml`. A dedicated launch file is necessary because `ros2 launch` does not forward `--ros-args` flags to launched nodes, so frame names must be configured via a params YAML file rather than command-line arguments.

### `yahboomcar_multi/param/X3_slam_robot1.yaml`

We add a SLAM Toolbox configuration for robot1:

```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: robot1/odom
    map_frame: map
    base_frame: robot1/base_footprint
    scan_topic: /robot1/scan
    use_sim_time: false
    mode: mapping
```

Without these overrides, SLAM defaults to the frame names `odom` and `base_footprint`, which do not exist in the namespaced TF tree. SLAM would publish `map → odom` instead of `map → robot1/odom`, breaking the TF chain our formation controller depends on.

### `yahboomcar_description/launch/description_X3_multi_launch.py`

We introduce a generic robot description launch that replaces the two separate per-robot files (`description_X3_multi_robot1.launch.py`, `description_X3_multi_robot2.launch.py`). It takes `robot_name` as a launch argument and dynamically resolves `yahboomcar_X3_<robot_name>.urdf` using `OpaqueFunction`.

---

## Bug Fixes in Existing Packages

### `yahboomcar_base_node/src/base_node_X3.cpp`, `base_node_x1.cpp`, `base_node_R2.cpp` — `last_vel_time_` epoch initialization

We identify a bug in `last_vel_time_`, a class member of type `rclcpp::Time`. In ROS2 Foxy, its default constructor sets it to nanoseconds = 0 (Unix epoch). On the very first `vel_raw` callback, `vel_dt_` was computed as:

```
current_unix_timestamp − 0 ≈ 1.78 × 10⁹ seconds
```

Any non-zero velocity reading (including sensor noise) multiplied by this value produced a massive position or heading jump in the integrated odometry. On robot1 this was masked by SLAM's continuous scan-matching correction. On robot2, AMCL could not overcome an odometry offset of tens of meters, causing `map → robot2/odom` to be wrong by ~24 m even after AMCL was given a correct initial pose.

We fix this by seeding `last_vel_time_` with the current time on the first message:

```cpp
if (last_vel_time_.nanoseconds() == 0) {
    last_vel_time_ = curren_time;
}
vel_dt_ = (curren_time - last_vel_time_).seconds();
```

---

## Namespace Compatibility Changes

We make the following changes to ensure the existing Yahboom multi-robot infrastructure works correctly with per-robot ROS2 namespaces (`/robot1/`, `/robot2/`).

### `yahboomcar_multi/param/X3_ekf_robot1.yaml` and `X3_ekf_robot2.yaml`

```yaml
# before:
odom0: /odom_raw

# after:
odom0: odom_raw
```

We find that the original used an absolute topic path, bypassing the `robot1/` namespace pushed by `PushRosNamespace`. Both robots' EKF nodes were subscribing to the same global `/odom_raw` topic. By changing to a relative path, each EKF subscribes to its own namespaced topic (`robot1/odom_raw` or `robot2/odom_raw`).

### `yahboomcar_multi/launch/X3_bringup_multi.launch.xml` — three changes

**1. Removed stale `/odom_raw` remap:** The original bringup contained `<remap from="/odom_raw" to="odom_raw"/>` as a workaround for the absolute-path EKF YAML bug above. With that bug fixed, we remove the remap as it is no longer needed.

**2. Added `base_link → laser` static transform:**

```xml
<node pkg="tf2_ros" exec="static_transform_publisher" name="base_link_to_laser"
      args="0.0435 5.258e-05 0.11 3.14 0 0 $(var robot_name)/base_link $(var robot_name)/laser"/>
```

We observe that SLAM Toolbox drops all incoming scans with `frame 'robot1/laser' ... reason 'Unknown'` unless TF contains a path from the laser frame to the robot base. The original bringup did not publish this transform. We source the mount values (x=0.0435 m, z=0.11 m, yaw=π) from the existing AMCL launch configs in `yahboomcar_multi`.

**3. Switched to generic description launch:**

```xml
<!-- before: -->
<include file="$(find-pkg-share yahboomcar_description)/launch/description_X3_multi_$(var robot_name).launch.py"/>

<!-- after: -->
<include file="$(find-pkg-share yahboomcar_description)/launch/description_X3_multi_launch.py">
    <arg name="robot_name" value="$(var robot_name)"/>
</include>
```

We replace file-name string interpolation (which required a separate `.launch.py` per robot name) with the new generic launch file that accepts `robot_name` as an argument.

---

## Summary Table

| File | Type | Change |
|---|---|---|
| `formation_control/` | New package | Hardware bridge: TF position, TF-differentiated velocity, formation controllers, env presets |
| `yahboomcar_multi/launch/X3_slam_robot1_launch.py` | New file | SLAM launch with correct namespaced frame params |
| `yahboomcar_multi/param/X3_slam_robot1.yaml` | New file | SLAM frame config (`robot1/odom`, `robot1/base_footprint`, `/robot1/scan`) |
| `yahboomcar_description/launch/description_X3_multi_launch.py` | New file | Generic description launch via `OpaqueFunction` (Foxy-compatible) |
| `yahboomcar_multi/launch/X3_bringup_multi.launch.xml` | Modified | Removed stale odom remap; added `base_link→laser` TF; generic description include |
| `yahboomcar_multi/param/X3_ekf_robot1.yaml` | Bug fix | `odom0: /odom_raw` → `odom0: odom_raw` (namespace-relative topic) |
| `yahboomcar_multi/param/X3_ekf_robot2.yaml` | Bug fix | Same as robot1 |
| `yahboomcar_base_node/src/base_node_X3.cpp` | Bug fix | Fixed `last_vel_time_` epoch-0 initialization |
| `yahboomcar_base_node/src/base_node_x1.cpp` | Bug fix | Same fixes as X3 |
| `yahboomcar_base_node/src/base_node_R2.cpp` | Bug fix | Same fixes as X3 |
| `formation.py` (simulation root) | Enhancement | Added `Formation.column()` static method |

---

## Next Steps

### Formation Control Hardware Validation

We were not able to test formation control end-to-end on hardware. The following need to be validated before the system can be considered working:

- **End-to-end run with `leader_follower` + `column`**: confirm robot1 drives forward along the path and robot2 maintains the target offset (`-0.5, 0` in robot1's frame).
- **Velocity estimation**: our current implementation differentiates TF positions at 20 Hz to estimate velocity. AMCL corrects robot2's map position at scan rate (~5–10 Hz), which can appear as a large velocity spike at the control rate and corrupt the `f_align` velocity-matching term in the leader-follower and behavior controllers. We propose replacing TF position differentiation with EKF odometry velocity from `/robotN/odom` (smooth, no AMCL jumps). The odometry velocity is in the robot body frame and must be rotated to the map frame using the robot's TF heading before storing in `states[i, 2:]`.
- **Gain tuning**: we tune controller gains (`k_leader`, `k_follow`, `k_tangent`, `w_align`, `w_avoid`) for the Python simulation where robots can move at up to 5 m/s. At the hardware cap of 0.15 m/s these ratios have not been validated and may need adjustment.
- **`behavior` controller**: not tested on hardware.
- **`virtual_structure` controller**: not tested on hardware.
- **`line`, `triangle`, `diamond` formations**: not tested on hardware.

### Infrastructure

- **AMCL for robot1**: currently robot1 relies on SLAM for localization throughout the run. For a more reliable setup, we suggest stopping SLAM after the map is built and replacing it with AMCL for robot1 as well, freeing compute and giving more stable localization for both robots.
- **Automatic AMCL convergence check**: our formation controller starts as soon as TF is available, regardless of whether AMCL has converged to the correct location. We propose a convergence check (e.g., particle cloud variance below a threshold) before enabling the controller, which would prevent the follower from driving toward a wrong position.
- **Three or more robots**: the `num_robots` parameter and `Formation.column()` support arbitrary N, but we have only attempted 2-robot operation on hardware. Additional robots require additional EKF YAML files (`X3_ekf_robot3.yaml`, etc.) and AMCL instances.
