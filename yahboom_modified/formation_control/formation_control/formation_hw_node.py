#!/usr/bin/env python3
import math

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from formation_control.behavior_controller import BehaviorController
from formation_control.formation import Formation
from formation_control.lf_controller import LeaderFollowerController
from formation_control.path import StraightLinePath
from formation_control.vs_controller import VirtualStructureController


def _quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _world_to_robot(vx_w: float, vy_w: float, heading: float):
    c, s = math.cos(heading), math.sin(heading)
    return c * vx_w + s * vy_w, -s * vx_w + c * vy_w


class FormationHardwareNode(Node):
    def __init__(self):
        super().__init__('formation_hw_node')

        self.declare_parameter('controller', 'leader_follower')
        self.declare_parameter('formation_type', 'column')
        self.declare_parameter('formation_spacing', 0.5)
        self.declare_parameter('num_robots', 2)
        self.declare_parameter('max_hw_speed', 0.15)
        self.declare_parameter('path_length', 2.0)
        self.declare_parameter('path_angle_deg', 0.0)
        self.declare_parameter('use_robot_heading', False)
        self.declare_parameter('robot_prefix', 'robot')
        self.declare_parameter('path_speed', 0.05)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('map_frame', 'map')

        controller_type = str(self.get_parameter('controller').value)
        formation_type = str(self.get_parameter('formation_type').value)
        spacing = float(self.get_parameter('formation_spacing').value)
        self.n = int(self.get_parameter('num_robots').value)
        self.max_speed = float(self.get_parameter('max_hw_speed').value)
        self.path_length = float(self.get_parameter('path_length').value)
        self.path_angle = math.radians(float(self.get_parameter('path_angle_deg').value))
        self.use_robot_heading = bool(self.get_parameter('use_robot_heading').value)
        self.robot_prefix = str(self.get_parameter('robot_prefix').value)
        self.path_speed = float(self.get_parameter('path_speed').value)
        rate = float(self.get_parameter('control_rate').value)
        self.dt = 1.0 / rate
        self.map_frame = str(self.get_parameter('map_frame').value)

        if formation_type == 'triangle':
            self.formation = Formation.triangle(spacing)
        elif formation_type == 'diamond':
            self.formation = Formation.diamond(spacing)
        elif formation_type == 'line':
            self.formation = Formation.line(self.n, spacing)
        else:
            self.formation = Formation.column(self.n, spacing)

        if controller_type == 'behavior':
            self.controller = BehaviorController()
        elif controller_type == 'virtual_structure':
            self.controller = VirtualStructureController()
        else:
            self.controller = LeaderFollowerController()

        # TF2 — positions come from map→robot/base_footprint, shared coordinate frame for all robots.
        # each robot's odometry frame has a different origin, so odom topics cannot be compared directly.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # per-robot state in map frame: [x, y, vx_world, vy_world]
        self.states = np.zeros((self.n, 4))
        self.headings = np.zeros(self.n)
        self._prev_xy = None   # set on first successful lookup to avoid a velocity spike
        self.path = None
        self.t = 0.0

        self.cmd_publishers = []
        for i in range(self.n):
            robot_name = f'{self.robot_prefix}{i + 1}'
            pub = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 1)
            self.cmd_publishers.append(pub)
            self.get_logger().info(
                f'Will look up {self.map_frame} → {robot_name}/base_footprint, '
                f'publishing to /{robot_name}/cmd_vel'
            )

        self.create_timer(self.dt, self._control_loop)
        self.get_logger().info(
            f'FormationHardwareNode: controller={controller_type}, '
            f'formation={formation_type}, n={self.n}, map_frame={self.map_frame}'
        )

    def _lookup_all_poses(self) -> bool:
        # populates self.states[:, :2] and self.headings from TF; returns False if any lookup fails.
        # all positions are in self.map_frame so both robots share a common coordinate system.
        new_xy = np.zeros((self.n, 2))
        new_headings = np.zeros(self.n)

        for i in range(self.n):
            robot_name = f'{self.robot_prefix}{i + 1}'
            target = f'{robot_name}/base_footprint'
            try:
                t = self.tf_buffer.lookup_transform(
                    self.map_frame, target, rclpy.time.Time()
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(
                    f'Waiting for TF {self.map_frame} → {target}: {e}',
                    throttle_duration_sec=3.0,
                )
                return False

            new_xy[i] = [t.transform.translation.x, t.transform.translation.y]
            new_headings[i] = _quat_to_yaw(t.transform.rotation)

        # first successful lookup: initialise position, zero velocity to avoid a spike.
        if self._prev_xy is None:
            self._prev_xy = new_xy.copy()

        for i in range(self.n):
            vx = (new_xy[i, 0] - self._prev_xy[i, 0]) / self.dt
            vy = (new_xy[i, 1] - self._prev_xy[i, 1]) / self.dt
            self.states[i] = [new_xy[i, 0], new_xy[i, 1], vx, vy]
            self.headings[i] = new_headings[i]

        self._prev_xy = new_xy
        return True

    def _init_path(self) -> None:
        start = self.states[0, :2].copy()
        angle = self.headings[0] if self.use_robot_heading else self.path_angle
        direction = np.array([math.cos(angle), math.sin(angle)])
        end = start + direction * self.path_length
        self.path = StraightLinePath(start, end)
        src = 'robot heading' if self.use_robot_heading else f'map angle {math.degrees(angle):.1f}°'
        self.get_logger().info(f'Path initialised ({src}): {start} → {end}')

    def _control_loop(self) -> None:
        if not self._lookup_all_poses():
            return

        if self.path is None:
            self._init_path()
            return

        for i in range(self.n):
            v_cmd = self.controller.compute_control(
                i, self.states.copy(), self.path, self.t, self.formation, obstacles=[]
            )

            speed = np.linalg.norm(v_cmd)
            if speed > self.max_speed:
                v_cmd = v_cmd * (self.max_speed / speed)

            # convert map-frame velocity command to robot body frame before publishing.
            vx_r, vy_r = _world_to_robot(float(v_cmd[0]), float(v_cmd[1]), self.headings[i])

            twist = Twist()
            twist.linear.x = vx_r
            twist.linear.y = vy_r
            self.cmd_publishers[i].publish(twist)

        self.t = min(1.0, self.t + self.path_speed * self.dt)

    def _stop_all(self) -> None:
        for pub in self.cmd_publishers:
            pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FormationHardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_all()
        node.destroy_node()
        rclpy.shutdown()
