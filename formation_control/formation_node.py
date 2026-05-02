import sys
import termios
import threading
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from behavior.behavior_controller import BehaviorController
from leader_follower.lf_controller import LeaderFollowerController
from virtual_structure.vs_controller import VirtualStructureController
from formation_control.hw_environment import build_formation, build_path
from formation_control.twist_converter import odom_to_state, world_to_body_twist

_CONTROLLERS = {
    'behavior': BehaviorController,
    'virtual_structure': VirtualStructureController,
    'leader_follower': LeaderFollowerController,
}

# Topics follow the namespacing pattern seen in ros2 topic list.
_ODOM_TOPICS = ['/robot1/odom', '/robot2/odom']
_CMD_TOPICS   = ['/robot1/cmd_vel', '/robot2/cmd_vel']


class FormationNode(Node):
    def __init__(self):
        super().__init__('formation_node')

        # --- parameters ---
        self.declare_parameter('controller', 'leader_follower')
        self.declare_parameter('formation_type', 'line')
        self.declare_parameter('formation_spacing', 0.5)
        self.declare_parameter('n_robots', 2)
        self.declare_parameter('path_type', 'straight')
        self.declare_parameter('path_length', 2.0)
        self.declare_parameter('waypoints_json', '[[0,0],[2,0]]')
        self.declare_parameter('path_speed', 0.02)
        self.declare_parameter('max_hw_speed', 0.3)
        self.declare_parameter('dt', 0.05)

        ctrl_name = self.get_parameter('controller').value
        if ctrl_name not in _CONTROLLERS:
            raise ValueError(f"Unknown controller '{ctrl_name}'. Choose from {list(_CONTROLLERS)}")
        self._controller = _CONTROLLERS[ctrl_name]()
        self.get_logger().info(f"Controller: {ctrl_name}")

        self._n_robots     = self.get_parameter('n_robots').value
        self._path_speed   = self.get_parameter('path_speed').value
        self._max_hw_speed = self.get_parameter('max_hw_speed').value
        self._dt           = self.get_parameter('dt').value

        # --- state ---
        self._states    = np.zeros((self._n_robots, 4))  # [x, y, vx_world, vy_world]
        self._headings  = np.zeros(self._n_robots)
        self._received  = [False] * self._n_robots
        self._path      = None  # built on first leader odom
        self._formation = None
        self._t         = 0.0
        self._paused    = True  # start paused; SPACE toggles

        # --- ROS2 interfaces ---
        self._cmd_pubs = [
            self.create_publisher(Twist, topic, 10)
            for topic in _CMD_TOPICS[:self._n_robots]
        ]
        for i, topic in enumerate(_ODOM_TOPICS[:self._n_robots]):
            self.create_subscription(
                Odometry, topic,
                lambda msg, idx=i: self._odom_cb(msg, idx),
                10,
            )

        self._timer = self.create_timer(self._dt, self._control_loop)
        self.get_logger().info('Waiting for odometry from all robots...')

        # Start keyboard listener in a background daemon thread.
        # Skipped gracefully if stdin is not a TTY (e.g. piped output).
        if sys.stdin.isatty():
            t = threading.Thread(target=self._keyboard_listener, daemon=True)
            t.start()
        else:
            self.get_logger().warn(
                'stdin is not a TTY — keyboard listener disabled. '
                'Run with output:=screen and an interactive terminal to use SPACE.'
            )

    def _keyboard_listener(self):
        """Read keypresses in raw terminal mode. SPACE toggles start/pause."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == ' ':
                    self._paused = not self._paused
                    if self._paused:
                        self._stop_robots()
                        self.get_logger().info('PAUSED  — press SPACE to resume')
                    else:
                        self.get_logger().info('RUNNING — press SPACE to pause')
                elif ch == '\x03':  # Ctrl+C
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _stop_robots(self):
        stop = Twist()
        for pub in self._cmd_pubs:
            pub.publish(stop)

    def _odom_cb(self, msg: Odometry, robot_idx: int):
        state, yaw = odom_to_state(msg)
        self._states[robot_idx]   = state
        self._headings[robot_idx] = yaw
        first_recv = not self._received[robot_idx]
        self._received[robot_idx] = True

        # Build path relative to leader (robot 0) on its very first odom message
        if robot_idx == 0 and first_recv and self._path is None:
            start_pos = state[:2].copy()
            self._path      = build_path(self, start_pos, yaw)
            self._formation = build_formation(self)
            self.get_logger().info(
                f'Path anchored at ({start_pos[0]:.2f}, {start_pos[1]:.2f}), '
                f'heading {yaw:.2f} rad. Formation: '
                f'{self.get_parameter("formation_type").value} '
                f'({self._formation.n} robots)'
            )
            self.get_logger().info('Ready — press SPACE to start')

    def _control_loop(self):
        if self._path is None or not all(self._received) or self._paused:
            return

        for i in range(self._n_robots):
            v_cmd = self._controller.compute_control(
                i, self._states, self._path, self._t, self._formation, []
            )
            speed = np.linalg.norm(v_cmd)
            if speed > self._max_hw_speed:
                v_cmd = v_cmd * (self._max_hw_speed / speed)

            twist = world_to_body_twist(float(v_cmd[0]), float(v_cmd[1]), self._headings[i])
            self._cmd_pubs[i].publish(twist)

        self._t = min(self._t + self._path_speed * self._dt, 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = FormationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robots()
        node.destroy_node()
        rclpy.shutdown()
