from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q) -> float:
    # extracts yaw from a geometry_msgs/Quaternion without tf_transformations
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def odom_to_state(msg: Odometry) -> tuple[np.ndarray, float]:
    # converts nav_msgs/Odometry to (state[4], yaw).
    # state = [x, y, vx_world, vy_world]; velocity is rotated from body frame to world frame.
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    # ROSMASTER-X3 is holonomic: odom reports both linear.x and linear.y in body frame
    vx_body = msg.twist.twist.linear.x
    vy_body = msg.twist.twist.linear.y
    c, s = math.cos(yaw), math.sin(yaw)
    vx_world = c * vx_body - s * vy_body
    vy_world = s * vx_body + c * vy_body

    return np.array([x, y, vx_world, vy_world]), yaw


def world_to_body_twist(vx_world: float, vy_world: float, theta: float) -> Twist:
    # rotates a world-frame velocity into a body-frame Twist (linear.x=fwd, linear.y=lateral, angular.z=0).
    twist = Twist()
    c, s = math.cos(theta), math.sin(theta)
    twist.linear.x = float( c * vx_world + s * vy_world)
    twist.linear.y = float(-s * vx_world + c * vy_world)
    twist.angular.z = 0.0
    return twist
