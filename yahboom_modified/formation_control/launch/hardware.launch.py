from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Named environment presets — each entry overrides only the keys it cares about.
_ENV_PRESETS = {
    'forward': {
        'path_length': 3.0,
        'max_hw_speed': 0.15,
        'use_robot_heading': True,   # align path with leader's heading at launch time
    },
}


def _make_node(context, *args, **kwargs):
    params = {
        'controller':        LaunchConfiguration('controller').perform(context),
        'formation_type':    LaunchConfiguration('formation_type').perform(context),
        'formation_spacing': float(LaunchConfiguration('formation_spacing').perform(context)),
        'num_robots':        int(LaunchConfiguration('num_robots').perform(context)),
        'path_length':       float(LaunchConfiguration('path_length').perform(context)),
        'path_angle_deg':    float(LaunchConfiguration('path_angle_deg').perform(context)),
        'max_hw_speed':      float(LaunchConfiguration('max_hw_speed').perform(context)),
        'path_speed':        float(LaunchConfiguration('path_speed').perform(context)),
        'map_frame':         LaunchConfiguration('map_frame').perform(context),
        'use_robot_heading': False,
    }

    env = LaunchConfiguration('env').perform(context)
    if env in _ENV_PRESETS:
        params.update(_ENV_PRESETS[env])

    return [
        Node(
            package='formation_control',
            executable='formation_hw',
            name='formation_hw_node',
            output='screen',
            parameters=[params],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='forward',
                              description='Named environment preset: default | forward'),
        DeclareLaunchArgument('controller', default_value='leader_follower',
                              description='behavior | virtual_structure | leader_follower'),
        DeclareLaunchArgument('formation_type', default_value='line',
                              description='line | triangle | diamond'),
        DeclareLaunchArgument('formation_spacing', default_value='0.5',
                              description='Inter-robot spacing (m)'),
        DeclareLaunchArgument('num_robots', default_value='2',
                              description='Number of robots'),
        DeclareLaunchArgument('path_length', default_value='2.0',
                              description='Trajectory length (m) — overridden by env preset'),
        DeclareLaunchArgument('path_angle_deg', default_value='0.0',
                              description='Path direction in degrees (0 = map +x); ignored when env=forward'),
        DeclareLaunchArgument('max_hw_speed', default_value='0.15',
                              description='Maximum velocity sent to robots (m/s) — overridden by env preset'),
        DeclareLaunchArgument('path_speed', default_value='0.05',
                              description='Rate of path parameter advance per second'),
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='TF root frame shared by both robots'),
        OpaqueFunction(function=_make_node),
    ])
