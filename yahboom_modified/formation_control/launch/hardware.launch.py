from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='leader_follower',
                              description='behavior | virtual_structure | leader_follower'),
        DeclareLaunchArgument('formation_type', default_value='line',
                              description='line | triangle | diamond'),
        DeclareLaunchArgument('formation_spacing', default_value='0.5',
                              description='Inter-robot spacing (m)'),
        DeclareLaunchArgument('num_robots', default_value='2',
                              description='Number of robots'),
        DeclareLaunchArgument('path_length', default_value='2.0',
                              description='Trajectory length (m)'),
        DeclareLaunchArgument('path_angle_deg', default_value='0.0',
                              description='Path direction in degrees (0 = +x axis)'),
        DeclareLaunchArgument('max_hw_speed', default_value='0.3',
                              description='Maximum velocity sent to robots (m/s)'),
        DeclareLaunchArgument('path_speed', default_value='0.05',
                              description='Rate of path parameter advance per second'),
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='TF frame shared by both robots after SLAM/AMCL'),

        Node(
            package='formation_control',
            executable='formation_hw',
            name='formation_hw_node',
            output='screen',
            parameters=[{
                'controller': LaunchConfiguration('controller'),
                'formation_type': LaunchConfiguration('formation_type'),
                'formation_spacing': LaunchConfiguration('formation_spacing'),
                'num_robots': LaunchConfiguration('num_robots'),
                'path_length': LaunchConfiguration('path_length'),
                'path_angle_deg': LaunchConfiguration('path_angle_deg'),
                'max_hw_speed': LaunchConfiguration('max_hw_speed'),
                'path_speed': LaunchConfiguration('path_speed'),
                'map_frame': LaunchConfiguration('map_frame'),
            }],
        ),
    ])
