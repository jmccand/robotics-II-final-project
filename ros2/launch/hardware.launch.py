from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('controller', default_value='leader_follower',
                              description='behavior | virtual_structure | leader_follower'),
        DeclareLaunchArgument('formation_type', default_value='line',
                              description='line | triangle | diamond'),
        DeclareLaunchArgument('formation_spacing', default_value='0.5',
                              description='Distance between robots in metres'),
        DeclareLaunchArgument('n_robots', default_value='2'),
        DeclareLaunchArgument('path_type', default_value='straight',
                              description='straight | waypoint'),
        DeclareLaunchArgument('path_length', default_value='2.0',
                              description='Length of straight path in metres'),
        DeclareLaunchArgument('waypoints_json', default_value='[[0,0],[2,0]]',
                              description='JSON array of [fwd,lat] offsets for waypoint paths'),
        DeclareLaunchArgument('path_speed', default_value='0.02',
                              description='Path parameter advance rate per control tick'),
        DeclareLaunchArgument('max_hw_speed', default_value='0.3',
                              description='Maximum speed sent to robots (m/s)'),
        DeclareLaunchArgument('dt', default_value='0.05',
                              description='Control loop period in seconds (20 Hz default)'),
    ]

    node = Node(
        package='formation_control',
        executable='formation_node',
        name='formation_node',
        output='screen',
        parameters=[{
            'controller':        ParameterValue(LaunchConfiguration('controller'),        value_type=str),
            'formation_type':    ParameterValue(LaunchConfiguration('formation_type'),    value_type=str),
            'formation_spacing': ParameterValue(LaunchConfiguration('formation_spacing'), value_type=float),
            'n_robots':          ParameterValue(LaunchConfiguration('n_robots'),          value_type=int),
            'path_type':         ParameterValue(LaunchConfiguration('path_type'),         value_type=str),
            'path_length':       ParameterValue(LaunchConfiguration('path_length'),       value_type=float),
            'waypoints_json':    ParameterValue(LaunchConfiguration('waypoints_json'),    value_type=str),
            'path_speed':        ParameterValue(LaunchConfiguration('path_speed'),        value_type=float),
            'max_hw_speed':      ParameterValue(LaunchConfiguration('max_hw_speed'),      value_type=float),
            'dt':                ParameterValue(LaunchConfiguration('dt'),                value_type=float),
        }],
    )

    return LaunchDescription(args + [node])
