# New file: replaces separate description_X3_multi_robot1/2.launch.py files with one parameterised launch,
# eliminating the need to add a new file per robot name.
# Uses OpaqueFunction for path construction — PathJoinSubstitution nested lists are not supported in Foxy.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _make_nodes(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    urdf_path = os.path.join(
        get_package_share_directory('yahboomcar_description'),
        'urdf',
        f'yahboomcar_X3_{robot_name}.urdf',
    )
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    return [
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=robot_name,
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            parameters=[{'robot_description': robot_description}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1',
                              description='Unique robot name used as namespace and URDF prefix'),
        OpaqueFunction(function=_make_nodes),
    ])
