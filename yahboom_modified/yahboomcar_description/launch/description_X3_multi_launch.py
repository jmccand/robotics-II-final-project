# New file: replaces separate description_X3_multi_robot1/2.launch.py files with one parameterised launch, eliminating the need to add a new file per robot name.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        name='robot_name', default_value='robot1',
        description='Unique robot name used as namespace and URDF prefix')
    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='false', choices=['true', 'false'],
        description='Launch joint_state_publisher_gui instead of joint_state_publisher')

    # Dynamically resolve yahboomcar_X3_<robot_name>.urdf
    urdf_path = PathJoinSubstitution([
        FindPackageShare('yahboomcar_description'),
        'urdf',
        ['yahboomcar_X3_', LaunchConfiguration('robot_name'), '.urdf'],
    ])

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=LaunchConfiguration('robot_name'),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    return LaunchDescription([
        robot_name_arg,
        gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
    ])
