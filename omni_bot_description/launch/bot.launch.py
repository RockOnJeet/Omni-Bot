import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launches robot_state_publisher node with robot URDF.

    Publishes: /robot_description parameter, TF transforms
    Subscribes: /joint_states
    Arguments: use_sim_time (default: true)
    """

    # Declare use_sim_time launch argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='gz',
        description='Set the robot model (gz or rviz)'
    )

    # Get package share directory
    desc_pkg = get_package_share_directory('omni_bot_description')

    # Get launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')

    # Log sim time setting
    sim_time_info = LogInfo(
        msg=['NOTE: Sim time set to ', use_sim_time]
    )

    # Log robot model setting
    robot_model_info = LogInfo(
        msg=['NOTE: Robot model set to ', robot_model]
    )

    # Path to xacro file
    xacro_file = PathJoinSubstitution([
        desc_pkg,
        'description',
        [robot_model, '_bot.urdf.xacro']
    ])

    # Process xacro to generate URDF
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Robot state publisher node
    # Publishes /robot_description and TF tree
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=[robot_model, '_robot_state_publisher'],
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_model_arg,
        robot_state_publisher_node,
        sim_time_info,
        robot_model_info
    ])
