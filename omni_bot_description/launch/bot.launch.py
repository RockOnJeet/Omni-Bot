import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch argument for use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # Get the package share directory
    pkg_share = get_package_share_directory('omni_bot_description')

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Info message about sim time
    sim_time_info = LogInfo(
        msg=['NOTE - use_sim_time: ', use_sim_time]
    )

    # Path to the xacro file
    xacro_file = os.path.join(pkg_share, 'description', 'bot.urdf.xacro')

    # Process the xacro file to generate URDF
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        sim_time_info
    ])
