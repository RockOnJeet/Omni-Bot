from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch file for Gazebo simulation of Omni Bot with ROS-Gazebo bridge and RViz2."""

    # Package shares
    desc_pkg = get_package_share_directory('omni_bot_description')
    hw_pkg = get_package_share_directory('omni_bot_hw')
    ydlidar_pkg = get_package_share_directory('ydlidar_ros2_driver')

    # Launch robot_state_publisher
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                desc_pkg,
                'launch',
                'bot.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_joint_state_publisher_gui': 'false'
        }.items()
    )

    # YDLidar ROS2 driver
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ydlidar_pkg,
                'launch',
                'ydlidar_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )

    # Command converter for wheels
    hw_node = Node(
        package='omni_bot_hw',
        executable='omni_controller_node',
        name='omni_controller_node',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        rsp_launch,
        ydlidar_launch,
        hw_node
    ])
