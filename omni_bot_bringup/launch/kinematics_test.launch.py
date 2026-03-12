#!/usr/bin/env python3
"""
Launch file for the omni_bot_base controller node.
Converts /cmd_vel to wheel velocities for 3-wheel omni-directional robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for base controller."""

    # Declare launch arguments
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.076',
        description='Wheel radius in meters (default: 0.076 m)'
    )

    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.4185',
        description='Robot radius (center to wheel) in meters (default: 0.4185 m)'
    )

    max_wheel_velocity_arg = DeclareLaunchArgument(
        'max_wheel_velocity',
        default_value='8.0',
        description='Maximum wheel velocity in rad/s (default: 8.0 rad/s)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Base controller node
    base_controller_node = Node(
        package='omni_bot_kinematics',
        executable='kinematics_node',
        name='omni_bot_kinematics_node',
        output='screen',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'robot_radius': LaunchConfiguration('robot_radius'),
            'max_wheel_velocity': LaunchConfiguration('max_wheel_velocity'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        wheel_radius_arg,
        robot_radius_arg,
        max_wheel_velocity_arg,
        use_sim_time_arg,
        base_controller_node,
    ])
