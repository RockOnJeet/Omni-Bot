#!/usr/bin/env python3
"""Launch odometry node with configurable parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for odometry node."""

    # Declare launch arguments
    encoder_radius_arg = DeclareLaunchArgument(
        'encoder_radius',
        default_value='0.05',
        description='Radius of encoder wheels (meters)'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='60.0',
        description='Odometry update rate (Hz)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Odometry node
    odometry_node = Node(
        package='omni_bot_odometry',
        executable='odometry_node',
        name='omni_bot_odometry_node',
        output='screen',
        parameters=[{
            'encoder_radius': LaunchConfiguration('encoder_radius'),
            'update_rate': LaunchConfiguration('update_rate'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        encoder_radius_arg,
        update_rate_arg,
        use_sim_time_arg,
        odometry_node,
    ])
