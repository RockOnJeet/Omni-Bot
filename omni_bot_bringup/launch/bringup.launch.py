from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Kinematics node (cmd_vel -> wheel velocities)
    kine_node = Node(
        package='omni_bot_kinematics',
        executable='kinematics_node',
        name='omni_bot_kinematics_node',
        output='screen',
        parameters=[{
            'wheel_radius': 0.076,
            'robot_radius': 0.4185,
            'use_sim_time': False
        }]
    )

    # Odometry node (encoder fusion, translation-only)
    odometry_node = Node(
        package='omni_bot_odometry',
        executable='odometry_node',
        name='omni_bot_odometry_node',
        output='screen',
        parameters=[{
            'encoder_radius': 0.05,
            'update_rate': 60.0,
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        kine_node,
        odometry_node
    ])
