from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for Omni Bot kinematics and odometry nodes."""
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    # Log sim time setting
    sim_time_info = LogInfo(
        msg=['NOTE: Sim time set to ', use_sim_time]
    )

    # Kinematics node (cmd_vel -> wheel velocities)
    kine_node = Node(
        package='omni_bot_bringup',
        executable='kinematics_node',
        name='omni_bot_kinematics_node',
        output='screen',
        parameters=[{
            'wheel_radius': 0.076,
            'robot_radius': 0.4185,
            'max_wheel_velocity': 28.0,
            'use_sim_time': use_sim_time
        }]
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='omni_twist_keybd',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        prefix=['gnome-terminal --']
    )

    return LaunchDescription([
        time_arg,
        sim_time_info,
        kine_node,
        teleop_node
    ])
