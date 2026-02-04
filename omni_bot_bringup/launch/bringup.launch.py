from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


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
        package='omni_bot_kinematics',
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

    # Odometry node (encoder fusion, translation-only)
    odometry_node = Node(
        package='omni_bot_odometry',
        executable='odometry_node',
        name='omni_bot_odometry_node',
        output='screen',
        parameters=[{
            'encoder_radius': 0.05,
            'update_rate': 60.0,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        time_arg,
        sim_time_info,
        kine_node,
        odometry_node
    ])
