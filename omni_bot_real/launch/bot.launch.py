from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    # Package shares
    desc_pkg = get_package_share_directory('omni_bot_description')

    # RViz2 visualization
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            desc_pkg,
            'config',
            'rviz',
            'gz_view.rviz'
        ])
    )
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                desc_pkg,
                'launch',
                'urdf.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 'robot_model': 'rviz'
            'use_joint_state_publisher_gui': 'false',
            'rviz_config_file': rviz_config_file
        }.items()
    )

    # Kinematics node (cmd_vel -> wheel velocities)
    base_controller_node = Node(
        package='omni_bot_base',
        executable='base_controller',
        name='omni_bot_base_controller',
        output='screen',
        parameters=[{
            'wheel_radius': 0.076,
            'robot_radius': 0.4185,
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
            'update_rate': 30.0,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        time_arg,
        rviz_config_arg,
        rviz_node,
        base_controller_node,
        odometry_node,
    ])
