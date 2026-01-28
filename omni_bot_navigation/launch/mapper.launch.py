import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Gazebo (Ignition / gz sim) launch for omni bot.

    Best-practice updates:
      * Uses ros_gz_sim (gz sim) instead of gazebo_ros (Classic not present).
      * Always uses sim time (gz = sim time true).
      * Minimal arguments: only optional 'world' path. Defaults to empty world.
      * Spawns robot via ros_gz_sim `create` using Xacro-generated URDF directly.
      * No dependency on ROS topics for spawning - uses -file flag with temp URDF.
    """

    # Package shares
    desc_pkg = get_package_share_directory('omni_bot_description')
    sim_pkg = get_package_share_directory('omni_bot_sim')
    nav_pkg = get_package_share_directory('omni_bot_navigation')

    # Launch arguments: keep only optional 'world'. Everything else has sensible defaults.
    world_arg = DeclareLaunchArgument(
        'world', default_value=PathJoinSubstitution([
            sim_pkg, 'worlds', 'custom.sdf'
        ]),
        description='Optional world file path (.sdf / .world). Empty = default custom world.'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            desc_pkg,
            'config',
            'rviz',
            'map_view.rviz'
        ])
    )

    time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )

    params_arg = DeclareLaunchArgument(
        name='use_params_file',
        default_value=os.path.join(
            nav_pkg,
            'config',
            'mapper_params_online_async.yaml'
        ),
        description='Path to the parameters file'
    )

    # Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_params_file = LaunchConfiguration('use_params_file')

    # GZ visualization (only launch if use_sim_time is true)
    gz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                sim_pkg,
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': world_file,
            'rviz_config': rviz_config_file,
            'gui': 'false'
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    # SLAM node - delayed to start after Gazebo is ready
    slam_node = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to fully initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        get_package_share_directory('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    ])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': use_params_file,
                }.items()
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        time_arg,
        world_arg,
        rviz_config_arg,
        params_arg,
        # Actions
        gz_node,
        slam_node,
    ])
