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
    """Complete automation launch: Gazebo + SLAM Toolbox + Nav2.

    Launch sequence:
      1. Mapper (Gazebo + SLAM Toolbox) starts immediately
      2. Navigation stack starts after 20s delay for SLAM initialization

    This ensures SLAM Toolbox publishes map → odom transform before Nav2 activates.
    """

    # Package shares
    nav_pkg = get_package_share_directory('omni_bot_navigation')

    # Launch arguments
    time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )

    params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            nav_pkg,
            'config',
            'mapper_params_online_async.yaml'
        ),
        description='SLAM Toolbox parameters file'
    )

    nav_params_arg = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(
            nav_pkg,
            'config',
            'nav2_params.yaml'
        ),
        description='Nav2 parameters file'
    )

    startup_delay_arg = DeclareLaunchArgument(
        'nav_startup_delay',
        default_value='20.0',
        description='Seconds to wait before launching Nav2 (for SLAM initialization)'
    )

    # Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav_params_file = LaunchConfiguration('nav_params_file')
    nav_startup_delay = LaunchConfiguration('nav_startup_delay')

    # Step 1: Launch mapper (Gazebo + SLAM Toolbox)
    mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav_pkg,
                'launch',
                'mapper.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_params_file': slam_params_file,
        }.items()
    )

    # Step 2: Launch Nav2 after delay for SLAM initialization
    nav_launch = TimerAction(
        period=nav_startup_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        nav_pkg,
                        'launch',
                        'navigation_launch.py'
                    ])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav_params_file,
                }.items()
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        time_arg,
        params_arg,
        nav_params_arg,
        startup_delay_arg,
        # Actions
        mapper_launch,
        nav_launch,
    ])
