import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch file for the navigation (with localization) node in omni_bot_navigation package."""

    # Package share directory
    nav_pkg = get_package_share_directory('omni_bot_navigation')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    localization_mode = LaunchConfiguration('localization_mode')
    localization_arg = DeclareLaunchArgument(
        name='localization_mode',
        default_value='slam',
        description='Switch localization source: "slam" or "amcl"'
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    path_arg = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=os.path.join(
            nav_pkg,
            'config',
            'rviz',
            'nav2_view.rviz'
        ),
        description='Path to the RViz configuration file'
    )

    # RViz Launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omni_bot_description'),
                'launch',
                'rviz.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config_file': rviz_config_file
        }.items()
    )

    # Navigation Launch
    nav2_params = os.path.join(
        nav_pkg, 'config', 'nav2_params.yaml'
    )
    map_subscribe_transient_local = PythonExpression(
        ['"true" if "', localization_mode, '" == "amcl" else "false"']
    )
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_pkg,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'map_subscribe_transient_local': map_subscribe_transient_local
        }.items()
    )

    # Log sim time setting
    sim_time_info = LogInfo(
        msg=['NOTE: Sim time set to ', use_sim_time]
    )

    # Log robot model setting
    localization_info = LogInfo(
        msg=['NOTE: Localization source set to ', localization_mode]
    )

    # Launch!
    return LaunchDescription([
        # Args
        time_arg,
        path_arg,
        localization_arg,
        # Log Info
        sim_time_info,
        localization_info,
        # Launches
        rviz_launch,
        # Navigation Launch (delayed by 10 seconds)
        TimerAction(
            period=10.0,
            actions=[nav_launch]
        )
    ])
