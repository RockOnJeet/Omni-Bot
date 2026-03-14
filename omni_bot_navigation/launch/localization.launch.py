import os
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch file for the localization node in omni_bot_navigation package."""

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

    localization_mode = LaunchConfiguration('mode')
    localization_arg = DeclareLaunchArgument(
        name='mode',
        default_value='slam',
        description='Switch localization source: "slam" or "amcl"'
    )

    map_file = LaunchConfiguration('map')
    map_arg = DeclareLaunchArgument(
        name='map',
        default_value=os.path.join(
            nav_pkg,
            'maps',
            'rs_v2',
            'rs_v2.yaml'
        ),
        description='Full path to map file to load'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    path_arg = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=os.path.join(
            nav_pkg,
            'config',
            'rviz',
            'map_view.rviz'
        ),
        description='Path to the RViz configuration file'
    )

    sim_time_info = LogInfo(
        msg=['NOTE: Sim time set to ', use_sim_time]
    )

    def _update_map(context, *args, **kwargs):
        map_value = map_file.perform(context)
        map_value = map_value[0:-5]  # Remove .yaml extension for SLAM toolbox
        slam_params_path = os.path.join(
            nav_pkg,
            'config',
            'mapper_params_localization.yaml'
        )
        with open(slam_params_path, "r", encoding="utf-8") as file:
            content = file.read()
        pattern = r"^(\s*map_file_name\s*:\s*)(.+)$"
        match = re.search(pattern, content, flags=re.MULTILINE)
        if not match:
            raise ValueError("map_file_name not found in YAML file")
        updated_content = re.sub(
            pattern,
            lambda m: f"{m.group(1)}{map_value}",
            content,
            count=1,
            flags=re.MULTILINE,
        )
        with open(slam_params_path, "w", encoding="utf-8") as file:
            file.write(updated_content)
        return []

    # SLAM Localization Launch (if localization_mode:=slam)
    slam_params_file = PathJoinSubstitution(
        [nav_pkg, 'config', 'mapper_params_localization.yaml']
    )
    slam_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch',
                        'localization_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params_file
                }.items(),
                condition=IfCondition(PythonExpression(
                    ['"', localization_mode, '" == "slam"']))
            )
        ]
    )

    # AMCL Localization Launch (if localization_mode:=amcl)
    amcl_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        nav2_pkg,
                        'launch',
                        'localization_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_file
                }.items(),
                condition=IfCondition(PythonExpression(
                    ['"', localization_mode, '" == "amcl"']))
            )
        ]
    )

    # RViz Launch (only if use_rviz:=true)
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
        }.items(),
        condition=IfCondition(use_rviz)
    )

    # Launch!
    return LaunchDescription([
        # Args
        time_arg,
        localization_arg,
        map_arg,
        rviz_arg,
        path_arg,
        # Log Info
        sim_time_info,
        # Update map file for SLAM toolbox based on map_arg
        OpaqueFunction(function=_update_map),
        # Launches
        rviz_launch,
        slam_launch,
        amcl_launch
    ])
