import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch file for the mapper node in omni_bot_navigation package."""

    # Package share directory
    nav_pkg = get_package_share_directory('omni_bot_navigation')
    slam_pkg = get_package_share_directory('slam_toolbox')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
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

    # Log sim time setting
    sim_time_info = LogInfo(
        msg=['NOTE: Sim time set to ', use_sim_time]
    )

    # Online Async Mapper Launch
    slam_params_file = PathJoinSubstitution(
        [nav_pkg, 'config', 'mapper_params_online_async.yaml']
    )
    mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_pkg,
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # Gazebo Launch (if use_sim_time:=true)
    custom_world = os.path.join(
        get_package_share_directory('omni_bot_sim'),
        'worlds',
        'custom.sdf'
    )
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omni_bot_sim'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': custom_world,
            'gui': 'false',
            'rviz_config_file': rviz_config_file
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    # HW Launch (if use_sim_time:=false)
    hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omni_bot_description'),
                'launch',
                'urdf.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_joint_state_publisher_gui': 'false',
            'rviz_config_file': rviz_config_file
        }.items(),
        condition=UnlessCondition(use_sim_time)
    )

    # Launch!
    return LaunchDescription([
        # Args
        time_arg,
        path_arg,
        # Log Info
        sim_time_info,
        # Launches
        gazebo_launch,
        hw_launch,
        # Launch SLAM at end
        mapper_launch
    ])
