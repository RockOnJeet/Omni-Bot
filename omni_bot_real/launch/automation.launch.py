import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess


def generate_launch_description():
    path = get_package_share_directory('omni_bot_real')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    use_rviz_path = LaunchConfiguration('use_rviz_path')
    path_arg = DeclareLaunchArgument(
        name='use_rviz_path',
        default_value=os.path.join(
            path,
            'config',
            'rviz',
            'auto_view.rviz'
        ),
        description='Path to the RViz configuration file'
    )

    # Other Launch Files
    mapper_args = {
        'use_sim_time': use_sim_time,
        'use_rviz_path': use_rviz_path,
        'use_params_file': os.path.join(
            path,
            'config',
            'mapper_params_localization.yaml'
        )
    }

    mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            path, 'launch', 'mapper.launch.py')),
        launch_arguments=mapper_args.items()
    )

    nav_args = {
        'use_sim_time': use_sim_time,
        'params_file': os.path.join(
            path,
            'config',
            'nav2_params.yaml'
        )
    }

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            path, 'launch', 'navigation_launch.py')),
        launch_arguments=nav_args.items()
    )

    # Launch!
    return LaunchDescription([
        # Arguments
        time_arg,
        path_arg,

        # Other Launch Files
        mapper_launch,
        nav_launch
    ])
