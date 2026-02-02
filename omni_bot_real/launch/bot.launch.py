from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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
            'use_sim_time': 'false',
            # 'robot_model': 'rviz'
            'use_joint_state_publisher_gui': 'false',
            'rviz_config_file': rviz_config_file
        }.items()
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node
    ])
