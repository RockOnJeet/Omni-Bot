import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription


def generate_launch_description():
    # Get package share directory (Modify if your package name is different)
    path = get_package_share_directory('omni_bot_description')

    # Arguments
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
            path,
            'config',
            'rviz',
            'rviz_view.rviz'
        ),
        description='Path to the RViz configuration file'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    # Launch!
    return LaunchDescription([
        # Arguments
        time_arg,
        path_arg,

        # Nodes
        rviz_node
    ])
