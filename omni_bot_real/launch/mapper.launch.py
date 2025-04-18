import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    path = get_package_share_directory('omni_bot_real')
    urdf_path = get_package_share_directory('omni_bot_description')

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
            'map_view.rviz'
        ),
        description='Path to the RViz configuration file'
    )
    
    use_params_file = LaunchConfiguration('use_params_file')
    params_arg = DeclareLaunchArgument(
        name='use_params_file',
        default_value=os.path.join(
            path,
            'config',
            'mapper_params_online_async.yaml'
        ),
        description='Path to the parameters file'
    )

    # Other Launch Files
    urdf_args = {
        'use_sim_time': use_sim_time,
        'use_rviz': 'true',
        'use_joint_state_publisher_gui': 'false',
        'rviz_path': use_rviz_path,
    }

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(urdf_path, 'launch', 'urdf.launch.py')),
        launch_arguments=urdf_args.items()
    )

    mapper_args = {
        'use_sim_time': use_sim_time,
        'slam_params_file': use_params_file,
    }
    mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            path, 'launch', 'online_async_launch.py')),
        launch_arguments=mapper_args.items()
    )
    
    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --'
    )
    teleop_launch = TimerAction(
        period=2.0,
        actions=[teleop_twist_keyboard_node]
    )

    # Launch!
    return LaunchDescription([
        # Arguments
        time_arg,
        path_arg,
        params_arg,

        # Other Launch Files
        urdf_launch,
        mapper_launch,
        teleop_launch
    ])
