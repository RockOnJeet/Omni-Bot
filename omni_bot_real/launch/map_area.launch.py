import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.conditions import IfCondition


def generate_launch_description():
    urdf_path = get_package_share_directory('omni_bot_description')
    real_path = get_package_share_directory('omni_bot_real')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start rviz'
    )

    # Nodes
    rviz_config = os.path.join(
        real_path,
        'config',
        'rviz',
        'map_view.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',

        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )

    bot_xacro = os.path.join(
        urdf_path,
        'description',
        'omni_bot.urdf.xacro'
    )
    robot_description = xacro.process_file(bot_xacro).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}]
    )

    # Run mapping node
    mapper_params = {
        'use_sim_time': use_sim_time
    }
    mapper_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(real_path, 'launch', 'online_async_launch.py')
      ),
      launch_arguments=mapper_params.items()
    )
    
    # Teleop node
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
        rviz_arg,

        # Nodes
        robot_state_publisher_node,
        rviz_node,
        
        # Launch Files
        mapper_launch,
        teleop_launch
    ])
