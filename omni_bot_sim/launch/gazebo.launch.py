import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, RegisterEventHandler

from launch.event_handlers import OnProcessExit


def generate_launch_description():
    urdf_path = get_package_share_directory('omni_bot_description')
    path = get_package_share_directory('omni_bot_sim')

    # Other Launch Files
    urdf_args = {
        'use_sim_time': 'true',
        'use_rviz': 'false',
        'use_joint_state_publisher_gui': 'false'
    }
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(urdf_path, 'launch', 'urdf.launch.py')),
        launch_arguments=urdf_args.items()
    )

    world_path = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_path)
    )

    # Nodes
    bot_args = [
        '-topic', 'robot_description',
        '-entity', 'omni_bot'
    ]
    bot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=bot_args,
        output='screen'
    )

    # Joint State Broadcaster
    jsb_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )
    jsb_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bot_spawn_node,
            on_exit=[jsb_node]
        )
    )
    
    # Omnidirectional Controller
    omni_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omnidirectional_controller']
    )
    omni_controller_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_node,
            on_exit=[omni_controller_node]
        )
    )

    return LaunchDescription([
        urdf_launch,
        gazebo_world,
        bot_spawn_node,
        # jsb_launch,
        # omni_controller_launch
    ])
