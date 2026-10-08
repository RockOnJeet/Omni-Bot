import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution
)
from launch_ros.actions import Node

from ros_gz_bridge.actions import RosGzBridge

import xacro


def generate_launch_description():
    # Package shares (Modify if your package names are different)
    desc_pkg = get_package_share_directory('omni_bot_description')
    sim_pkg = get_package_share_directory('omni_bot_sim')

    # Env vars (NVIDIA GPU only)
    set_nv_offload = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    set_glx_vendor = SetEnvironmentVariable(
        '__GLX_VENDOR_LIBRARY_NAME', 'nvidia')

    # Arguments
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Optional world file path (.sdf / .world). Empty = default empty world.'
    )

    # Setup the launch file within Description for RViz and Robot State Publisher
    desc_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                desc_pkg,
                'launch',
                'urdf.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': 'true',
            'use_joint_state_publisher_gui': 'false',
            # 'use_rviz_path': os.path.join(
            #     sim_pkg,
            #     'config',
            #     'rviz',
            #     'gz_view.rviz'
            # )
        }.items()
    )

    # Setup Gazebo sim launch (no bots/entities spawned yet)
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r -v4 '), world],
            'on_exit_shutdown': 'true'  # Ensures shutdown of all nodes upon Gazebo exit
        }.items()
    )

    # Spawn robot directly using URDF string (no ROS topic dependency)
    xacro_file = os.path.join(desc_pkg, 'description', 'gz_bot.urdf.xacro')
    robot_description_content = xacro.process_file(
        xacro_file).toxml()  # type: ignore
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_description_content,
                   '-name', 'Omni-Bot'],
        output='screen'
    )

    # ros_gz_bridge for topic bridging between ROS and Gazebo
    # Source: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=PathJoinSubstitution([
            sim_pkg,
            'config',
            'ros_gz_bridge.yaml'
        ]),
        create_own_container=False,  # Don't create container (standalone node)
        use_composition=False,       # Run as standalone node, not composed
        use_respawn=False,          # Don't respawn on crash
        log_level='info'            # Info-level logging for bridge diagnostics
        # Note: extra_bridge_params can be added here if additional runtime params needed
    )

    # Optional: ROS2 Controller launch
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    omni_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'omni_controller'
        ],
        parameters=[os.path.join(
            sim_pkg,
            'config',
            'gz_controller.yaml'
        )],
    )

    # Setup Reset Topic
    reset_node = Node(
        package='omni_bot_sim',
        executable='reset_node',
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        # Arguments
        world_arg,
        set_nv_offload,  # ! Remove this if don't have NVIDIA GPU
        set_glx_vendor,  # ! Remove this if don't have NVIDIA GPU

        # Actions
        desc_file,
        gz_sim_launch,
        spawn_entity,
        ros_gz_bridge,
        joint_state_broadcaster,
        omni_controller,
        reset_node
    ])
