import os
import xacro
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    IfElseSubstitution,
)
from ament_index_python.packages import get_package_share_directory
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    """Launch file for Gazebo simulation of Omni Bot with ROS-Gazebo bridge and RViz2."""

    # Package shares
    desc_pkg = get_package_share_directory('omni_bot_description')
    sim_pkg = get_package_share_directory('omni_bot_sim')

    # Launch arguments: keep only optional 'world'. Everything else has sensible defaults.
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Optional world file path (.sdf / .world). Empty = default empty world.'
    )

    gui = LaunchConfiguration('gui')
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable Gazebo GUI. Set to false for headless server-only mode.'
    )

    # Gz Harmonic Launch (World only)
    gz_args = [
        TextSubstitution(text='-r -v4 '),
        IfElseSubstitution(gui, '', TextSubstitution(text='-s ')),
        world
    ]
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': gz_args,
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
                   '-name', 'Omni_Bot'],
        output='screen'
    )

    # ros_gz_bridge for topic bridging between ROS and Gazebo
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

    # Launch robot_state_publisher for ROS2_Control
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                desc_pkg,
                'launch',
                'bot.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_joint_state_publisher_gui': 'false'
        }.items()
    )

    # ros2_control spawners
    omni_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_controller'],
        ros_arguments=['--remap', 'use_sim_time:=true'],
        output='screen'
    )

    # Static TF replacing URDF LiDAR frame (parity)
    tf2_lidar_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf2_lidar_broadcaster',
        # Origin matches URDF: parent=base_footprint, xyz="0 0 0.25"
        # Source: src/omni_bot_description/description/gz_bot.urdf.xacro line 60
        arguments=['0', '0', '0.25', '0', '0',
                   '0', 'base_footprint', 'lidar_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        world_arg,
        gui_arg,
        # Actions
        tf2_lidar_broadcaster,
        gz_sim_launch,
        spawn_entity,
        ros_gz_bridge,
        rsp_launch,
        # ros2_control spawners (after robot spawned)
        omni_controller_spawner
    ])
