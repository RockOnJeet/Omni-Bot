
import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution
)
from ament_index_python.packages import get_package_share_directory
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    """Gazebo (Ignition / gz sim) launch for omni bot.

    Best-practice updates:
      * Uses ros_gz_sim (gz sim) instead of gazebo_ros (Classic not present).
      * Always uses sim time (gz = sim time true).
      * Minimal arguments: only optional 'world' path. Defaults to empty world.
      * Spawns robot via ros_gz_sim `create` using Xacro-generated URDF directly.
      * No dependency on ROS topics for spawning - uses -file flag with temp URDF.
    """

    # Launch arguments: keep only optional 'world'. Everything else has sensible defaults.
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Optional world file path (.sdf / .world). Empty = default empty world.'
    )

    # Configurations
    world = LaunchConfiguration('world')

    # Package shares
    desc_pkg = get_package_share_directory('omni_bot_description')
    sim_pkg = get_package_share_directory('omni_bot_sim')

    # Path to xacro file (using gz_bot.urdf.xacro for Gazebo simulation)
    xacro_file = os.path.join(desc_pkg, 'description', 'gz_bot.urdf.xacro')

    # Process xacro to generate URDF string
    robot_description_content = xacro.process_file(
        xacro_file).toxml()  # type: ignore

    # Compose gz_args: run immediately (-r), verbosity 3, and optional world path
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r -v3 '), world],
            'on_exit_shutdown': 'true'  # Ensures shutdown of all nodes upon Gazebo exit
        }.items()
    )

    # Spawn robot directly using URDF string (no ROS topic dependency)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_description_content,
                   '-name', 'Omni_Bot'],
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

    # RViz2 visualization
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                desc_pkg,
                'launch',
                'urdf.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_model': 'rviz'
        }.items()
    )

    return LaunchDescription([
        # Arguments
        world_arg,
        # Actions
        gz_sim_launch,
        spawn_entity,
        ros_gz_bridge,
        rviz_node
    ])
