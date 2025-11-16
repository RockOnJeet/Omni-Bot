
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
      * Always uses sim time and the 'gz' namespace.
      * Minimal arguments: only optional 'world' path. Defaults to empty world.
      * Spawns robot via ros_gz_sim `create` using Xacro-generated URDF (-string).
      * Keeps robot_state_publisher via included `bot.launch.py` for TF tree.
    """

    # Launch arguments: keep only optional 'world'. Everything else has sensible defaults.
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Optional world file path (.sdf / .world). Empty = default empty world.'
    )

    # Configurations
    world = LaunchConfiguration('world')

    # Package shares
    descr_pkg = get_package_share_directory('omni_bot_description')
    sim_pkg = get_package_share_directory('omni_bot_sim')

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

    # Include robot description launch (robot_state_publisher)
    bot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                descr_pkg,
                'launch',
                'bot.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_model': 'gz'     # Redundant (Default is 'gz')
        }.items()
    )

    # Spawn using ros_gz_sim create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='gz',
        arguments=['-topic', 'robot_description',
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
        create_own_container=False,
        namespace='gz',
        use_composition=False,
        use_respawn=False,
        log_level='info',
        bridge_params=''
    )

    # RViz2 visualization
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                descr_pkg,
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
        # Argument
        world_arg,
        # Actions
        gz_sim_launch,
        bot_launch,
        spawn_entity,
        ros_gz_bridge,
        rviz_node
    ])
