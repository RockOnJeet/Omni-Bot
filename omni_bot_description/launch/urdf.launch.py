import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition


def generate_launch_description():
    """
    Launch file for visualizing the robot URDF in RViz2 with joint control.

    This launch file starts:
    - bot.launch.py: Includes robot_state_publisher (from existing launch file)
    - joint_state_publisher_gui: GUI for manually controlling joint positions
    - rviz2: Visualization tool with custom configuration
    """

    # Declare launch argument for use_sim_time
    # Set to 'false' by default for manual testing with joint_state_publisher_gui
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true (Default: false)'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='gz',
        description='Set the robot model (gz[Default] or rviz)'
    )

    # Get the package share directory
    pkg_share = get_package_share_directory('omni_bot_description')

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')

    # Path to the custom RViz configuration file
    rviz_config_file = PathJoinSubstitution([
        pkg_share, 'config', 'rviz', [robot_model, '_view.rviz']
    ])

    # Include the bot.launch.py file which handles robot_state_publisher
    # This reuses existing launch logic instead of duplicating code
    bot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bot.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'robot_model': robot_model}.items()
    )

    # Joint State Publisher GUI node
    # Provides sliders to manually control joint positions for testing
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        namespace=[robot_model],
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    # RViz2 node with custom configuration
    # Visualizes the robot model and TF tree
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        namespace=[robot_model],
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_model_arg,
        bot_launch,
        joint_state_publisher_gui_node,
        rviz_node
    ])
