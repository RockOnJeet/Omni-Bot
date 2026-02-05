import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition


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

    use_jsp = LaunchConfiguration('use_joint_state_publisher_gui')
    jsp_arg = DeclareLaunchArgument(
        name='use_joint_state_publisher_gui',
        default_value='true',
        description='Whether to start joint_state_publisher_gui'
    )

    # XACRO -> URDF conversion
    bot_xacro = os.path.join(
        path,
        'description',
        'rviz_bot.urdf.xacro'
    )
    bot_urdf = xacro.process_file(bot_xacro).toxml()  # type: ignore

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',

        output='screen',
        parameters=[{'robot_description': bot_urdf,
                     'use_sim_time': use_sim_time}]
    )

    # Joint State Publisher GUI
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_jsp)
    )

    # Launch!
    return LaunchDescription([
        # Arguments
        time_arg,
        jsp_arg,

        # Nodes
        robot_state_publisher_node,
        jsp_gui_node
    ])
