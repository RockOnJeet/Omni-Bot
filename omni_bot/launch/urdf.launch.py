import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

from launch.conditions import IfCondition

def generate_launch_description():
  path = get_package_share_directory('omni_bot')

  # Arguments
  use_sim_time = LaunchConfiguration('use_sim_time')
  time_arg = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation/Gazebo clock'
  )

  use_rviz = LaunchConfiguration('use_rviz')
  rviz_arg = DeclareLaunchArgument(
    name='use_rviz',
    default_value='true',
    description='Whether to start rviz'
  )

  use_jsp = LaunchConfiguration('use_joint_state_publisher_gui')
  jsp_arg = DeclareLaunchArgument(
    name='use_joint_state_publisher_gui',
    default_value='true',
    description='Whether to start joint_state_publisher_gui'
  )

  # Nodes
  rviz_config = os.path.join(
    path,
    'config',
    'rviz',
    'urdf_view.rviz'
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
    path, 
    'description',
    'omni_bot.urdf.xacro'
  )
  robot_description = xacro.process_file(bot_xacro).toxml()

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
  )

  jsp_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher',
    output='screen',
    condition=IfCondition(use_jsp)
  )

  # Launch!
  return LaunchDescription([
    # Arguments
    time_arg,
    rviz_arg,
    jsp_arg,

    # Nodes
    robot_state_publisher_node,
    rviz_node,
    jsp_gui_node
  ])

