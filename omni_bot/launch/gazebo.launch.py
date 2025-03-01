import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, RegisterEventHandler

from launch.event_handlers import OnProcessExit

def generate_launch_description():
  path = get_package_share_directory('omni_bot')

  # Other Launch Files
  urdf_args = {
    'use_sim_time': 'true',
    'use_rviz': 'false',
    'use_joint_state_publisher_gui': 'false'
  }
  urdf_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(path, 'launch', 'urdf.launch.py')),
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

  # 
  
  return LaunchDescription([
    urdf_launch,
    gazebo_world,
    bot_spawn_node,

  ])


