import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, RegisterEventHandler

from launch.event_handlers import OnProcessExit


def generate_launch_description():
    path = get_package_share_directory('omni_bot')

    # Driver Launcher
    driver_args = {
        'wheel_names': ['front_wheel_joint', 'left_wheel_joint', 'right_wheel_joint'],
        'wheel_radius': 0.076,
        'robot_radius': 0.4185,
        'encoder_names': ['X_encoder_joint', 'Y_encoder_joint'],
        'encoder_radius': 0.05,
        'ticks_per_rev': 400,
        'port': '/dev/ttyUSB0',
        'baudrate': 115200,
        'refresh_rate': 60
    }
    driver_node = Node(
        package='omni_controller',
        executable='omni_controller_node',
        name='omni_controller',
        output='screen',
        parameters=[driver_args]
    )
    
    # Lidar Launcher
    lidar_path = get_package_share_directory('ydlidar_ros2_driver')
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(lidar_path, 'launch', 'ydlidar_launch.py')])
    )
    
    
    # Launch!
    return LaunchDescription([
        driver_node,
        lidar_node
    ])
