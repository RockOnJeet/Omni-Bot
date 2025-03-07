from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omni_controller',
            executable='omni_controller_node',
            name='omni_controller',
            output='screen',
            parameters=[{
                # Change these values to match your robot
                'wheel_names': ['wheel1', 'wheel2', 'wheel3'],
                'wheel_radius': 0.03,
                'robot_radius': 0.2,
                'ticks_per_rev': 400,
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'frame_rate': 30
            }]
        )
    ])
