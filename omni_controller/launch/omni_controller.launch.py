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
                # Change these values to match your robot's configuration
                'wheel_names': ['front_wheel_joint', 'left_wheel_joint', 'right_wheel_joint'],  # Names of the wheels
                'wheel_radius': 0.065,  # Radius of the wheels (m)
                'robot_radius': 0.4185,  # Radius of the robot (m)
                'encoder_names': ['X_encoder_joint', 'Y_encoder_joint'],  # Names of the encoders
                'encoder_radius': 0.05,  # Radius of Encoder Wheel (m)
                'ticks_per_rev': 400,   # Encoder resolution
                'port': '/dev/ttyACM0',  # Serial port
                'baudrate': 115200,     # Baudrate
                'refresh_rate': 30    # Refresh rate of the controller
            }]
        )
    ])
