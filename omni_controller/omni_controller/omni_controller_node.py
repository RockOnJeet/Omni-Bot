import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster, TransformStamped
from tf_transformations import quaternion_from_euler as quat_from_euler
from sensor_msgs.msg import JointState

import time
import math
import serial
import threading


class OmniControllerNode(Node):
    def __init__(self):
        super().__init__('omni_controller_node')

        self.declare_parameter('wheel_names', ['wheel1', 'wheel2', 'wheel3'])
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('ticks_per_rev', 400)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_rate', 30)

        # Get parameters
        self.wheel_names = self.get_parameter(
            'wheel_names').get_parameter_value().string_array
        self.wheel_radius = self.get_parameter(
            'wheel_radius').get_parameter_value().double_value
        self.robot_radius = self.get_parameter(
            'robot_radius').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter(
            'ticks_per_rev').get_parameter_value().integer_value
        self.port = self.get_parameter(
            'port').get_parameter_value().string_value
        self.baudrate = self.get_parameter(
            'baudrate').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter(
            'frame_rate').get_parameter_value().integer_value

        self.timer = self.create_timer(
            1.0/self.frame_rate, self.update_odometry)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.vel_callback, 10)

        # Initialize Odom Message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'

        # Initialize TF Message
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = 'odom'
        self.tf_msg.child_frame_id = 'base_link'

        # Initialize Joint State
        self.joint_msg = JointState()
        self.joint_msg.name = self.wheel_names
        self.joint_msg.position = [0]*3
        self.joint_msg.velocity = [0]*3

        # Initialize Serial Port
        self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
        timer = time.monotonic()
        while self.serial.read() != b'!':
            if time.monotonic() - timer > 5:
                self.get_logger().error(f'Failed to initialize serial port {self.port}')
                self.destroy_node()
                return
            self.serial.write(b'?')
        self.serial.write(b'!')
        self.serial.flush()

        self.serial_thread = threading.Thread(target=self.serial_read)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def serial_read(self):
        while rclpy.ok():
            try:
                data = self.serial.readline().decode().strip()
                if data.startswith('{') and data.endswith('}'):
                    data = data[1:-1].split(',')
                    if len(data) == 3:
                        self.joint_msg.position = [int(x) for x in data]
                    else:
                        self.get_logger().warn('Invalid format: %s', data)
                else:
                    self.get_logger().warn('Invalid data: %s', data)
            except serial.SerialException as e:
                self.get_logger().error('Serial error: %s', e)
                break

    def vel_callback(self, msg):
        # TODO: Convert Twist to Wheel Velocities (Forward Kinematics)
        pass

    def update_odometry(self):
        # TODO: Update Odom and TF Messages (Inverse Kinematics)
        pass

    def publish_topics(self):
        # TODO: Publish Odom, TF, and Joint States
        pass

    def destroy_node(self):
        self.serial.close()
        return super().destroy_node()


# Main Function
def main(args=None):
    rclpy.init(args=args)
    node = OmniControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
