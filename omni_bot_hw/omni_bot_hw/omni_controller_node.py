import rclpy
from rclpy.node import Node
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# from tf2_ros import TransformBroadcaster, TransformStamped
# from tf_transformations import quaternion_from_euler as quat_from_euler
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu

import time
import math
import serial
import threading


class OmniControllerNode(Node):
    def __init__(self):
        super().__init__('omni_controller_node')

        # Declare parameters
        self.declare_parameter('wheel_names', [
                               'wheel_joint_0', 'wheel_joint_1', 'wheel_joint_2'])   # Names of the wheels
        # Names of the encoders
        self.declare_parameter(
            'encoder_names', ['encoder_joint_X', 'encoder_joint_Y'])
        # Serial port
        self.declare_parameter('port', '/dev/ttyUSB0')
        # Baudrate
        self.declare_parameter('baudrate', 115200)
        # Refresh rate of the controller
        self.declare_parameter('refresh_rate', 30)

        # Get parameters
        self.wheel_names = self.get_parameter(
            'wheel_names').get_parameter_value().string_array_value
        self.encoder_names = self.get_parameter(
            'encoder_names').get_parameter_value().string_array_value
        self.port = self.get_parameter(
            'port').get_parameter_value().string_value
        self.baudrate = self.get_parameter(
            'baudrate').get_parameter_value().integer_value
        self.refresh_rate = self.get_parameter(
            'refresh_rate').get_parameter_value().integer_value

        self.timer = self.create_timer(
            1.0/self.refresh_rate, self.publish_topics)

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)

        # Initialize Variables
        self.yaw = 0.0
        self.wheel_pwm = [0]*3  # [pwm1, pwm2, pwm3]
        self.enc_ang = [0.0]*2  # [angX, angY]
        self.enc_ang_vel = [0.0]*2  # [ang_velX, ang_velY]

        # Initialize Joint State
        self.joint_msg = JointState()
        self.joint_msg.name = self.encoder_names + self.wheel_names # type: ignore

        # Initialize IMU Message (we'll publish yaw only)
        self.imu_msg = Imu()

        # Initialize Serial Port
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.serial.flush()  # Clear buffer

            # Wait for Arduino to initialize
            timer = time.monotonic()
            while self.serial.read() != b'#':
                if time.monotonic() - timer > 12:
                    self.get_logger().error(
                        f'Failed to initialize serial port {self.port}')
                    self.destroy_node()
                    return
                self.serial.write(b'?')
            self.serial.flush()
            self.serial.write(b'!')  # Acknowledge
            self.get_logger().info(
                f'Serial port {self.port} opened successfully')

            self.serial_thread = threading.Thread(target=self.serial_read)
            self.serial_thread.daemon = True
            self.serial_thread.start()
        except serial.SerialException as e:
            self.get_logger().warn(
                f'Failed to open serial port {self.port}: {e}')
            self.destroy_node()

    def serial_read(self):
        while rclpy.ok():
            try:
                data = self.serial.readline().decode().strip()
                if data.startswith('{') and data.endswith('}'):
                    data = data[1:-1].split('|')
                    if len(data) == 5:  # 4 if yaw is not published
                        self.enc_ang = [float(data[0]), float(data[1])]
                        self.enc_ang_vel = [float(data[2]), float(data[3])]
                        self.yaw = float(data[4])
                    else:
                        self.get_logger().warn(f'Invalid format: {data}')
                elif data == '!':
                    self.get_logger().error('Acknowledge received during Operation!')
                else:
                    self.get_logger().warn(f'Invalid data: {data}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                break

    def publish_topics(self):
        current_time = self.get_clock().now().to_msg()

        # Publish Joint States
        self.joint_msg.header.stamp = current_time
        self.joint_msg.position = self.enc_ang + [0.0]*3
        self.joint_msg.velocity = self.enc_ang_vel + [0.0]*3
        self.joint_pub.publish(self.joint_msg)

        # Publish IMU with yaw-only orientation (roll=pitch=0)
        half_yaw = self.yaw / 2.0
        self.imu_msg.header.stamp = current_time
        self.imu_msg.orientation.x = 0.0
        self.imu_msg.orientation.y = 0.0
        self.imu_msg.orientation.z = math.sin(half_yaw)
        self.imu_msg.orientation.w = math.cos(half_yaw)
        self.imu_pub.publish(self.imu_msg)

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
