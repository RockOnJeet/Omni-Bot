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

        # Declare parameters
        self.declare_parameter('wheel_names', [
                               'front_wheel_joint', 'left_wheel_joint', 'right_wheel_joint'])   # Names of the wheels
        # Radius of the wheels (m)
        self.declare_parameter('wheel_radius', 0.07)
        # Radius of the robot (m)
        self.declare_parameter('robot_radius', 0.45)
        # Names of the encoders
        self.declare_parameter(
            'encoder_names', ['X_encoder_joint', 'Y_encoder_joint'])
        # Radius of Encoder Wheel (m)
        self.declare_parameter('encoder_radius', 0.05)
        # Encoder resolution
        self.declare_parameter('ticks_per_rev', 400)
        # Serial port
        self.declare_parameter('port', '/dev/ttyACM0')
        # Baudrate
        self.declare_parameter('baudrate', 115200)
        # Refresh rate of the controller
        self.declare_parameter('refresh_rate', 30)

        # Get parameters
        self.wheel_names = self.get_parameter(
            'wheel_names').get_parameter_value().string_array_value
        self.wheel_radius = self.get_parameter(
            'wheel_radius').get_parameter_value().double_value
        self.robot_radius = self.get_parameter(
            'robot_radius').get_parameter_value().double_value
        self.encoder_names = self.get_parameter(
            'encoder_names').get_parameter_value().string_array_value
        self.encoder_radius = self.get_parameter(
            'encoder_radius').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter(
            'ticks_per_rev').get_parameter_value().integer_value
        self.port = self.get_parameter(
            'port').get_parameter_value().string_value
        self.baudrate = self.get_parameter(
            'baudrate').get_parameter_value().integer_value
        self.refresh_rate = self.get_parameter(
            'refresh_rate').get_parameter_value().integer_value

        self.timer = self.create_timer(
            1.0/self.refresh_rate, self.update_odometry)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.vel_callback, 10)

        # Initialize Variables
        self.pose = [0.0]*3  # [x, y, theta]
        self.wheel_pwm = [0]*3  # [pwm1, pwm2, pwm3]
        self.enc_ang = [0.0]*2  # [angX, angY]
        self.enc_ang_vel = [0.0]*2  # [ang_velX, ang_velY]

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
        self.joint_msg.name = self.encoder_names
        self.joint_msg.position = [0.0]*3
        self.joint_msg.velocity = [0.0]*3

        # Initialize Serial Port
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.serial.flush()  # Clear buffer

            # Wait for Arduino to initialize
            timer = time.monotonic()
            while self.serial.read() != b'!':
                if time.monotonic() - timer > 5:
                    self.get_logger().error(
                        f'Failed to initialize serial port {self.port}')
                    self.destroy_node()
                    return
                self.serial.write(b'?')
            self.serial.write(b'!')  # Acknowledge
            self.serial.flush()
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
                    if len(data) == 5:
                        self.enc_ang = [float(data[0]), float(data[1])]
                        self.enc_ang_vel = [float(data[2]), float(data[3])]
                        self.pose[2] = float(data[4])
                    else:
                        self.get_logger().warn('Invalid format: %s', data)
                elif data == '!':
                    self.get_logger().error('Acknowledge received during Operation!')

                else:
                    self.get_logger().warn('Invalid data: %s', data)
            except serial.SerialException as e:
                self.get_logger().error('Serial error: %s', e)
                break

    def vel_callback(self, msg):
        # Convert Twist to Wheel Velocities (Forward Kinematics)
        self.wheel_pwm[0] = (self.robot_radius *
                             msg.angular.z - msg.linear.y * 2.0) / self.wheel_radius
        self.wheel_pwm[1] = (self.robot_radius * msg.angular.z + 0.5 *
                             msg.linear.y * 2.0 + math.sin(math.pi/3) * msg.linear.x * 2.0) / self.wheel_radius
        self.wheel_pwm[2] = (self.robot_radius * msg.angular.z + 0.5 *
                             msg.linear.y * 2.0 - math.sin(math.pi/3) * msg.linear.x * 2.0) / self.wheel_radius

        # Map Recieved Velocities to PWM Range
        for i in range(3):
            # By default, Twist values for linear is 0.5 and angular is 1.0
            # Map [-20, 20] to [-250, 250]
            self.wheel_pwm[i] = int(self.wheel_pwm[i] * 250 / 20)

            # Constrain PWM to [-250, 250]
            self.wheel_pwm[i] = max(-250, min(250, self.wheel_pwm[i]))

        # Send Wheel Velocities to Serial Port
        self.serial.write(
            f'[{self.wheel_pwm[0]}|{self.wheel_pwm[1]}|{self.wheel_pwm[2]}]\n'.encode())
        self.serial.flush()
        self.get_logger().info(f'Wheel PWM: {self.wheel_pwm}')

    def update_odometry(self):
        # Update Odom using Wheel Velocities (Inverse Kinematics)
        timestamp = self.get_clock().now().to_msg()
        dt: float = 1 / self.refresh_rate

        # Parse encoder values
        self.pose[0] += self.enc_ang_vel[0] * self.encoder_radius * dt
        self.pose[1] += self.enc_ang_vel[1] * self.encoder_radius * dt
        self.pose[2] += 0.0  # TODO: IMU sets this

        # DEBUG
        # self._logger.info(f'Pose: {self.pose}')

        # Publish Odom, TF, and Joint States
        self.publish_topics(timestamp)

    def publish_topics(self, current_time):
        # Publish Odom
        self.odom_msg.header.stamp = current_time
        self.odom_msg.pose.pose.position.x = self.pose[0]
        self.odom_msg.pose.pose.position.y = self.pose[1]
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(self.odom_msg)

        # Publish Joint States
        self.joint_msg.header.stamp = current_time
        self.joint_msg.position = self.enc_ang
        self.joint_msg.velocity = self.enc_ang_vel
        self.joint_pub.publish(self.joint_msg)

        # Publish TF
        self.tf_msg.header.stamp = current_time
        self.tf_msg.transform.translation.x = self.pose[0]
        self.tf_msg.transform.translation.y = self.pose[1]
        self.tf_msg.transform.rotation.x = 0.0
        self.tf_msg.transform.rotation.y = 0.0
        self.tf_msg.transform.rotation.z = 0.0
        self.tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(self.tf_msg)

    def destroy_node(self):
        self.serial.close()
        return super().destroy_node()


# Main Function
def main(args=None):
    # Destroy previous nodes if any
    if not rclpy.is_shutdown():
        rclpy.shutdown()

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
