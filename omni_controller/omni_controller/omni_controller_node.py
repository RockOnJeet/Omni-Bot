import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster, TransformStamped
from tf_transformations import quaternion_from_euler as quat_from_euler, euler_from_quaternion as euler_from_quat
from sensor_msgs.msg import JointState, Imu, LaserScan
from std_msgs.msg import Float64MultiArray

import time
import math


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
        self.refresh_rate = self.get_parameter(
            'refresh_rate').get_parameter_value().integer_value

        self.timer = self.create_timer(
            1.0/self.refresh_rate, self.update_odometry)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.wheel_speed_pub = self.create_publisher(
            Float64MultiArray, '/sim/omnidirectional_controller/commands', 10)
        self.lidar_pub = self.create_publisher(
            LaserScan, '/scan', 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/sim/joint_states', self.joint_read, 10)
        self.vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.vel_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/gazebo_ros_lidar/out', self.lidar_callback, 10)

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
        
        # Initialize Lidar Message
        self.lidar_msg = LaserScan()

    # def serial_read(self):
    #     while rclpy.ok():
    #         try:
    #             data = self.serial.readline().decode().strip()
    #             if data.startswith('{') and data.endswith('}'):
    #                 data = data[1:-1].split('|')
    #                 if len(data) == 5:  # 4 if yaw is not published
    #                     self.enc_ang = [float(data[0]), float(data[1])]
    #                     self.enc_ang_vel = [float(data[2]), float(data[3])]
    #                     self.pose[2] = float(data[4])
    #                 else:
    #                     self.get_logger().warn(f'Invalid format: {data}')
    #             elif data == '!':
    #                 self.get_logger().error('Acknowledge received during Operation!')
    #             else:
    #                 self.get_logger().warn(f'Invalid data: {data}')
    #         except serial.SerialException as e:
    #             self.get_logger().error(f'Serial error: {e}')
    #             break
    def joint_read(self, msg):
        self.enc_ang = msg.position[:2]
        self.enc_ang_vel = msg.velocity[:2]
        # self.get_logger().info(f'Enc Ang: {self.enc_ang}, Enc Ang Vel: {self.enc_ang_vel}')
        
    def imu_callback(self, msg: Imu):
        # Update the pose with the IMU data
        self.pose[2] = euler_from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2]
        # self.get_logger().info(f'IMU Pose: {self.pose[2]}')
    
    def lidar_callback(self, msg: LaserScan):
        # Update the Lidar message with the received data
        self.lidar_msg = msg

    def vel_callback(self, msg):
        # Convert Twist to Wheel Velocities (Forward Kinematics)
        self.wheel_pwm[0] = (-self.robot_radius *
                             msg.angular.z - 1.5 * msg.linear.y) / self.wheel_radius
        self.wheel_pwm[1] = (-self.robot_radius * msg.angular.z + 0.5 *
                             1.5 * msg.linear.y + math.sin(math.pi/3) * 1.5 * msg.linear.x) / self.wheel_radius
        self.wheel_pwm[2] = (-self.robot_radius * msg.angular.z + 0.5 *
                             1.5 * msg.linear.y - math.sin(math.pi/3) * 1.5 * msg.linear.x) / self.wheel_radius

        # Map Recieved Velocities to PWM Range
        for i in range(3):
            # By default, Twist values for linear is 0.5 and angular is 1.0
            # Map [-20, 20] to [-250, 250]
            # self.wheel_pwm[i] = int(self.wheel_pwm[i] * 250 / 20)

            # Constrain PWM to [-8, 8]
            self.wheel_pwm[i] = max(-8, min(8, self.wheel_pwm[i]))

        # Send Wheel Velocities to Serial Port
        # self.serial.write(
        #     f'[{self.wheel_pwm[0]}|{self.wheel_pwm[1]}|{self.wheel_pwm[2]}]'.encode())
        # self.serial.flush()
        # self.get_logger().info(f'Wheel PWM: {self.wheel_pwm}')
        
        # Publish Wheel Velocities
        msg = Float64MultiArray()
        msg.data.append(self.wheel_pwm[0])
        msg.data.append(self.wheel_pwm[1])
        msg.data.append(self.wheel_pwm[2])
        self.wheel_speed_pub.publish(msg)

    def update_odometry(self):
        # Update Odom using Wheel Velocities (Inverse Kinematics)
        timestamp = self.get_clock().now().to_msg()
        dt: float = 1 / self.refresh_rate

        # Parse encoder values
        self.pose[0] += (self.enc_ang_vel[1] * math.cos(self.pose[2]) +
                         self.enc_ang_vel[0] * math.sin(self.pose[2])) * self.encoder_radius * dt
        self.pose[1] -= (self.enc_ang_vel[0] * math.cos(self.pose[2]) -
                         self.enc_ang_vel[1] * math.sin(self.pose[2])) * self.encoder_radius * dt
        self.pose[2] += 0.0  # TODO: Gyro sets this

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
        self.odom_msg.pose.pose.orientation.z = quat_from_euler(
            0.0, 0.0, self.pose[2])[2]
        self.odom_msg.pose.pose.orientation.w = quat_from_euler(
            0.0, 0.0, self.pose[2])[3]
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
        self.tf_msg.transform.rotation.z = quat_from_euler(
            0.0, 0.0, self.pose[2])[2]
        self.tf_msg.transform.rotation.w = quat_from_euler(
            0.0, 0.0, self.pose[2])[3]
        self.tf_broadcaster.sendTransform(self.tf_msg)
        
        # Publish Lidar Data
        self.lidar_msg.header.stamp = current_time
        self.lidar_pub.publish(self.lidar_msg)

    def destroy_node(self):
        # self.serial.close()
        return super().destroy_node()


# Main Function
def main(args=None):
    rclpy.init(args=args)
    node = OmniControllerNode()
    node.get_logger().info('Omni Controller Node Started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
