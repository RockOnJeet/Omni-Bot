import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class OmniControllerNode(Node):
    def __init__(self):
        super().__init__('omni_controller')

        # Radius of the wheels (m)
        self.declare_parameter('wheel_radius', 0.07)
        self.wheel_radius = self.get_parameter(
            'wheel_radius').get_parameter_value().double_value

        # Radius of the robot (m)
        self.declare_parameter('robot_radius', 0.45)
        self.robot_radius = self.get_parameter(
            'robot_radius').get_parameter_value().double_value

        self.wheel_pwm = [0]*3  # Initialize wheel PWM values
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.vel_callback, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, '/omnidirectional_controller/commands', 10)

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

            # Constrain PWM to [-20, 20]
            self.wheel_pwm[i] = max(-8, min(8, self.wheel_pwm[i]))
        
        # self.get_logger().info(f'Wheel PWM: {self.wheel_pwm}')
        
        # Send Wheel Velocities to Controller
        msg = Float64MultiArray()
        msg.data.append(self.wheel_pwm[0])
        msg.data.append(self.wheel_pwm[1])
        msg.data.append(self.wheel_pwm[2])
        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = OmniControllerNode()
    node.get_logger().info('Mapping Twist to Velocities...')
    # Spin the node to keep it active
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
