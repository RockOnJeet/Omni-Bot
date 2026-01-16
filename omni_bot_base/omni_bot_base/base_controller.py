#!/usr/bin/env python3
"""
Omni-directional robot base controller.
Converts cmd_vel (Twist) messages to individual wheel velocities for a 3-wheel omni robot.

Kinematics equations:
- front_wheel:  (-R*ω - 1.5*Vy) / r
- left_wheel:   (-R*ω + 0.75*Vy + 1.3*Vx) / r
- right_wheel:  (-R*ω + 0.75*Vy - 1.3*Vx) / r

Where:
- Vx, Vy: Linear velocities (m/s) in robot frame
- ω: Angular velocity (rad/s)
- R: Robot radius (m)
- r: Wheel radius (m)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class OmniBaseController(Node):
    """ROS 2 node for omni-directional robot base control."""

    def __init__(self):
        super().__init__('omni_bot_base_controller')

        # Declare and get parameters
        self.declare_parameter('wheel_radius', 0.076)  # meters
        self.declare_parameter('robot_radius', 0.4185)  # meters
        self.declare_parameter('max_wheel_velocity', 8.0)  # rad/s
        self.declare_parameter('cmd_vel_timeout', 0.5)  # seconds

        self.wheel_radius = self.get_parameter(
            'wheel_radius').get_parameter_value().double_value
        self.robot_radius = self.get_parameter(
            'robot_radius').get_parameter_value().double_value
        self.max_wheel_vel = self.get_parameter(
            'max_wheel_velocity').get_parameter_value().double_value
        self.cmd_vel_timeout = self.get_parameter(
            'cmd_vel_timeout').get_parameter_value().double_value

        # Initialize cmd_vel timeout tracking
        self.last_cmd_vel_time = self.get_clock().now()
        self.is_timed_out = False

        # Log parameters
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'Robot radius: {self.robot_radius} m')
        self.get_logger().info(
            f'Max wheel velocity: ±{self.max_wheel_vel} rad/s')
        self.get_logger().info(
            f'cmd_vel timeout: {self.cmd_vel_timeout} s')

        # Subscriber: /cmd_vel (Twist)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher: /omni_controller/commands (Float64MultiArray)
        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/omni_controller/commands',
            10
        )

        # Timer: Check for cmd_vel timeout (20 Hz)
        self.timeout_check_timer = self.create_timer(
            0.05,  # 20 Hz
            self.check_cmd_vel_timeout
        )

        self.get_logger().info('Omni base controller initialized')
        self.get_logger().info('Subscribed to: /cmd_vel')
        self.get_logger().info('Publishing to: /omni_controller/commands')

    def inverse_kinematics(self, vx, vy, omega):
        """
        Compute wheel velocities from robot velocities.

        Args:
            vx (float): Linear velocity in x-direction (m/s), robot frame
            vy (float): Linear velocity in y-direction (m/s), robot frame
            omega (float): Angular velocity (rad/s)

        Returns:
            tuple: (front_wheel_vel, left_wheel_vel, right_wheel_vel) in rad/s
        """
        R = self.robot_radius
        r = self.wheel_radius

        # Compute wheel velocities (rad/s)
        # Front wheel (120° from x-axis)
        front_vel = (-R * omega - 1.5 * vy) / r

        # Left wheel (240° from x-axis)
        left_vel = (-R * omega + 0.75 * vy + 1.3 * vx) / r

        # Right wheel (0° from x-axis)
        right_vel = (-R * omega + 0.75 * vy - 1.3 * vx) / r

        return front_vel, left_vel, right_vel

    def clamp_velocity(self, velocity):
        """Clamp wheel velocity to max limits."""
        return max(-self.max_wheel_vel, min(self.max_wheel_vel, velocity))

    def check_cmd_vel_timeout(self):
        """
        Check if cmd_vel has timed out and stop the robot if necessary.
        Called periodically by the timeout_check_timer.
        """
        current_time = self.get_clock().now()
        time_since_last_cmd = (
            current_time - self.last_cmd_vel_time).nanoseconds / 1e9

        # If timeout occurred, stop the robot
        if time_since_last_cmd > self.cmd_vel_timeout:
            if not self.is_timed_out:
                # First timeout detection - log warning and stop robot
                self.get_logger().warn(
                    f'cmd_vel timeout ({self.cmd_vel_timeout}s) - stopping robot'
                )
                self.publish_wheel_velocities(0.0, 0.0, 0.0)
                self.is_timed_out = True
        else:
            # Reset timeout flag when receiving commands again
            if self.is_timed_out:
                self.get_logger().info('cmd_vel resumed - timeout cleared')
                self.is_timed_out = False

    def publish_wheel_velocities(self, front_vel, left_vel, right_vel):
        """
        Publish wheel velocities to the controller.

        Args:
            front_vel (float): Front wheel velocity (rad/s)
            left_vel (float): Left wheel velocity (rad/s)
            right_vel (float): Right wheel velocity (rad/s)
        """
        wheel_cmd = Float64MultiArray()
        wheel_cmd.data = [front_vel, left_vel, right_vel]
        self.wheel_cmd_pub.publish(wheel_cmd)

    def cmd_vel_callback(self, msg):
        """
        Process incoming cmd_vel message and publish wheel velocities.

        Args:
            msg (Twist): Velocity command in robot frame
        """
        # Update last cmd_vel timestamp
        self.last_cmd_vel_time = self.get_clock().now()

        # Extract velocities from Twist message
        vx = msg.linear.x  # m/s
        vy = msg.linear.y  # m/s
        omega = msg.angular.z  # rad/s

        # Compute wheel velocities
        front_vel, left_vel, right_vel = self.inverse_kinematics(vx, vy, omega)

        # Apply velocity constraints
        front_vel = self.clamp_velocity(front_vel)
        left_vel = self.clamp_velocity(left_vel)
        right_vel = self.clamp_velocity(right_vel)

        # Publish wheel velocities
        self.publish_wheel_velocities(front_vel, left_vel, right_vel)

        # Log commanded velocities (throttled to avoid spam)
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(omega) > 0.01:
            self.get_logger().debug(
                f'cmd_vel: vx={vx:.2f}, vy={vy:.2f}, ω={omega:.2f} → '
                f'wheels: [{front_vel:.2f}, {left_vel:.2f}, {right_vel:.2f}] rad/s'
            )


def main(args=None):
    """Main entry point for the base controller node."""
    rclpy.init(args=args)
    node = OmniBaseController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
