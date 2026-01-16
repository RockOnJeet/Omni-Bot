#!/usr/bin/env python3
"""
Simple test script to verify Phase 1: Kinematics Node
Tests cmd_vel to wheel velocity conversion independently
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time


class KinematicsTest(Node):
    def __init__(self):
        super().__init__('kinematics_test')

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for wheel commands
        self.wheel_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/omni_controller/commands',
            self.wheel_cmd_callback,
            10
        )

        self.received_wheels = None
        self.get_logger().info('Test node initialized')

    def wheel_cmd_callback(self, msg):
        """Store received wheel commands"""
        self.received_wheels = msg.data
        self.get_logger().info(
            f'Received wheels: {[f"{v:.2f}" for v in msg.data]}')

    def test_forward(self):
        """Test forward motion (vx=1.0)"""
        self.get_logger().info('\n=== Test 1: Forward motion (vx=1.0) ===')
        cmd = Twist()
        cmd.linear.x = 1.0
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)

    def test_strafe_right(self):
        """Test strafe right (vy=-1.0)"""
        self.get_logger().info('\n=== Test 2: Strafe right (vy=-1.0) ===')
        cmd = Twist()
        cmd.linear.y = -1.0
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)

    def test_rotation(self):
        """Test rotation (omega=1.0)"""
        self.get_logger().info('\n=== Test 3: Rotation (omega=1.0) ===')
        cmd = Twist()
        cmd.angular.z = 1.0
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)

    def test_combined(self):
        """Test combined motion"""
        self.get_logger().info('\n=== Test 4: Combined (vx=0.5, vy=0.5, omega=0.5) ===')
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.linear.y = 0.5
        cmd.angular.z = 0.5
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)

    def run_tests(self):
        """Run all tests"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Phase 1: Kinematics Node Test')
        self.get_logger().info('='*50)

        time.sleep(1)  # Wait for connections

        self.test_forward()
        self.test_strafe_right()
        self.test_rotation()
        self.test_combined()

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Tests complete!')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsTest()

    # Create executor for spinning
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Spin in background thread
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Run tests
        node.run_tests()
        time.sleep(2)  # Wait for final messages
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
