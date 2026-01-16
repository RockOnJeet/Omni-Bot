#!/usr/bin/env python3
"""
Wheel TF broadcaster for omnidirectional robot.

Subscribes to:
  - /omni_controller/commands (std_msgs/Float64MultiArray) wheel velocities from base_controller

Publishes:
  - TF: base_link → wheel_joint_0, wheel_joint_1, wheel_joint_2

Purpose:
  Integrates commanded wheel velocities to compute joint angles and publish as TF frames.
  This provides visual feedback for RViz without requiring physical encoder feedback.
  Wheels are defined as "continuous" joints in the URDF for ros2_control compatibility,
  but visualization still needs rotating wheel visuals.

Wheel layout (omnidirectional 3-wheel robot):
  - Wheel 0 (front):   120° from x-axis
  - Wheel 1 (left):    240° from x-axis
  - Wheel 2 (right):   0° from x-axis
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math


class WheelTFBroadcaster(Node):
    """Publishes wheel joint transforms from commanded velocities."""

    def __init__(self):
        super().__init__('wheel_tf_broadcaster')

        # Parameters
        self.declare_parameter('wheel_radius', 0.076)  # meters
        self.declare_parameter('update_rate', 60.0)     # Hz
        self.declare_parameter('base_side_length', 1.05)  # meters (from URDF)
        self.declare_parameter('wheel_offset', 0.02)  # meters (from URDF)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.update_rate = self.get_parameter('update_rate').value
        base_side_length = self.get_parameter('base_side_length').value
        self.wheel_offset = self.get_parameter('wheel_offset').value

        # Compute bot_radius from URDF formula: base_side_length / sqrt(3) - 0.215
        self.bot_radius = base_side_length / math.sqrt(3) - 0.215

        # State: joint angles for each wheel (radians)
        self.wheel_angles = [0.0, 0.0, 0.0]
        self.wheel_velocities = [0.0, 0.0, 0.0]  # rad/s

        self.last_time: Optional[Time] = None

        # Subscriber: wheel velocity commands
        self.wheel_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/omni_controller/commands',
            self.wheel_cmd_callback,
            10
        )

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for periodic TF publishing
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_wheel_transforms
        )

        self.get_logger().info(
            f'Wheel TF broadcaster started: wheel_radius={self.wheel_radius}m, '
            f'bot_radius={self.bot_radius:.3f}m, rate={self.update_rate}Hz'
        )

    def get_wheel_position(self, wheel_index: int) -> tuple:
        """
        Compute wheel position relative to base_link from URDF geometry.

        Matches the wheel_preset macro from wheels.xacro:
          angle = i * 2π/3  (0°, 120°, 240°)
          x = (bot_radius + wheel_offset) * cos(angle)
          y = (bot_radius + wheel_offset) * sin(angle)
          z = 0

        Args:
            wheel_index: Wheel number (0=front/120°, 1=left/240°, 2=right/0°)

        Returns:
            tuple: (x, y, z) position in meters relative to base_link
        """
        angle = wheel_index * 2.0 * math.pi / 3.0  # 0°, 120°, 240°
        radius = self.bot_radius + self.wheel_offset

        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = 0.0

        return (x, y, z)

    def wheel_cmd_callback(self, msg: Float64MultiArray):
        """
        Extract wheel velocities from /omni_controller/commands.

        Args:
            msg: Float64MultiArray with data=[front_vel, left_vel, right_vel] in rad/s
        """
        if len(msg.data) >= 3:
            self.wheel_velocities = list(msg.data[:3])

    def publish_wheel_transforms(self):
        """
        Integrate wheel velocities to update joint angles and publish TF.

        Called at fixed rate (update_rate Hz).
        """
        current_time = self.get_clock().now()

        # Compute time delta
        if self.last_time is None:
            dt = 1.0 / self.update_rate
        else:
            dt_rclpy = current_time - self.last_time
            dt = dt_rclpy.nanoseconds / 1e9

        self.last_time = current_time

        # Integrate velocities → angles
        # for i in range(3):
        #     self.wheel_angles[i] += self.wheel_velocities[i] * dt
        #     # Normalize to [0, 2π) to prevent unbounded growth
        #     self.wheel_angles[i] = self.wheel_angles[i] % (2.0 * math.pi)

        # Publish TF frames for each wheel
        # Wheel frame naming: wheel_0, wheel_1, wheel_2 (child links from URDF)
        for i in range(3):
            transform = TransformStamped()

            # Header
            transform.header.stamp = current_time.to_msg()
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = f'wheel_{i}'

            # Position: compute from URDF geometry
            x, y, z = self.get_wheel_position(i)
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z

            # Rotation: wheel orientation from URDF (rpy="0 π/2 angle")
            # angle = i * 2π/3
            wheel_angle = i * 2.0 * math.pi / 3.0

            # Convert RPY(0, π/2, angle) to quaternion
            # R = Rz(angle) * Ry(π/2) * Rx(0)
            cy = math.cos(wheel_angle / 2.0)
            sy = math.sin(wheel_angle / 2.0)
            cp = math.cos(math.pi / 4.0)  # π/2 divided by 2
            sp = math.sin(math.pi / 4.0)

            # Quaternion multiplication: Rz(angle) * Ry(π/2)
            qx = cy * sp
            qy = sy * sp
            qz = sy * cp
            qw = cy * cp

            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            # Broadcast
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Main entry point for the wheel TF broadcaster node."""
    rclpy.init(args=args)
    node = WheelTFBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
