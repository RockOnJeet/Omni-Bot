#!/usr/bin/env python3
"""
Encoder-based odometry node for omnidirectional robot.

Subscribes to:
  - /joint_states (sensor_msgs/JointState) from ros2_control

Publishes:
  - /odom (nav_msgs/Odometry) — robot pose estimate
  - TF: odom → base_link

Simplified version: Translation-only (no rotation from IMU yet).
Uses X/Y encoder velocities to estimate position in world frame.

Source: Phase 2 implementation (IMPLEMENTATION_PROGRESS.md)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros


class OdometryNode(Node):
    """Publishes odometry from encoder data (translation-only)."""

    def __init__(self):
        super().__init__('omni_bot_odometry_node')

        # Parameters
        self.declare_parameter('encoder_radius', 0.05)  # meters
        self.declare_parameter('update_rate', 60.0)     # Hz
        # Note: use_sim_time is automatically declared by ROS 2, don't redeclare it

        self.encoder_radius = self.get_parameter('encoder_radius').value
        self.update_rate = self.get_parameter('update_rate').value

        # State variables
        self.x = 0.0  # Position in world frame (meters)
        self.y = 0.0
        self.theta = 0.0  # Fixed at 0 (no rotation yet)

        self.last_time: Optional[Time] = None
        self.encoder_x_vel = 0.0  # rad/s
        self.encoder_y_vel = 0.0  # rad/s

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for odometry updates
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_odometry
        )

        self.get_logger().info(
            f'Odometry node started: encoder_radius={self.encoder_radius}m, '
            f'rate={self.update_rate}Hz'
        )

    def joint_state_callback(self, msg: JointState):
        """
        Extract encoder velocities from /joint_states.

        Expected joint names from ros2_control:
          - encoder_joint_X
          - encoder_joint_Y
        """
        try:
            # Find encoder joint indices
            x_idx = msg.name.index('encoder_joint_X')
            y_idx = msg.name.index('encoder_joint_Y')

            # Extract velocities (rad/s)
            if len(msg.velocity) > max(x_idx, y_idx):
                self.encoder_x_vel = msg.velocity[x_idx]
                self.encoder_y_vel = msg.velocity[y_idx]
        except (ValueError, IndexError) as e:
            # Joint not found or velocity array too short
            self.get_logger().warn(
                f'Encoder joints not found in /joint_states: {e}',
                throttle_duration_sec=5.0
            )

    def publish_odometry(self):
        """
        Integrate encoder velocities to estimate position and publish odometry.

        Simplified version:
          - Assumes theta = 0 (no rotation)
          - Robot frame motion: dx_robot = encoder_X_vel * r * dt
                                dy_robot = encoder_Y_vel * r * dt
          - World frame: X += dx_robot, Y += dy_robot (since theta=0)
        """
        current_time = self.get_clock().now()

        # Initialize timestamp on first call
        if self.last_time is None:
            self.last_time = current_time
            return

        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return  # Avoid division by zero or negative time

        # Robot frame displacement from encoders (meters)
        dx_robot = self.encoder_x_vel * self.encoder_radius * dt
        dy_robot = -self.encoder_y_vel * self.encoder_radius * \
            dt  # Negated to fix axis inversion

        # World frame update (simplified: theta=0, so robot frame = world frame)
        # TODO (Phase 4): Add rotation from IMU
        self.x += dx_robot
        self.y += dy_robot
        # self.theta remains 0

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (identity quaternion: no rotation)
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Velocity (linear only, in robot frame)
        odom_msg.twist.twist.linear.x = dx_robot / dt if dt > 0 else 0.0
        odom_msg.twist.twist.linear.y = dy_robot / dt if dt > 0 else 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = 0.0  # No rotation yet

        self.odom_pub.publish(odom_msg)

        # Publish TF: odom → base_footprint
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.tf_broadcaster.sendTransform(transform)

        # Update timestamp
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
