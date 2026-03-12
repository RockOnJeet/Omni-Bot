#!/usr/bin/env python3
"""
Encoder-based odometry node for omnidirectional robot.

Subscribes to:
    - /joint_states (sensor_msgs/JointState) from ros2_control
    - /imu (sensor_msgs/Imu) for heading

Publishes:
    - /odom (nav_msgs/Odometry) — robot pose estimate
    - TF: odom → base_link

Fusion approach:
    - X/Y translation from encoders
    - Heading (yaw) from IMU orientation

Source: Phase 2 implementation (IMPLEMENTATION_PROGRESS.md)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros


class OdometryNode(Node):
    """Publishes odometry from encoders and IMU yaw."""

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
        self.theta = 0.0  # Updated from IMU yaw when available

        self.last_time: Optional[Time] = None
        self.encoder_x_vel = 0.0  # rad/s
        self.encoder_y_vel = 0.0  # rad/s
        self.imu_orientation = Quaternion()
        self.imu_yaw = 0.0
        self.imu_angular_z = 0.0
        self.imu_has_orientation = False

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            50
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
        Uses IMU yaw to rotate encoder-derived translation into the world frame.
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
        dy_robot = -self.encoder_y_vel * self.encoder_radius * dt  # Fix axis inversion

        # World frame update using IMU yaw when available
        theta = self.imu_yaw if self.imu_has_orientation else self.theta
        dx_world = math.cos(theta) * dx_robot - math.sin(theta) * dy_robot
        dy_world = math.sin(theta) * dx_robot + math.cos(theta) * dy_robot

        self.x += dx_world
        self.y += dy_world
        self.theta = theta

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation from IMU when available
        if self.imu_has_orientation:
            odom_msg.pose.pose.orientation = self.imu_orientation
        else:
            odom_msg.pose.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=0.0, w=1.0)

        # Velocity (linear only, in robot frame)
        odom_msg.twist.twist.linear.x = dx_robot / dt if dt > 0 else 0.0
        odom_msg.twist.twist.linear.y = dy_robot / dt if dt > 0 else 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = self.imu_angular_z if self.imu_has_orientation else 0.0

        self.odom_pub.publish(odom_msg)

        # Publish TF: odom → base_footprint
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

        # Update timestamp
        self.last_time = current_time

    def imu_callback(self, msg: Imu):
        """Cache latest IMU orientation and yaw for heading integration."""
        if not self._quaternion_valid(msg.orientation):
            self.imu_has_orientation = False
            self.get_logger().warn('Invalid IMU orientation received.')
            return

        self.imu_orientation = msg.orientation
        self.imu_yaw = self._yaw_from_quaternion(msg.orientation)
        self.imu_angular_z = msg.angular_velocity.z
        self.imu_has_orientation = True

    @staticmethod
    def _yaw_from_quaternion(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_valid(q: Quaternion) -> bool:
        return not (q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0)


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
