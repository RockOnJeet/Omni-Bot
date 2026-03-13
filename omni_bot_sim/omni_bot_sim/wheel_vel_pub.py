import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray

from .wheel_ff import omega_to_pwm, WHEEL_DB

# These are your measured per-wheel slopes from calibration
WHEEL_SLOPE = {
    1: 0.099376,   # front
    2: 0.130052,   # rear_left  ← faster motor, gets proportionally more torque
    3: 0.104018,   # rear_right
}

# Must match <damping> value in your SDF joint dynamics
# This is the only tuning knob — set it once in SDF and mirror here
SDF_JOINT_DAMPING = 0.45   # Nm·s/rad — tune until spin-up matches real


def pwm_to_effort(wheel_id: int, pwm: int) -> float:
    db = WHEEL_DB[wheel_id]
    if abs(pwm) < db:
        return 0.0
    # torque that produces same steady-state omega as real motor at this PWM
    return SDF_JOINT_DAMPING * WHEEL_SLOPE[wheel_id] * pwm


class WheelVelPublisher(Node):
    def __init__(self):
        super().__init__('wheel_vel_publisher')
        self.subscriber_ = self.create_subscription(
            Float64MultiArray,
            'omni_controller/commands',
            self.wheel_vel_callback,
            10
        )
        self.publishers_ = []
        for i in range(3):
            pub = self.create_publisher(
                Float64, f'omni_controller/commands/wheel_{i}', 10)
            self.publishers_.append(pub)
        self.get_logger().info('WheelVelPublisher node has been started.')

    def wheel_vel_callback(self, msg):
        for i in range(3):
            wheel_id = i + 1
            pwm = omega_to_pwm(wheel_id, msg.data[i])
            effort = pwm_to_effort(wheel_id, pwm)

            wheel_vel_msg = Float64()
            wheel_vel_msg.data = effort
            self.publishers_[i].publish(wheel_vel_msg)
            self.get_logger().debug(
                f'wheel_{i}: omega={msg.data[i]:.3f} → pwm={pwm:+4d} → effort={effort:.4f} Nm')


def main(args=None):
    rclpy.init(args=args)
    wheel_vel_publisher = WheelVelPublisher()
    rclpy.spin(wheel_vel_publisher)
    wheel_vel_publisher.destroy_node()
    rclpy.shutdown()
