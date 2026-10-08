import os
import subprocess

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import WorldControl, WorldReset
from ros_gz_interfaces.srv import ControlWorld

from std_msgs.msg import Bool

import xacro


class GazeboResetNode(Node):
    def __init__(self):
        super().__init__('gazebo_reset_node')

        self.reset_sub = self.create_subscription(
            Bool,
            '/world/reset',
            self.reset_callback,
            10
        )

        self.reset_client = self.create_client(
            ControlWorld,
            '/world/empty/control'
        )

        self.spawn_timer = None

        self.get_logger().info(
            'Monitoring topic "/world/reset" for reset signal...'
        )

    def reset_callback(self, msg):
        if not msg.data:
            return

        self.get_logger().info('Gazebo reset requested.')

        if not self.reset_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error(
                'Gazebo ControlWorld service is unavailable.'
            )
            return

        request = ControlWorld.Request()
        request.world_control = WorldControl(
            reset=WorldReset(all=True)
        )

        future = self.reset_client.call_async(request)
        future.add_done_callback(self.reset_complete_callback)

    def reset_complete_callback(self, future):
        try:
            response = future.result()

            if not response.success:
                self.get_logger().error(
                    'Gazebo reset request failed.'
                )
                return

        except Exception as e:
            self.get_logger().error(
                f'Gazebo reset service call failed: {e}'
            )
            return

        self.get_logger().info(
            'Gazebo reset completed successfully. Waiting 0.1 second(s)...'
        )

        # Delay spawning by exactly 1 second.
        if self.spawn_timer is not None:
            self.spawn_timer.cancel()

        self.spawn_timer = self.create_timer(
            1.0,
            self.spawn_omni_bot,
            callback_group=None
        )

    def spawn_omni_bot(self):
        # One-shot timer.
        if self.spawn_timer is not None:
            self.spawn_timer.cancel()
        self.spawn_timer = None

        self.get_logger().info('Spawning Omni-Bot...')
        # Spawn robot directly using URDF string (no ROS topic dependency)
        desc_pkg = get_package_share_directory('omni_bot_description')
        xacro_file = os.path.join(desc_pkg, 'description', 'gz_bot.urdf.xacro')
        robot_description_content = xacro.process_file(
            xacro_file).toxml()  # type: ignore
        try:
            subprocess.Popen([
                'ros2',
                'run',
                'ros_gz_sim',
                'create',
                '-string',
                robot_description_content,
                '-name',
                'Omni-Bot',
            ])
        except Exception as e:
            self.get_logger().error(
                f'Failed to spawn Omni-Bot: {e}'
            )

        # Joint State Broadcaster Spawner
        try:
            subprocess.Popen([
                'ros2',
                'run',
                'controller_manager',
                'spawner',
                'joint_state_broadcaster',
            ])
        except Exception as e:
            self.get_logger().error(
                f'Failed to spawn joint_state_broadcaster: {e}'
            )

        # Omni Controller Spawner
        sim_pkg = get_package_share_directory('omni_bot_sim')
        try:
            subprocess.Popen([
                'ros2',
                'run',
                'controller_manager',
                'spawner',
                'omni_controller',
                '--param-file',
                os.path.join(sim_pkg, 'config', 'gz_controller.yaml'),
            ])
        except Exception as e:
            self.get_logger().error(
                f'Failed to spawn omni_controller: {e}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = GazeboResetNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
