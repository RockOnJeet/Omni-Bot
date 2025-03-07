import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import threading
from time import sleep as delay

class HardwareControllerNode(Node):
  def __init__(self):
    super().__init__('hardware_controller_node')
    
    self.declare_parameter('port', '/dev/ttyUSB0')
    self.declare_parameter('baudrate', 115200)
    self.declare_parameter('frame_rate', 30)
    
    # Get parameters
    self.port = self.get_parameter('port').get_parameter_value().string_value
    self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
    self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
    
    self.timer = self.create_timer(1.0/self.frame_rate, self.update_hardware)
    
    self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
    
    # Initialize Joint State
    self.joint_msg = JointState()
    self.joint_msg.name = ['joint1', 'joint2', 'joint3']
    self.joint_msg.position = [0]*3
    self.joint_msg.velocity = [0]*3
    
    # Initialize Serial Port
    self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
    while self.serial.read() != b'!':
      self.serial.write(b'?')
  
  def update_hardware(self):
    # TODO
    pass

def main(args=None):
  rclpy.init(args=args)
  node = HardwareControllerNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
