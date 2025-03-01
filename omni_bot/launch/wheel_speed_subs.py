import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import serial
import serial.tools.list_ports  # For Serial Port Selection
# import tkinter as tk    # For GUI
# from tkinter import ttk    # For Combobox

MCUSerial = None

def isMCUConnected(baudRate): # Console version
  global MCUSerial

  try:
    def confirmBaudrate(BaudRate):
      while MCUSerial.read() != b'?':
        pass
      MCUSerial.write(b'!')
      return True

    ports = list(serial.tools.list_ports.comports())
    port_info = [f"{port.device} ({port.manufacturer})" for port in ports if port.manufacturer is not None]
    # DEBUG
    # port_info = ["COM3 (Arduino)", "COM4 (STM32)", "COM5 (ESP32)"]

    if len(port_info) == 0:
      return False

    [print(f'{i+1}. {port}') for i, port in enumerate(port_info)] # Example: 1. COM3 (Arduino)
    port = input("Enter the port number or type 'q' to quit: ")
    if port == 'q':
      return False

    try:
      port = int(port)
      if port < 1 or port > len(port_info):
        print("Invalid port number")
        return False
      port = port_info[port-1].split("(")[0].strip()  # Get the port name
      MCUSerial = serial.Serial(port, baudrate=baudRate, timeout=0.1)
      if not confirmBaudrate(baudRate):
        MCUSerial.close()
        return False
      # DEBUG
      print(f'Opened {MCUSerial.port}')
      return True
    except ValueError:
      print("Invalid port number")
      return False
  except KeyboardInterrupt:
    return False

# def isMCUConnected(baudRate): # GUI version
#     global MCUSerial

#     ports = list(serial.tools.list_ports.comports())
#     port_info = [f"{port.device} ({port.manufacturer})" for port in ports]
#     # DEBUG
#     # port_info = ["COM3 (Arduino)", "COM4 (STM32)", "COM5 (ESP32)"]

#     if len(port_info) == 0:
#         return False

#     GUI = tk.Tk()
#     GUI.title("Select Serial Port")

#     def onSelect(event):
#         port = menu.get()
#         confirm_button["state"] = "normal" if port else "disabled"

#     def onConfirmation():
#         global MCUSerial

#         port = menu.get().split("(")[0].strip()  # Get the port name
#         MCUSerial = serial.Serial(port, baudrate=baudRate, timeout=0.1)
#         if not confirmBaudrate(baudRate):
#             MCUSerial.close()
#             onClose()
#         # DEBUG
#         print(f'Opened {MCUSerial.port}')
#         GUI.destroy()

#     def onClose():
#         GUI.destroy()
#         sys.exit()

#     def confirmBaudrate(baudRate):
#         while MCUSerial.read() != b'?':
#             pass
#         return True

#     # Close the GUI when the window is closed
#     GUI.protocol("WM_DELETE_WINDOW", onClose)

#     menu = ttk.Combobox(GUI, values=port_info, state="readonly")
#     menu.set("Select a Serial Port")  # Set the default value
#     # Call onSelect when a port is selected
#     menu.bind("<<ComboboxSelected>>", onSelect)
#     menu.pack(padx=10, pady=10)  # Add padding to the menu

#     confirm_button = tk.Button(
#         GUI, text="Confirm", command=onConfirmation, state="disabled")
#     confirm_button.pack(pady=10)

#     GUI.mainloop()

#     return True

class WheelVelocitySubscriber(Node):
  def __init__(self):
    super().__init__('wheel_velocity_subscriber') # Initialize the node
    print("Wheel velocity subscriber node initialized")
    self.subscription = self.create_subscription(
      JointState,
      '/joint_states',
      self.listener_callback,
      100)
    self.wheel_velocities = []

  def listener_callback(self, msg):
    # DEBUG
    # print(f"Received message: {msg}")
    self.wheel_velocities = list(msg.velocity)
    self.wheel_velocities = [int(v / 10 * 215) for v in msg.velocity]
    # print(f"Wheel velocities: {self.wheel_velocities}")
    if not self.sendWheelVelocities():
      self.get_logger().warn("Failed to send wheel velocities")

  def sendWheelVelocities(self):
    global MCUSerial
    if MCUSerial is not None:
      try:
        string = f"[{','.join(map(str, self.wheel_velocities))}]"
        # print(string.encode())
        MCUSerial.write(string.encode())
        MCUSerial.flush() # Flush the output buffer
        current_time = time.time()
        while(MCUSerial.in_waiting == 0 and time.time() - current_time < 0.1):  # Wait for the MCU to respond
          pass
        if MCUSerial.in_waiting > 0:
          if MCUSerial.read() == b'!':
            return True
          else:
            return False
        else:
          return False
      except serial.SerialException:
        return False


def main(args=None):
  # Initialize the serial port
  # arduinoReady = True
  arduinoReady = isMCUConnected(57600)
  if arduinoReady:
    print("Arduino connected")
  else:
    print("No serial ports available")
    return
  rclpy.init(args=args)
  wheel_velocity_subscriber = WheelVelocitySubscriber()
  rclpy.spin(wheel_velocity_subscriber)
  wheel_velocity_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
