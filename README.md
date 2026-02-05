# Omni-Bot
*An Extensive Research and Trial on Ground-Based Autonomous Holonomic Driven Robots*

---

## Table of Contents
1. [Overview](#1-overview)
2. [Our Goals](#2-our-goals)
3. [What We Currently Use](#3-what-we-currently-use)
    - [Hardware](#hardware)
    - [Software](#software)
    - [Packages](#packages)
4. [How to Run the Project](#4-how-to-run-the-project)
    - [Installation](#installation)
5. [Problems Currently Being Faced](#5-problems-currently-being-faced)
6. [Bill of Materials Required](#6-bill-of-materials-required)
7. [Future Applications/Features](#7-future-applicationsfeatures)
8. [References](#8-references)

---

## 1. Overview
<!-- Update this section with a brief overview of the project. -->
*Omni-Bot* is a research project focused on developing a ground-based autonomous robot with holonomic drive capabilities. The project aims to explore the potential of *omnidirectional movement* in robotics, leveraging advanced sensors and control algorithms to achieve high levels of autonomy and maneuverability.

## 2. Our Goals
<!-- List the main objectives and milestones for the project. -->
- [x] Develop a custom chassis supporting 3-Wheel Omnidirectional Drive
- [ ] Implement Low Level Control Stack that drives the motors based on the desired velocity
- [ ] Utilize ROS2 for high-level control and navigation
- [ ] Integrate LIDAR and IMU sensors for environment perception
- [ ] Implement SLAM (Simultaneous Localization and Mapping) capabilities
- [ ] Develop a user-friendly interface for teleoperation
- [ ] Conduct extensive testing and validation in various environments

## 3. What We Currently Use
### Hardware
**Chassis:** Unique Aluminum Chassis ([CAD](https://github.com/RockOnJeet/Omni-Bot/blob/main/CAD%20Files/Full%20Assembly.pdf))
  
| Component       | Model/Type                        |
| --------------- | --------------------------------- |
| Motor Driver    | Cytron MD10C DC Motor Driver      |
| Motors          | 3x 24V DC Planetary Motors        |
| Microcontroller | Arduino Nano                      |
| Microcomputer   | Raspberry Pi 4 Model B (8GB RAM)  |
| Power Supply    | 14.8V LiPo Battery Pack (5000mAh) |
| LIDAR           | YDLidar G2                        |
| IMU             | MPU6050                           |

### Software
- **Operating System:** Ubuntu 24.04 LTS
- **ROS Version:** ROS 2 Jazzy Jalisco
- **Simulator:** Gazebo (gz sim) - Modern Gazebo using `ros_gz` packages
- **Visualization Tool:** RViz2

### Packages
- **Bot Description:** [`omni_bot_description`](./omni_bot_description/) - URDF/Xacro models with separate variants for RViz and Gazebo simulation
- **Simulation:** [`omni_bot_sim`](./omni_bot_sim/) - Gazebo (gz sim) integration with `ros_gz_bridge` for topic bridging
- **Hardware Driver:** *In Development* - Server-side `omni_bot_real` and companion computer `omni_controller`

> **Note:** This project has been migrated from ROS 2 Humble + Gazebo Classic to ROS 2 Jazzy + Gazebo (gz sim).

---

## 4. How to Run the Project

### Installation

#### Development Machine (Workstation)

1. **Install ROS 2 Jazzy:**
   ```bash
   # Ubuntu 24.04 LTS required
   # Follow official installation: https://docs.ros.org/en/jazzy/Installation.html
   ```

2. **Install Gazebo and ROS-Gazebo Bridge:**
   ```bash
   sudo apt update
   sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
                    ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                    ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
   ```

3. **Clone the Repository:**
   ```bash
   git clone https://github.com/RockOnJeet/Omni-Bot.git
   cd Omni-Bot
   ```

4. **Build the Workspace:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

### Running the Simulation

1. **Visualize URDF in RViz2 Only:**
   ```bash
   ros2 launch omni_bot_description rviz.launch.py
   ```
   - Opens RViz2 with the robot model
   - Includes joint state publisher GUI for manual joint control

2. **Run Full Gazebo Simulation:**
   ```bash
   ros2 launch omni_bot_sim gazebo.launch.py
   ```
   - Launches Gazebo (gz sim) with the robot
   - Starts RViz2 for visualization
   - Bridges topics between Gazebo and ROS
   - Uses sim_time automatically

3. **Optional: Custom World:**
   ```bash
   ros2 launch omni_bot_sim gazebo.launch.py world:=/path/to/custom.sdf
   ```

#### Hardware (Raspberry Pi) - *In Development*

1. **Install ROS 2 Jazzy** on Raspberry Pi 4 (Ubuntu 24.04)
2. **Install Sensor Drivers:**
   - YDLidar ROS 2 Driver: [GitHub](https://github.com/YDLIDAR/ydlidar_ros2_driver)
   - MPU6050 driver (if applicable)
3. **Upload Arduino Firmware** to the microcontroller
4. **Launch Hardware Interface** (*to be implemented*)
---

## 5. Problems Currently Being Faced
- Need for a more robust and efficient SLAM algorithm.
- Proper calibration and fusion of sensor data (LIDAR and IMU).
- Challenges in real-time control and navigation.
- Limited Power Supply for extended operation.
- Limited testing environments for real-world scenarios.

## 6. Bill of Materials Required
<!-- TODO: Provide a detailed bill of materials. -->
*(This section will be updated with a detailed list of components, including links to purchase them.)*

## 7. Future Applications/Features
<!-- Outline potential future features and applications. -->
- **Enhanced Teleoperation Interface:** Develop an intuitive, user-friendly interface for remote control and monitoring.
- **Modular Design:** Implement a modular architecture for easy upgrades and maintenance.
- **Advanced Autonomous Navigation:** Enhance the robot's ability to navigate complex environments without human intervention.
- **Distributed Multi-Robot Coordination:** Enable communication and coordination among multiple robots for distributed tasks.

## 8. References

### Software Documentation
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html): Official ROS 2 Jazzy installation and usage guide
- [ROS 2 Jazzy Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html): Step-by-step tutorials for ROS 2
- [Gazebo (gz sim) Documentation](https://gazebosim.org/docs): Modern Gazebo simulator documentation
- [ros_gz GitHub](https://github.com/gazebosim/ros_gz): ROS 2 + Gazebo integration packages
- [URDF/Xacro Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html): Robot description format guides

### Hardware Resources
- [YDLidar G2 Official](https://www.ydlidar.com/products/view/1.html): Specifications and datasheet
- [YDLidar ROS 2 Driver](https://github.com/YDLIDAR/ydlidar_ros2_driver): Official ROS 2 driver
- [Arduino Documentation](https://www.arduino.cc/en/Guide/HomePage): Setup and programming guides
- [MPU6050 IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/): Datasheet and usage

### Robotics Concepts
- [Omni-Wheels Overview](https://en.wikipedia.org/wiki/Omni_wheel): Theory and applications
- [Holonomic Drive Systems](https://www.wevolver.com/article/holonomic-robot): Omnidirectional mechanisms
- [SLAM Algorithms](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping): Mapping and localization

### Learning Resources
- [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics): Excellent ROS 2 video tutorials
- [e-Yantra IIT Bombay](https://e-yantra.org/): Robotics projects and resources
- [Robotics Stack Exchange](https://robotics.stackexchange.com/): Community Q&A platform
- [The Construct](https://www.theconstructsim.com/): Online ROS courses and simulations
