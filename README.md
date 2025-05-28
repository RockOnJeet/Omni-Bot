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
- [x] Implement Low Level Control Stack that drives the motors based on the desired velocity
- [x] Utilize ROS2 for high-level control and navigation
- [x] Integrate LIDAR and IMU sensors for environment perception
- [x] Implement SLAM (Simultaneous Localization and Mapping) capabilities
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
- **Operating System:** Ubuntu 22.04 LTS
- **ROS Version:** ROS2 Humble
- **Simulator:** Gazebo 11 (Classic)
- **Visualization Tool:** RViz2

### Packages
- **Bot Description:** Custom [`omni_bot_description`](https://github.com/RockOnJeet/Omni-Bot/tree/ROS2-Humble/omni_bot_description)
- **Digital Twin:** [`omni_bot_sim`](https://github.com/RockOnJeet/Omni-Bot/tree/ROS2-Humble/omni_bot_sim) with custom [`omnidirectional_controllers`](https://github.com/RockOnJeet/Omni-Bot/tree/ROS2-Humble/omnidirectional_controllers)
- **Hardware Driver:** Server-side [`omni_bot_real`](https://github.com/RockOnJeet/Omni-Bot/tree/ROS2-Humble/omni_bot_real) and companion computer [`omni_controller`](https://github.com/RockOnJeet/Omni-Bot/tree/Raspberry-Pi/omni_controller)
- **Full Hardware Control (Outdated):** Arduino-only [`Omni_Bot V3.0`](https://github.com/RockOnJeet/Omni-Bot/tree/v3.0.0-alpha)

---

## 4. How to Run the Project

### Installation
On the server side, install the following packages:
- **ROS2 Humble:** Follow the instructions [here](https://docs.ros.org/en/humble/Installation.html).
- **Gazebo:** Follow the instructions [here](https://gazebosim.org/docs/latest/ros_installation/).
  - **NOTE:** As of now, the project is compatible with Gazebo Classic (version 11), ***now deprecated.*** The transition to Gazebo Fortress (version 12) is in progress and will be updated soon. Till then, please use Gazebo Classic or attempt a migration using the [migration guide](https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/).

On the companion computer (Raspberry Pi), install the following:
- **ROS2 Humble:** Follow the instructions [here](https://docs.ros.org/en/humble/Installation.html).
  - **NOTE:** Ensure that the Raspberry Pi has sufficient resources and a ***stable internet connection*** for offloading heavy computations to the server.
- **YDLidar ROS2 Driver:** Follow the instructions [here](https://github.com/YDLIDAR/ydlidar_ros2_driver).
- **Microcontroller:** Upload the [firmware](https://github.com/RockOnJeet/Omni-Bot/blob/Raspberry-Pi/omni_controller/resource/omni_arduino/omni_arduino.ino).
To run the project, follow these steps:
1. **Clone the Repository:**

   <span style="color:blue;">**Server Side:**</span>
   ```bash
   git clone -b ROS2-Humble https://github.com/RockOnJeet/Omni-Bot.git
   cd Omni-Bot
   ```
   <span style="color:green;">**Client Side (Raspberry Pi):**</span>
   ```bash
   git clone -b Raspberry-Pi https://github.com/RockOnJeet/Omni-Bot.git
   cd Omni-Bot
   ```
2. **Install Dependencies:**
   ```bash
   sudo apt update
   sudo apt install -y python3-colcon-common-extensions
   ```
3. **Build the Project:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
4. **Visualize in RViz2:** Open a new terminal and source the workspace:
      ```bash
      source install/setup.bash
      ```
   1. For visualizing the digital twin only <span style="color:blue;">**[Server Side]:**</span>
      ```bash
      ros2 launch omni_bot_description urdf.launch.py
      ```
    2. For running the digital twin <span style="color:blue;">**[Server Side]:**</span>
        ```bash
        ros2 launch omni_bot_sim gazebo.launch.py
        ```
        Control using `teleop_twist_keyboard`:
        ```bash
        ros2 run teleop_twist_keyboard teleop_twist_keyboard
        ```
    3. For running the real robot (hardware) <span style="color:blue;">**[Server Side]:**</span>
       * For mapping the environment:
          ```bash
          ros2 launch omni_bot_real mapper.launch.py
          ```
          Control using `teleop_twist_keyboard`:
          ```bash
          ros2 run teleop_twist_keyboard teleop_twist_keyboard
          ```
       * For autonomous navigation:
          ```bash
          ros2 launch omni_bot_real automation.launch.py
          ```
          Regain control using `teleop_twist_keyboard`:
          ```bash
          ros2 run teleop_twist_keyboard teleop_twist_keyboard
          ```
    4. For running the companion computer (Raspberry Pi) <span style="color:green;">**[Client Side]:**</span>
        ```bash
        ros2 launch omni_bot bot.launch.py
        ```
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
<!-- Add references and resources. -->
### Software
- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html): Comprehensive guide for ROS2, detailing installation, configuration, and usage.
- [Gazebo Documentation](https://gazebosim.org/docs/latest/getstarted/): Official Gazebo documentation for installation and usage.
- [YDLidar ROS2 Driver](https://www.ydlidar.com/products/view/1.html): Official YDLidar G2 datasheet and ROS2 driver documentation.
- [Arduino IDE](https://www.arduino.cc/en/software): Download and installation instructions for the Arduino IDE.

### Hardware
- [YDLidar G2](https://www.ydlidar.com/service_support/download.html?gid=1): Specifications and usage instructions for the YDLidar G2 sensor.
- [Arduino Documentation](https://www.arduino.cc/en/Guide/HomePage): Official Arduino guides for setup and programming.

### Unique Motion Systems
- [Omni-Wheels](https://en.wikipedia.org/wiki/Omni_wheel): Information about omni-wheels and their applications in robotics.
- [Omni-Directional Drive Systems](https://www.wevolver.com/article/holonomic-robot): Overview of omni-directional drive mechanisms.
- [SLAM Algorithms](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping): Insights into SLAM algorithms used for mapping and navigation.

### Inspiration and Research
  - [ROS2 Tutorials:](https://docs.ros.org/en/humble/Tutorials.html) Official ROS2 tutorials for beginners and advanced users.
  - [Articulated Robotics:](https://www.youtube.com/@ArticulatedRobotics) YouTube channel with tutorials on ROS2 and robotics, including a tutorial on building a Differential Drive Robot.
  - [Eyantra:](https://e-yantra.org/) A robotics initiative by IIT Bombay that provides resources and projects related to robotics and automation.
  - [Robotics Stack Exchange:](https://robotics.stackexchange.com/) A question-and-answer site for professional and hobbyist roboticists.
