# Gazebo Simulation - Quick Start Guide

## System Requirements

- **OS**: Ubuntu 24.04 (or compatible)
- **ROS 2**: Jazzy Jalisco
- **Required Packages**:
  - `ros-jazzy-ros-gz-sim`
  - `ros-jazzy-xacro`
  - `omni_bot_description` (URDF/Xacro + robot_state_publisher)

## Installation

Install required packages if not already installed:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim \
                 ros-jazzy-xacro
```

## Build

```bash
cd ~/ROSCodes/Omni-Bot
colcon build --packages-select omni_bot_sim --symlink-install
source install/setup.bash
```

## Run

To launch Gazebo with the robot spawned in the default empty world:

```bash
ros2 launch omni_bot_sim gazebo.launch.py
```

> **Note**: The launch always uses:
> - **Namespace**: `gz`
> - **Sim Time**: `true` (always enabled)
> - **Robot Model**: `gz` variant (from `omni_bot_description`)
> - **Entity Name**: `omni_bot`
> - **Spawn Pose**: (x=0, y=0, z≈0, yaw=0) — simulator defaults

## Optional: Custom World

To launch with a custom world file:

```bash
ros2 launch omni_bot_sim gazebo.launch.py world:=/absolute/path/to/world.sdf
```

Leave `world` empty (default) for the standard empty Gazebo world.

## What happens when you launch

1. **Gazebo (gz sim)** starts via `ros_gz_sim/launch/gz_sim.launch.py`
2. **Robot State Publisher** launches from `omni_bot_description/launch/bot.launch.py` under `gz` namespace
3. **Robot spawning** expands the Xacro (`gz_bot.urdf.xacro`) and spawns the entity using `ros_gz_sim create`

## Troubleshooting

**Gazebo doesn't start:**
- Verify Gazebo tools are installed:
  ```bash
  gz sim --help
  ros2 pkg list | grep ros_gz
  ```

**Robot doesn't appear in simulation:**
- Check that `/gz/robot_state_publisher` is running.
- Verify `/gz/robot_description` topic is being published:
  ```bash
  ros2 topic list | grep robot_description
  ros2 topic echo /gz/robot_description --once
  ```

**Xacro errors:**
- Ensure `omni_bot_description` is built and sourced.
- Check for malformed or missing files under `omni_bot_description/description/`.

**Build errors:**
- Make sure all dependencies are installed.
- Source ROS 2: `source /opt/ros/jazzy/setup.bash`.
