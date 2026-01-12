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

> **Note**: The launch file configuration:
> - **Sim Time**: `true` (always enabled for all nodes)
> - **Robot Model**: Uses `gz_bot.urdf.xacro` variant for Gazebo physics
> - **Entity Name**: `Omni_Bot`
> - **Spawn Method**: Direct URDF string spawn (no topic dependency)
> - **Default World**: `empty.sdf` (standard Gazebo empty world)

## Optional: Custom World

To launch with a custom world file:

```bash
ros2 launch omni_bot_sim gazebo.launch.py world:=/path/to/world.sdf
```

Or relative to the package:
```bash
ros2 launch omni_bot_sim gazebo.launch.py world:=warehouse.sdf
```

## What Happens When You Launch

1. **Gazebo (gz sim)** starts with:
   - Auto-run enabled (`-r` flag)
   - Verbosity level 3 (`-v3`)
   - Custom or default world loaded
   - Shutdown on exit configured

2. **Robot Spawning**:
   - Xacro file `gz_bot.urdf.xacro` processed to URDF
   - Robot spawned using `ros_gz_sim create` with URDF string
   - No dependency on `/robot_description` topic

3. **ROS-Gazebo Bridge** (`ros_gz_bridge`):
   - Bridges topics between Gazebo and ROS 2
   - Configuration from `config/ros_gz_bridge.yaml`
   - Currently bridges: `/clock`, `/joint_states`
   - Runs as standalone node (no composition)

4. **RViz2 Visualization**:
   - Launches via `omni_bot_description/launch/urdf.launch.py`
   - Uses `robot_model:=rviz` variant for display
   - Sim time enabled for synchronization

## Troubleshooting

**Gazebo doesn't start:**
- Verify Gazebo installation:
  ```bash
  gz sim --help
  ros2 pkg list | grep ros_gz
  ```
- Check if another Gazebo instance is running:
  ```bash
  ps aux | grep gz
  ```

**Robot doesn't appear in simulation:**
- Check Gazebo entity list:
  ```bash
  gz model --list
  ```
- Verify spawn command succeeded (check terminal output)
- Try respawning:
  ```bash
  gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 300 --req 'sdf_filename: "/path/to/model.sdf"'
  ```

**Bridge topics not working:**
- List active bridges:
  ```bash
  ros2 topic list | grep -E '(clock|joint_states)'
  ```
- Check bridge configuration:
  ```bash
  ros2 param list /ros_gz_bridge
  ```
- Verify Gazebo topics:
  ```bash
  gz topic -l
  ```

**RViz shows no robot:**
- Ensure `robot_state_publisher` is running:
  ```bash
  ros2 node list | grep robot_state_publisher
  ```
- Check `/robot_description` topic:
  ```bash
  ros2 topic echo /robot_description --once
  ```
- Set Fixed Frame to `base_link` in RViz

**Xacro processing errors:**
- Test xacro independently:
  ```bash
  xacro $(ros2 pkg prefix omni_bot_description)/share/omni_bot_description/description/gz_bot.urdf.xacro
  ```
- Check for missing mesh files or malformed XML

**Build errors:**
- Install all dependencies:
  ```bash
  cd ~/ROSCodes/Omni-Bot
  rosdep install --from-paths src --ignore-src -r -y
  ```
- Source ROS 2:
  ```bash
  source /opt/ros/jazzy/setup.bash
  ```
- Clean and rebuild:
  ```bash
  rm -rf build install log
  colcon build --symlink-install
  ```
