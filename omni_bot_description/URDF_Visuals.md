# URDF Visualization - Quick Start Guide

## System Requirements

- **OS**: Ubuntu 24.04 (or compatible)
- **ROS 2**: Jazzy Jalisco
- **Required Packages**:
  - `ros-jazzy-robot-state-publisher`
  - `ros-jazzy-joint-state-publisher-gui`
  - `ros-jazzy-rviz2`
  - `ros-jazzy-xacro`

## Installation

Install required packages if not already installed:

```bash
sudo apt update
sudo apt install ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-rviz2 \
                 ros-jazzy-xacro
```

## Build

```bash
cd ~/ROSCodes/Omni-Bot
colcon build --packages-select omni_bot_description --symlink-install
source install/setup.bash
```

## Run

### Robot Description Only

Publish only the `/robot_description` topic (no visualization):

```bash
ros2 launch omni_bot_description bot.launch.py
```

Optional parameters:
- `robot_model:=rviz` (default) or `robot_model:=gz`
- `use_sim_time:=false` (default) or `use_sim_time:=true`

### Full Visualization with Joint Control

Launch RViz2 with robot visualization and joint control GUI:

```bash
ros2 launch omni_bot_description rviz.launch.py
```

Optional parameters:
```bash
ros2 launch omni_bot_description rviz.launch.py robot_model:=rviz use_sim_time:=false
```

**Parameters:**
- `robot_model`: Choose URDF variant
  - `rviz` (default): Optimized for visualization, includes visual meshes
  - `gz`: Optimized for Gazebo physics simulation
- `use_sim_time`: Time source
  - `false` (default): Use system time
  - `true`: Use `/clock` topic (required when running with Gazebo)

## Troubleshooting

**RViz shows no robot:**
- Check that `/robot_description` topic is being published.
- Verify Fixed Frame is set to `base_link` in RViz.

**No joint sliders:**
- Ensure your URDF has movable joints defined.
- Check that joint types are not `fixed`.

**Build errors:**
- Make sure all dependencies are installed.
- Source ROS 2: `source /opt/ros/jazzy/setup.bash`.
