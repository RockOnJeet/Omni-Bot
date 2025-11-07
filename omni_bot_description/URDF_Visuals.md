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

For just the `robot_description` topic:

```bash
ros2 launch omni_bot_description bot.launch.py
```

To launch the URDF visualization with manual joint control:

```bash
ros2 launch omni_bot_description urdf.launch.py
```

> Note: For different models (Gz & RViz), change parameters:
> ```bash
> robot_mode:=gz (/rviz)
> ```
> Default opens `gz`.

## Optional: Change Sim Time

Add additional parameter to the above:
```bash
use_sim_time:=true (/false)
```

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
