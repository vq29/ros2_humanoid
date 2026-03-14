# ROS2 Humanoid Arm Pick-and-Place Simulation

A ROS2 Jazzy-based humanoid robot manipulation system in simulation. The robot autonomously detects an object, plans a motion trajectory using MoveIt2, grasps the object, and places it at a target location.

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-purple)
![MoveIt2](https://img.shields.io/badge/MoveIt2-latest-green)

## System Architecture

```
Camera → Object Detection Node → Target Pose → MoveIt2 Planner → Joint Trajectory → Robot Arm Controller → Gripper
```

```
┌─────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  RGB-D       │────▶│  Object Detector │────▶│  Target Pose    │
│  Camera      │     │  (OpenCV)        │     │  Publisher       │
└─────────────┘     └──────────────────┘     └────────┬────────┘
                                                       │
                                                       ▼
┌─────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Gazebo      │◀───│  ros2_control    │◀───│  MoveIt2        │
│  Simulation  │    │  Controllers     │    │  Motion Planner │
└─────────────┘     └──────────────────┘     └─────────────────┘
```

## Tech Stack

| Component | Technology |
|---|---|
| OS | Ubuntu 24.04 |
| ROS2 | Jazzy Jalisco |
| Simulator | Gazebo Harmonic (gz-sim) |
| Motion Planning | MoveIt2 + MoveIt Task Constructor |
| Controllers | ros2_control |
| Vision | OpenCV + simulated RGB-D camera |
| Visualization | RViz2 |

## Package Structure

| Package | Description |
|---|---|
| `humanoid_description` | Robot URDF/Xacro model, meshes, and RViz config |
| `humanoid_gazebo` | Gazebo world, launch files, and controller configs |
| `humanoid_moveit_config` | MoveIt2 configuration (SRDF, kinematics, planners) |
| `humanoid_perception` | OpenCV-based object detection node |
| `humanoid_bringup` | Top-level launch files and pick-and-place pipeline |

## Prerequisites

```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# Gazebo Harmonic + ROS2 bridge
sudo apt install ros-jazzy-ros-gz

# MoveIt2
sudo apt install ros-jazzy-moveit

# ros2_control
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# Additional dependencies
sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher ros-jazzy-controller-manager \
    ros-jazzy-gz-ros2-control
    
# Python dependencies
pip install opencv-python numpy
```

## Build

```bash
cd ~/ros2_humanoid
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### 1. Visualize the Robot in RViz
```bash
ros2 launch humanoid_description display.launch.py
```

### 2. Launch Gazebo Simulation
```bash
ros2 launch humanoid_gazebo simulation.launch.py
```

### 3. Launch MoveIt2 + Gazebo
```bash
ros2 launch humanoid_moveit_config moveit_gazebo.launch.py
```

### 4. Run Full Pick-and-Place Demo
```bash
ros2 launch humanoid_bringup pick_place_demo.launch.py
```

## Robot Model

Custom simplified humanoid with:
- **Torso:** Fixed rectangular base (0.4m × 0.3m × 0.5m)
- **Head:** Cosmetic sphere on top
- **Arm:** 6-DOF serial manipulator (shoulder pitch/roll/yaw, elbow pitch, wrist pitch/roll)
- **Gripper:** 2-finger parallel gripper (prismatic joints)
- **Camera:** RGB-D sensor mounted on head

## License

MIT License
