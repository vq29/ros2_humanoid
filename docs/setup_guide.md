# Ubuntu Setup Guide

Complete setup instructions for the Ubuntu 24.04 laptop.

## 1. Install ROS2 Jazzy

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop -y

# Source ROS2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Install Dependencies

```bash
# Gazebo Harmonic + ROS2 bridge
sudo apt install ros-jazzy-ros-gz -y

# MoveIt2
sudo apt install ros-jazzy-moveit -y

# ros2_control
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers -y
sudo apt install ros-jazzy-gz-ros2-control -y

# Additional tools
sudo apt install ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-controller-manager \
    ros-jazzy-cv-bridge -y

# Build tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Python dependencies
pip install opencv-python numpy
```

## 3. Clone and Build

```bash
# Clone from GitHub
cd ~
git clone https://github.com/vq29/ros2_humanoid.git
cd ros2_humanoid

# Install rosdep dependencies
sudo rosdep init  # Only needed once
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Source workspace
echo "source ~/ros2_humanoid/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 4. Verify Installation

```bash
# Check URDF
ros2 run xacro xacro src/humanoid_description/urdf/humanoid.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf

# Visualize in RViz
ros2 launch humanoid_description display.launch.py
```

## 5. Run the Demo

```bash
# Option 1: Step by step
ros2 launch humanoid_gazebo simulation.launch.py           # Gazebo only
ros2 launch humanoid_moveit_config moveit_gazebo.launch.py  # MoveIt + Gazebo

# Option 2: Full demo
ros2 launch humanoid_bringup pick_place_demo.launch.py
```

## SSH Setup (for Windows → Ubuntu Remote Development)

```bash
# On Ubuntu
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh

# Get your IP
ip addr show | grep inet

# On Windows (PowerShell) — generate SSH key and copy
ssh-keygen -t ed25519
ssh-copy-id user@UBUNTU_IP

# VS Code Remote-SSH: connect to user@UBUNTU_IP
```
