---
sidebar_position: 2
title: ROS 2 Setup and Installation
---

# ROS 2 Setup and Installation

This chapter guides you through setting up ROS 2 on Ubuntu 22.04 LTS, which is the recommended platform for robotics development.

## System Requirements

- Ubuntu 22.04 (Jammy Jellyfish) - recommended
- At least 4GB RAM (8GB+ recommended)
- 20GB free disk space
- A dedicated development environment (physical machine or virtual machine)

## Installation Steps

### 1. Set Locale
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2. Add ROS 2 Repository
First, add the ROS 2 repository to your system:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install ROS 2
Update your package list and install the ROS 2 Desktop package:

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

For a minimal installation (without GUI tools), install only the ROS base:

```bash
sudo apt install ros-humble-ros-base
```

### 4. Environment Setup
Add ROS 2 to your environment:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Install colcon build tools
Install the tools required for building ROS 2 packages:

```bash
sudo apt install python3-colcon-common-extensions
```

### 6. Install additional dependencies
Install Python packages needed for development:

```bash
pip3 install -U argcomplete
pip3 install -U rosdep
sudo rosdep init
rosdep update
```

## Creating a ROS 2 Workspace

### 1. Create the workspace directory:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Build the workspace:
```bash
colcon build
```

### 3. Source the workspace:
```bash
source install/setup.bash
```

To automatically source your workspace when opening a new terminal, add the following to your `~/.bashrc`:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Testing Your Installation

### 1. Run the talker demo:
Open a new terminal and run:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

### 2. Run the listener demo:
In another terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see messages being published by the talker and received by the listener.

## Essential ROS 2 Commands

Here are some key commands you'll use frequently:

- `ros2 topic list` - List all active topics
- `ros2 service list` - List all active services
- `ros2 node list` - List all active nodes
- `ros2 param list <node_name>` - List parameters of a specific node
- `ros2 run <package_name> <executable_name>` - Run a node
- `ros2 launch <package_name> <launch_file>` - Launch multiple nodes

## Development Tools

### 1. RViz2 - 3D Visualization Tool
RViz2 is the 3D visualization tool for ROS 2:
```bash
ros2 run rviz2 rviz2
```

### 2. rqt - GUI Tools Suite
rqt provides a set of GUI tools for ROS 2:
```bash
ros2 run rqt_gui rqt_gui
```

### 3. ros2doctor - System Diagnostics
Check your ROS 2 installation:
```bash
ros2 doctor
```

## Setting Up Your Development Environment

### 1. Install VS Code with ROS extension
```bash
sudo snap install code --classic
```

Install the ROS extension in VS Code for syntax highlighting and debugging support.

### 2. Install additional tools
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

## Troubleshooting Common Issues

### 1. Permission errors with ROS 2
If you encounter permission errors, make sure you're not running ROS 2 commands with sudo in normal usage.

### 2. Network configuration
If running ROS 2 across multiple machines, configure your network settings properly:
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 3. Resource limitations
For systems with limited resources, consider using the ROS 2 base installation instead of the full desktop package.

## Virtual Environment Setup (Optional)

For managing Python dependencies, it's recommended to use a virtual environment:

```bash
python3 -m venv ~/ros2_env
source ~/ros2_env/bin/activate
pip3 install -U pip setuptools
```

## Docker Setup (Alternative)

For a containerized approach, you can use ROS 2 Docker images:

```bash
docker pull ros:humble
docker run -it ros:humble
```

## Next Steps

With ROS 2 properly installed, you can now:

1. Create your first ROS 2 package
2. Explore the ROS 2 tutorials
3. Set up your development environment
4. Begin working with ROS 2 concepts like nodes, topics, and services

The installation process is fundamental to your ROS 2 journey. Take time to familiarize yourself with the basic commands and tools before proceeding to more advanced topics. Understanding the workspace structure and build system is crucial for effective ROS 2 development.

## Verification Checklist

Before moving on, verify that:
- [ ] ROS 2 commands work correctly (e.g., `ros2 --version`)
- [ ] You can run basic demos (talker/listener)
- [ ] Your environment is properly sourced
- [ ] You have a working workspace
- [ ] Essential tools are installed and accessible

This setup provides a solid foundation for developing sophisticated ROS 2 applications, particularly for humanoid robotics where reliable communication and real-time performance are critical.