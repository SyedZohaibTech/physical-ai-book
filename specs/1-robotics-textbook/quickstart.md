# Quickstart Guide: Textbook Development Environment

This guide provides the steps to set up the development environment for working on the "Physical AI & Humanoid Robotics" textbook.

**Primary Target Platform**: Ubuntu 22.04

## 1. Core Dependencies

First, ensure your system is up-to-date and has essential build tools.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install build-essential git python3-pip python3-venv -y
```

## 2. Docusaurus Website Setup

This covers the setup for the textbook's static site.

```bash
# Navigate to the website directory
cd website

# Install Node.js dependencies
npm install

# Run the Docusaurus development server
npm run start
```

The website should now be available at `http://localhost:3000`. Content for the textbook is located in the `website/docs` directory.

## 3. Robotics Environment Setup

This is a high-level overview. Each module in the textbook will have more detailed setup instructions.

### 3.1. ROS 2 Humble Installation

Follow the official ROS 2 documentation to install **ROS 2 Humble Hawksbill**.

```bash
# Follow the official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# Make sure to also install colcon, the ROS 2 build tool
sudo apt install ros-humble-colcon-common-extensions
```

### 3.2. Gazebo Simulator

Install Gazebo, which is used for physics simulation in Module 2.

```bash
# Install Gazebo for ROS 2 Humble
sudo apt install ros-humble-gazebo-*
```

### 3.3. Python Dependencies

The Python examples will have their own dependencies. A `requirements.txt` will be provided for each example project in the `/examples` directory.

```bash
# Example usage for a project
cd examples/module-1-ros2/
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 4. Development Workflow

1.  **Content Generation**: Create or edit `.md`/`.mdx` files in the `website/docs` directory. Follow the established structure.
2.  **Example Code**: Add or modify Python ROS 2 packages within the `/examples` directory. Each example should be a self-contained ROS 2 package.
3.  **Diagrams**: Use Mermaid.js syntax inside Markdown files for all diagrams.
4.  **Testing**:
    - Run `npm run build` in the `website` directory to ensure the Docusaurus site builds without errors.
    - Run `pytest` within each Python example's directory to execute unit tests.
    - Use `colcon build` and `colcon test` for ROS 2 package testing.
