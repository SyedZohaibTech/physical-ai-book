---
title: Glossary
sidebar_position: 100
---

# Glossary of Terms

This glossary provides definitions for key terms and acronyms used throughout the "Physical AI & Humanoid Robotics" textbook.

## A

**Action (ROS 2)**: A type of ROS 2 communication for long-running, goal-oriented tasks that provide feedback during execution and a final result upon completion. Consists of a goal, result, and feedback.

**AI (Artificial Intelligence)**: The simulation of human intelligence processes by machines, especially computer systems. These processes include learning, reasoning, and self-correction.

**AI Agent**: An autonomous entity that observes through sensors and acts upon an environment using actuators. In robotics, often refers to software that makes decisions for a robot.

**AMENT**: The build system used by ROS 2. It is a set of tools to compile and link packages.

**API (Application Programming Interface)**: A set of defined methods of communication among various components.

## C

**Colcon**: The build tool used in ROS 2. It orchestrates the compilation of multiple packages in a workspace.

**Cognitive Planning**: The process by which an intelligent agent (often using an LLM) breaks down high-level goals into a sequence of executable sub-tasks, reasoning about the environment and available actions.

**Controller (ROS 2)**: A software component that manages a robot's joints or actuators to achieve a desired state (e.g., position, velocity, effort).

**Costmap (Nav2)**: A 2D grid used by the Nav2 stack to represent the robot's environment, indicating traversability and obstacles for path planning.

## D

**Depth Camera**: A sensor that captures an image where each pixel's value represents its distance from the camera, providing 3D spatial information.

**Digital Twin**: A virtual, physics-based, 1:1 representation of a physical object or system, connected to the same control software as its real-world counterpart.

**Docusaurus**: A static site generator that helps you build optimized websites quickly. Used for building this textbook.

## G

**Gazebo**: A powerful open-source 3D robotics simulator, known for its high-fidelity physics engine.

**GPU (Graphics Processing Unit)**: A specialized electronic circuit designed to rapidly manipulate and alter memory to accelerate the creation of images in a frame buffer for output to a display device. Essential for AI and robotics.

## H

**Human-Robot Interaction (HRI)**: The study of interactions between humans and robots.

## I

**IMU (Inertial Measurement Unit)**: An electronic device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, using a combination of accelerometers, gyroscopes, and magnetometers.

**Isaac ROS**: NVIDIA's collection of hardware-accelerated ROS 2 packages (GEMs) that leverage GPUs for performance-critical robotics tasks like perception and navigation.

**Isaac Sim**: NVIDIA's robotics simulation application, built on Omniverse, focused on photorealistic rendering and physics simulation for AI training and development.

## J

**Joint (URDF)**: Defines the connection and relative motion between two links in a robot model (e.g., revolute, prismatic, fixed).

**JointState (ROS 2 Message)**: A standard ROS 2 message type (`sensor_msgs/msg/JointState`) used to convey the current state (position, velocity, effort) of a robot's joints.

## L

**Launch File (ROS 2)**: A Python script used to automate the startup of multiple ROS 2 nodes, programs, and configurations with a single command.

**LiDAR (Light Detection and Ranging)**: A remote sensing method that uses light in the form of a pulsed laser to measure ranges (variable distances) to the Earth. In robotics, used for mapping and obstacle detection.

**Link (URDF)**: Represents a rigid part of a robot model in a URDF file, describing its visual, collision, and inertial properties.

**LLM (Large Language Model)**: A type of artificial intelligence program that can generate and understand human-like language, often used for tasks like text generation, translation, and reasoning.

**Localization**: The process by which a robot determines its own position and orientation within a known map.

## M

**Mapping**: The process by which a robot builds a representation (a map) of its environment.

**Message (ROS 2)**: A data structure used for communication between ROS 2 nodes over topics.

**Middleware**: Software that provides common services and capabilities to applications beyond those offered by the operating system. ROS is a robotics middleware.

## N

**Nav2 (Navigation2)**: The ROS 2 navigation stack, providing functionalities for autonomous movement, including global and local path planning, and obstacle avoidance.

**NITROS (NVIDIA Isaac Transport for ROS)**: A technology in Isaac ROS that enables high-performance, zero-copy data transfer between CPU and GPU for ROS 2 messages.

**Node (ROS 2)**: The smallest executable unit in a ROS 2 system, responsible for a specific task (e.g., a camera driver node, a motor controller node).

## P

**Perception**: The process by which a robot interprets sensory data (from cameras, LiDAR, etc.) to understand its environment.

**Point Cloud**: A set of data points in a three-dimensional coordinate system, typically generated by 3D scanners (e.g., LiDAR, depth cameras), representing the external surface of an object or environment.

**Prompt Engineering**: The process of designing and refining input prompts for large language models to elicit desired outputs.

**Publish/Subscribe**: A messaging pattern where senders (publishers) do not program messages to be sent directly to specific receivers (subscribers), but instead characterize published messages into classes without knowledge of which subscribers, if any, there may be.

**Pytest**: A popular Python testing framework used for writing unit and integration tests.

## R

**rclpy**: The official Python client library for ROS 2, used to write ROS 2 nodes in Python.

**Request/Response**: A communication pattern where a client sends a request to a server, and the server processes the request and sends a response back to the client. Used in ROS 2 Services.

**ROS (Robot Operating System)**: An open-source robotics middleware suite that provides libraries and tools to help software developers create robot applications.

**ROS 2**: The next generation of ROS, designed with improved real-time capabilities, security, and multi-robot support.

**ros2_control**: A generic framework for robot control in ROS 2, providing a standardized interface for hardware abstraction and controller management.

**ROS-TCP-Connector**: A plugin for Unity that enables communication between a Unity simulation and a ROS 2 network over TCP.

**RViz**: A 3D visualization tool for ROS, allowing users to view sensor data, robot models, and navigation trajectories.

## S

**SDF (Simulation Description Format)**: An XML format used by Gazebo to describe robots, environments, and other simulation elements. An extension of URDF.

**SLAM (Simultaneous Localization and Mapping)**: The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

**Service (ROS 2)**: A type of ROS 2 communication for one-shot, request-response interactions between nodes.

**Speech-to-Text**: The process of converting spoken words into written text.

## T

**TF2 (Transformations)**: A ROS 2 library that keeps track of coordinate frames over time, allowing easy transformation of data between different frames (e.g., robot base, camera, world).

**Topic (ROS 2)**: A named bus over which ROS 2 nodes exchange messages using a publish/subscribe mechanism.

## U

**Unity**: A popular real-time 3D development platform (game engine) increasingly used for high-fidelity robotics simulation and synthetic data generation.

**URDF (Unified Robot Description Format)**: An XML file format used in ROS to describe the physical and kinematic properties of a robot.

## V

**VLA (Vision-Language-Action)**: A robotics paradigm that combines visual perception, natural language understanding (often with LLMs), and robot action execution to enable robots to perform complex tasks based on high-level human commands.

**VSLAM (Visual SLAM)**: A type of SLAM that uses one or more cameras as its primary sensor for simultaneous localization and mapping.

## W

**Whisper (OpenAI)**: A general-purpose speech recognition model by OpenAI, capable of transcribing speech into text with high accuracy.
