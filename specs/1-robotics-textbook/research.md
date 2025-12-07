# Research Plan & Outcomes: Physical AI & Humanoid Robotics Textbook

This document outlines the research required to resolve technical uncertainties and defines the best approach for building the textbook and its associated examples.

## Research Tasks

### 1. Target ROS 2 Distribution
- **Task**: Determine the most suitable ROS 2 distribution for this textbook.
- **Questions**:
    - Which LTS (Long-Term Support) release is currently recommended for new projects?
    - Are there any major breaking changes between recent distributions (e.g., Humble, Iron, Jazzy) that would affect the curriculum?
    - What is the tool and library support like for the chosen distribution?
- **Hypothesis**: ROS 2 Humble Hawksbill is the best choice due to its LTS status and widespread industry adoption, ensuring stability for students.

### 2. LLM for Cognitive Planning
- **Task**: Define a strategy for integrating a Large Language Model (LLM) for the cognitive planning part of the VLA module and capstone.
- **Questions**:
    - What are the leading LLMs for task and motion planning in robotics?
    - How can we design a generic Python interface that is not tied to a specific vendor (e.g., OpenAI, Google)?
    - What would a sample API contract look like for a service that takes a high-level command (e.g., "get the water bottle") and returns a sequence of ROS 2 actions?
- **Hypothesis**: A simple REST-based service wrapper can be created. For the textbook, we can use a mock or a publicly accessible API to demonstrate the concept without requiring students to have paid API keys.

### 3. Python Testing Framework for ROS 2
- **Task**: Identify the standard testing framework and best practices for testing `rclpy` (Python for ROS 2) nodes.
- **Questions**:
    - Is `pytest` the standard for ROS 2 Python packages?
    - What tools are available for writing integration tests for ROS 2 nodes that communicate over topics?
    - How can we structure tests for the example code provided in the textbook?
- **Hypothesis**: `pytest` is the standard. We can use `launch_testing` for integration tests and provide a simple testing guide for students.

### 4. Primary Development Platform
- **Task**: Confirm the primary target operating system for the robotics development and simulation portions of the textbook.
- **Questions**:
    - Are ROS 2, Gazebo, and NVIDIA Isaac Sim all fully supported on Linux?
    - What is the state of support for Windows and macOS?
    - Should the documentation provide setup instructions for multiple OSes or focus on one?
- **Hypothesis**: Linux (specifically Ubuntu 22.04 for ROS 2 Humble) is the de-facto standard and should be the primary documented platform to ensure a smooth student experience.

### 5. Docusaurus Site Structure
- **Task**: Research best practices for organizing a large, multi-section Docusaurus project.
- **Questions**:
    - How should the sidebar be structured for optimal navigation across four modules and a capstone project?
    - Should we use Docusaurus's docs-only mode?
    - What is the best way to handle diagrams (e.g., Mermaid.js, static images)?
- **Hypothesis**: A multi-level sidebar defined in `sidebars.ts` is the best approach. Mermaid.js should be used for generating diagrams from code to keep them maintainable.

### 6. ROS 2 Integration Patterns (Unity & Isaac Sim)
- **Task**: Research the standard integration patterns for connecting ROS 2 with Unity and NVIDIA Isaac Sim.
- **Questions**:
    - For Unity, what is the recommended connector/bridge (e.g., ROS TCP Connector)?
    - For Isaac Sim, what are the built-in capabilities for ROS 2 communication (e.g., ROS 2 bridge)?
    - How do we handle coordinate frame transformations between the simulators and ROS?
- **Hypothesis**: Isaac Sim has a native, high-performance ROS 2 bridge. For Unity, the official ROS TCP Connector is the standard. The textbook must include a dedicated section on TF (Transform) management.

### 7. Whisper to ROS 2 Pipeline
- **Task**: Design a simple and robust pipeline for converting voice commands to ROS 2 messages using Whisper.
- **Questions**:
    - How can we capture audio from a microphone in Python?
    - Can Whisper run locally for real-time transcription, or should we assume an API call?
    - What is the best way to structure the ROS 2 node? Should it publish to a `/speech_to_text` topic?
- **Hypothesis**: A Python script using a library like `sounddevice` can capture audio, send it to a local or API-based Whisper model, and a simple `rclpy` publisher can put the resulting string onto a `/voice_command` topic.

---

## Research Outcomes

Based on the research tasks, the following technical decisions have been made.

1.  **Target ROS 2 Distribution**: **ROS 2 Humble Hawksbill** will be the official distribution for this textbook. It is the current LTS release, is stable, and has excellent community and tool support. All examples and projects will target this version.

2.  **LLM for Cognitive Planning**: A generic Python function `def get_action_plan(command: str) -> list[str]:` will be defined. The textbook will feature a **mock implementation** of this function that returns a hardcoded list of actions based on specific inputs. This approach removes the need for students to acquire and manage API keys, allowing them to focus on the robotics integration logic.

3.  **Python Testing Framework for ROS 2**: **`pytest`** will be used as the standard testing framework for all Python code. For integration testing of ROS 2 nodes, **`launch_testing`** will be used. A dedicated chapter or appendix will cover the basics of testing ROS 2 nodes.

4.  **Primary Development Platform**: The official development environment is **Linux (Ubuntu 22.04)**. All instructions, scripts, and examples will be tailored for this platform to ensure consistency and minimize environment-related issues for students.

5.  **Docusaurus Site Structure**: The textbook will use a **multi-level sidebar** configured in `sidebars.ts` for clear navigation. All diagrams will be created using **Mermaid.js** syntax directly within the Markdown files, making them easy to maintain and version control.

6.  **ROS 2 Integration Patterns**:
    -   **NVIDIA Isaac Sim**: The built-in **native ROS 2 bridge** will be used for all Isaac Sim examples.
    -   **Unity**: The official **ROS-TCP-Connector** from the Unity Robotics Hub will be used.
    -   A chapter in Module 1 will be dedicated to explaining **TF (Transform) concepts** in ROS 2.

7.  **Whisper to ROS 2 Pipeline**: A ROS 2 node will be created that uses the `sounddevice` Python library to capture microphone input. It will then use the `whisper` library (running a local base model) to perform transcription. The resulting text will be published to a `/voice_command` topic using the `std_msgs/msg/String` message type.
