---
title: "3. LLM to ROS Actions: The Cognitive Core"
sidebar_label: "LLM to ROS Actions"
sidebar_position: 3
---

# 3. LLM to ROS Actions: The Cognitive Core

We can now turn spoken commands into text. The next crucial step in our VLA pipeline is to convert this natural language text into a sequence of executable robot actions. This is the **cognitive core** of our robot's intelligence, typically handled by a Large Language Model (LLM).

## The Role of the LLM in Robotics

An LLM's strength lies in its ability to understand and generate human-like text, reason about tasks, and even generate code. In robotics, we can leverage these capabilities to:

1.  **Task Decomposition**: Break down a complex, high-level instruction ("Make me coffee") into a series of smaller, more manageable sub-tasks ("Go to kitchen," "Get mug," "Brew coffee").
2.  **Action Sequencing**: Order these sub-tasks logically to achieve the overall goal.
3.  **Parameter Grounding**: Extract key entities and parameters from the natural language command (e.g., "red mug," "table," "5 meters") and map them to the robot's capabilities and environment.
4.  **Error Handling**: Potentially reason about failures and suggest recovery actions.

## Prompt Engineering for Robot Actions

The key to getting an LLM to generate robot actions is **prompt engineering**. We provide the LLM with a carefully crafted prompt that includes:
-   **System Role**: Defining the LLM's role (e.g., "You are a helpful robot assistant.").
-   **Available Tools/Actions**: Listing the robot's capabilities (e.g., `navigate_to(location)`, `pick_up(object_name)`).
-   **Context**: Information about the robot's current state and environment.
-   **User Command**: The natural language instruction.
-   **Desired Output Format**: Specifying how the LLM should output the action sequence (e.g., a JSON list of actions).

### Example Prompt (Simplified)

```
You are a helpful robot assistant. Your goal is to translate human commands into a sequence of executable robot actions.

Available actions:
- navigate_to(location_name: str) -> bool: Navigates the robot to a predefined location. Returns True on success.
- pick_up(object_name: str) -> bool: Picks up a specified object. Returns True on success.
- say(phrase: str) -> None: Makes the robot speak a phrase.

Current known locations: "kitchen", "bedroom", "living_room"
Current known objects: "mug", "book", "water_bottle"

Human command: "Go to the kitchen and grab the mug."

Please output a JSON list of actions the robot should perform.
```

The LLM, given this prompt, might respond with:
```json
[
    {"action": "navigate_to", "args": {"location_name": "kitchen"}},
    {"action": "pick_up", "args": {"object_name": "mug"}}
]
```

## Mock LLM for the Textbook

As discussed in the Research Outcomes, we will use a **mock LLM** implementation for this textbook. This avoids the complexity of setting up API keys, managing costs, and dealing with potential latency issues of real LLM APIs. The focus remains on the *integration* of the LLM's output with the robot's systems.

Our mock LLM will be a Python function that takes a text command and returns a predefined sequence of actions based on simple string matching.

```python
# ~/ros2_ws/src/llm_planner_pkg/llm_planner_pkg/mock_llm_planner.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.srv import GetActionPlan # Our custom service from contracts.md
import json

class MockLLMPlanner(Node):

    def __init__(self):
        super().__init__('mock_llm_planner')
        self.srv = self.create_service(GetActionPlan, 'get_action_plan', self.get_action_plan_callback)
        self.get_logger().info('Mock LLM Planner Service Ready.')

    def get_action_plan_callback(self, request, response):
        self.get_logger().info(f'Received command: "{request.command}"')
        
        action_plan = []

        if "go to kitchen and grab the mug" in request.command.lower():
            action_plan.append("navigate_to(kitchen)")
            action_plan.append("pick_up(mug)")
        elif "go to bedroom" in request.command.lower():
            action_plan.append("navigate_to(bedroom)")
        elif "tell me a joke" in request.command.lower():
            action_plan.append("say(Why did the robot cross the road? To get to the other bytes!)")
        else:
            action_plan.append("say(I'm sorry, I don't understand that command.)")
        
        response.action_plan = action_plan
        self.get_logger().info(f'Responding with plan: {action_plan}')
        return response

def main(args=None):
    rclpy.init(args=args)
    mock_llm_planner = MockLLMPlanner()
    rclpy.spin(mock_llm_planner)
    mock_llm_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Setup and Running

1.  **Define Custom Service**: Ensure you have defined the `custom_interfaces/srv/GetActionPlan.srv` as specified in `contracts.md`.
2.  **Create ROS 2 package**:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python llm_planner_pkg --dependencies rclpy custom_interfaces
    ```
3.  **Place code**: Put the `mock_llm_planner.py` file into `~/ros2_ws/src/llm_planner_pkg/llm_planner_pkg/`.
4.  **Update `setup.py`**: Add an entry point for your node in `~/ros2_ws/src/llm_planner_pkg/setup.py` under `entry_points`:
    ```python
    entry_points={
        'console_scripts': [
            'mock_llm_planner = llm_planner_pkg.mock_llm_planner:main',
        ],
    },
    ```
5.  **Build**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
6.  **Run**:
    ```bash
    ros2 run llm_planner_pkg mock_llm_planner
    ```
7.  **Test the Service**: In another terminal, call the service:
    ```bash
    ros2 service call /get_action_plan custom_interfaces/srv/GetActionPlan "command: 'Go to the kitchen and grab the mug.'"
    ```

This mock planner acts as a stand-in for a powerful LLM, demonstrating how a robot can receive a natural language command and internally generate a structured plan. In the next chapter, we'll build the system that executes these plans.
