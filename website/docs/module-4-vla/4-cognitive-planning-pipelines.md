---
title: "4. Cognitive Planning Pipelines"
sidebar_label: "Cognitive Planning Pipelines"
sidebar_position: 4
---

# 4. Cognitive Planning Pipelines

We now have the ability to translate voice commands into text using Whisper and to generate a sequence of abstract actions from an LLM (or our mock LLM). The next challenge is to build a **Cognitive Planning Pipeline** that takes this abstract action plan and translates it into concrete, executable ROS 2 commands for our robot.

This pipeline is the bridge between high-level intelligence and low-level robot execution.

## The Role of the Action Executor

The action executor is a ROS 2 node that:
1.  **Subscribes** to the `/voice_command` topic from the Whisper node.
2.  **Calls** the LLM service (our `mock_llm_planner`) to get an action plan.
3.  **Executes** each action in the plan by calling the appropriate ROS 2 service, action client, or publishing to a topic.
4.  **Provides Feedback**: In a more advanced system, it would monitor the execution of each action and provide feedback or handle failures.

## Designing Robot Skills

For the action executor to work, we need to define a set of **robot skills**. Each skill corresponds to a capability of the robot that can be invoked by the action executor. These skills wrap the underlying ROS 2 services, actions, or topic publications.

For example, if the LLM generates the action `navigate_to(kitchen)`, the action executor needs a `navigate_to` skill that:
-   Translates "kitchen" into a specific target pose for Nav2.
-   Sends a goal to the Nav2 action server (`/navigate_to_pose`).
-   Waits for the result.

Similarly, for `pick_up(mug)`, we need a `pick_up` skill that:
-   Locates the mug using perception (which we'll cover in the next chapter).
-   Plans a grasping trajectory.
-   Sends commands to the robot's arm to execute the grasp.

### Example: A Simple Action Executor

Let's build a simplified action executor that can handle the mock LLM's output. For now, we'll implement `navigate_to` and `say` using ROS 2 services and topics.

```python
# ~/ros2_ws/src/action_executor_pkg/action_executor_pkg/executor_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.srv import GetActionPlan # From our LLM planner
from rclpy.action import ActionClient # For Nav2
from nav2_msgs.action import NavigateToPose # For Nav2
from geometry_msgs.msg import PoseStamped # For Nav2 goal

import json
import time

class ActionExecutor(Node):

    def __init__(self):
        super().__init__('action_executor')
        self.get_logger().info('Action Executor Node Started.')

        # Subscribe to voice commands
        self.voice_command_subscription = self.create_subscription(
            String,
            '/voice_command',
            self.voice_command_callback,
            10
        )

        # Create client for LLM service
        self.llm_client = self.create_client(GetActionPlan, 'get_action_plan')
        while not self.llm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LLM service not available, waiting...')

        # Create client for Nav2 NavigateToPose action
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Navigation action server not available, waiting...')
        
        # Publisher for robot speech (mock for now)
        self.speech_publisher = self.create_publisher(String, '/robot_speech', 10)

        self.get_logger().info('All services and action servers are connected.')

    async def voice_command_callback(self, msg):
        self.get_logger().info(f'Received voice command: "{msg.data}"')
        
        # 1. Call LLM to get action plan
        llm_request = GetActionPlan.Request()
        llm_request.command = msg.data
        
        self.get_logger().info('Requesting action plan from LLM...')
        llm_response = await self.llm_client.call_async(llm_request)
        
        if llm_response.action_plan:
            self.get_logger().info(f'Received plan: {llm_response.action_plan}')
            # 2. Execute each action in the plan
            for action_str in llm_response.action_plan:
                self.get_logger().info(f'Executing: {action_str}')
                if action_str.startswith("navigate_to"):
                    location = action_str.split('(')[1].split(')')[0]
                    await self.execute_navigate_to(location)
                elif action_str.startswith("say"):
                    phrase = action_str.split('(')[1].split(')')[0]
                    self.execute_say(phrase)
                else:
                    self.get_logger().warn(f"Unknown action: {action_str}")
        else:
            self.get_logger().warn("LLM returned an empty plan.")

    async def execute_navigate_to(self, location):
        self.get_logger().info(f"Executing navigation to: {location}")
        # Mocking specific poses for locations (in a real system, this would come from a map)
        if location == "kitchen":
            target_x, target_y = 5.0, 0.0
        elif location == "bedroom":
            target_x, target_y = 0.0, 5.0
        else:
            self.get_logger().warn(f"Unknown location: {location}")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.w = 1.0 # No rotation

        self.get_logger().info(f"Sending navigation goal to Nav2: {target_x}, {target_y}")
        future = self.nav_action_client.send_goal_async(goal_msg)
        goal_handle = await future

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result = await result_future
        
        if result.status == ActionClient.GoalStatus.SUCCEEDED:
            self.get_logger().info(f'Navigation to {location} succeeded!')
        else:
            self.get_logger().error(f'Navigation to {location} failed with status: {result.status}')


    def execute_say(self, phrase):
        self.get_logger().info(f"Robot says: {phrase}")
        msg = String()
        msg.data = phrase
        self.speech_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    action_executor = ActionExecutor()
    # Use rclpy.spin_until_future_complete for async operations
    rclpy.spin(action_executor) 
    action_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Setup and Running

1.  **Custom Interface**: Ensure `custom_interfaces` package and `GetActionPlan.srv` are built.
2.  **Create ROS 2 package**:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python action_executor_pkg --dependencies rclpy std_msgs custom_interfaces nav2_msgs geometry_msgs
    ```
3.  **Place code**: Put `executor_node.py` into `~/ros2_ws/src/action_executor_pkg/action_executor_pkg/`.
4.  **Update `setup.py`**: Add an entry point.
5.  **Build**: `colcon build` and `source install/setup.bash`.
6.  **Run**: You'll need to run several nodes simultaneously:
    -   Your Whisper node (`ros2 run voice_command_pkg whisper_node`)
    -   Your Mock LLM Planner (`ros2 run llm_planner_pkg mock_llm_planner`)
    -   **Crucially, a running Nav2 stack** (e.g., in Isaac Sim or Gazebo)
    -   Your Action Executor (`ros2 run action_executor_pkg executor_node`)

### Testing the Pipeline

With all components running:
1.  Speak a command into your microphone, e.g., "Go to the kitchen".
2.  The Whisper node transcribes it and publishes to `/voice_command`.
3.  The Action Executor subscribes to this, calls the LLM service with the text.
4.  The Mock LLM returns `["navigate_to(kitchen)"]`.
5.  The Action Executor translates "kitchen" into a `NavigateToPose` goal and sends it to Nav2.
6.  Nav2 (in your simulation) plans and executes the movement.
7.  If you speak "Tell me a joke", the robot will "say" the joke via `/robot_speech` topic.

This complete pipeline demonstrates how a robot can receive natural language input, parse it into a plan, and execute that plan using its underlying ROS 2 capabilities. The modular design allows you to easily swap out components, for instance, replacing the mock LLM with a real one.
