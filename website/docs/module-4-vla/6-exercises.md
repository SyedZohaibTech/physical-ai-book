---
title: "6. Module 4 Exercises"
sidebar_label: "Exercises"
sidebar_position: 6
---

# 6. Module 4 Exercises

These exercises will challenge you to build and integrate the Vision-Language-Action (VLA) pipeline components we've discussed.

## Exercise 1: Customize Your Whisper Node

**Goal**: Modify the `whisper_node.py` to change its audio capture duration and the Whisper model it uses.

1.  Open `whisper_node.py` from Chapter 2.
2.  Modify the node's parameters (e.g., `audio_duration`, `whisper_model`) either directly in the code or by launching the node with `ros2 run` and passing arguments:
    ```bash
    ros2 run voice_command_pkg whisper_node --ros-args -p audio_duration:=5.0 -p whisper_model:='tiny'
    ```
3.  **Verify**: Confirm the node now records for 5 seconds (or your chosen duration) and, if you chose a smaller model like 'tiny', you might notice a slight change in transcription accuracy or speed.

## Exercise 2: Extend the Mock LLM Planner

**Goal**: Add new command-action mappings to your `mock_llm_planner.py`.

1.  Open `mock_llm_planner.py` from Chapter 3.
2.  Add at least two new natural language commands and their corresponding action sequences. For example:
    -   "Find the book and tell me where it is." -> `["say(The book is over there.)", "find_object(book)"]` (you don't have to implement `find_object` yet, just make it an action string).
    -   "Dance for me." -> `["perform_dance_routine()"]`
3.  Rebuild your `llm_planner_pkg` package.
4.  Run your `mock_llm_planner` node.
5.  **Verify**: Use `ros2 service call /get_action_plan custom_interfaces/srv/GetActionPlan "command: 'Your new command here.'"` and check if the returned `action_plan` is as expected.

## Exercise 3: Implement a Custom Action Skill

**Goal**: Extend the `ActionExecutor` to handle a new custom robot skill.

This exercise builds on Exercise 2. Let's assume you added `perform_dance_routine()` as a new action.

1.  Open `executor_node.py` from Chapter 4.
2.  Add a new `elif` block in the `voice_command_callback` loop to detect your new action.
3.  Implement a new `async` function, for example, `execute_dance_routine(self)`. For now, this function can simply print a message like `self.get_logger().info("Robot is dancing!")` and publish to your `/robot_speech` topic to say "I am dancing!". In a real robot, this would trigger a complex movement sequence.
4.  Rebuild your `action_executor_pkg` package.
5.  Run all necessary nodes: Whisper, Mock LLM, and Action Executor.
6.  **Verify**: Speak your new command ("Dance for me!") into the microphone. Check the logs of the `action_executor` node and the `/robot_speech` topic.

## Exercise 4 (Advanced): Basic Object Detection Integration

**Goal**: Simulate a basic object detection system and integrate it with your `ActionExecutor`.

1.  **Create a mock object detector node**:
    - Create a new Python ROS 2 node, say `mock_object_detector.py`.
    - This node should periodically publish a `geometry_msgs/msg/PoseStamped` message to a `/detected_objects` topic. The message's `header.frame_id` could be 'base_link', and its `pose.position` could represent a known object's location (e.g., `x=1.0, y=0.5, z=0.0`).
    - Publish a `std_msgs/msg/String` message alongside this, e.g., `/detected_object_name` with `data: "mug"`. (For simplicity, assume it only detects one object type).
2.  **Modify `ActionExecutor`**:
    - Open `executor_node.py` from Chapter 5.
    - Subscribe to the `/detected_objects` and `/detected_object_name` topics. Store the latest detected object's name and pose in internal variables.
    - Add a new action to your mock LLM, e.g., "pick up the object." -> `["pick_up_detected_object()"]`.
    - Implement `async def execute_pick_up_detected_object(self)` in `ActionExecutor`. This function should retrieve the stored detected object's pose and simulate sending a manipulation command (e.g., print a message with the pose).
3.  **Verify**:
    - Run the `mock_object_detector`, Whisper, Mock LLM, and Action Executor nodes.
    - Speak "Pick up the object."
    - Observe the `ActionExecutor` logs to see if it correctly identifies and "picks up" the object at the simulated detected pose.

---

This concludes the VLA module. These exercises demonstrate how the power of language models can be combined with visual perception and robot control to create intelligent, adaptable robotic agents. In the final capstone project, we will bring all these modules together to build a complete autonomous humanoid.
