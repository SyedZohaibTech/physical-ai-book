---
title: '6. Module 1 Exercises'
sidebar_label: 'Exercises'
sidebar_position: 6
---

# 6. Module 1 Exercises

It's time to put your new knowledge into practice. These exercises are designed to solidify your understanding of the core ROS 2 concepts covered in this module.

## Exercise 1: The Personalized Talker

**Goal**: Modify the `publisher_node.py` (the "talker") to publish a message with your own name.

1.  Open the `publisher_node.py` file you created in Chapter 3.
2.  In the `timer_callback` function, change the `msg.data` line to include your name. For example:
    ```python
    msg.data = f'Hello from [Your Name]! Count: {self.i}'
    ```
3.  Re-build your workspace:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```
4.  Source the workspace and run your talker and the original listener.
    ```bash
    # Terminal 1
    source ~/ros2_ws/install/setup.bash
    ros2 run py_pubsub publisher_node

    # Terminal 2
    source ~/ros2_ws/install/setup.bash
    ros2 run py_pubsub subscriber_node
    ```
5.  **Verify**: Confirm that the listener node's output now shows your personalized message.

## Exercise 2: The Loud Listener

**Goal**: Modify the `subscriber_node.py` (the "listener") to "shout" the message it receives by converting it to uppercase.

1.  Open the `subscriber_node.py` file.
2.  In the `listener_callback` function, modify the `get_logger().info()` call to print the uppercase version of the received message data.
    - **Hint**: In Python, you can use the `.upper()` string method.
3.  Re-build, source, and run both nodes.
4.  **Verify**: Check the output of your listener node. It should now be printing the "Hello World" message in all capital letters.

## Exercise 3: A Simple Service

**Goal**: Create a new ROS 2 package and implement a simple "add two integers" service. This is a classic ROS 2 tutorial that is excellent for understanding services.

1.  Create a new package named `py_simple_service`.
2.  You will need a custom service definition. Create a new directory `srv` inside a new directory `custom_interfaces` (at the workspace level, `~/ros2_ws/src/`) and add a file named `AddTwoInts.srv` with the following content:
    ```c
    int64 a
    int64 b
    ---
    int64 sum
    ```
3.  Write a **service server node** that:
    - Creates a service named `add_two_ints`.
    - Its callback function takes a request, adds `a` and `b`, and returns the `sum` in the response.
4.  Write a **service client node** that:
    - Calls the `add_two_ints` service with two numbers (e.g., 5 and 10).
    - Logs the response it gets back from the server.
5.  **Challenge**: Can you make the client node take the two numbers as command-line arguments?

**Reference**: The official ROS 2 documentation has a detailed tutorial on writing services in Python that will be very helpful for this exercise.

## Exercise 4: The Waving Robot

**Goal**: Modify the `joint_publisher_node.py` to make the robot arm "wave" by moving only the elbow joint.

1.  Open the `joint_publisher_node.py` file from Chapter 5.
2.  In the `timer_callback`, modify the angle calculations.
    - Set the `shoulder_joint` to a fixed position (e.g., `0.0` radians).
    - Keep the sine wave calculation for the `elbow_joint`.
3.  Build and run your node along with `robot_state_publisher` and RViz.
4.  **Verify**: You should see the robot's upper arm stay still while the forearm moves back and forth, creating a waving motion.

---

Completing these exercises will give you a strong, practical foundation in ROS 2. In the next module, we'll take these concepts and apply them to a simulated robot in a virtual world.
