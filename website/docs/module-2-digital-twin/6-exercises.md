---
title: '6. Module 2 Exercises'
sidebar_label: 'Exercises'
sidebar_position: 6
---

# 6. Module 2 Exercises

These exercises will help you master the fundamentals of creating and interacting with robotic simulations in Gazebo. You will need a working ROS 2 and Gazebo installation to complete them.

## Exercise 1: Build a World

**Goal**: Create a simple "world" file for Gazebo that contains a ground plane and a few basic shapes.

1.  An SDF world file is a simple XML file. Create a new file named `my_world.sdf`.
2.  Start with the basic structure:
    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="default">
        <!-- We will add objects here -->
      </world>
    </sdf>
    ```
3.  Add a ground plane. This is essential for most simulations.
    ```xml
    <include>
      <uri>model://ground_plane</uri>
    </include>
    ```
4.  Add a light source.
    ```xml
    <include>
      <uri>model://sun</uri>
    </include>
    ```
5.  Now, add a static (non-moving) box to the world. You'll need to define its `pose` (position and orientation) and its `link` with `collision` and `visual` properties.
    ```xml
    <model name="box_obstacle">
      <static>true</static>
      <pose>2.0 0.0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 1 1</size></box></geometry>
        </visual>
      </link>
    </model>
    ```
6.  **Launch it!** You can load your world directly with Gazebo:
    ```bash
    gazebo my_world.sdf
    ```
7.  **Verify**: You should see a world with a ground plane and a floating box. Try adding other shapes like spheres and cylinders.

## Exercise 2: Spawn a Robot

**Goal**: Use a ROS 2 launch file to start Gazebo and automatically spawn your robot URDF into the simulation.

1.  Make sure your robot's URDF file from Module 1 has been updated to include `<collision>` and `<inertial>` tags for each link.
2.  Create a ROS 2 launch file (e.g., `spawn_robot.launch.py`).
3.  Inside the launch file, you will need to:
    - Include the `Gazebo.launch.py` launch file from the `gazebo_ros` package.
    - Start a `robot_state_publisher` node that is configured to use your robot's URDF.
    - Start a `spawn_entity.py` node from the `gazebo_ros` package. This node will call the `/spawn_entity` service to add your robot to the simulation.
4.  **Run your launch file**:
    ```bash
    ros2 launch your_package_name spawn_robot.launch.py
    ```
5.  **Verify**: Gazebo should launch, and your robot arm model should appear in the world. Check the ROS 2 topics to see `/tf` and other topics being published.

## Exercise 3: Add a Camera Sensor

**Goal**: Add a camera to your robot's URDF and view its output in RViz.

1.  Edit your robot's URDF file (or a copy of it).
2.  Following the example in Chapter 4, add a `<sensor>` block for a camera and the `libgazebo_ros_camera.so` plugin. Attach it to one of your robot's links.
3.  Relaunch the simulation using the launch file from Exercise 2, making sure it's using your new URDF file.
4.  In a new terminal, start RViz:
    ```bash
    rviz2
    ```
5.  In RViz, click the "Add" button and choose "By topic". Find the image topic that your camera plugin is publishing (e.g., `/camera/image_raw`) and add it.
6.  **Verify**: You should see a new window in RViz showing the view from your robot's simulated camera. Move objects in front of the robot in Gazebo and see them appear in the RViz view.

## Exercise 4 (Advanced): Closed-Loop Control

**Goal**: Combine everything to control the robot in Gazebo using your own publisher node.

1.  You will need to install and configure `ros2_control` for Gazebo. This is a complex topic, and you should follow the official `gazebo_ros2_control` tutorials.
2.  Your goal is to create a launch file that starts:
    - Gazebo with your world.
    - Your robot, spawned into Gazebo.
    - The `ros2_control` plugins and controllers (`joint_state_broadcaster` and `joint_trajectory_controller`).
3.  Write a simple Python script that sends a goal to the `joint_trajectory_controller`'s action server (`/follow_joint_trajectory`). The goal should command the arm to move to a specific position.
4.  **Verify**: When you run your Python script, you should see the robot arm move to the commanded position in Gazebo.

---

These exercises provide a glimpse into the powerful capabilities of modern robotics simulation. Having a robust digital twin is an essential skill for any robotics engineer. In the next module, we will begin to give our simulated robot a true AI brain.
