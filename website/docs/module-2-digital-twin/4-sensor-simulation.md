---
title: '4. Simulating Sensors'
sidebar_label: 'Sensor Simulation'
sidebar_position: 4
---

# 4. Simulating Sensors: The Robot's Senses

A robot is blind and deaf without its sensors. A critical function of a digital twin is to generate realistic sensor data that our perception and navigation algorithms can consume. In this chapter, we'll explore how to add and configure common robotic sensors in our simulators.

The process is always the same:
1.  **Attach** a sensor to a link in your robot's URDF or SDF model.
2.  **Add** a plugin that tells the simulator how the sensor works and where to publish its data.
3.  **Subscribe** to the corresponding ROS 2 topic in your code to receive the simulated data.

## 1. Cameras

Cameras are the "eyes" of the robot, providing rich visual information about the world.

### Gazebo
In Gazebo, you use the `gazebo_ros_camera` plugin. You add a `<sensor>` tag to your robot description file, attach it to a link (like the robot's "head"), and configure its properties.

```xml
<gazebo reference="head_link">
  <sensor type="camera" name="head_camera_sensor">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.396</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>head_camera</namespace>
        <argument>--ros-args -r image_raw:=image_raw</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

This SDF snippet does the following:
- Creates a camera sensor and attaches it to `head_link`.
- Sets the update rate to 30 Hz.
- Defines the image properties (resolution, field of view).
- Loads the `libgazebo_ros_camera.so` plugin.
- Tells the plugin to publish the images to the `/head_camera/image_raw` topic.

Your ROS 2 perception nodes can now subscribe to `/head_camera/image_raw` to get `sensor_msgs/msg/Image` data, just as if it were a real camera.

### Unity
In Unity, you simply add a `Camera` object to your scene and attach it to your robot model. The Unity Robotics Hub provides a `ROSPublisher` script that can be configured to publish the camera's rendered image to a ROS 2 topic. This is particularly powerful because you get the benefit of Unity's high-quality rendering pipeline for your simulated camera feed.

## 2. LiDAR (Laser Scanners)

LiDAR is a crucial sensor for navigation and mapping, providing precise distance measurements.

### Gazebo
The process is very similar to the camera, but this time we use the `gazebo_ros_ray_sensor` plugin. "Ray sensor" is the generic Gazebo term for sensors that work by casting rays into the world, like LiDAR and sonar.

```xml
<gazebo reference="base_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0.1 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>20</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.57</min_angle>
          <max_angle>1.57</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>10.0</max>
      </range>
    </ray>
    <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>lidar</namespace>
        <argument>--ros-args -r scan:=scan</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```
This plugin simulates a laser scanner by casting 720 rays in a 180-degree arc and then publishes the results as a `sensor_msgs/msg/LaserScan` message to the `/lidar/scan` topic.

## 3. IMUs (Inertial Measurement Units)

An IMU is a device that measures a robot's orientation, angular velocity, and linear acceleration. It's essential for balance and estimating the robot's state.

### Gazebo
Gazebo provides the `gazebo_ros_imu_sensor` plugin.

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>imu</namespace>
        <argument>--ros-args -r out:=data</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```
This plugin simulates the noise and biases of a real IMU and publishes a `sensor_msgs/msg/Imu` message to the `/imu/data` topic.

## 4. Depth Cameras

A depth camera is a special type of camera where each pixel represents a distance instead of a color. They are extremely useful for 3D perception.

### Gazebo
You can use the same `gazebo_ros_camera` plugin as a regular camera, but you change the image format and add depth camera-specific settings. The plugin can publish multiple synchronized topics:
- `/camera/image_raw` (the color image)
- `/camera/depth/image_raw` (the depth image)
- `/camera/points` (a `PointCloud2` message generated from the depth data)

This ability to generate perfect, per-pixel depth information and point clouds is one of the great advantages of simulation.

In the next chapter, we'll put this all together to build a complete simulation of our humanoid robot that can be controlled and monitored entirely through ROS 2.
