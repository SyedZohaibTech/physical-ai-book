---
sidebar_position: 5
title: URDF for Humanoid Robots
---

# URDF for Humanoid Robots

URDF (Unified Robot Description Format) is an XML format for representing a robot model. For humanoid robots, URDF describes the kinematic and dynamic properties of the robot's body structure.

## URDF Structure

A URDF file consists of:
- **Links**: Rigid bodies with visual and collision geometry
- **Joints**: Connections between links defining motion constraints
- **Sensors**: Camera, LIDAR, IMU specifications
- **Actuators**: Motor and transmission properties

## Simple Humanoid URDF Example
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.4 0.2 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  
  <!-- Head Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.04"/>
    </inertial>
  </link>
</robot>
```

## Complete Humanoid Robot URDF

For humanoid robots, a more complete URDF would include all major body parts:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.3"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.5 0.5 1.0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="2"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="arm_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.125"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Hand -->
  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_lower_arm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="5" velocity="1"/>
  </joint>

  <link name="left_hand">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
      <material name="hand_color">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.075"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Arm (symmetric to left) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="arm_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="2"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="arm_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.125"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_lower_arm"/>
    <child link="right_hand"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="5" velocity="1"/>
  </joint>

  <link name="right_hand">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
      <material name="hand_color"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.075"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.075 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="1"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.3 0.3 0.9 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.0072"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.0" effort="50" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="leg_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="foot_color">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right Leg (symmetric to left) -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.075 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="1"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="leg_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.0072"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.0" effort="50" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="leg_color"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="foot_color"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>
</robot>
```

## Xacro for Complex Humanoid URDFs

Xacro (XML Macros) is a preprocessor for URDF that allows for more maintainable and readable robot descriptions:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.6" />
  <xacro:property name="torso_width" value="0.3" />
  <xacro:property name="torso_depth" value="0.2" />

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Macro for a simple joint -->
  <xacro:macro name="simple_joint" params="name type parent child origin_xyz axis_xyz lower upper">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${origin_xyz}" rpy="0 0 0"/>
      <axis xyz="${axis_xyz}"/>
      <limit lower="${lower}" upper="${upper}" effort="100" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <!-- Macro for a simple link -->
  <xacro:macro name="simple_link" params="name mass geometry_type *geometry_origin *geometry *inertia">
    <link name="${name}">
      <visual>
        <xacro:insert_block name="geometry_origin"/>
        <xacro:insert_block name="geometry"/>
        <material name="white"/>
      </visual>
      <collision>
        <xacro:insert_block name="geometry_origin"/>
        <xacro:insert_block name="geometry"/>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <xacro:insert_block name="inertia"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Torso -->
  <simple_link name="torso" mass="5.0">
    <origin xyz="0 0 0"/>
    <geometry>
      <box size="${torso_width} ${torso_depth} ${torso_height}"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
  </simple_link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <simple_joint name="neck_joint" type="revolute" 
                parent="torso" child="head" 
                origin_xyz="0 0 ${torso_height/2}" 
                axis_xyz="0 1 0" 
                lower="${-M_PI/3}" upper="${M_PI/3}"/>

  <simple_link name="head" mass="1.0">
    <origin xyz="0 0 0"/>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
  </simple_link>

  <!-- Function to create arms -->
  <xacro:macro name="arm" params="side shoulder_pos shoulder_axis elbow_axis wrist_axis">
    <!-- Shoulder -->
    <simple_joint name="${side}_shoulder_joint" type="revolute"
                  parent="torso" child="${side}_upper_arm"
                  origin_xyz="${shoulder_pos}" 
                  axis_xyz="${shoulder_axis}"
                  lower="${-M_PI/2}" upper="${M_PI/2}"/>

    <simple_link name="${side}_upper_arm" mass="1.0">
      <origin xyz="0 0 0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.0025"/>
    </simple_link>

    <!-- Elbow -->
    <simple_joint name="${side}_elbow_joint" type="revolute"
                  parent="${side}_upper_arm" child="${side}_lower_arm"
                  origin_xyz="0 0 0.3" 
                  axis_xyz="${elbow_axis}"
                  lower="${-M_PI/2}" upper="${M_PI/2}"/>

    <simple_link name="${side}_lower_arm" mass="0.5">
      <origin xyz="0 0 0.125"/>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </simple_link>

    <!-- Wrist -->
    <simple_joint name="${side}_wrist_joint" type="revolute"
                  parent="${side}_lower_arm" child="${side}_hand"
                  origin_xyz="0 0 0.25" 
                  axis_xyz="${wrist_axis}"
                  lower="${-M_PI/4}" upper="${M_PI/4}"/>

    <simple_link name="${side}_hand" mass="0.3">
      <origin xyz="0 0 0.075"/>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </simple_link>
  </xacro:macro>

  <!-- Create arms -->
  <xacro:arm side="left" 
             shoulder_pos="0.2 0 0.2" 
             shoulder_axis="0 1 0" 
             elbow_axis="0 0 1" 
             wrist_axis="0 1 0"/>
             
  <xacro:arm side="right" 
             shoulder_pos="-0.2 0 0.2" 
             shoulder_axis="0 1 0" 
             elbow_axis="0 0 1" 
             wrist_axis="0 1 0"/>

  <!-- Function to create legs -->
  <xacro:macro name="leg" params="side hip_pos knee_axis ankle_axis">
    <!-- Hip -->
    <simple_joint name="${side}_hip_joint" type="revolute"
                  parent="torso" child="${side}_upper_leg"
                  origin_xyz="${hip_pos}" 
                  axis_xyz="0 0 1"
                  lower="${-M_PI/3}" upper="${M_PI/3}"/>

    <simple_link name="${side}_upper_leg" mass="2.0">
      <origin xyz="0 0 -0.2"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.0072"/>
    </simple_link>

    <!-- Knee -->
    <simple_joint name="${side}_knee_joint" type="revolute"
                  parent="${side}_upper_leg" child="${side}_lower_leg"
                  origin_xyz="0 0 -0.4" 
                  axis_xyz="${knee_axis}"
                  lower="0" upper="${M_PI/2}"/>

    <simple_link name="${side}_lower_leg" mass="1.5">
      <origin xyz="0 0 -0.2"/>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.005"/>
    </simple_link>

    <!-- Ankle -->
    <simple_joint name="${side}_ankle_joint" type="revolute"
                  parent="${side}_lower_leg" child="${side}_foot"
                  origin_xyz="0 0 -0.4" 
                  axis_xyz="${ankle_axis}"
                  lower="${-M_PI/6}" upper="${M_PI/6}"/>

    <simple_link name="${side}_foot" mass="0.5">
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </simple_link>
  </xacro:macro>

  <!-- Create legs -->
  <xacro:leg side="left" 
             hip_pos="0.075 0 0" 
             knee_axis="0 0 1" 
             ankle_axis="0 1 0"/>
             
  <xacro:leg side="right" 
             hip_pos="-0.075 0 0" 
             knee_axis="0 0 1" 
             ankle_axis="0 1 0"/>

</robot>
```

## Visualization in RViz

To visualize your humanoid URDF in RViz:

1. Launch RViz:
```bash
ros2 run rviz2 rviz2
```

2. Add a RobotModel display and set the Robot Description parameter to your URDF parameter name.

3. Make sure your URDF is loaded as a parameter in your ROS 2 system:
```bash
ros2 param set /robot_state_publisher robot_description "$(cat your_humanoid.urdf)"
```

## URDF Best Practices for Humanoid Robots

### 1. Accurate Inertial Properties
For humanoid robots, accurate inertial properties are crucial for:
- Balance control algorithms
- Physics simulation
- Collision detection

### 2. Proper Joint Limits
Set realistic joint limits based on:
- Physical constraints of actuators
- Safe operational ranges
- Human-like motion constraints

### 3. Consistent Naming Convention
Use a consistent naming scheme for links and joints:
- `left_arm_upper`, `left_arm_lower` for arms
- `right_leg_upper`, `right_leg_lower` for legs
- `torso`, `head`, etc. for main body parts

### 4. Collision Avoidance
Ensure proper collision geometry to prevent:
- Self-collision during complex movements
- Incorrect physics simulation
- Planning failures

### 5. Mass Distribution
Consider the realistic mass distribution of humanoid robots:
- Heavy torso with most mass
- Proportional limb masses
- Proper center of mass placement

## Advanced URDF Features

### Transmission Elements
For controlling actuators:

```xml
<transmission name="left_hip_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_hip_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_hip_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo Integration
For simulation in Gazebo:

```xml
<gazebo reference="left_foot">
  <mu1>0.9</mu1>
  <mu2>0.9</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <material>Gazebo/Blue</material>
</gazebo>
```

URDF is fundamental to humanoid robotics as it provides the kinematic and dynamic model needed for control, simulation, and planning. A well-structured URDF enables the development of sophisticated humanoid behaviors and ensures accurate simulation results.