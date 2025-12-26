---
sidebar_position: 2
title: Gazebo Environment Setup and Configuration
---

# Gazebo Environment Setup and Configuration

Setting up Gazebo for humanoid robotics simulation requires careful configuration to ensure accurate physics simulation and proper integration with ROS 2. This chapter covers the complete setup process for creating effective simulation environments.

## Installing Gazebo

### System Requirements
Before installing Gazebo, ensure your system meets the requirements:
- Ubuntu 22.04 LTS (recommended)
- At least 8GB RAM (16GB recommended for complex simulations)
- Dedicated GPU with OpenGL 3.3+ support
- 20GB free disk space

### Installation Process
For ROS 2 Humble Hawksbill with Gazebo Garden:

```bash
# Install Gazebo
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Install Gazebo standalone (if needed)
sudo apt install gazebo
```

### Verification
Test the installation:
```bash
gz sim --version
```

## Gazebo World Files

World files define the simulation environment in Gazebo. Here's a basic humanoid simulation world:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Include the default sun and ground plane -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Configure physics engine -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    
    <!-- Atmosphere properties -->
    <atmosphere type="adiabatic"/>
    
    <!-- Lighting configuration -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Plugins for ROS 2 integration -->
    <plugin filename="gz-sim-ros2-control-system" name="gz::sim::systems::Ros2Control">
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
      <parameters>[/controller_manager]</parameters>
    </plugin>
    
    <!-- Plugins for state publishing -->
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <joint_name>j1</joint_name>
    </plugin>
  </world>
</sdf>
```

## Custom Environment Creation

### 1. Creating Simple Environments
Start with basic environments to test humanoid locomotion:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_humanoid">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Flat ground with appropriate friction -->
    <physics name="ode" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
    
    <!-- Add a humanoid robot model -->
    <include>
      <uri>model://my_humanoid_robot</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>
    
    <!-- Add obstacles for navigation testing -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1667</iyy>
            <iyz>0</iyz>
            <izz>0.1667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### 2. Complex Terrain Environments
For more advanced humanoid testing:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="complex_terrain">
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Custom terrain -->
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://terrain.png</uri>
              <size>100 100 10</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>file://terrain.png</uri>
              <size>100 100 10</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
    
    <!-- Stairs for testing locomotion -->
    <model name="stairs">
      <pose>5 0 0 0 0 0</pose>
      <link name="step1">
        <pose>0 0 0.1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>2 2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 2 0.2</size>
            </box>
          </geometry>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
      <link name="step2">
        <pose>0 0 0.3 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>2 2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 2 0.2</size>
            </box>
          </geometry>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <physics name="ode" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

## Physics Configuration for Humanoid Robots

### 1. Time Step Configuration
For stable humanoid simulation, use appropriate time steps:
```xml
<physics name="1ms" type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1ms time step -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
</physics>
```

### 2. Contact Parameters
Fine-tune contact parameters for realistic humanoid interactions:
```xml
<ode>
  <constraints>
    <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
    <contact_surface_layer>0.001</contact_surface_layer>
    <cfm>0.000001</cfm>  <!-- Constraint Force Mixing -->
    <erp>0.2</erp>      <!-- Error Reduction Parameter -->
  </constraints>
</ode>
```

### 3. Solver Configuration
Optimize solver parameters for humanoid dynamics:
```xml
<solver>
  <type>quick</type>
  <iters>100</iters>    <!-- More iterations for stability -->
  <sor>1.3</sor>        <!-- Successive Over-Relaxation -->
</solver>
```

## Gazebo Launch Files

Create ROS 2 launch files to automate Gazebo environment setup:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('my_humanoid_package'),
            'worlds',
            'humanoid_world.sdf'
        ),
        description='SDF world file'
    )
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            'gui': 'true'
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(
                os.path.join(
                    get_package_share_directory('my_humanoid_package'),
                    'urdf',
                    'humanoid.urdf'
                )
            ).read()
        }]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/joint_states', 'dynamixel/joint_states')]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gazebo,
        robot_state_publisher,
        joint_state_publisher
    ])
```

## Environment Optimization

### 1. Performance Tuning
For complex humanoid simulations:
- Reduce rendering quality during heavy computation
- Use simplified collision geometry where possible
- Optimize physics parameters for your specific use case
- Limit the number of active objects in the scene

### 2. Stability Considerations
For stable humanoid simulation:
- Use smaller time steps (0.001s or less)
- Increase solver iterations
- Adjust contact parameters carefully
- Ensure proper mass distribution in URDF

### 3. Realism vs Performance Trade-offs
Balance realism and performance:
- High-fidelity physics for control development
- Simplified models for AI training
- Detailed rendering for perception tasks
- Efficient models for large-scale testing

## Debugging and Visualization

### 1. Gazebo GUI Tools
Use Gazebo's built-in tools for debugging:
- Model inspector to check properties
- Force visualization to see contact forces
- Physics statistics to monitor performance
- Camera views from multiple angles

### 2. ROS 2 Integration Debugging
Monitor simulation through ROS 2 topics:
```bash
# Check robot state
ros2 topic echo /joint_states

# Monitor TF transforms
ros2 run tf2_tools view_frames

# Check sensor data
ros2 topic echo /camera/image_raw
```

## Advanced Environment Features

### 1. Dynamic Objects
Add dynamic objects for interaction testing:
- Moving platforms
- Objects that can be manipulated
- Dynamic obstacles
- Interactive elements

### 2. Weather Simulation
For outdoor humanoid testing:
- Wind effects
- Rain simulation
- Variable lighting conditions
- Environmental effects

### 3. Multi-Robot Environments
For multi-humanoid scenarios:
- Communication between robots
- Coordination tasks
- Collision avoidance
- Team behaviors

Proper environment setup is crucial for effective humanoid robot simulation. The configuration must balance computational efficiency with physical accuracy to enable realistic testing of humanoid behaviors while maintaining real-time performance for interactive development.