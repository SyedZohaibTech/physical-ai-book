---
title: Appendix
sidebar_position: 101
---

# Appendix: Quick Reference Guides

This appendix provides quick reference guides and supplementary information to assist you in your robotics development journey.

## A. ROS 2 Cheatsheet

### Common `ros2` Commands

| Command                     | Description                                                                 |
| :-------------------------- | :-------------------------------------------------------------------------- |
| `ros2 run <pkg> <executable>` | Runs an executable from a ROS 2 package.                                    |
| `ros2 launch <pkg> <launch_file>` | Starts a launch file.                                                     |
| `ros2 topic list`           | Lists all active ROS 2 topics.                                              |
| `ros2 topic echo <topic_name>` | Displays messages being published on a topic.                               |
| `ros2 topic pub <topic_name> <msg_type> <args>` | Publishes data to a topic from the command line.          |
| `ros2 service list`         | Lists all active ROS 2 services.                                            |
| `ros2 service call <service_name> <srv_type> <args>` | Calls a service from the command line.                |
| `ros2 node list`            | Lists all active ROS 2 nodes.                                               |
| `ros2 node info <node_name>` | Displays information about a node (pubs, subs, services, actions, params).  |
| `ros2 param list`           | Lists all parameters on all nodes.                                          |
| `ros2 param get <node> <param>` | Gets the value of a parameter.                                              |
| `ros2 pkg list`             | Lists all installed ROS 2 packages.                                         |
| `ros2 interface show <msg/srv/action_type>` | Shows the definition of a message, service, or action type. |

### Build System (`colcon`)

| Command                     | Description                                                                 |
| :-------------------------- | :-------------------------------------------------------------------------- |
| `colcon build`              | Builds all packages in the current workspace.                               |
| `colcon build --packages-select <pkg_name>` | Builds a specific package.                                  |
| `source install/setup.bash` | Sources the workspace to make executables and libraries available.          |

## B. URDF Elements Cheatsheet

### Key URDF Tags

| Tag             | Description                                                                 | Child Tags (Common)                                        |
| :-------------- | :-------------------------------------------------------------------------- | :--------------------------------------------------------- |
| `<robot>`       | The root element of a URDF file.                                            | `<link>`, `<joint>`, `<material>`                          |
| `<link>`        | Defines a rigid body part of the robot.                                     | `<visual>`, `<collision>`, `<inertial>`                    |
| `<joint>`       | Defines the connection and motion between two links.                        | `<parent>`, `<child>`, `<origin>`, `<axis>`, `<limit>`     |
| `<visual>`      | Describes the visual properties of a link.                                  | `<geometry>`, `<material>`, `<origin>`                     |
| `<collision>`   | Describes the collision properties of a link.                               | `<geometry>`, `<origin>`                                   |
| `<inertial>`    | Describes the mass and inertia tensor of a link.                            | `<mass>`, `<inertia>`, `<origin>`                          |
| `<geometry>`    | Defines the shape of a link (e.g., box, cylinder, sphere, mesh).            |                                                            |
| `<material>`    | Defines the color and texture for visual elements.                          | `<color>`, `<texture>`                                     |
| `<origin>`      | Specifies the position and orientation of an element relative to its parent. | `xyz` (position), `rpy` (roll, pitch, yaw rotation)        |
| `<axis>`        | Defines the axis of rotation for revolute joints.                           | `xyz` (vector representing the axis)                       |
| `<limit>`       | Defines the physical limits of a joint (e.g., position, velocity, effort).  | `lower`, `upper`, `effort`, `velocity`                     |

## C. Python `rclpy` Snippets

### Minimal Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello: {self.i}'
        self.publisher_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Minimal Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Minimal Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Replace with your custom service

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Returning sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Minimal Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Replace with your custom service
import sys

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()
```

## D. Common ROS 2 Message Types

| Message Type                     | Description                                                                 | Package            |
| :------------------------------- | :-------------------------------------------------------------------------- | :----------------- |
| `std_msgs/msg/String`            | Simple text string.                                                         | `std_msgs`         |
| `std_msgs/msg/Header`            | Contains timestamp and frame ID information.                                | `std_msgs`         |
| `geometry_msgs/msg/Pose`         | Position (x,y,z) and orientation (quaternion x,y,z,w).                      | `geometry_msgs`    |
| `geometry_msgs/msg/PoseStamped`  | A `Pose` with a `Header` (timestamp and frame ID).                          | `geometry_msgs`    |
| `geometry_msgs/msg/Twist`        | Linear (x,y,z) and angular (x,y,z) velocities.                              | `geometry_msgs`    |
| `sensor_msgs/msg/Image`          | Raw or compressed image data from a camera.                                 | `sensor_msgs`      |
| `sensor_msgs/msg/CameraInfo`     | Camera calibration parameters.                                              | `sensor_msgs`      |
| `sensor_msgs/msg/LaserScan`      | Data from a 2D laser scanner (LiDAR).                                       | `sensor_msgs`      |
| `sensor_msgs/msg/PointCloud2`    | Data from a 3D point cloud sensor (e.g., depth camera, 3D LiDAR).           | `sensor_msgs`      |
| `sensor_msgs/msg/Imu`            | Data from an Inertial Measurement Unit (IMU).                               | `sensor_msgs`      |
| `sensor_msgs/msg/JointState`     | State of a robot's joints (positions, velocities, efforts).                 | `sensor_msgs`      |
| `nav_msgs/msg/Odometry`          | Robot's estimated position and velocity relative to its starting point.     | `nav_msgs`         |
| `tf2_msgs/msg/TFMessage`         | Contains multiple `geometry_msgs/msg/TransformStamped` messages for TF2.    | `tf2_msgs`         |
