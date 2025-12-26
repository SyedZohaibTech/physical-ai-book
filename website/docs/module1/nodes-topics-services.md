---
sidebar_position: 3
title: Nodes, Topics, and Services
---

# Nodes, Topics, and Services

Understanding the communication patterns in ROS 2 is fundamental to building distributed robotic systems. This chapter explores the three primary communication mechanisms.

## ROS 2 Nodes

A node is a participant in the ROS 2 graph. Nodes are the fundamental building blocks of ROS 2 applications.

### Creating a Simple Node
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node has been started!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle
In ROS 2, nodes follow a specific lifecycle that includes:
- Unconfigured state
- Inactive state
- Active state
- Finalized state

This lifecycle management is particularly important for safety-critical robotic systems like humanoid robots where components need to be properly initialized and shut down.

### Node Parameters
Nodes can accept parameters at runtime, allowing for configuration without recompilation:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('robot_name', 'my_robot')
        
        frequency = self.get_parameter('frequency').value
        robot_name = self.get_parameter('robot_name').value
        
        self.get_logger().info(f'Frequency: {frequency}, Robot: {robot_name}')
```

## Topics (Publisher/Subscriber Pattern)

Topics enable asynchronous, many-to-many communication through a publish/subscribe model.

### Publisher Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Subscriber Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### Quality of Service (QoS) Settings

QoS policies allow fine-tuning of communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Define a QoS profile
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Use it in publisher/subscriber
publisher = self.create_publisher(String, 'topic', qos_profile)
```

Common QoS settings include:
- **Reliability**: BEST_EFFORT or RELIABLE
- **Durability**: VOLATILE or TRANSIENT_LOCAL
- **History**: KEEP_LAST or KEEP_ALL
- **Depth**: Number of messages to store in history

## Services (Request/Reply Pattern)

Services provide synchronous, request/reply communication.

### Service Server
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}')
        return response
```

### Service Client
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions

Actions are used for long-running tasks with feedback and goal management.

### Action Server
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
                
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

### Action Client
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')
```

## Communication Patterns in Humanoid Robotics

### Sensor Data Distribution
In humanoid robots, sensor data is typically published on topics:
- IMU data for balance control
- Joint position/velocity/effort feedback
- Camera images for perception
- Force/torque sensor readings

### Control Commands
Control commands are often sent via services or actions:
- Trajectory execution requests
- Gait parameter adjustments
- Emergency stop commands

### Coordination Between Subsystems
Humanoid robots have multiple coordinated subsystems:
- Perception system publishing object detections
- Planning system publishing navigation goals
- Control system publishing joint commands
- Behavior system orchestrating complex behaviors

## Best Practices

### 1. Use Appropriate QoS Settings
Choose QoS settings based on your application requirements:
- Use RELIABLE for critical data
- Use BEST_EFFORT for high-frequency sensor data
- Adjust history depth based on message importance

### 2. Handle Node Lifecycle Properly
Implement proper lifecycle management for robustness:
- Initialize resources in constructors
- Clean up resources in destructors
- Handle parameter changes gracefully

### 3. Design Clear Message Interfaces
- Use standard message types when possible
- Create custom messages for domain-specific data
- Document message contents and expected ranges

### 4. Error Handling
- Implement timeout handling for service calls
- Monitor connection status
- Implement fallback behaviors

### 5. Performance Considerations
- Minimize message size for high-frequency topics
- Use appropriate publishing rates
- Consider message compression for large data (e.g., images)

## Debugging Communication Issues

### 1. Check Topic Connectivity
```bash
# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /topic_name

# Check topic info
ros2 topic info /topic_name
```

### 2. Monitor Service Calls
```bash
# List all services
ros2 service list

# Call a service from command line
ros2 service call /service_name service_type "{request_fields: values}"
```

Understanding these communication patterns is essential for building robust humanoid robots. Each pattern serves a specific purpose and choosing the right one for your use case is critical for system performance and reliability.