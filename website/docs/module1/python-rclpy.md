---
sidebar_position: 4
title: Python Integration with rclpy
---

# Python Integration with rclpy

The `rclpy` library is the Python client library for ROS 2, providing the Python API to interact with ROS 2 concepts like nodes, topics, services, parameters, and actions.

## Publisher Example

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

## Subscriber Example

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

## Service Client/Server Implementation

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

## Parameters

ROS 2 allows nodes to have configurable parameters that can be changed at runtime:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('sensor_enabled', True)
        
        # Get parameter values
        frequency = self.get_parameter('frequency').value
        robot_name = self.get_parameter('robot_name').value
        sensor_enabled = self.get_parameter('sensor_enabled').value
        
        self.get_logger().info(f'Frequency: {frequency}, Robot: {robot_name}, Sensor: {sensor_enabled}')
        
        # Set callback for parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)
    
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'frequency' and param.type_ == Parameter.Type.DOUBLE:
                self.get_logger().info(f'Frequency changed to: {param.value}')
        return SetParametersResult(successful=True)
```

## Advanced rclpy Concepts

### Timers

Timers are used to execute callbacks at regular intervals:

```python
import rclpy
from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        
        # Create a timer that calls the callback every 500ms
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback executed: {self.counter}')
        self.counter += 1
```

### Clock and Time

ROS 2 provides sophisticated time management capabilities:

```python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg

class TimeNode(Node):
    def __init__(self):
        super().__init__('time_node')
        
        # Get current ROS time
        current_time = self.get_clock().now()
        self.get_logger().info(f'Current ROS time: {current_time}')
        
        # Create a timer with a specific time period
        self.timer = self.create_timer(1.0, self.time_callback)
    
    def time_callback(self):
        # Get time since node started
        time_since_start = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'Seconds since start: {time_since_start:.2f}')
```

### Waitables

Waitables allow custom synchronization mechanisms:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import Future

class WaitableNode(Node):
    def __init__(self):
        super().__init__('waitable_node')
        
        # Create a future that will be completed later
        self.future = Future()
        
        # Add future to the executor
        self.add_waitable(self.future)
        
        # Schedule completion of the future
        self.create_timer(2.0, self.complete_future)
    
    def complete_future(self):
        self.future.set_result('Future completed!')
        self.get_logger().info('Future has been completed')
```

## Lifecycle Nodes

For more robust systems, especially in humanoid robotics, lifecycle nodes provide better state management:

```python
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

class LifecycleExampleNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_example_node')
        self.get_logger().info('Lifecycle node created')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring node')
        # Initialize resources here
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating node')
        # Activate components here
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating node')
        # Deactivate components here
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up node')
        # Clean up resources here
        return TransitionCallbackReturn.SUCCESS
```

## Message and Service Customization

### Creating Custom Messages

To create custom messages, define them in a `.msg` file:

```
# In msg/JointState.msg
string name
float64 position
float64 velocity
float64 effort
```

Then use them in Python:

```python
from my_robot_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
    
    def publish_joint_state(self, name, pos, vel, effort):
        msg = JointState()
        msg.name = name
        msg.position = pos
        msg.velocity = vel
        msg.effort = effort
        self.publisher.publish(msg)
```

### Custom Services

Define services in `.srv` files:

```
# In srv/MoveJoint.srv
string joint_name
float64 target_position
---
bool success
string message
```

Use them in Python:

```python
from my_robot_msgs.srv import MoveJoint

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.srv = self.create_service(MoveJoint, 'move_joint', self.move_joint_callback)
    
    def move_joint_callback(self, request, response):
        # Perform joint movement
        success = self.move_single_joint(request.joint_name, request.target_position)
        
        response.success = success
        if success:
            response.message = f'Successfully moved {request.joint_name} to {request.target_position}'
        else:
            response.message = f'Failed to move {request.joint_name}'
        
        return response
```

## Threading and Concurrency

rclpy provides several execution models for handling concurrency:

### Single-threaded Executor

```python
import rclpy
from rclpy.executors import SingleThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    
    node1 = MinimalPublisher()
    node2 = MinimalSubscriber()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
```

### Multi-threaded Executor

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    
    node1 = MinimalPublisher()
    node2 = MinimalSubscriber()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node1)
    executor.add_node(node2)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
```

## Error Handling and Best Practices

### Exception Handling

```python
import rclpy
from rclpy.node import Node

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        try:
            self.setup_components()
        except Exception as e:
            self.get_logger().error(f'Failed to initialize: {e}')
            raise
    
    def setup_components(self):
        # Setup code that might fail
        pass
    
    def safe_publish(self, publisher, msg):
        try:
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish message: {e}')
```

### Resource Management

```python
import rclpy
from rclpy.node import Node

class ResourceManagedNode(Node):
    def __init__(self):
        super().__init__('resource_managed_node')
        
        # Initialize resources
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Register cleanup function
        import atexit
        atexit.register(self.cleanup)
    
    def cleanup(self):
        """Cleanup function to release resources"""
        self.get_logger().info('Cleaning up resources...')
        # Close files, disconnect from hardware, etc.
    
    def destroy_node(self):
        """Override to ensure cleanup"""
        self.cleanup()
        super().destroy_node()
```

## Python-Specific Considerations

### Memory Management

When working with large data (like images or point clouds), be mindful of memory usage:

```python
import rclpy
from sensor_msgs.msg import Image
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
    def image_callback(self, msg):
        # Convert image without unnecessary copying
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        
        # Process image
        processed_img = self.process_image(img)
        
        # Ensure we don't hold references to large data longer than necessary
        del img  # Explicitly delete if needed
```

### Type Hints for Better Code

Using type hints improves code readability and helps with debugging:

```python
from typing import Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.timer import Timer
from rclpy.qos import QoSProfile

class TypedNode(Node):
    def __init__(self):
        super().__init__('typed_node')
        self.publisher: Optional[rclpy.publisher.Publisher] = None
        self.timer: Optional[Timer] = None
        self.qos_profile: QoSProfile = QoSProfile(depth=10)
        
        self.setup_components()
    
    def setup_components(self) -> None:
        self.publisher = self.create_publisher(String, 'topic', self.qos_profile)
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self) -> None:
        if self.publisher:
            msg = String()
            msg.data = 'Hello with types!'
            self.publisher.publish(msg)
```

The rclpy library provides a powerful and Pythonic way to develop ROS 2 applications. Its integration with Python's ecosystem allows for rapid development and prototyping, which is particularly valuable in the complex domain of humanoid robotics where rapid iteration is often necessary.