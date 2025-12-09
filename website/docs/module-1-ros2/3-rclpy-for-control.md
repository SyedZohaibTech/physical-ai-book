---
title: '3. Writing Your First ROS 2 Node with rclpy'
sidebar_label: 'Controlling with rclpy'
sidebar_position: 3
---

# 3. Hands-On: Writing Your First ROS 2 Node with `rclpy`

Theory is great, but robotics is all about building things that work. In this chapter, we'll write our first ROS 2 nodes using `rclpy`, the official Python client library for ROS 2. We'll create a classic "talker" (publisher) and "listener" (subscriber) to see topics in action.

## Setting Up Your ROS 2 Package

Before we write code, we need a place to put it. In ROS 2, code is organized into **packages**. Let's create one.

First, create a workspace directory to hold your packages:
```bash
# Create a new directory for your ROS 2 workspace
mkdir -p ~/ros2_ws/src

# Navigate into the source directory
cd ~/ros2_ws/src
```

Now, use the ROS 2 command-line tools to create a new package. We'll call it `py_pubsub`:
```bash
# Create a new Python package
ros2 pkg create --build-type ament_python py_pubsub --dependencies rclpy std_msgs
```
This command creates a new directory named `py_pubsub` with all the necessary files for a Python ROS 2 package, including `package.xml` and `setup.py`.

## Creating the Publisher Node (Talker)

Now let's write the code for our publisher node. This node will publish a "Hello, World" message with a counter to a topic every second.

Create a new file inside your package: `~/ros2_ws/src/py_pubsub/py_pubsub/publisher_node.py`.

```python
# ~/ros2_ws/src/py_pubsub/py_pubsub/publisher_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # Create a publisher on the 'chatter' topic
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        # Create a timer that calls the timer_callback function every 1 second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

1.  **`import`**: We import `rclpy`, the `Node` class, and the `String` message type from `std_msgs.msg`.
2.  **`MinimalPublisher` class**: Our node is defined as a class that inherits from `Node`.
3.  **`__init__`**: In the constructor, we call the parent `Node` constructor, giving our node the name `'minimal_publisher'`.
4.  **`create_publisher`**: This is the key line. We create a publisher that will send messages of type `String` on a topic named `'chatter'`. The `10` is the queue size—a quality of service (QoS) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.
5.  **`create_timer`**: We create a timer that will execute `timer_callback` every `1.0` seconds.
6.  **`timer_callback`**: This function is our main loop. It creates a `String` message, populates its `data` field, publishes it with `self.publisher_.publish(msg)`, and logs it to the console.
7.  **`main` function**: This is the standard entry point for a ROS 2 node. It initializes `rclpy`, creates an instance of our node, and then calls `rclpy.spin()`. `spin()` is what keeps the node alive and processing callbacks (like our timer).

## Creating the Subscriber Node (Listener)

Next, let's create the node that will listen to the `'chatter'` topic.

Create a new file: `~/ros2_ws/src/py_pubsub/py_pubsub/subscriber_node.py`.

```python
# ~/ros2_ws/src/py_pubsub/py_pubsub/subscriber_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # Create a subscriber on the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

1.  **`create_subscription`**: This is the subscriber equivalent of `create_publisher`. We subscribe to the `'chatter'` topic, expecting messages of type `String`.
2.  **`self.listener_callback`**: This is the crucial part. We provide a callback function that will be executed every time a message is received on the topic.
3.  **`listener_callback`**: This function receives the message object (`msg`) as its argument. Here, we simply log the contents of the message to the console.

## Building and Running the Nodes

Before you can run the nodes, you need to:
1.  **Add entry points** to your `setup.py` file so ROS 2 knows about your executables.
2.  **Build** your workspace.
3.  **Source** your workspace to make the new executables available.

After following the steps in the official ROS 2 documentation for setting up your package, you can finally run the nodes.

Open two separate terminals. In both, make sure you have sourced your ROS 2 setup files (`source /opt/ros/humble/setup.bash` and `source ~/ros2_ws/install/setup.bash`).

**In Terminal 1, run the talker:**
```bash
ros2 run py_pubsub publisher_node
# Expected output:
# [INFO] [minimal_publisher]: Publishing: "Hello World: 0"
# [INFO] [minimal_publisher]: Publishing: "Hello World: 1"
# ...
```

**In Terminal 2, run the listener:**
```bash
ros2 run py_pubsub subscriber_node
# Expected output:
# [INFO] [minimal_subscriber]: I heard: "Hello World: 0"
# [INFO] [minimal_subscriber]: I heard: "Hello World: 1"
# ...
```

Congratulations! You have just created your first distributed robotic application with ROS 2. The publisher and subscriber are two completely separate programs communicating anonymously over a shared topic. This is the fundamental pattern you will use to build increasingly complex systems throughout this textbook.
