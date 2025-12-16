---
sidebar_position: 2
title: "ROS 2 Foundations"
---

# ROS 2 Foundations: The Robotic Nervous System

This chapter introduces you to the core concepts of ROS 2 (Robot Operating System 2), the framework that serves as the nervous system for modern robots. You'll learn about nodes, topics, services, and how they work together to create distributed robotic applications.

## What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Unlike a traditional operating system, ROS 2 is middleware that provides services designed for a heterogeneous computer cluster:
- Hardware abstraction
- Low-level device control
- Implementation of commonly used functionality
- Message-passing between processes
- Package management

## Core Concepts

### Nodes

A **node** is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of ROS 2 applications.

```python
# Example: Simple ROS 2 Node
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Minimal node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key characteristics of nodes:**
- Each node runs in its own process
- Nodes communicate with each other through topics and services
- Multiple nodes can be run simultaneously
- Nodes can be written in different languages (C++, Python, etc.)

### Topics and Messages

**Topics** enable asynchronous communication between nodes using a publish/subscribe model. Nodes that publish data are called publishers, and nodes that receive data are called subscribers.

```python
# Publisher example
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# Subscriber example
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

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Services

**Services** provide synchronous request/response communication between nodes. Services are useful when you need to get a specific response from a particular node.

```python
# Service server example
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

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# Service client example
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
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

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Files

Launch files allow you to start multiple nodes with a single command and manage complex robotic systems more easily. They can be written in Python or XML.

### Python Launch File

```python
# launch_example.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='talker',
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener',
        ),
    ])
```

### Running Launch Files

```bash
# Create launch directory in your package
mkdir -p launch

# Run the launch file
ros2 launch your_package launch_example.py
```

## Architecture Overview

ROS 2 uses a distributed architecture where nodes can run on the same or different machines. The key architectural components are:

### DDS (Data Distribution Service)
- Provides the underlying communication mechanism
- Handles message routing between nodes
- Supports multiple DDS implementations (Fast DDS, Cyclone DDS, RTI Connext)

### Parameters
- Dynamic configuration system
- Nodes can store and retrieve configuration values
- Parameters can be set at startup or changed during runtime

```python
# Parameter example
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters
        self.declare_parameter('my_parameter', 'default_value')

        # Get parameter value
        my_param = self.get_parameter('my_parameter').value
        self.get_logger().info(f'Parameter value: {my_param}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Setting Up Your Environment

### Creating a ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### Understanding the Command Line Tools

ROS 2 provides several command-line tools for debugging and monitoring:

```bash
# List all active nodes
ros2 node list

# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Call a service
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# List parameters of a node
ros2 param list node_name
```

## Best Practices

1. **Node Design**: Keep nodes focused on a single responsibility
2. **Topic Naming**: Use descriptive, consistent naming conventions
3. **Message Types**: Use standard message types when possible
4. **Error Handling**: Always include proper error handling and logging
5. **Resource Management**: Clean up resources properly in node destruction

## Summary

ROS 2 provides the foundational infrastructure for modern robotics applications. Understanding nodes, topics, services, and launch files is essential for building distributed robotic systems. In the next chapter, we'll explore how to integrate Python with ROS 2 to create powerful control nodes for your robots.