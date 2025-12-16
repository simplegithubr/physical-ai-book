---
sidebar_position: 3
title: "Python Integration with Robotics"
---

# Python Integration with Robotics: Bridging AI and Control

This chapter explores how to integrate Python with ROS 2 using rclpy, the Python client library for ROS 2. Python is the dominant language in AI and machine learning, making it crucial for creating intelligent robotic systems.

## Why Python for Robotics?

Python has become the de facto standard for AI and machine learning applications. Integrating Python with ROS 2 allows you to:
- Leverage powerful AI libraries (TensorFlow, PyTorch, scikit-learn)
- Rapidly prototype robotic applications
- Implement complex algorithms with minimal code
- Interface with computer vision and perception systems

## Introduction to rclpy

**rclpy** is the Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl) and enables Python developers to create ROS 2 nodes.

### Installation and Setup

```bash
# rclpy comes pre-installed with ROS 2
# Verify installation
python3 -c "import rclpy; print('rclpy available')"
```

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class PythonRobotNode(Node):
    def __init__(self):
        super().__init__('python_robot_node')
        self.get_logger().info('Python Robot Node initialized')

def main(args=None):
    rclpy.init(args=args)

    node = PythonRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Control Nodes

### Publisher Node Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publisher
        self.publisher = self.create_publisher(Float32, 'sensor_data', 10)

        # Create timer for periodic publishing
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

        # Initialize sensor simulation variables
        self.time_offset = 0.0

        self.get_logger().info('Sensor publisher started')

    def publish_sensor_data(self):
        msg = Float32()

        # Simulate sensor data (e.g., distance sensor with sine wave pattern)
        sensor_value = 1.0 + 0.5 * math.sin(self.time_offset)
        msg.data = sensor_value

        self.publisher.publish(msg)
        self.get_logger().info(f'Published sensor data: {msg.data:.3f}')

        self.time_offset += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Sensor publisher stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class ControlSubscriber(Node):
    def __init__(self):
        super().__init__('control_subscriber')

        # Create subscriber
        self.subscription = self.create_subscription(
            Float32,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Initialize control variables
        self.recent_values = []
        self.control_output = 0.0

        self.get_logger().info('Control subscriber initialized')

    def sensor_callback(self, msg):
        # Store recent values for filtering
        self.recent_values.append(msg.data)
        if len(self.recent_values) > 10:
            self.recent_values.pop(0)

        # Simple moving average filter
        if len(self.recent_values) > 0:
            avg_value = sum(self.recent_values) / len(self.recent_values)

            # Implement control logic
            self.control_output = self.compute_control(avg_value)

            self.get_logger().info(
                f'Sensor: {msg.data:.3f}, Filtered: {avg_value:.3f}, Control: {self.control_output:.3f}'
            )

    def compute_control(self, sensor_value):
        # Simple proportional controller
        target_value = 1.0
        error = target_value - sensor_value
        control_gain = 2.0
        return control_gain * error

def main(args=None):
    rclpy.init(args=args)
    node = ControlSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Control subscriber stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Event Loops and Callbacks

Understanding event loops and callbacks is crucial for creating responsive robotic applications.

### Understanding the ROS 2 Event Loop

The ROS 2 event loop handles message processing, service calls, and timer callbacks. When you call `rclpy.spin()`, it enters a loop that:

1. Checks for incoming messages
2. Executes callbacks for received messages
3. Runs timer callbacks at specified intervals
4. Processes service requests
5. Handles parameter updates

### Advanced Callback Patterns

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
import threading
import time

class AdvancedCallbackNode(Node):
    def __init__(self):
        super().__init__('advanced_callback_node')

        # Multiple subscribers
        self.sensor_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_callback, 10)

        self.command_sub = self.create_subscription(
            String, 'robot_command', self.command_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.velocity_pub = self.create_publisher(Float32, 'motor_velocity', 10)

        # Timer for periodic tasks
        self.timer = self.create_timer(0.5, self.periodic_task)

        # Shared data with thread safety
        self.lock = threading.Lock()
        self.laser_data = None
        self.last_command = "idle"
        self.is_running = True

        self.get_logger().info('Advanced callback node initialized')

    def laser_callback(self, msg):
        """Handle laser scan data"""
        with self.lock:
            self.laser_data = {
                'ranges': msg.ranges,
                'min_range': msg.range_min,
                'max_range': msg.range_max,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max
            }

        # Process laser data immediately
        self.process_obstacle_detection()

    def command_callback(self, msg):
        """Handle robot commands"""
        with self.lock:
            self.last_command = msg.data
            self.get_logger().info(f'Received command: {msg.data}')

        # Execute command-specific logic
        self.execute_command(msg.data)

    def periodic_task(self):
        """Periodic task executed by timer"""
        with self.lock:
            status_msg = String()
            status_msg.data = f'Running - Command: {self.last_command}'
            self.status_pub.publish(status_msg)

    def process_obstacle_detection(self):
        """Process laser data for obstacle detection"""
        with self.lock:
            if self.laser_data:
                ranges = self.laser_data['ranges']
                min_distance = min(ranges) if ranges else float('inf')

                if min_distance < 1.0:  # Obstacle within 1 meter
                    self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')
                    self.avoid_obstacle()

    def execute_command(self, command):
        """Execute specific robot commands"""
        if command == 'move_forward':
            self.move_forward()
        elif command == 'turn_left':
            self.turn_left()
        elif command == 'stop':
            self.stop_robot()

    def move_forward(self):
        """Move robot forward"""
        vel_msg = Float32()
        vel_msg.data = 1.0  # Set forward velocity
        self.velocity_pub.publish(vel_msg)
        self.get_logger().info('Moving forward')

    def turn_left(self):
        """Turn robot left"""
        vel_msg = Float32()
        vel_msg.data = 0.5  # Differential velocity for turning
        self.velocity_pub.publish(vel_msg)
        self.get_logger().info('Turning left')

    def stop_robot(self):
        """Stop robot movement"""
        vel_msg = Float32()
        vel_msg.data = 0.0
        self.velocity_pub.publish(vel_msg)
        self.get_logger().info('Robot stopped')

    def avoid_obstacle(self):
        """Simple obstacle avoidance behavior"""
        vel_msg = Float32()
        vel_msg.data = -0.5  # Move backward briefly
        self.velocity_pub.publish(vel_msg)
        self.get_logger().info('Avoiding obstacle')

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedCallbackNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Advanced callback node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Python Robotics Libraries Integration

### Computer Vision with OpenCV

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscriber for camera images
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        # Create publisher for processed images
        self.processed_pub = self.create_publisher(Image, 'camera/processed', 10)

        self.get_logger().info('Vision node initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process image (example: edge detection)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Convert back to ROS message
            processed_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
            processed_msg.header = msg.header  # Preserve header

            # Publish processed image
            self.processed_pub.publish(processed_msg)

            self.get_logger().info('Processed image and published result')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Vision node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Machine Learning Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import numpy as np
from sklearn.linear_model import LinearRegression

class MLControlNode(Node):
    def __init__(self):
        super().__init__('ml_control_node')

        # Create subscriber for sensor data
        self.sensor_sub = self.create_subscription(
            Float32MultiArray, 'sensor_array', self.sensor_callback, 10)

        # Create publisher for control commands
        self.control_pub = self.create_publisher(String, 'ml_control_output', 10)

        # Initialize ML model
        self.model = LinearRegression()
        self.is_trained = False
        self.training_data = []

        self.get_logger().info('ML control node initialized')

    def sensor_callback(self, msg):
        sensor_values = np.array(msg.data).reshape(1, -1)

        if self.is_trained:
            # Use trained model to predict action
            prediction = self.model.predict(sensor_values)
            action = self.determine_action(prediction[0])

            control_msg = String()
            control_msg.data = action
            self.control_pub.publish(control_msg)

            self.get_logger().info(f'Predicted action: {action}')
        else:
            # Collect training data (in real application, you'd have a training phase)
            self.collect_training_data(sensor_values)

    def determine_action(self, prediction_value):
        """Map prediction to robot action"""
        if prediction_value > 0.7:
            return "move_forward"
        elif prediction_value > 0.3:
            return "turn_right"
        elif prediction_value > -0.3:
            return "turn_left"
        else:
            return "stop"

    def collect_training_data(self, sensor_values):
        """Collect data for training (simplified example)"""
        # In a real scenario, you'd collect labeled data
        # This is just a placeholder for the concept
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MLControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ML control node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling and Debugging

### Robust Error Handling

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import traceback

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')

        self.publisher = self.create_publisher(String, 'robust_topic', 10)
        self.timer = self.create_timer(1.0, self.robust_timer_callback)

        self.get_logger().info('Robust node initialized')

    def robust_timer_callback(self):
        try:
            # Perform potentially risky operation
            result = self.perform_calculation()

            msg = String()
            msg.data = f'Result: {result}'
            self.publisher.publish(msg)

        except ValueError as e:
            self.get_logger().error(f'Value error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    def perform_calculation(self):
        # Example calculation that might fail
        import random
        value = random.random()

        if value < 0.1:  # Simulate occasional failure
            raise ValueError("Simulated calculation error")

        return value * 100

def main(args=None):
    rclpy.init(args=args)
    node = RobustNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Robust node stopped by user')
    except Exception as e:
        node.get_logger().error(f'Fatal error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Considerations

### Optimizing Python Nodes

1. **Use appropriate timer rates**: Don't create timers that fire too frequently
2. **Minimize data copying**: Be mindful of large message types
3. **Use efficient data structures**: Consider numpy arrays for numerical computations
4. **Profile your code**: Use profiling tools to identify bottlenecks

### Memory Management

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import weakref

class MemoryEfficientNode(Node):
    def __init__(self):
        super().__init__('memory_efficient_node')

        self.subscription = self.create_subscription(
            String, 'input_topic', self.memory_safe_callback, 10)

        # Use weak references to avoid circular references
        self.processed_messages = []

        # Limit message history to prevent memory leaks
        self.max_history = 100

    def memory_safe_callback(self, msg):
        # Process message
        processed = self.process_message(msg)

        # Store with size limit
        self.processed_messages.append(processed)
        if len(self.processed_messages) > self.max_history:
            self.processed_messages.pop(0)  # Remove oldest

    def process_message(self, msg):
        return f"Processed: {msg.data}"
```

## Summary

Python integration with ROS 2 through rclpy enables powerful robotic applications that combine AI capabilities with robotic control systems. Key takeaways include:

1. **Event-driven architecture**: Understanding callbacks and the ROS 2 event loop
2. **Integration capabilities**: Connecting AI libraries with robotic systems
3. **Error handling**: Implementing robust error management
4. **Performance considerations**: Optimizing Python nodes for real-time applications

In the next chapter, we'll explore how to describe robots using URDF and connect these descriptions to our ROS 2 control systems.