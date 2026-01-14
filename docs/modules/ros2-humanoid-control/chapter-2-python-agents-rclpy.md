---
sidebar_position: 3
title: 'Chapter 2: Python Agents with rclpy'
description: 'Writing ROS 2 nodes in Python and bridging AI logic to robot controllers'
---

# Chapter 2: Python Agents with rclpy

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2, providing Python bindings for the ROS 2 middleware. It allows Python developers to create ROS 2 nodes, publish and subscribe to topics, and make service calls, making it an excellent choice for AI engineers who prefer Python for developing intelligent agents.

Python's simplicity and rich ecosystem of AI libraries make `rclpy` a powerful tool for bridging AI algorithms to robot control systems.

## Setting Up Your First rclpy Node

To create a basic ROS 2 node in Python, you'll need to import the necessary modules and implement the basic node structure:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Publishers and Subscribers

### Publishers

Publishers send data to topics. Here's an example of creating a publisher that sends sensor data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)

        # Simulate sensor readings every 0.1 seconds
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        self.sensor_value = 0.0

    def publish_sensor_data(self):
        msg = Float32()
        # In a real system, this would come from an actual sensor
        msg.data = self.sensor_value
        self.publisher_.publish(msg)
        self.sensor_value += 0.1  # Simulate changing sensor value
```

### Subscribers

Subscribers receive data from topics. Here's an example of creating a subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Services in Python

Services provide synchronous request-reply communication. Here's how to implement a service server and client:

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
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Bridging AI Logic to Robot Controllers

One of the key benefits of ROS 2 is its ability to connect AI algorithms with robot control systems. Here's an example of how to bridge AI decision-making with robot control:

### AI Agent Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class AIAgent(Node):
    """
    An AI agent that makes decisions based on sensor input
    and sends commands to the robot controller
    """

    def __init__(self):
        super().__init__('ai_agent')

        # Subscribe to sensor data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_input',
            self.sensor_callback,
            10)

        # Publish commands to robot controller
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.current_sensor_data = None

    def sensor_callback(self, msg):
        """Process incoming sensor data"""
        self.current_sensor_data = msg.data
        self.make_decision()

    def make_decision(self):
        """AI logic to determine robot action"""
        if self.current_sensor_data:
            cmd_msg = Twist()

            # Simple AI logic: if sensor detects obstacle, stop or turn
            if 'obstacle' in self.current_sensor_data.lower():
                cmd_msg.linear.x = 0.0  # Stop linear motion
                cmd_msg.angular.z = 0.5  # Turn right
            else:
                cmd_msg.linear.x = 0.5  # Move forward
                cmd_msg.angular.z = 0.0  # No turning

            # Send command to robot controller
            self.cmd_publisher.publish(cmd_msg)
            self.get_logger().info(f'Sent command: linear={cmd_msg.linear.x}, angular={cmd_msg.angular.z}')
```

## Working with Parameters

ROS 2 nodes can accept parameters that can be configured at runtime:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('safety_distance', 0.5)

        # Access parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value

        self.get_logger().info(f'Max speed: {self.max_speed}, Safety distance: {self.safety_distance}')
```

## Best Practices for Python ROS 2 Development

### 1. Error Handling

Always include proper error handling in your nodes:

```python
def timer_callback(self):
    try:
        # Your node logic here
        processed_data = self.process_sensor_data()
        self.publisher_.publish(processed_data)
    except Exception as e:
        self.get_logger().error(f'Error in timer callback: {str(e)}')
```

### 2. Resource Cleanup

Ensure proper cleanup of resources:

```python
def destroy_node(self):
    # Clean up timers, publishers, subscribers
    if hasattr(self, 'timer') and self.timer is not None:
        self.timer.cancel()
    super().destroy_node()
```

### 3. Threading Considerations

Be aware of threading issues in ROS 2 Python nodes:

```python
# For CPU-intensive tasks, consider using a separate thread
import threading
from rclpy.executors import MultiThreadedExecutor

def cpu_intensive_task(self):
    # Move heavy computation to a separate thread
    thread = threading.Thread(target=self.heavy_computation)
    thread.start()
```

## Integrating with AI Libraries

Python's rich AI ecosystem integrates seamlessly with `rclpy`. Here's an example using TensorFlow for perception:

```python
import rclpy
from rclpy.node import Node
import tensorflow as tf
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

class AIPerceptionNode(Node):

    def __init__(self):
        super().__init__('ai_perception_node')

        # Load pre-trained model
        self.model = tf.keras.models.load_model('/path/to/model')

        # Subscribe to camera data
        self.image_subscription = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            10)

        # Publish recognition results
        self.result_publisher = self.create_publisher(String, 'recognition_result', 10)

    def image_callback(self, msg):
        # Convert ROS image message to format for TensorFlow
        image_data = self.convert_ros_image_to_numpy(msg)

        # Run inference
        prediction = self.model.predict(np.expand_dims(image_data, axis=0))

        # Publish result
        result_msg = String()
        result_msg.data = f'Detected object: {prediction}'
        self.result_publisher.publish(result_msg)
```

## Summary

In this chapter, we've explored:

- Creating ROS 2 nodes using the `rclpy` Python client library
- Implementing publishers and subscribers for asynchronous communication
- Setting up services for synchronous request-reply communication
- Bridging AI logic to robot controllers using ROS 2 messaging
- Working with parameters for configurable behavior
- Best practices for developing robust Python nodes
- Integrating AI libraries with ROS 2 systems

The combination of Python's ease of use and ROS 2's distributed architecture makes `rclpy` an excellent choice for implementing AI agents that interact with robotic systems.

## Learning Objectives Review

After completing this chapter, you should be able to:
- Create ROS 2 nodes in Python using rclpy
- Implement publishers, subscribers, and services in Python
- Bridge AI logic to robot controllers using ROS 2 messaging
- Apply best practices for Python ROS 2 development
- Integrate AI libraries with ROS 2 systems

## Previous and Next Steps

Previous: [Chapter 1: ROS 2 Fundamentals](./chapter-1-ros2-fundamentals.md)

Continue to [Chapter 3: Humanoid Modeling with URDF](./chapter-3-urdf-modeling.md) to learn about Unified Robot Description Format and its role in ROS 2.