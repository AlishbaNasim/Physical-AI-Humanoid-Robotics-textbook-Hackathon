---
sidebar_position: 5
---

# Code Examples for ROS 2 Communication Patterns

## Introduction

This section provides practical code examples demonstrating the implementation of ROS 2 communication patterns in Python. These examples will help you understand how to implement nodes, topics, services, and actions in your humanoid robotics applications.

## Node Implementation Example

Here's a basic ROS 2 node implementation in Python:

```python
import rclpy
from rclpy.node import Node

class BasicNode(Node):
    def __init__(self):
        super().__init__('basic_node')
        self.get_logger().info('Basic node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = BasicNode()

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

## Publisher Example

Here's how to implement a publisher node that sends messages to a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
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
    publisher_node = PublisherNode()

    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber Example

Here's how to implement a subscriber node that receives messages from a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = SubscriberNode()

    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Server Example

Here's how to implement a service server:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server_node = ServiceServerNode()

    try:
        rclpy.spin(service_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        service_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Client Example

Here's how to implement a service client:

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client_node = ServiceClientNode()

    try:
        response = client_node.send_request(2, 3)
        if response:
            print(f'Result: {response.sum}')
        else:
            print('Service call failed')
    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Action Server Example

Here's how to implement an action server:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class ActionServerNode(Node):
    def __init__(self):
        super().__init__('action_server_node')
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
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    action_server_node = ActionServerNode()

    try:
        rclpy.spin(action_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        action_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Action Client Example

Here's how to implement an action client:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class ActionClientNode(Node):
    def __init__(self):
        super().__init__('action_client_node')
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

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

def main(args=None):
    rclpy.init(args=args)
    action_client_node = ActionClientNode()

    try:
        action_client_node.send_goal(10)
        rclpy.spin(action_client_node)
    except KeyboardInterrupt:
        pass
    finally:
        action_client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Package Configuration

To use these examples, you'll need to configure your package appropriately. In your `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>your_package_name</name>
  <version>0.0.0</version>
  <description>Package for ROS 2 communication examples</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>example_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

And in your `setup.py`:

```python
from setuptools import setup

package_name = 'your_package_name'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Package for ROS 2 communication examples',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = your_package_name.publisher_node:main',
            'subscriber_node = your_package_name.subscriber_node:main',
            'service_server = your_package_name.service_server:main',
            'service_client = your_package_name.service_client:main',
            'action_server = your_package_name.action_server:main',
            'action_client = your_package_name.action_client:main',
        ],
    },
)
```

## Running the Examples

To run these examples:

1. Create a new ROS 2 package
2. Copy the example code into appropriately named Python files
3. Update the package.xml and setup.py files
4. Build the package: `colcon build`
5. Source the workspace: `source install/setup.bash`
6. Run the nodes: `ros2 run your_package_name node_name`

## Summary

These examples demonstrate the fundamental communication patterns in ROS 2. Understanding these patterns is essential for building complex robotic systems that require coordinated behavior between multiple components. Each pattern serves a specific purpose and choosing the right one for your use case is crucial for building robust and efficient robotic applications.