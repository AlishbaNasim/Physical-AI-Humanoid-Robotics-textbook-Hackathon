---
sidebar_position: 2
---

# Setting Up Python Agents with rclpy

## Introduction

Setting up Python agents with rclpy involves creating properly structured nodes that can participate in the ROS 2 ecosystem. This section covers the essential steps to create robust, well-structured Python agents for humanoid robotics applications.

## Basic Node Setup

### Creating a Basic Node

The foundation of any Python agent in ROS 2 is a properly initialized node:

```python
import rclpy
from rclpy.node import Node

class PythonAgent(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('python_agent')

        # Log initialization
        self.get_logger().info('Python Agent initialized')

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the agent node
    agent = PythonAgent()

    try:
        # Keep the node running
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Interrupted by user')
    finally:
        # Clean up
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Package Structure

A well-structured Python agent package should follow this organization:

```
my_robot_agent/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── my_robot_agent
├── my_robot_agent/
│   ├── __init__.py
│   ├── agent_node.py
│   ├── agent_logic.py
│   └── utils.py
└── test/
    └── test_agent.py
```

## Environment Setup

### Dependencies

Ensure your `package.xml` includes necessary dependencies:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>builtin_interfaces</depend>
```

### Setup Configuration

Your `setup.py` should properly configure entry points:

```python
from setuptools import setup

setup(
    name='my_robot_agent',
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/my_robot_agent']),
        ('share/my_robot_agent', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Python agent for humanoid robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_agent = my_robot_agent.agent_node:main',
        ],
    },
)
```

## Advanced Node Configuration

### Parameter Declaration

Use parameters to make your agent configurable:

```python
class ConfigurableAgent(Node):
    def __init__(self):
        super().__init__('configurable_agent')

        # Declare parameters with default values
        self.declare_parameter('agent_frequency', 10.0)
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('debug_mode', False)

        # Access parameter values
        self.frequency = self.get_parameter('agent_frequency').value
        self.robot_name = self.get_parameter('robot_name').value
        self.debug_mode = self.get_parameter('debug_mode').value

        self.get_logger().info(f'Agent configured for {self.robot_name}')
```

### Timer Integration

Use timers for periodic operations:

```python
class TimedAgent(Node):
    def __init__(self):
        super().__init__('timed_agent')

        # Create a timer for periodic tasks
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback
        )

        # Counter for demonstration
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback executed: {self.counter}')
        self.counter += 1
```

## Error Handling and Robustness

### Exception Handling

Implement proper error handling in your agents:

```python
class RobustAgent(Node):
    def __init__(self):
        super().__init__('robust_agent')

        try:
            # Initialize components that might fail
            self.setup_components()
        except Exception as e:
            self.get_logger().error(f'Failed to initialize: {e}')
            raise

    def setup_components(self):
        # Component initialization with error handling
        pass

    def safe_execute(self, func, *args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            self.get_logger().error(f'Error in {func.__name__}: {e}')
            return None
```

## Logging Best Practices

Configure logging appropriately for debugging and monitoring:

```python
class LoggingAgent(Node):
    def __init__(self):
        super().__init__('logging_agent')

        # Use different log levels appropriately
        self.get_logger().debug('Debug information')
        self.get_logger().info('Normal operation info')
        self.get_logger().warn('Warning message')
        self.get_logger().error('Error occurred')
        self.get_logger().fatal('Fatal error')
```

## Lifecycle Management

### Proper Shutdown

Ensure resources are properly cleaned up:

```python
class LifecycleAgent(Node):
    def __init__(self):
        super().__init__('lifecycle_agent')

        # Register cleanup function
        import atexit
        atexit.register(self.cleanup)

    def cleanup(self):
        # Clean up resources
        self.get_logger().info('Cleaning up resources...')
        # Close files, connections, etc.

    def destroy_node(self):
        # Additional cleanup before node destruction
        self.cleanup()
        super().destroy_node()
```

## Testing Setup

Create a basic test structure:

```python
import unittest
import rclpy
from my_robot_agent.agent_node import PythonAgent

class TestPythonAgent(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = PythonAgent()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_creation(self):
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'python_agent')

if __name__ == '__main__':
    unittest.main()
```

## Build and Execution

### Building the Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_agent
source install/setup.bash
```

### Running the Agent

```bash
ros2 run my_robot_agent my_agent
```

## Summary

Setting up Python agents with rclpy requires attention to proper node initialization, package structure, error handling, and lifecycle management. Following these patterns ensures your agents are robust, maintainable, and integrate well with the broader ROS 2 ecosystem.