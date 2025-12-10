---
sidebar_position: 2
---

# ROS 2 Composition and Component Architecture

## Introduction

ROS 2 composition is a powerful feature that allows multiple nodes to run within the same process, improving performance and reducing overhead. This is particularly important for humanoid robots where multiple control components need to run with minimal latency and maximum efficiency.

## Understanding ROS 2 Composition

### Composition vs. Traditional Nodes

Traditional ROS 2 nodes run in separate processes with inter-process communication overhead. Composed nodes run within the same process, sharing memory space and eliminating communication overhead between components.

### Benefits for Humanoid Robots

- **Reduced Latency**: Faster communication between control components
- **Lower Overhead**: Less process management overhead
- **Better Performance**: More efficient resource utilization
- **Improved Determinism**: More predictable execution timing

## Creating Composable Nodes

### Composable Node Structure

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace composition_examples
{
class ComposableNode : public rclcpp::Node
{
public:
  ComposableNode(const rclcpp::NodeOptions & options)
  : Node("composable_node", options)
  {
    // Initialize components
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });

    publisher_ = this->create_publisher<std_msgs::msg::String>("output", 10);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace composition_examples

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with the ROS 2 node factory
RCLCPP_COMPONENTS_REGISTER_NODE(composition_examples::ComposableNode)
```

### Python Composable Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor
import threading
import time

class ComposableNode(Node):
    def __init__(self, node_name='composable_node'):
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter('update_rate', 50, ParameterDescriptor(
            description='Update rate for the control loop in Hz'
        ))

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value

        # Create subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Create publishers
        self.command_pub = self.create_publisher(
            String,
            '/robot_commands',
            10
        )

        # Create timer for control loop
        self.control_timer = self.create_timer(
            1.0 / self.update_rate,
            self.control_loop
        )

        self.get_logger().info(f'Composable node initialized with update rate: {self.update_rate}Hz')

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        # Process joint state data
        self.get_logger().debug(f'Received joint state with {len(msg.name)} joints')

    def control_loop(self):
        """Main control loop"""
        # Implement control logic here
        cmd_msg = String()
        cmd_msg.data = f'Control tick at {time.time()}'
        self.command_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    node = ComposableNode()

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

## Composition Example: Humanoid Control System

### Complete Composable Control System

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Float64MultiArray
from builtin_interfaces.msg import Duration
import numpy as np
from collections import deque
import threading
import time

class StateEstimatorNode(Node):
    def __init__(self, node_name='state_estimator'):
        super().__init__(node_name)

        # Subscriptions
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Publishers
        self.state_pub = self.create_publisher(
            String, '/robot_state', 10
        )

        # State variables
        self.joint_positions = {}
        self.joint_velocities = {}
        self.orientation = None

        # Timer
        self.timer = self.create_timer(0.01, self.publish_state)  # 100Hz

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        self.orientation = msg.orientation

    def publish_state(self):
        # Publish state information
        state_msg = String()
        state_msg.data = f'State update: {len(self.joint_positions)} joints'
        self.state_pub.publish(state_msg)

class TrajectoryGeneratorNode(Node):
    def __init__(self, node_name='trajectory_generator'):
        super().__init__(node_name)

        # Subscriptions
        self.command_sub = self.create_subscription(
            String, '/motion_command', self.command_callback, 10
        )

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory', 10
        )

        # Joint names
        self.joint_names = [
            'l_hip_joint', 'l_knee_joint', 'l_ankle_joint',
            'r_hip_joint', 'r_knee_joint', 'r_ankle_joint'
        ]

        # Current trajectory
        self.current_trajectory = None

    def command_callback(self, msg):
        command = msg.data
        if command == 'walk':
            self.generate_walk_trajectory()
        elif command == 'stand':
            self.generate_stand_trajectory()

    def generate_walk_trajectory(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        # Create walking pattern (simplified)
        for i in range(20):  # 20 points
            point = JointTrajectoryPoint()
            t = i * 0.1  # 0.1 second intervals

            # Generate walking pattern
            positions = [
                0.1 * np.sin(2 * np.pi * t),      # l_hip
                0.05 * np.sin(4 * np.pi * t),     # l_knee
                -0.1 * np.sin(2 * np.pi * t),     # l_ankle
                0.1 * np.sin(2 * np.pi * t + np.pi),  # r_hip
                0.05 * np.sin(4 * np.pi * t + np.pi), # r_knee
                -0.1 * np.sin(2 * np.pi * t + np.pi)  # r_ankle
            ]

            point.positions = positions
            point.time_from_start = Duration(sec=int(t), nanosec=int((t-int(t)) * 1e9))
            trajectory.points.append(point)

        self.trajectory_pub.publish(trajectory)

    def generate_stand_trajectory(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=1, nanosec=0)

        trajectory.points = [point]
        self.trajectory_pub.publish(trajectory)

class ControllerNode(Node):
    def __init__(self, node_name='controller'):
        super().__init__(node_name)

        # Subscriptions
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory', self.trajectory_callback, 10
        )

        # Publishers
        self.command_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10
        )

        # Control variables
        self.current_target = {}
        self.current_position = {}
        self.control_history = deque(maxlen=100)

        # PID parameters
        self.kp = 10.0
        self.ki = 0.1
        self.kd = 1.0

        # Timer for control loop
        self.control_timer = self.create_timer(0.001, self.control_loop)  # 1kHz

    def trajectory_callback(self, msg):
        if msg.points:
            # Use first point as current target
            point = msg.points[0]
            for i, joint_name in enumerate(msg.joint_names):
                if i < len(point.positions):
                    self.current_target[joint_name] = point.positions[i]

    def control_loop(self):
        commands = Float64MultiArray()

        # Simple PID control for each joint
        for joint_name in self.current_target:
            target_pos = self.current_target.get(joint_name, 0.0)
            current_pos = self.current_position.get(joint_name, 0.0)

            error = target_pos - current_pos
            control_output = self.kp * error  # Simplified PID

            commands.data.append(control_output)

        if commands.data:
            self.command_pub.publish(commands)

class ComposedHumanoidControl(Node):
    def __init__(self):
        super().__init__('composed_humanoid_control')

        # Create component nodes
        self.state_estimator = StateEstimatorNode('state_estimator')
        self.trajectory_generator = TrajectoryGeneratorNode('trajectory_generator')
        self.controller = ControllerNode('controller')

        self.get_logger().info('Composed humanoid control system initialized')

def main(args=None):
    rclpy.init(args=args)

    # Create the composed node
    composed_node = ComposedHumanoidControl()

    try:
        # Spin the composed node (this spins all components)
        rclpy.spin(composed_node)
    except KeyboardInterrupt:
        composed_node.get_logger().info('Shutting down composed control system')
    finally:
        composed_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Composition Launch Files

### Composition Container Launch

```python
import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for composed nodes."""

    container = ComposableNodeContainer(
        name='humanoid_control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='composition_examples',
                plugin='composition_examples::StateEstimatorNode',
                name='state_estimator',
                parameters=[{'update_rate': 100}]
            ),
            ComposableNode(
                package='composition_examples',
                plugin='composition_examples::TrajectoryGeneratorNode',
                name='trajectory_generator',
                parameters=[{'motion_type': 'walking'}]
            ),
            ComposableNode(
                package='composition_examples',
                plugin='composition_examples::ControllerNode',
                name='controller',
                parameters=[{'control_frequency': 1000}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

## C++ Composition Example

### CMakeLists.txt for Composition

```cmake
cmake_minimum_required(VERSION 3.8)
project(composition_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create a library with your composable node
add_library(composition_examples SHARED
  src/composable_node.cpp
)
target_compile_features(composition_examples PRIVATE c_std_99 cxx_std_17)
ament_target_dependencies(composition_examples
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
)

# Register the node as a component
rclcpp_components_register_nodes(composition_examples "composition_examples::ComposableNode")

install(TARGETS
  composition_examples
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Performance Optimization

### Shared Memory Communication

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import multiprocessing
import threading
from collections import defaultdict

class OptimizedComposedNode(Node):
    def __init__(self):
        super().__init__('optimized_composed_node')

        # Shared data structures for internal communication
        self.shared_data = defaultdict(lambda: None)
        self.lock = threading.Lock()

        # Create internal timers for different components
        self.state_timer = self.create_timer(0.01, self.state_update)  # 100Hz
        self.control_timer = self.create_timer(0.001, self.control_update)  # 1kHz
        self.communication_timer = self.create_timer(0.1, self.communication_update)  # 10Hz

        self.get_logger().info('Optimized composed node initialized')

    def state_update(self):
        """State estimation at 100Hz"""
        with self.lock:
            # Update internal state
            self.shared_data['last_state_update'] = self.get_clock().now().nanoseconds * 1e-9

    def control_update(self):
        """Control loop at 1kHz"""
        with self.lock:
            # Access shared state data
            last_update = self.shared_data.get('last_state_update', 0)
            # Perform control calculations
            control_output = self.calculate_control_output()

            # Store results for other components
            self.shared_data['control_output'] = control_output

    def communication_update(self):
        """Communication with external nodes at 10Hz"""
        with self.lock:
            control_output = self.shared_data.get('control_output', 0)

            # Publish to external topics
            msg = String()
            msg.data = f'Control output: {control_output}'
            # self.external_publisher.publish(msg)

    def calculate_control_output(self):
        """Calculate control output (simplified)"""
        return 42.0  # Placeholder calculation

def main(args=None):
    rclpy.init(args=args)

    node = OptimizedComposedNode()

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

## Best Practices for Composition

### Component Design Guidelines

1. **Clear Interfaces**: Define clear input/output interfaces between components
2. **Appropriate Update Rates**: Match update rates to component requirements
3. **Resource Management**: Share resources appropriately between components
4. **Error Handling**: Implement proper error handling and recovery
5. **Parameter Configuration**: Use parameters to configure components
6. **Logging**: Implement proper logging for debugging composed systems

### Performance Considerations

- **Timing Constraints**: Ensure timing requirements are met across all components
- **Memory Usage**: Monitor memory usage of composed nodes
- **CPU Utilization**: Balance CPU usage across components
- **Real-time Performance**: Consider real-time requirements for control systems

### Safety Considerations

- **Component Isolation**: Ensure components don't interfere with each other
- **Failure Handling**: Handle component failures gracefully
- **Resource Limits**: Set appropriate limits on resource usage
- **Monitoring**: Implement monitoring for composed systems

## Testing Composed Systems

### Unit Testing Composable Nodes

```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from composition_examples.composable_node import ComposableNode

class TestComposableNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = ComposableNode('test_node')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_creation(self):
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'test_node')

    def test_parameters(self):
        self.node.declare_parameter('test_param', 42)
        self.assertEqual(self.node.get_parameter('test_param').value, 42)

if __name__ == '__main__':
    unittest.main()
```

## Summary

ROS 2 composition provides significant benefits for humanoid robot control systems by allowing multiple nodes to run in the same process, reducing communication overhead and improving performance. Properly designed composable nodes can enhance the real-time performance and efficiency of humanoid robot control pipelines while maintaining modularity and maintainability.