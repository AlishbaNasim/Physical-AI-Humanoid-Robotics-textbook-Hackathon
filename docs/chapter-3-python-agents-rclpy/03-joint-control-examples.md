---
sidebar_position: 3
---

# Joint Control Examples with rclpy

## Introduction

Joint control is fundamental to humanoid robotics, enabling precise movement and manipulation. This section demonstrates how to implement joint control systems using rclpy, covering both basic and advanced control patterns.

## Joint State Interface

### Subscribing to Joint States

The `sensor_msgs/JointState` message is the standard way to receive joint position, velocity, and effort information:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Store the latest joint state
        self.latest_joint_state = None

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg
        self.get_logger().debug(f'Received joint state for {len(msg.name)} joints')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()

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

### Publishing Joint Commands

To control joints, publish to the appropriate command topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        # Publisher for joint commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Timer for sending commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example joint positions

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()

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

## Joint Trajectory Control

### Using JointTrajectory Messages

For more sophisticated control, use the `trajectory_msgs/JointTrajectory` message:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointTrajectoryController(Node):
    def __init__(self):
        super().__init__('joint_trajectory_controller')

        # Publisher for joint trajectories
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Joint names for the robot
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'joint5', 'joint6', 'joint7'
        ]

    def send_trajectory(self, positions, time_from_start=5.0):
        """Send a trajectory with specified positions"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(time_from_start), nanosec=0)

        trajectory_msg.points = [point]

        self.publisher.publish(trajectory_msg)
        self.get_logger().info(f'Sent trajectory to {len(positions)} joints')

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryController()

    # Send a sample trajectory
    node.send_trajectory([1.0, -0.5, 0.0, 0.5, -1.0, 0.0, 0.5])

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

## PID Controller Implementation

### Basic PID Controller

Implement a simple PID controller for joint control:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class PIDJointController(Node):
    def __init__(self):
        super().__init__('pid_joint_controller')

        # PID parameters
        self.kp = 10.0  # Proportional gain
        self.ki = 0.1   # Integral gain
        self.kd = 0.5   # Derivative gain

        # Joint control variables
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.target_positions = [0.0, 0.0, 0.0]
        self.current_positions = [0.0, 0.0, 0.0]
        self.previous_errors = [0.0, 0.0, 0.0]
        self.integral_errors = [0.0, 0.0, 0.0]

        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/position_commands',
            10
        )

        # Control timer
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

    def joint_state_callback(self, msg):
        """Update current joint positions from joint state"""
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self.current_positions[i] = msg.position[idx]

    def control_loop(self):
        """Main control loop implementing PID control"""
        commands = Float64MultiArray()

        for i in range(len(self.joint_names)):
            # Calculate error
            error = self.target_positions[i] - self.current_positions[i]

            # Calculate derivative
            derivative = error - self.previous_errors[i]

            # Update integral
            self.integral_errors[i] += error

            # Calculate PID output
            output = (
                self.kp * error +
                self.ki * self.integral_errors[i] +
                self.kd * derivative
            )

            # Update previous error
            self.previous_errors[i] = error

            # Add to commands
            commands.data.append(output)

        # Publish commands
        self.command_pub.publish(commands)

def main(args=None):
    rclpy.init(args=args)
    node = PIDJointController()

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

## Advanced Joint Control Patterns

### Multi-Joint Synchronization

Coordinate multiple joints for synchronized movement:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class SynchronizedJointController(Node):
    def __init__(self):
        super().__init__('synchronized_joint_controller')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Define joint groups
        self.left_arm_joints = ['l_shoulder_joint', 'l_elbow_joint', 'l_wrist_joint']
        self.right_arm_joints = ['r_shoulder_joint', 'r_elbow_joint', 'r_wrist_joint']

        # Timer for periodic control
        self.timer = self.create_timer(0.1, self.periodic_control)

    def create_trajectory_msg(self, joint_names, positions, velocities=None, time_from_start=2.0):
        """Create a trajectory message for specified joints"""
        msg = JointTrajectory()
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions

        if velocities:
            point.velocities = velocities
        else:
            point.velocities = [0.0] * len(positions)

        point.time_from_start = Duration(sec=int(time_from_start), nanosec=0)
        msg.points = [point]

        return msg

    def periodic_control(self):
        """Execute periodic control actions"""
        # Example: Move both arms in a coordinated pattern
        left_positions = [math.sin(self.get_clock().now().nanoseconds * 1e-9),
                         math.cos(self.get_clock().now().nanoseconds * 1e-9), 0.0]
        right_positions = [math.sin(self.get_clock().now().nanoseconds * 1e-9 + math.pi),
                          math.cos(self.get_clock().now().nanoseconds * 1e-9 + math.pi), 0.0]

        # Create and publish trajectories
        left_msg = self.create_trajectory_msg(self.left_arm_joints, left_positions)
        right_msg = self.create_trajectory_msg(self.right_arm_joints, right_positions)

        self.publisher.publish(left_msg)
        self.publisher.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SynchronizedJointController()

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

## Safety Considerations

### Joint Limit Checking

Implement safety checks to prevent joint limit violations:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

class SafeJointController(Node):
    def __init__(self):
        super().__init__('safe_joint_controller')

        # Define joint limits (example values)
        self.joint_limits = {
            'joint1': (-3.14, 3.14),
            'joint2': (-1.57, 1.57),
            'joint3': (-2.36, 2.36),
        }

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

    def check_joint_limits(self, joint_names, positions):
        """Check if positions are within joint limits"""
        for joint_name, position in zip(joint_names, positions):
            if joint_name in self.joint_limits:
                min_limit, max_limit = self.joint_limits[joint_name]
                if position < min_limit or position > max_limit:
                    self.get_logger().warn(
                        f'Joint {joint_name} position {position} exceeds limits '
                        f'[{min_limit}, {max_limit}]'
                    )
                    # Clamp to limits
                    clamped_pos = max(min_limit, min(max_limit, position))
                    idx = joint_names.index(joint_name)
                    positions[idx] = clamped_pos
        return positions

    def send_safe_trajectory(self, joint_names, positions):
        """Send trajectory after checking joint limits"""
        safe_positions = self.check_joint_limits(joint_names, positions.copy())

        # Create and send trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names

        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        point = JointTrajectoryPoint()
        point.positions = safe_positions
        point.time_from_start = Duration(sec=2, nanosec=0)
        trajectory_msg.points = [point]

        self.publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafeJointController()

    # Example: Send a trajectory with potentially unsafe positions
    node.send_safe_trajectory(['joint1', 'joint2', 'joint3'], [5.0, -2.0, 1.0])

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

## Summary

Joint control in humanoid robotics requires careful consideration of control patterns, safety limits, and synchronization. These examples demonstrate various approaches to joint control using rclpy, from basic position control to advanced trajectory following with safety checks. Understanding these patterns is essential for developing robust humanoid robotic systems.