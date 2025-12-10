---
sidebar_position: 3
---

# Python Command to Movement: Complete Workflow

## Introduction

The workflow from Python command to physical movement in humanoid robots involves multiple layers of software and hardware coordination. This section details the complete pipeline from high-level Python commands to actual robot movement, covering all intermediate steps and considerations.

## Complete Command-to-Movement Pipeline

### High-Level Command Interface

The entry point for commanding robot movement typically starts with a high-level Python interface:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import time

class MovementCommander(Node):
    def __init__(self):
        super().__init__('movement_commander')

        # Publishers for different movement types
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )
        self.action_pub = self.create_publisher(String, '/robot_action', 10)

        # Timer for demonstration
        self.demo_timer = self.create_timer(5.0, self.demo_movement_sequence)

        self.get_logger().info('Movement commander initialized')

    def move_to_pose(self, x, y, theta):
        """Send navigation command to move to specific pose"""
        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.angular.z = theta
        self.cmd_vel_pub.publish(twist_msg)

    def execute_joint_trajectory(self, joint_names, positions, duration=2.0):
        """Execute a joint trajectory"""
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration-int(duration)) * 1e9))

        trajectory.points = [point]
        self.joint_traj_pub.publish(trajectory)

    def perform_action(self, action_name):
        """Perform a predefined action"""
        action_msg = String()
        action_msg.data = action_name
        self.action_pub.publish(action_msg)

    def demo_movement_sequence(self):
        """Demonstrate different movement commands"""
        self.get_logger().info('Executing movement sequence...')

        # Example: Move to a specific position
        self.move_to_pose(1.0, 0.0, 0.0)  # Move forward 1m
        time.sleep(2)

        # Example: Execute a joint trajectory
        joint_names = ['l_shoulder_joint', 'l_elbow_joint', 'r_shoulder_joint', 'r_elbow_joint']
        positions = [0.5, -0.3, 0.5, -0.3]  # Raise both arms
        self.execute_joint_trajectory(joint_names, positions, 2.0)
        time.sleep(3)

        # Example: Perform a predefined action
        self.perform_action('wave_hello')

def main(args=None):
    rclpy.init(args=args)
    commander = MovementCommander()

    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        commander.get_logger().info('Shutting down movement commander')
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Middleware Communication Layer

### ROS 2 Message Handling

The command flows through ROS 2 middleware to reach the appropriate controllers:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import threading
import time

class MiddlewareHandler(Node):
    def __init__(self):
        super().__init__('middleware_handler')

        # Subscriptions
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.position_command_pub = self.create_publisher(
            Float64MultiArray,
            '/position_commands',
            10
        )

        # Internal state
        self.current_joint_positions = {}
        self.target_positions = {}
        self.command_buffer = []

        # Processing timer
        self.process_timer = self.create_timer(0.001, self.process_commands)  # 1kHz

        self.get_logger().info('Middleware handler initialized')

    def trajectory_callback(self, msg):
        """Process incoming trajectory commands"""
        if not msg.points:
            return

        # Process the trajectory points
        for point in msg.points:
            # Create target positions dictionary
            target_dict = {}
            for i, joint_name in enumerate(msg.joint_names):
                if i < len(point.positions):
                    target_dict[joint_name] = point.positions[i]

            # Add to command buffer
            self.command_buffer.append({
                'target_positions': target_dict,
                'time_from_start': point.time_from_start.sec + point.time_from_start.nanosec * 1e-9,
                'velocities': point.velocities if point.velocities else [],
                'accelerations': point.accelerations if point.accelerations else []
            })

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def process_commands(self):
        """Process commands and generate position commands"""
        if not self.command_buffer:
            return

        # Get the next command from buffer
        command = self.command_buffer[0]

        # Generate position commands based on current state and target
        position_cmd = Float64MultiArray()

        for joint_name, target_pos in command['target_positions'].items():
            current_pos = self.current_joint_positions.get(joint_name, 0.0)

            # Simple proportional control (in practice, use more sophisticated controllers)
            error = target_pos - current_pos
            output = 10.0 * error  # Simple P controller

            # Limit output to prevent excessive commands
            output = max(-10.0, min(10.0, output))

            position_cmd.data.append(output)

        if position_cmd.data:
            self.position_command_pub.publish(position_cmd)

        # Remove processed command if it's time-based
        if len(self.command_buffer) > 0:
            self.command_buffer.pop(0)

def main(args=None):
    rclpy.init(args=args)
    handler = MiddlewareHandler()

    try:
        rclpy.spin(handler)
    except KeyboardInterrupt:
        pass
    finally:
        handler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Control System Implementation

### Joint Controller with Interpolation

A more sophisticated controller that handles trajectory interpolation:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
import numpy as np
import math
from collections import deque

class AdvancedJointController(Node):
    def __init__(self):
        super().__init__('advanced_joint_controller')

        # Subscriptions
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/position_commands',
            10
        )

        # Controller state
        self.current_positions = {}
        self.current_velocities = {}
        self.active_trajectory = None
        self.trajectory_start_time = None
        self.current_trajectory_point = 0

        # PID controller parameters
        self.kp = 100.0
        self.ki = 0.1
        self.kd = 10.0

        # Control loop timer (1kHz)
        self.control_timer = self.create_timer(0.001, self.control_loop)

        # Trajectory interpolation parameters
        self.interpolation_buffer = deque(maxlen=100)

        self.get_logger().info('Advanced joint controller initialized')

    def trajectory_callback(self, msg):
        """Process new trajectory command"""
        if not msg.points or not msg.joint_names:
            return

        self.get_logger().info(f'Received trajectory with {len(msg.points)} points for {len(msg.joint_names)} joints')

        # Store the new trajectory
        self.active_trajectory = {
            'joint_names': msg.joint_names,
            'points': msg.points,
            'start_time': self.get_clock().now().nanoseconds * 1e-9
        }

        # Reset trajectory tracking
        self.current_trajectory_point = 0
        self.trajectory_start_time = self.active_trajectory['start_time']

    def joint_state_callback(self, msg):
        """Update current joint states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_velocities[name] = msg.velocity[i]

    def interpolate_trajectory(self, target_time):
        """Interpolate trajectory to get desired position at target_time"""
        if not self.active_trajectory:
            return {}

        points = self.active_trajectory['points']
        joint_names = self.active_trajectory['joint_names']

        if not points:
            return {}

        # Find the appropriate segment
        current_time = target_time - self.trajectory_start_time

        # If we've passed the last point, hold the final position
        if current_time >= points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9:
            final_point = points[-1]
            return {joint_names[i]: final_point.positions[i]
                   for i in range(min(len(joint_names), len(final_point.positions)))}

        # Find the current segment
        for i in range(len(points) - 1):
            t1 = points[i].time_from_start.sec + points[i].time_from_start.nanosec * 1e-9
            t2 = points[i+1].time_from_start.sec + points[i+1].time_from_start.nanosec * 1e-9

            if t1 <= current_time <= t2:
                # Linear interpolation between points
                if t2 != t1:  # Avoid division by zero
                    alpha = (current_time - t1) / (t2 - t1)
                else:
                    alpha = 0

                interpolated_positions = {}
                for j, joint_name in enumerate(joint_names):
                    if j < len(points[i].positions) and j < len(points[i+1].positions):
                        pos1 = points[i].positions[j]
                        pos2 = points[i+1].positions[j]
                        interpolated_positions[joint_name] = pos1 + alpha * (pos2 - pos1)

                return interpolated_positions

        # If we get here, we're before the first point or after the last
        if current_time < points[0].time_from_start.sec + points[0].time_from_start.nanosec * 1e-9:
            # Before first point, return first point positions
            first_point = points[0]
            return {joint_names[i]: first_point.positions[i]
                   for i in range(min(len(joint_names), len(first_point.positions)))}

        return {}

    def control_loop(self):
        """Main control loop running at 1kHz"""
        current_time = self.get_clock().now().nanoseconds * 1e-9

        if not self.active_trajectory:
            # No active trajectory, publish zero commands
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [0.0] * len(self.current_positions)
            self.command_pub.publish(cmd_msg)
            return

        # Get desired positions from trajectory
        desired_positions = self.interpolate_trajectory(current_time)

        if not desired_positions:
            return

        # Generate control commands using PID control
        cmd_msg = Float64MultiArray()

        for joint_name in self.active_trajectory['joint_names']:
            if joint_name in self.current_positions and joint_name in desired_positions:
                current_pos = self.current_positions[joint_name]
                desired_pos = desired_positions[joint_name]

                # Calculate error
                error = desired_pos - current_pos

                # Simple PID control (in practice, use proper PID with integral and derivative terms)
                control_output = self.kp * error

                # Apply limits
                control_output = max(-100.0, min(100.0, control_output))

                cmd_msg.data.append(control_output)
            else:
                # Joint not available, send zero command
                cmd_msg.data.append(0.0)

        # Publish commands
        if cmd_msg.data:
            self.command_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = AdvancedJointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down advanced joint controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hardware Interface Layer

### Simulated Hardware Interface

The final layer that interfaces with actual hardware (or simulation):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import threading
import time
import numpy as np

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')

        # Subscriptions for commands
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            '/position_commands',
            self.command_callback,
            10
        )

        # Publishers for joint states
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Simulated joint properties
        self.joint_names = [
            'l_hip_joint', 'l_knee_joint', 'l_ankle_joint',
            'r_hip_joint', 'r_knee_joint', 'r_ankle_joint',
            'l_shoulder_joint', 'l_elbow_joint',
            'r_shoulder_joint', 'r_elbow_joint'
        ]

        # Simulated joint states
        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.current_velocities = {name: 0.0 for name in self.joint_names}
        self.current_efforts = {name: 0.0 for name in self.joint_names}

        # Command processing
        self.command_buffer = []
        self.command_lock = threading.Lock()

        # Physics simulation parameters
        self.joint_damping = {name: 0.1 for name in self.joint_names}
        self.joint_friction = {name: 0.05 for name in self.joint_names}

        # Simulation timer (1kHz)
        self.sim_timer = self.create_timer(0.001, self.simulation_step)

        self.get_logger().info('Hardware interface initialized')

    def command_callback(self, msg):
        """Process incoming position commands"""
        with self.command_lock:
            # Process the command
            if len(msg.data) == len(self.joint_names):
                # Apply commands to each joint with some physics simulation
                for i, joint_name in enumerate(self.joint_names):
                    if i < len(msg.data):
                        # In a real system, this would send the command to the physical actuator
                        command = msg.data[i]

                        # Simulate actuator response with damping and friction
                        current_pos = self.current_positions[joint_name]
                        current_vel = self.current_velocities[joint_name]

                        # Simple physics simulation
                        acceleration = command - self.joint_damping[joint_name] * current_vel - self.joint_friction[joint_name] * np.sign(current_vel)

                        # Update state (Euler integration)
                        new_vel = current_vel + acceleration * 0.001  # dt = 1ms
                        new_pos = current_pos + new_vel * 0.001

                        # Update simulated joint state
                        self.current_positions[joint_name] = new_pos
                        self.current_velocities[joint_name] = new_vel
                        self.current_efforts[joint_name] = command

    def simulation_step(self):
        """Simulation step that publishes joint states"""
        # Create and publish joint state message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = list(self.joint_names)
        joint_state_msg.position = [self.current_positions[name] for name in self.joint_names]
        joint_state_msg.velocity = [self.current_velocities[name] for name in self.joint_names]
        joint_state_msg.effort = [self.current_efforts[name] for name in self.joint_names]

        self.joint_state_pub.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    hw_interface = HardwareInterface()

    try:
        rclpy.spin(hw_interface)
    except KeyboardInterrupt:
        hw_interface.get_logger().info('Shutting down hardware interface')
    finally:
        hw_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Integration Example

### Full System Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import numpy as np
import time

class CompleteMovementSystem(Node):
    def __init__(self):
        super().__init__('complete_movement_system')

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        self.command_pub = self.create_publisher(
            String, '/system_command', 10
        )

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # System state
        self.current_joint_states = {}
        self.system_status = 'idle'

        # Demo timer
        self.demo_timer = self.create_timer(10.0, self.run_demo_sequence)

        self.get_logger().info('Complete movement system initialized')

    def joint_state_callback(self, msg):
        """Update joint state information"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }

    def execute_arm_wave(self):
        """Execute an arm waving motion"""
        self.get_logger().info('Executing arm wave motion')

        trajectory = JointTrajectory()
        trajectory.joint_names = ['l_shoulder_joint', 'l_elbow_joint', 'r_shoulder_joint', 'r_elbow_joint']

        # Create wave pattern with multiple points
        time_step = 0.2  # 5Hz
        num_points = 20

        for i in range(num_points):
            point = JointTrajectoryPoint()

            t = i * time_step

            # Create wave motion
            l_shoulder_pos = 0.5 * np.sin(2 * np.pi * 0.5 * t)  # 0.5Hz wave
            l_elbow_pos = -0.3 * np.sin(2 * np.pi * 0.5 * t + np.pi/2)  # Phase shifted
            r_shoulder_pos = 0.5 * np.sin(2 * np.pi * 0.5 * t + np.pi)  # Opposite phase
            r_elbow_pos = -0.3 * np.sin(2 * np.pi * 0.5 * t + 3*np.pi/2)  # Phase shifted

            point.positions = [l_shoulder_pos, l_elbow_pos, r_shoulder_pos, r_elbow_pos]
            point.time_from_start = Duration(sec=int(t), nanosec=int((t-int(t)) * 1e9))

            trajectory.points.append(point)

        self.trajectory_pub.publish(trajectory)

    def execute_walk_pattern(self):
        """Execute a walking pattern"""
        self.get_logger().info('Executing walk pattern')

        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'l_hip_joint', 'l_knee_joint', 'l_ankle_joint',
            'r_hip_joint', 'r_knee_joint', 'r_ankle_joint'
        ]

        # Create simple walking pattern
        time_step = 0.1
        num_steps = 50

        for i in range(num_steps):
            point = JointTrajectoryPoint()
            t = i * time_step

            # Walking gait pattern
            l_hip = 0.2 * np.sin(2 * np.pi * 0.5 * t)
            l_knee = 0.1 * np.sin(4 * np.pi * 0.5 * t)
            l_ankle = -0.2 * np.sin(2 * np.pi * 0.5 * t)

            r_hip = 0.2 * np.sin(2 * np.pi * 0.5 * t + np.pi)
            r_knee = 0.1 * np.sin(4 * np.pi * 0.5 * t + np.pi)
            r_ankle = -0.2 * np.sin(2 * np.pi * 0.5 * t + np.pi)

            point.positions = [l_hip, l_knee, l_ankle, r_hip, r_knee, r_ankle]
            point.time_from_start = Duration(sec=int(t), nanosec=int((t-int(t)) * 1e9))

            trajectory.points.append(point)

        self.trajectory_pub.publish(trajectory)

    def run_demo_sequence(self):
        """Run a complete demo sequence"""
        self.get_logger().info('Starting demo sequence...')

        # Sequence: idle -> wave arms -> walk -> return to idle
        time.sleep(1)

        # Wave arms
        self.execute_arm_wave()
        time.sleep(5)

        # Walk
        self.execute_walk_pattern()
        time.sleep(6)

        # Return to neutral position
        self.return_to_neutral()

    def return_to_neutral(self):
        """Return all joints to neutral position"""
        self.get_logger().info('Returning to neutral position')

        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'l_hip_joint', 'l_knee_joint', 'l_ankle_joint',
            'r_hip_joint', 'r_knee_joint', 'r_ankle_joint',
            'l_shoulder_joint', 'l_elbow_joint',
            'r_shoulder_joint', 'r_elbow_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(trajectory.joint_names)
        point.time_from_start = Duration(sec=2, nanosec=0)

        trajectory.points = [point]
        self.trajectory_pub.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    system = CompleteMovementSystem()

    try:
        rclpy.spin(system)
    except KeyboardInterrupt:
        system.get_logger().info('Shutting down complete movement system')
    finally:
        system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling and Safety

### Safety Monitor for Command Pipeline

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool, String
import numpy as np

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Subscriptions
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Publishers
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.safety_status_pub = self.create_publisher(String, '/safety_status', 10)

        # Joint safety limits
        self.joint_limits = {
            'l_hip_joint': (-2.0, 2.0),
            'l_knee_joint': (0.0, 2.5),
            'l_ankle_joint': (-0.5, 0.5),
            'r_hip_joint': (-2.0, 2.0),
            'r_knee_joint': (0.0, 2.5),
            'r_ankle_joint': (-0.5, 0.5),
            'l_shoulder_joint': (-3.0, 3.0),
            'l_elbow_joint': (-2.0, 0.0),
            'r_shoulder_joint': (-3.0, 3.0),
            'r_elbow_joint': (-2.0, 0.0),
        }

        # Safety thresholds
        self.max_joint_velocity = 5.0  # rad/s
        self.max_tilt_angle = 1.0  # rad (about 57 degrees)
        self.max_command_frequency = 100  # Hz

        # State tracking
        self.current_positions = {}
        self.previous_positions = {}
        self.previous_times = {}
        self.command_count = 0
        self.last_command_time = 0

        # Safety state
        self.emergency_stop_active = False

        # Safety check timer (100Hz)
        self.safety_timer = self.create_timer(0.01, self.safety_check)

    def joint_callback(self, msg):
        """Process joint state for safety checks"""
        current_time = self.get_clock().now().nanoseconds * 1e-9

        for i, name in enumerate(msg.name):
            if name in self.joint_limits:
                if i < len(msg.position):
                    # Store current position
                    self.current_positions[name] = msg.position[i]

                    # Check joint limits
                    min_limit, max_limit = self.joint_limits[name]
                    if msg.position[i] < min_limit or msg.position[i] > max_limit:
                        self.get_logger().error(f'JOINT LIMIT VIOLATION: {name} = {msg.position[i]}, limits = [{min_limit}, {max_limit}]')
                        self.trigger_emergency_stop()
                        return

                    # Check velocity limits if we have previous data
                    if name in self.previous_positions:
                        dt = current_time - self.previous_times.get(name, current_time - 0.01)
                        if dt > 0:
                            velocity = abs(msg.position[i] - self.previous_positions[name]) / dt
                            if velocity > self.max_joint_velocity:
                                self.get_logger().warn(f'High velocity on {name}: {velocity} rad/s')
                                # Don't trigger emergency stop for velocity, but log it

                # Store for next iteration
                if i < len(msg.position):
                    self.previous_positions[name] = msg.position[i]
                    self.previous_times[name] = current_time

    def imu_callback(self, msg):
        """Check robot orientation for safety"""
        if self.emergency_stop_active:
            return

        # Convert quaternion to roll/pitch angles
        orientation = msg.orientation
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z

        # Roll calculation
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch calculation
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Check if robot is tilted beyond safe limits
        if abs(roll) > self.max_tilt_angle or abs(pitch) > self.max_tilt_angle:
            self.get_logger().error(f'Dangerous orientation: Roll={roll:.2f}, Pitch={pitch:.2f}')
            self.trigger_emergency_stop()

    def safety_check(self):
        """Perform comprehensive safety checks"""
        if self.emergency_stop_active:
            return

        # Publish safety status
        status_msg = String()
        status_msg.data = 'SAFE' if not self.emergency_stop_active else 'EMERGENCY_STOP'
        self.safety_status_pub.publish(status_msg)

    def trigger_emergency_stop(self):
        """Trigger emergency stop and halt all movement"""
        if not self.emergency_stop_active:
            self.get_logger().fatal('EMERGENCY STOP ACTIVATED - ALL MOTION HALTED')
            self.emergency_stop_active = True

            # Publish emergency stop command
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()

    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        safety_monitor.get_logger().info('Shutting down safety monitor')
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### Optimized Command Pipeline

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from collections import deque
import time

class OptimizedCommandPipeline(Node):
    def __init__(self):
        super().__init__('optimized_command_pipeline')

        # High-frequency subscriptions and publications
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            1  # Minimal queue for real-time performance
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1  # Minimal queue for real-time performance
        )

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/position_commands',
            1  # Minimal queue for real-time performance
        )

        # Optimized data structures
        self.joint_names = []
        self.current_positions = np.array([])
        self.current_velocities = np.array([])
        self.target_positions = np.array([])
        self.control_gains = np.array([])  # kp values

        # Circular buffer for trajectory smoothing
        self.trajectory_buffer = deque(maxlen=10)

        # Timing optimization
        self.last_control_time = 0
        self.control_period = 0.001  # 1kHz

        # High-frequency timer
        self.control_timer = self.create_timer(
            self.control_period,
            self.optimized_control_loop,
            clock=self.get_clock()
        )

        self.get_logger().info('Optimized command pipeline initialized')

    def trajectory_callback(self, msg):
        """Optimized trajectory callback"""
        if not msg.points or not msg.joint_names:
            return

        # Update joint names if they've changed
        if msg.joint_names != self.joint_names:
            self.joint_names = msg.joint_names
            self.current_positions = np.zeros(len(self.joint_names))
            self.current_velocities = np.zeros(len(self.joint_names))
            self.target_positions = np.zeros(len(self.joint_names))
            self.control_gains = np.full(len(self.joint_names), 100.0)  # Default kp

        # Process trajectory points efficiently
        if msg.points:
            # Use the first point as immediate target
            first_point = msg.points[0]
            if len(first_point.positions) == len(self.joint_names):
                self.target_positions = np.array(first_point.positions)

    def joint_state_callback(self, msg):
        """Optimized joint state callback"""
        # Update only the joints we care about
        for i, name in enumerate(msg.name):
            try:
                idx = self.joint_names.index(name)
                if i < len(msg.position):
                    self.current_positions[idx] = msg.position[i]
                if i < len(msg.velocity):
                    self.current_velocities[idx] = msg.velocity[i]
            except ValueError:
                # Joint not in our list, ignore
                continue

    def optimized_control_loop(self):
        """High-performance control loop"""
        current_time = self.get_clock().now().nanoseconds * 1e-9

        # Check if it's time to run control (in case timer isn't perfectly accurate)
        if current_time - self.last_control_time < self.control_period:
            return

        self.last_control_time = current_time

        # Check if we have valid data
        if len(self.current_positions) == 0 or len(self.target_positions) == 0:
            return

        # Vectorized control calculation (much faster than loops)
        errors = self.target_positions - self.current_positions
        control_outputs = self.control_gains * errors

        # Apply output limits efficiently
        control_outputs = np.clip(control_outputs, -100.0, 100.0)

        # Publish using numpy array conversion
        cmd_msg = Float64MultiArray()
        cmd_msg.data = control_outputs.tolist()  # Convert to list for ROS message

        if len(cmd_msg.data) > 0:
            self.command_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    pipeline = OptimizedCommandPipeline()

    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pipeline.get_logger().info('Shutting down optimized command pipeline')
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

The complete Python command to movement workflow involves multiple layers of software coordination:

1. **Command Interface**: High-level Python commands that specify desired movements
2. **Middleware**: ROS 2 message passing between different system components
3. **Control System**: Trajectory interpolation and PID control algorithms
4. **Hardware Interface**: Communication with physical actuators or simulation
5. **Safety Systems**: Monitoring and emergency stop functionality
6. **Performance Optimization**: Efficient data structures and algorithms

Each layer must work seamlessly together to achieve smooth, safe, and responsive robot movement. Proper design of this pipeline is critical for humanoid robot performance and safety.