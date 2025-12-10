---
sidebar_position: 4
---

# Sensor Integration with rclpy

## Introduction

Sensor integration is critical for humanoid robots to perceive and interact with their environment. This section covers how to integrate various sensor types using rclpy, including cameras, IMUs, force/torque sensors, and other common robotic sensors.

## Camera Integration

### Subscribing to Camera Topics

Camera data is typically published as `sensor_msgs/Image` messages:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # Create CV bridge for image conversion
        self.bridge = CvBridge()

        # Subscribe to camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.subscription  # Prevent unused variable warning

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process the image (example: convert to grayscale)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Display the image
            cv2.imshow('Camera Image', gray_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## IMU Integration

### Processing IMU Data

Inertial Measurement Unit (IMU) data is published as `sensor_msgs/Imu` messages:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Subscribe to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Store previous orientation for drift calculation
        self.previous_orientation = None

        # Store orientation history for filtering
        self.orientation_history = []

    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract orientation (quaternion)
        orientation = msg.orientation

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        # Extract angular velocity
        angular_velocity = msg.angular_velocity

        # Extract linear acceleration
        linear_acceleration = msg.linear_acceleration

        # Log the processed data
        self.get_logger().info(
            f'IMU Data - Roll: {math.degrees(roll):.2f}°, '
            f'Pitch: {math.degrees(pitch):.2f}°, '
            f'Yaw: {math.degrees(yaw):.2f}°'
        )

        # Store for history analysis
        self.orientation_history.append((roll, pitch, yaw))
        if len(self.orientation_history) > 100:
            self.orientation_history.pop(0)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = IMUProcessor()

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

## Force/Torque Sensor Integration

### Processing Force/Torque Data

Force/Torque sensors provide data as `geometry_msgs/Wrench` messages:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64
import numpy as np

class ForceTorqueProcessor(Node):
    def __init__(self):
        super().__init__('force_torque_processor')

        # Subscribe to force/torque data
        self.subscription = self.create_subscription(
            Wrench,
            '/ft_sensor/wrench',
            self.wrench_callback,
            10
        )

        # Publishers for processed data
        self.force_magnitude_pub = self.create_publisher(
            Float64, '/ft_sensor/force_magnitude', 10
        )

        # Store history for filtering
        self.force_history = []
        self.torque_history = []

    def wrench_callback(self, msg):
        """Process force/torque data"""
        # Extract force components
        force = np.array([msg.force.x, msg.force.y, msg.force.z])
        force_magnitude = np.linalg.norm(force)

        # Extract torque components
        torque = np.array([msg.torque.x, msg.torque.y, msg.torque.z])
        torque_magnitude = np.linalg.norm(torque)

        # Calculate force magnitude
        force_msg = Float64()
        force_msg.data = float(force_magnitude)
        self.force_magnitude_pub.publish(force_msg)

        # Store in history
        self.force_history.append(force_magnitude)
        self.torque_history.append(torque_magnitude)

        if len(self.force_history) > 50:
            self.force_history.pop(0)
            self.torque_history.pop(0)

        # Log the processed data
        self.get_logger().info(
            f'Force: {force_magnitude:.2f}N, Torque: {torque_magnitude:.2f}Nm'
        )

        # Check for contact detection
        if force_magnitude > 10.0:  # Threshold for contact detection
            self.get_logger().info('Contact detected!')

def main(args=None):
    rclpy.init(args=args)
    node = ForceTorqueProcessor()

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

## LIDAR Integration

### Processing LIDAR Data

LIDAR data is published as `sensor_msgs/LaserScan` messages:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class LIDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Subscribe to LIDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher for obstacle distance
        self.obstacle_distance_pub = self.create_publisher(
            Float32, '/obstacle_distance', 10
        )

        # Configuration parameters
        self.min_obstacle_distance = 1.0  # meters
        self.front_angle_range = 30  # degrees

    def scan_callback(self, msg):
        """Process LIDAR scan data"""
        # Convert angle range to indices
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Get distances in front of the robot (within front_angle_range)
        front_distances = []
        center_index = len(msg.ranges) // 2

        # Calculate indices for front angle range
        angle_range_indices = int(self.front_angle_range * np.pi / 180 / angle_increment)

        start_idx = max(0, center_index - angle_range_indices // 2)
        end_idx = min(len(msg.ranges), center_index + angle_range_indices // 2)

        for i in range(start_idx, end_idx):
            if i < len(msg.ranges):
                range_val = msg.ranges[i]
                if range_val >= msg.range_min and range_val <= msg.range_max:
                    front_distances.append(range_val)

        if front_distances:
            min_distance = min(front_distances)

            # Publish minimum distance
            distance_msg = Float32()
            distance_msg.data = min_distance
            self.obstacle_distance_pub.publish(distance_msg)

            # Log obstacle information
            if min_distance < self.min_obstacle_distance:
                self.get_logger().warn(f'Obstacle detected: {min_distance:.2f}m')
            else:
                self.get_logger().info(f'Clear path: {min_distance:.2f}m')
        else:
            self.get_logger().info('No valid measurements in front')

def main(args=None):
    rclpy.init(args=args)
    node = LIDARProcessor()

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

## Multi-Sensor Fusion

### Combining Multiple Sensors

Combine data from multiple sensors for better perception:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import math

class MultiSensorFusion(Node):
    def __init__(self):
        super().__init__('multi_sensor_fusion')

        # Subscribe to multiple sensor topics
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers for fused data
        self.tilt_publisher = self.create_publisher(
            Float64, '/robot_tilt', 10
        )

        self.safe_distance_publisher = self.create_publisher(
            Float64, '/safe_distance', 10
        )

        # Store sensor data
        self.current_pitch = 0.0
        self.min_scan_distance = float('inf')

        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.1, self.fusion_callback)

    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract orientation
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        self.current_pitch = pitch

    def scan_callback(self, msg):
        """Process LIDAR data"""
        valid_distances = [
            r for r in msg.ranges
            if msg.range_min <= r <= msg.range_max
        ]

        if valid_distances:
            self.min_scan_distance = min(valid_distances)
        else:
            self.min_scan_distance = float('inf')

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def fusion_callback(self):
        """Fusion logic combining multiple sensors"""
        # Calculate tilt-based safety factor
        tilt_factor = math.cos(self.current_pitch)

        # Calculate safe distance considering tilt
        if self.min_scan_distance != float('inf'):
            safe_distance = self.min_scan_distance * tilt_factor
        else:
            safe_distance = float('inf')

        # Publish fused results
        tilt_msg = Float64()
        tilt_msg.data = float(self.current_pitch)
        self.tilt_publisher.publish(tilt_msg)

        safe_msg = Float64()
        safe_msg.data = safe_distance
        self.safe_distance_publisher.publish(safe_msg)

        # Log fusion results
        self.get_logger().info(
            f'Fused Data - Pitch: {math.degrees(self.current_pitch):.2f}°, '
            f'Safe Distance: {safe_distance:.2f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorFusion()

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

## Sensor Calibration and Filtering

### Implementing Kalman Filter for Sensor Data

Apply filtering to improve sensor accuracy:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
import numpy as np

class SensorFilter(Node):
    def __init__(self):
        super().__init__('sensor_filter')

        # Subscribe to sensor data
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Kalman filter parameters
        self.kalman_filters = {}  # One filter per joint

        # Process noise and measurement noise
        self.process_noise = 0.1
        self.measurement_noise = 0.5

    def initialize_kalman_filter(self, joint_name):
        """Initialize Kalman filter for a specific joint"""
        # State: [position, velocity]
        # Measurement: [position]
        self.kalman_filters[joint_name] = {
            'state': np.array([0.0, 0.0]),  # [position, velocity]
            'covariance': np.eye(2) * 1000.0,  # High initial uncertainty
            'process_noise': np.eye(2) * self.process_noise,
            'measurement_noise': self.measurement_noise
        }

    def kalman_update(self, joint_name, measurement, dt=0.01):
        """Apply Kalman filter to sensor measurement"""
        if joint_name not in self.kalman_filters:
            self.initialize_kalman_filter(joint_name)

        kf = self.kalman_filters[joint_name]

        # Prediction step
        # State transition matrix (constant velocity model)
        F = np.array([[1, dt],
                      [0, 1]])

        # Control input matrix (none in this case)
        B = np.array([[0.5 * dt**2],
                      [dt]])

        # Process noise covariance
        Q = kf['process_noise']

        # Predict state
        predicted_state = F @ kf['state']

        # Predict covariance
        predicted_covariance = F @ kf['covariance'] @ F.T + Q

        # Update step
        # Measurement matrix (we only measure position)
        H = np.array([[1, 0]])

        # Measurement noise covariance
        R = kf['measurement_noise']

        # Innovation
        innovation = measurement - H @ predicted_state
        innovation_covariance = H @ predicted_covariance @ H.T + R

        # Kalman gain
        kalman_gain = predicted_covariance @ H.T @ np.linalg.inv(innovation_covariance)

        # Update state
        updated_state = predicted_state + kalman_gain @ innovation

        # Update covariance
        updated_covariance = (np.eye(2) - kalman_gain @ H) @ predicted_covariance

        # Store updated values
        kf['state'] = updated_state
        kf['covariance'] = updated_covariance

        return updated_state[0]  # Return filtered position

    def joint_callback(self, msg):
        """Process joint state with filtering"""
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                raw_position = msg.position[i]

                # Apply Kalman filter
                filtered_position = self.kalman_update(joint_name, raw_position)

                # Log results
                self.get_logger().info(
                    f'Joint {joint_name}: Raw={raw_position:.3f}, '
                    f'Filtered={filtered_position:.3f}'
                )

def main(args=None):
    rclpy.init(args=args)
    node = SensorFilter()

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

Sensor integration is fundamental to humanoid robotics, enabling robots to perceive their environment and make informed decisions. This section demonstrated how to integrate various sensor types using rclpy, from basic camera and IMU integration to advanced multi-sensor fusion and filtering techniques. Proper sensor integration is essential for creating responsive and capable humanoid robotic systems.