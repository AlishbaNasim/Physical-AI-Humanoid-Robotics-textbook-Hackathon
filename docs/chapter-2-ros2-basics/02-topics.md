---
sidebar_position: 2
---

# ROS 2 Topics for Inter-Node Communication

## Introduction to Topics

Topics are one of the primary communication mechanisms in ROS 2, enabling asynchronous, many-to-many communication between nodes. They implement a publish-subscribe pattern where publishers send messages to a topic and subscribers receive those messages.

### How Topics Work

In the publish-subscribe pattern:
- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- Multiple publishers and subscribers can exist for the same topic
- Communication is asynchronous and decoupled

## Topic Characteristics

### Unidirectional Data Flow
Topics provide a one-way communication channel from publishers to subscribers. This is ideal for streaming data like sensor readings, robot states, or camera images.

### Asynchronous Communication
Publishers and subscribers don't need to be synchronized. Publishers can send messages regardless of whether subscribers are ready, and subscribers receive messages as they become available.

### Many-to-Many Communication
Multiple publishers can send to the same topic, and multiple subscribers can receive from the same topic. The data from all publishers is combined and delivered to all subscribers.

## Quality of Service (QoS)

ROS 2 provides Quality of Service settings to control communication behavior:

- **Reliability**: Reliable (all messages delivered) or Best Effort (messages may be lost)
- **Durability**: Transient Local (replay last message to new subscribers) or Volatile (no replay)
- **History**: Keep All messages or Keep Last N messages
- **Depth**: Size of the message queue

## Practical Use Cases in Humanoid Robotics

Topics are commonly used for:
- **Sensor Data**: Camera images, LIDAR scans, IMU readings
- **Robot State**: Joint positions, velocities, and efforts
- **Perception Results**: Detected objects, person tracking
- **System Status**: Battery levels, system health

### Example Topic Names
- `/joint_states`: Current joint positions and velocities
- `/camera/image_raw`: Raw camera images
- `/imu/data`: Inertial measurement unit data
- `/tf`: Transform data between coordinate frames

## Message Types

Each topic has a specific message type that defines the structure of the data. Common message types in robotics include:
- `sensor_msgs`: Sensor data (images, joint states, IMU, etc.)
- `geometry_msgs`: Geometric data (poses, points, vectors)
- `std_msgs`: Basic data types (integers, floats, strings)
- `nav_msgs`: Navigation-related messages

## Implementation Considerations

When using topics:
- Choose appropriate QoS settings based on your application's requirements
- Use standard message types when possible for compatibility
- Consider bandwidth when publishing large messages (like images)
- Be aware of message frequency and processing capabilities

## Best Practices

- Use descriptive topic names following ROS naming conventions
- Consider the frequency of messages to avoid overwhelming the system
- Use appropriate message types for your data
- Implement proper error handling for communication failures

## Summary

Topics provide the asynchronous, many-to-many communication essential for distributed robotic systems. Understanding how to properly use topics with appropriate QoS settings is crucial for building robust humanoid robotic applications.