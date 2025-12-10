---
sidebar_position: 4
---

# ROS 2 Actions for Goal-Oriented Communication

## Introduction to Actions

Actions in ROS 2 provide a communication pattern for long-running tasks that require feedback and goal management. Unlike services, which are synchronous and blocking, actions allow for asynchronous execution with continuous feedback and the ability to cancel ongoing tasks. This makes them ideal for operations like navigation, manipulation, or any process that takes a significant amount of time to complete.

### How Actions Work

The action communication pattern involves three message types:
- **Goal**: The request sent to start an action
- **Feedback**: Continuous updates on the progress of the action
- **Result**: The final outcome of the action when completed

Actions also support cancellation, allowing clients to request that a running action be stopped.

## Action Characteristics

### Goal-Feedback-Result Pattern
- **Goal**: Sent by the client to request a specific action
- **Feedback**: Continuous updates sent by the server during execution
- **Result**: Final outcome sent by the server when the action completes

### Asynchronous Execution
Actions run asynchronously, allowing the client to continue other operations while the action is executing. The client can monitor progress through feedback messages.

### Cancellation Support
Clients can request cancellation of an action in progress, and servers can implement cancellation logic.

### Goal Status Tracking
Actions provide status information (active, succeeded, aborted, canceled, etc.) allowing clients to track the state of their requests.

## Action Structure

An action definition consists of:
- **Goal**: The data structure for the goal request
- **Result**: The data structure for the final result
- **Feedback**: The data structure for intermediate feedback
- **Separators**: Lines with `---` separating the different message parts

## Practical Use Cases in Humanoid Robotics

Actions are commonly used for:
- **Navigation**: Moving to a specific location with feedback on progress
- **Manipulation**: Grasping or moving objects with status updates
- **Trajectory Execution**: Following motion paths with progress feedback
- **Calibration**: Long-running calibration procedures
- **Data Processing**: Time-consuming analysis tasks

### Example Action Calls
- `move_base`: Navigate to a goal position with feedback on progress
- `joint_trajectory`: Execute a sequence of joint positions over time
- `pick_and_place`: Perform object manipulation with status updates
- `mapping`: Build a map of the environment with progress feedback

## Action vs. Service vs. Topic Decision

Use actions when you need:
- Long-running operations with progress feedback
- The ability to cancel ongoing operations
- Goal-oriented behavior with status tracking
- Continuous feedback during execution

Use services when you need:
- Synchronous request-response behavior
- Immediate results without intermediate feedback
- Simple, quick operations

Use topics when you need:
- Streaming data without specific goals
- Continuous updates without request-response pattern
- Many-to-many communication

## Implementation Considerations

When using actions:
- Implement proper feedback publishing at appropriate intervals
- Handle cancellation requests gracefully
- Design goal messages with appropriate parameters
- Consider the impact of long-running operations on system responsiveness
- Implement proper error handling for various failure modes

## Best Practices

- Use descriptive action names following ROS naming conventions
- Design feedback messages to provide meaningful progress information
- Implement appropriate cancellation logic in action servers
- Use appropriate time limits and timeouts for goals
- Document expected behavior for different scenarios (success, failure, cancellation)
- Consider the state management requirements of your action servers

## Action States

Actions can be in various states:
- **PENDING**: Goal received but not yet started
- **ACTIVE**: Goal is currently being processed
- **PREEMPTED**: Goal was canceled or replaced
- **SUCCEEDED**: Goal completed successfully
- **ABORTED**: Goal failed during execution
- **RECALLED**: Goal was canceled before execution started

## Error Handling

Actions can fail in various ways:
- Goal validation errors
- Execution failures
- Cancellation requests
- Timeouts

Implement comprehensive error handling to ensure robust operation.

## Summary

Actions provide the sophisticated communication pattern needed for long-running, goal-oriented operations in robotic systems. Understanding when and how to use actions is crucial for building responsive robotic applications that can handle complex, time-consuming tasks with proper feedback and control.