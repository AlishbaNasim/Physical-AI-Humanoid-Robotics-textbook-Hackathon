---
sidebar_position: 3
---

# ROS 2 Services for Request-Response Communication

## Introduction to Services

Services in ROS 2 provide synchronous, request-response communication between nodes. Unlike topics which provide asynchronous, streaming communication, services allow a client to send a request to a server and wait for a response. This is ideal for operations that require a specific outcome or result.

### How Services Work

The service communication pattern involves:
- **Service Server**: A node that provides a specific service
- **Service Client**: A node that requests the service
- **Request Message**: The data sent from client to server
- **Response Message**: The data sent from server back to client

This is a synchronous, one-to-one communication pattern where the client waits for the server to process the request and return a response.

## Service Characteristics

### Synchronous Communication
Unlike topics, service calls are synchronous. The client waits for the server to process the request and return a response before continuing execution.

### Request-Response Pattern
Each service call consists of a request from the client and a response from the server. This makes services ideal for operations that have a clear input and output.

### One-to-One Communication
While multiple clients can call the same service, each call is handled individually by the server.

## Service Structure

A service definition consists of:
- **Request**: The data structure sent from client to server
- **Response**: The data structure sent from server to client
- **Separator**: A line with `---` separating request and response

## Practical Use Cases in Humanoid Robotics

Services are commonly used for:
- **Configuration**: Setting parameters or modes
- **Action Commands**: Requesting specific actions with confirmation
- **Data Queries**: Requesting specific information from a node
- **Calibration**: Performing calibration procedures with results
- **Initialization**: Starting up systems with success confirmation

### Example Service Calls
- `set_parameters`: Change configuration parameters
- `get_state`: Request current robot state
- `execute_trajectory`: Request execution of a motion trajectory
- `save_map`: Request saving a map with success confirmation

## Service vs. Topic Decision

Use services when you need:
- A specific response to a request
- Confirmation that an operation completed successfully
- Synchronous execution
- Request-response interaction pattern

Use topics when you need:
- Streaming data
- Asynchronous communication
- Many-to-many communication
- Continuous updates

## Implementation Considerations

When using services:
- Consider the blocking nature of service calls in your application design
- Implement appropriate timeouts to avoid indefinite waiting
- Handle service call failures gracefully
- Design service interfaces to be idempotent when possible

## Best Practices

- Use descriptive service names following ROS naming conventions
- Design services with clear, well-defined request and response messages
- Implement proper error handling and timeouts
- Consider the impact of blocking calls on your application's responsiveness
- Document the expected behavior and possible responses

## Error Handling

Service calls can fail for various reasons:
- Service server not available
- Request timeout
- Invalid request parameters
- Server processing error

Always implement error handling for service calls to ensure robust operation.

## Summary

Services provide the synchronous, request-response communication needed for operations that require confirmation or specific results. Understanding when and how to use services is essential for building complete robotic applications that require coordinated operations.