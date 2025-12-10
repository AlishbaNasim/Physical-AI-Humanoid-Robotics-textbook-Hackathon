---
sidebar_position: 1
---

# URDF Basics for Humanoid Models

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including links, joints, and their relationships. For humanoid robotics, URDF is essential for simulation, visualization, and control.

## URDF Structure

A URDF file describes a robot as a collection of **links** connected by **joints**:

- **Links**: Rigid bodies with physical properties (mass, inertia, visual, collision)
- **Joints**: Connections between links that define degrees of freedom
- **Materials**: Visual properties for rendering
- **Transmissions**: Mapping between actuators and joints (for control)

## Basic URDF Elements

### Robot Definition

Every URDF file starts with a robot tag:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- Robot content goes here -->
</robot>
```

### Link Definition

A link represents a rigid body with various properties:

```xml
<link name="base_link">
    <!-- Inertial properties for physics simulation -->
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>

    <!-- Visual properties for rendering -->
    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <box size="0.2 0.2 0.2"/>
        </geometry>
        <material name="blue">
            <color rgba="0 0 1 1"/>
        </material>
    </visual>

    <!-- Collision properties for physics -->
    <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <box size="0.2 0.2 0.2"/>
        </geometry>
    </collision>
</link>
```

### Joint Definition

Joints connect links and define their relative motion:

```xml
<joint name="joint_name" type="revolute">
    <parent link="parent_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Joint Types

URDF supports several joint types:

- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement (welded connection)
- **floating**: 6 DOF (rarely used)
- **planar**: Movement in a plane (rarely used)

## Coordinate Systems

URDF uses the right-handed coordinate system:
- **X**: Forward
- **Y**: Left
- **Z**: Up

Rotations follow the right-hand rule:
- **Roll**: Rotation around X-axis
- **Pitch**: Rotation around Y-axis
- **Yaw**: Rotation around Z-axis

## Visual and Collision Properties

### Geometry Types

URDF supports several geometry types:

```xml
<!-- Box -->
<geometry>
    <box size="0.1 0.2 0.3"/>
</geometry>

<!-- Cylinder -->
<geometry>
    <cylinder radius="0.1" length="0.2"/>
</geometry>

<!-- Sphere -->
<geometry>
    <sphere radius="0.1"/>
</geometry>

<!-- Mesh -->
<geometry>
    <mesh filename="package://my_robot/meshes/link.stl"/>
</geometry>
```

### Inertial Properties

Proper inertial properties are crucial for physics simulation:

```xml
<inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <!-- Inertia tensor (calculated for the shape) -->
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
</inertial>
```

## Material Definitions

Define materials for visualization:

```xml
<material name="red">
    <color rgba="1 0 0 1"/>
</material>

<material name="green">
    <color rgba="0 1 0 1"/>
</material>

<material name="blue">
    <color rgba="0 0 1 1"/>
</material>
```

## Complete Example

Here's a simple two-link robot as an example:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
    <!-- Base link -->
    <link name="base_link">
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
        </inertial>

        <visual>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <geometry>
                <cylinder radius="0.1" length="0.1"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>

        <collision>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <geometry>
                <cylinder radius="0.1" length="0.1"/>
            </geometry>
        </collision>
    </link>

    <!-- Arm link -->
    <link name="arm_link">
        <inertial>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <mass value="0.5"/>
            <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
        </inertial>

        <visual>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <geometry>
                <box size="0.05 0.05 0.2"/>
            </geometry>
            <material name="red">
                <color rgba="1 0 0 1"/>
            </material>
        </visual>

        <collision>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <geometry>
                <box size="0.05 0.05 0.2"/>
            </geometry>
        </collision>
    </link>

    <!-- Joint connecting base and arm -->
    <joint name="arm_joint" type="revolute">
        <parent link="base_link"/>
        <child link="arm_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
        <dynamics damping="0.1" friction="0.0"/>
    </joint>
</robot>
```

## Validation and Tools

### URDF Validation

Validate your URDF files using:
- `check_urdf` command: `check_urdf /path/to/robot.urdf`
- `urdf_to_graphiz`: Visualize the kinematic tree

### Visualization

Visualize URDF models in:
- RViz with RobotModel plugin
- Gazebo simulation environment
- MeshLab or other 3D viewers

## Best Practices

1. **Start Simple**: Begin with a basic model and add complexity gradually
2. **Use Proper Units**: Always use SI units (meters, kilograms, seconds)
3. **Validate Inertias**: Ensure inertia values are physically realistic
4. **Check Mass Properties**: Verify that masses are appropriate for the robot size
5. **Test in Simulation**: Always test URDF in simulation before use
6. **Use Xacro**: For complex robots, use Xacro to reduce redundancy

## Common Pitfalls

- **Invalid Inertias**: Non-positive definite inertia matrices
- **Missing Masses**: Links without mass values
- **Inconsistent Units**: Mixing different unit systems
- **Kinematic Loops**: Closed loops that aren't properly modeled
- **Improper Origins**: Incorrect coordinate system definitions

## Summary

URDF provides the foundation for robot modeling in ROS, defining the physical and visual properties of robots. Understanding URDF basics is essential for creating humanoid robot models that can be simulated, visualized, and controlled effectively.