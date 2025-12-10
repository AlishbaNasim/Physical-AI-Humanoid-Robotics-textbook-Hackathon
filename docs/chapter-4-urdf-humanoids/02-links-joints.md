---
sidebar_position: 2
---

# Links and Joints in Humanoid URDF Models

## Introduction

In humanoid robot models, links and joints form the fundamental structure that defines the robot's kinematic chain. Understanding how to properly define links and joints is crucial for creating accurate and functional humanoid robot models in URDF.

## Link Definition in Detail

### Link Properties

Each link in a humanoid robot model must define several key properties:

```xml
<link name="link_name">
    <!-- Inertial properties for physics simulation -->
    <inertial>
        <origin xyz="x y z" rpy="roll pitch yaw"/>
        <mass value="mass_value"/>
        <inertia ixx="ixx" ixy="ixy" ixz="ixz" iyy="iyy" iyz="iyz" izz="izz"/>
    </inertial>

    <!-- Visual properties for rendering -->
    <visual>
        <origin xyz="x y z" rpy="roll pitch yaw"/>
        <geometry>
            <!-- Geometry definition -->
        </geometry>
        <material name="material_name"/>
    </visual>

    <!-- Collision properties for physics -->
    <collision>
        <origin xyz="x y z" rpy="roll pitch yaw"/>
        <geometry>
            <!-- Geometry definition -->
        </geometry>
    </collision>
</link>
```

### Link Naming Convention

For humanoid robots, use descriptive names that reflect the body part:

- **Torso**: `torso`, `base_link`, `pelvis`
- **Head**: `head`, `head_link`
- **Arms**: `l_upper_arm`, `l_lower_arm`, `r_upper_arm`, `r_lower_arm`
- **Hands**: `l_hand`, `r_hand`
- **Legs**: `l_upper_leg`, `l_lower_leg`, `r_upper_leg`, `r_lower_leg`
- **Feet**: `l_foot`, `r_foot`

## Joint Definition in Detail

### Joint Properties

Each joint defines the connection between two links:

```xml
<joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <origin xyz="x y z" rpy="roll pitch yaw"/>
    <axis xyz="x y z"/>
    <limit lower="min_value" upper="max_value" effort="max_effort" velocity="max_velocity"/>
    <dynamics damping="damping_value" friction="friction_value"/>
</joint>
```

### Joint Types for Humanoid Robots

Different joint types are appropriate for different parts of a humanoid robot:

#### Revolute Joints
For joints with limited rotation range (e.g., elbows, knees):

```xml
<joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="0.5" effort="100" velocity="2"/>
    <dynamics damping="0.5" friction="0.1"/>
</joint>
```

#### Continuous Joints
For joints with unlimited rotation (e.g., shoulders, hips):

```xml
<joint name="shoulder_joint" type="continuous">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <origin xyz="0.05 0.15 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.2" friction="0.05"/>
</joint>
```

#### Fixed Joints
For permanently connected parts (e.g., sensors mounted on links):

```xml
<joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

## Humanoid Robot Kinematic Structure

### Typical Humanoid Configuration

A humanoid robot typically has this kinematic structure:

```
    pelvis (base)
        ├── torso
        │   ├── head
        │   ├── l_shoulder
        │   │   ├── l_upper_arm
        │   │   │   ├── l_lower_arm
        │   │   │   │   └── l_hand
        │   │   │   └── l_forearm
        │   │   └── l_arm
        │   ├── r_shoulder
        │   │   ├── r_upper_arm
        │   │   │   ├── r_lower_arm
        │   │   │   │   └── r_hand
        │   │   │   └── r_forearm
        │   │   └── r_arm
        │   ├── l_hip
        │   │   ├── l_upper_leg
        │   │   │   └── l_lower_leg
        │   │   │       └── l_foot
        │   └── r_hip
        │       ├── r_upper_leg
        │       │   └── r_lower_leg
        │       │       └── r_foot
```

### Complete Example: Simple Humanoid Torso

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_torso">
    <!-- Pelvis (base link) -->
    <link name="pelvis">
        <inertial>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <mass value="5.0"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
        </inertial>

        <visual>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <geometry>
                <box size="0.2 0.3 0.2"/>
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1"/>
            </material>
        </visual>

        <collision>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <geometry>
                <box size="0.2 0.3 0.2"/>
            </geometry>
        </collision>
    </link>

    <!-- Torso -->
    <link name="torso">
        <inertial>
            <origin xyz="0 0 0.3" rpy="0 0 0"/>
            <mass value="4.0"/>
            <inertia ixx="0.15" ixy="0.0" ixz="0.0" iyy="0.15" iyz="0.0" izz="0.15"/>
        </inertial>

        <visual>
            <origin xyz="0 0 0.3" rpy="0 0 0"/>
            <geometry>
                <box size="0.15 0.25 0.6"/>
            </geometry>
            <material name="light_gray">
                <color rgba="0.7 0.7 0.7 1"/>
            </material>
        </visual>

        <collision>
            <origin xyz="0 0 0.3" rpy="0 0 0"/>
            <geometry>
                <box size="0.15 0.25 0.6"/>
            </geometry>
        </collision>
    </link>

    <!-- Head -->
    <link name="head">
        <inertial>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <mass value="2.0"/>
            <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
        </inertial>

        <visual>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <geometry>
                <sphere radius="0.1"/>
            </geometry>
            <material name="skin">
                <color rgba="1 0.8 0.6 1"/>
            </material>
        </visual>

        <collision>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <geometry>
                <sphere radius="0.1"/>
            </geometry>
        </collision>
    </link>

    <!-- Hip joint (pelvis to torso) -->
    <joint name="torso_joint" type="revolute">
        <parent link="pelvis"/>
        <child link="torso"/>
        <origin xyz="0 0 0.2" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="200" velocity="2"/>
        <dynamics damping="1.0" friction="0.2"/>
    </joint>

    <!-- Neck joint (torso to head) -->
    <joint name="neck_joint" type="revolute">
        <parent link="torso"/>
        <child link="head"/>
        <origin xyz="0 0 0.6" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-0.78" upper="0.78" effort="50" velocity="3"/>
        <dynamics damping="0.5" friction="0.1"/>
    </joint>
</robot>
```

## Joint Limits and Safety

### Appropriate Joint Limits

For humanoid robots, joint limits should reflect human-like ranges of motion:

```xml
<!-- Shoulder joint - typical human range -->
<joint name="shoulder_joint" type="revolute">
    <limit lower="-2.0" upper="2.0" effort="100" velocity="2"/>
</joint>

<!-- Elbow joint - typically -5 to 160 degrees -->
<joint name="elbow_joint" type="revolute">
    <limit lower="-0.09" upper="2.79" effort="80" velocity="2.5"/>
</joint>

<!-- Knee joint - typically 0 to 150 degrees -->
<joint name="knee_joint" type="revolute">
    <limit lower="0.0" upper="2.62" effort="150" velocity="2"/>
</joint>
```

### Safety Considerations

Include safety margins in joint limits:

```xml
<!-- Use limits that are slightly more restrictive than physical limits -->
<joint name="dangerous_joint" type="revolute">
    <limit lower="-1.4" upper="1.4" effort="100" velocity="1"/>
    <!-- Actual physical limits might be -1.57 to 1.57 -->
</joint>
```

## Transmissions for Control

Define transmissions to connect joints to actuators:

```xml
<transmission name="elbow_transmission">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="elbow_joint">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="elbow_motor">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

## Gazebo-Specific Elements

For simulation in Gazebo, add Gazebo-specific elements:

```xml
<gazebo reference="link_name">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Blue</material>
</gazebo>
```

## Best Practices for Humanoid Models

1. **Use Consistent Naming**: Follow a consistent naming scheme for all links and joints
2. **Realistic Mass Properties**: Use mass values that reflect real-world humanoid proportions
3. **Appropriate Joint Limits**: Set limits that reflect human-like ranges of motion
4. **Proper Origins**: Ensure joint origins connect links correctly in 3D space
5. **Validate Kinematics**: Check that the kinematic chain is properly connected
6. **Test in Simulation**: Always test the model in simulation before use

## Common Issues and Solutions

### Kinematic Chain Issues
- **Disconnected links**: Ensure every link (except the base) is connected by a joint
- **Multiple parents**: Each link (except the base) should have exactly one parent

### Physical Property Issues
- **Zero mass**: All links must have a positive mass value
- **Invalid inertias**: Ensure inertia values follow physical constraints

### Joint Issues
- **Incorrect axes**: Joint axes should align with intended motion
- **Improper limits**: Joint limits should reflect physical capabilities

## Summary

Properly defining links and joints is fundamental to creating functional humanoid robot models. The kinematic structure must accurately represent the robot's physical connections and degrees of freedom, with appropriate physical properties for realistic simulation and control.