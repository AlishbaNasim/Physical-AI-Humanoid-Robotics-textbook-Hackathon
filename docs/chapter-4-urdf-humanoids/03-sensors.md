---
sidebar_position: 3
---

# Sensors in Humanoid URDF Models

## Introduction

Sensors are critical components of humanoid robots, providing perception capabilities that enable the robot to interact with its environment. This section covers how to properly define and integrate various sensor types in URDF models for humanoid robots.

## Sensor Integration Overview

Sensors in URDF are typically represented as additional links connected to the main robot structure through fixed joints. This approach allows for proper coordinate frame definition and transformation in the robot's kinematic chain.

### Basic Sensor Structure

```xml
<!-- Sensor link definition -->
<link name="sensor_link_name">
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.01"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>

    <!-- Visual representation (optional) -->
    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <box size="0.02 0.02 0.01"/>
        </geometry>
        <material name="black">
            <color rgba="0 0 0 1"/>
        </material>
    </visual>
</link>

<!-- Fixed joint connecting sensor to main structure -->
<joint name="sensor_joint_name" type="fixed">
    <parent link="main_link_name"/>
    <child link="sensor_link_name"/>
    <origin xyz="x_offset y_offset z_offset" rpy="roll pitch yaw"/>
</joint>
```

## Camera Sensors

### RGB Camera Definition

```xml
<link name="camera_link">
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.1"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>

    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <box size="0.03 0.03 0.01"/>
        </geometry>
        <material name="black">
            <color rgba="0 0 0 1"/>
        </material>
    </visual>
</link>

<joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for camera simulation -->
<gazebo reference="camera_link">
    <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <camera name="head_camera">
            <horizontal_fov>1.047</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.1</near>
                <far>30.0</far>
            </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <frame_name>camera_link</frame_name>
            <topic_name>camera/image_raw</topic_name>
        </plugin>
    </sensor>
</gazebo>
```

### Depth Camera Definition

```xml
<link name="depth_camera_link">
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.2"/>
        <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
    </inertial>
</link>

<joint name="depth_camera_joint" type="fixed">
    <parent link="head"/>
    <child link="depth_camera_link"/>
    <origin xyz="0.05 0.05 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="depth_camera_link">
    <sensor name="depth_camera" type="depth">
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <camera name="depth_head_camera">
            <horizontal_fov>1.047</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
            </image>
            <clip>
                <near>0.1</near>
                <far>10.0</far>
            </clip>
        </camera>
        <plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
            <frame_name>depth_camera_link</frame_name>
            <depth_image_topic_name>depth/image_raw</depth_image_topic_name>
            <point_cloud_topic_name>depth/points</point_cloud_topic_name>
        </plugin>
    </sensor>
</gazebo>
```

## IMU Sensors

### IMU Integration

```xml
<link name="imu_link">
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.01"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
</link>

<joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <imu>
            <angular_velocity>
                <x>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>0.001</stddev>
                    </noise>
                </x>
                <y>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>0.001</stddev>
                    </noise>
                </y>
                <z>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>0.001</stddev>
                    </noise>
                </z>
            </angular_velocity>
            <linear_acceleration>
                <x>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>0.017</stddev>
                    </noise>
                </x>
                <y>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>0.017</stddev>
                    </noise>
                </y>
                <z>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>0.017</stddev>
                    </noise>
                </z>
            </linear_acceleration>
        </imu>
    </sensor>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <frame_name>imu_link</frame_name>
        <topic_name>imu/data</topic_name>
        <gaussian_noise>0.001</gaussian_noise>
    </plugin>
</gazebo>
```

## Force/Torque Sensors

### Wrench Sensor Integration

```xml
<link name="ft_sensor_link">
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.05"/>
        <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0005"/>
    </inertial>
</link>

<joint name="ft_sensor_joint" type="fixed">
    <parent link="l_wrist"/>
    <child link="ft_sensor_link"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="ft_sensor_link">
    <sensor name="ft_sensor" type="force_torque">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <force_torque>
            <frame>child</frame>
            <measure_direction>child_to_parent</measure_direction>
        </force_torque>
    </sensor>
    <plugin name="ft_sensor_plugin" filename="libgazebo_ros_ft_sensor.so">
        <frame_name>ft_sensor_link</frame_name>
        <topic_name>ft_sensor/wrench</topic_name>
    </plugin>
</gazebo>
```

## LIDAR Sensors

### 2D LIDAR Integration

```xml
<link name="lidar_link">
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.5"/>
        <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>

    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <cylinder radius="0.05" length="0.05"/>
        </geometry>
        <material name="gray">
            <color rgba="0.5 0.5 0.5 1"/>
        </material>
    </visual>
</link>

<joint name="lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <ray>
            <scan>
                <horizontal>
                    <samples>720</samples>
                    <resolution>1</resolution>
                    <min_angle>-3.14159</min_angle>
                    <max_angle>3.14159</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.1</min>
                <max>30.0</max>
                <resolution>0.01</resolution>
            </range>
        </ray>
    </sensor>
    <plugin name="lidar_plugin" filename="libgazebo_ros_laser.so">
        <frame_name>lidar_link</frame_name>
        <topic_name>scan</topic_name>
    </plugin>
</gazebo>
```

## Multiple Sensor Integration Example

Here's a complete example showing multiple sensors on a humanoid head:

```xml
<?xml version="1.0"?>
<robot name="humanoid_head_with_sensors">
    <!-- Head link -->
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

    <!-- RGB Camera -->
    <link name="camera_link">
        <inertial>
            <mass value="0.1"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>

    <joint name="camera_joint" type="fixed">
        <parent link="head"/>
        <child link="camera_link"/>
        <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
    </joint>

    <!-- IMU -->
    <link name="imu_link">
        <inertial>
            <mass value="0.01"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
        </inertial>
    </link>

    <joint name="imu_joint" type="fixed">
        <parent link="head"/>
        <child link="imu_link"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
    </joint>

    <!-- Gazebo plugins -->
    <gazebo reference="camera_link">
        <sensor name="camera" type="camera">
            <always_on>true</always_on>
            <update_rate>30</update_rate>
            <camera name="head_camera">
                <horizontal_fov>1.047</horizontal_fov>
                <image>
                    <width>640</width>
                    <height>480</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.1</near>
                    <far>30.0</far>
                </clip>
            </camera>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                <frame_name>camera_link</frame_name>
                <topic_name>head_camera/image_raw</topic_name>
            </plugin>
        </sensor>
    </gazebo>

    <gazebo reference="imu_link">
        <sensor name="imu_sensor" type="imu">
            <always_on>true</always_on>
            <update_rate>100</update_rate>
        </sensor>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
            <frame_name>imu_link</frame_name>
            <topic_name>imu/data</topic_name>
        </plugin>
    </gazebo>
</robot>
```

## Sensor Placement Considerations

### Field of View
- Position cameras with appropriate field of view for the intended application
- Consider overlapping fields of view for stereo vision or redundancy

### Coverage
- Ensure adequate sensor coverage of the robot's workspace
- Position sensors to minimize blind spots

### Protection
- Position sensors to minimize damage from impacts
- Consider sensor placement relative to robot limbs that might block the field of view

### Calibration
- Position sensors with known geometric relationships for calibration
- Consider accessibility for maintenance and recalibration

## Best Practices

1. **Proper Coordinate Frames**: Ensure sensor frames are correctly oriented for intended use
2. **Realistic Noise Models**: Include appropriate noise models for simulation
3. **Consistent Units**: Use consistent units throughout sensor definitions
4. **Validation**: Test sensor placement in simulation before physical implementation
5. **Documentation**: Document sensor specifications and frame relationships
6. **Safety**: Position sensors to avoid interference with robot motion

## Gazebo Sensor Plugins

Common Gazebo sensor plugins for humanoid robots:
- `libgazebo_ros_camera.so`: RGB camera
- `libgazebo_ros_openni_kinect.so`: Depth camera
- `libgazebo_ros_imu.so`: IMU sensor
- `libgazebo_ros_laser.so`: LIDAR sensor
- `libgazebo_ros_ft_sensor.so`: Force/torque sensor
- `libgazebo_ros_p3d.so`: 3D position sensor

## Summary

Proper sensor integration in humanoid URDF models is essential for enabling perception capabilities. Sensors should be carefully positioned and configured with appropriate physical and simulation properties to enable effective robot perception and interaction with the environment.