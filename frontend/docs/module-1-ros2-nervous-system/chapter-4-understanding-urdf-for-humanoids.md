---
title: Understanding URDF for Humanoids
sidebar_position: 4
---

# Understanding URDF for Humanoids: Robot Modeling and Structure

## Overview

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. For humanoid robots, URDF defines the robot's physical structure, kinematic properties, and visual/collision representations.

## URDF Fundamentals

URDF describes a robot as links connected by joints:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Link with visual, collision, and inertial properties -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joint connecting links -->
  <joint name="base_to_upper_body" type="fixed">
    <parent link="base_link"/>
    <child link="upper_body"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>
```

## Robot Links and Joints Concepts

### Links
A link represents a rigid body with:
- **Visual**: How the link appears in visualization
- **Collision**: How the link interacts in simulation
- **Inertial**: Physical properties for dynamics

### Joints
Joints define how links connect. Types include:
- **Fixed**: No movement
- **Revolute**: Single axis rotation with limits
- **Continuous**: Single axis rotation without limits
- **Prismatic**: Single axis translation

## Inertial Properties and Dynamics

For accurate simulation, each link needs:
- **Mass**: The mass of the link
- **Inertia Tensor**: How mass is distributed

For a cylinder: Ixx = Iyy = (1/12)*m*(3*r² + h²), Izz = (1/2)*m*r²

## Visual vs Collision Meshes

- **Visual meshes**: High-resolution for appearance, don't affect physics
- **Collision meshes**: Simplified for performance, affect physics simulation

## Humanoid Skeleton Structure

Humanoid robots mimic human structure:
- **Degrees of Freedom**: Joints that replicate human movement
- **Proportions**: Human-like ratios for natural movement

A basic humanoid has 20-30+ degrees of freedom:
- **Leg**: 6 DOF (hip: 3, knee: 1, ankle: 2)
- **Arm**: 7 DOF (shoulder: 3, elbow: 1, wrist: 3)
- **Trunk/Head**: 2-4 DOF

## Complete URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Pelvis -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.25"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.15" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.15 0.15 0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.25" iyz="0.0" izz="0.15"/>
    </inertial>
  </link>
</robot>
```

## URDF Best Practices

- Use Xacro for complex robots to simplify repetitive elements
- Use descriptive, consistent names for links and joints
- Validate URDF files with `check_urdf` command
- Comment complex sections to explain design decisions

## Summary

URDF is fundamental to humanoid robotics in ROS, providing the description of the robot's physical structure. Proper URDF modeling is essential for simulation, visualization, kinematics, and dynamics in humanoid robot applications.

## References

1. ROS 2 Documentation Team. (2023). ROS 2 Humble Hawksbill Documentation. Open Robotics. https://docs.ros.org/en/humble/
2. Witt, J., Lawitzky, G., Wiedemeyer, T., & Beetz, M. (2020). Humanoid Robot Control and Perception Using ROS 2. Journal of Intelligent & Robotic Systems, 99(3-4), 871-887.