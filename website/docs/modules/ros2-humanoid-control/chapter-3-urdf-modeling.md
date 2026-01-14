---
sidebar_position: 4
title: 'Chapter 3: Humanoid Modeling with URDF'
description: 'Understanding Unified Robot Description Format and its role in ROS 2 and simulation'
---

# Chapter 3: Humanoid Modeling with URDF

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links, joints, inertial properties, visual meshes, and collision geometries.

URDF plays a crucial role in ROS 2 by enabling:
- Robot visualization in RViz
- Physics simulation in Gazebo and other simulators
- Kinematic analysis and inverse kinematics
- Motion planning algorithms

## The Purpose of URDF

URDF serves as the standard way to represent robot models in the ROS ecosystem. It allows developers to:

### Define Physical Structure
- Specify the geometric shape and size of robot parts
- Describe how different parts are connected via joints
- Define mass properties and inertial characteristics
- Set visual appearance and materials

### Enable Simulation
- Create accurate physics models for robot simulation
- Test control algorithms in a virtual environment before deployment
- Validate robot designs without building physical prototypes

### Support Advanced Algorithms
- Provide kinematic models for motion planning
- Enable collision detection and avoidance
- Facilitate robot calibration and state estimation

## Links, Joints, and Kinematic Chains

### Links

A **link** in URDF represents a rigid body part of the robot. Each link can have:

- **Visual properties**: How the link appears in visualization
- **Collision properties**: Geometry used for collision detection
- **Inertial properties**: Mass, center of mass, and inertia tensor

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joints

A **joint** connects two links and defines their relative motion. URDF supports several joint types:

- **Fixed**: No relative motion between links
- **Revolute**: Rotational motion around a single axis (with limits)
- **Continuous**: Unlimited rotational motion around a single axis
- **Prismatic**: Linear motion along a single axis (with limits)
- **Floating**: 6 degrees of freedom (6DOF) motion
- **Planar**: Motion on a plane

```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0.2 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

### Kinematic Chains

A **kinematic chain** is a series of links connected by joints that form a path from a base link to an end effector. In humanoid robots, examples include:

- **Arm chain**: From torso to hand
- **Leg chain**: From hip to foot
- **Head chain**: From neck to head

Kinematic chains are essential for:
- Forward kinematics (calculating end effector position from joint angles)
- Inverse kinematics (calculating joint angles to achieve desired end effector position)
- Motion planning and trajectory generation

## URDF Structure for Humanoid Robots

Humanoid robots have complex kinematic structures with multiple limbs. A typical humanoid URDF includes:

### Torso and Head
- Base link representing the main body
- Neck joint for head movement
- Head link with sensors (cameras, IMUs)

### Arms
- Shoulder joints for upper arm movement
- Elbow joints for lower arm movement
- Wrist joints for hand positioning
- Hand/End effector definitions

### Legs
- Hip joints for upper leg movement
- Knee joints for lower leg movement
- Ankle joints for foot positioning
- Foot links for ground contact

Here's a simplified example of a humanoid arm structure:

```xml
<!-- Left Arm -->
<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso_link"/>
  <child link="left_upper_arm_link"/>
  <origin xyz="0.2 0.15 0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>

<link name="left_upper_arm_link">
  <visual>
    <geometry>
      <capsule length="0.3" radius="0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <capsule length="0.3" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>

<joint name="left_elbow_yaw" type="revolute">
  <parent link="left_upper_arm_link"/>
  <child link="left_lower_arm_link"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.0" upper="2.0" effort="100" velocity="1"/>
</joint>

<link name="left_lower_arm_link">
  <visual>
    <geometry>
      <capsule length="0.25" radius="0.04"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <capsule length="0.25" radius="0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.5"/>
    <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
  </inertial>
</link>
```

## URDF's Role in ROS 2 and Simulation

### ROS 2 Integration

URDF integrates with ROS 2 through several mechanisms:

#### Robot State Publisher
The `robot_state_publisher` package takes a URDF and joint positions to publish the resulting transforms to the TF2 tree, allowing other ROS nodes to understand the spatial relationship between different parts of the robot.

```python
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state_node')

        # Load URDF from parameter or file
        robot_description = self.get_parameter_or(
            'robot_description',
            '/path/to/robot.urdf'
        ).value

        # Parse URDF
        robot = URDF.from_xml_string(robot_description)

        # Use URDF for kinematic calculations
        tree = kdl_tree_from_urdf_model(robot)
```

#### Visualization
URDF models are visualized in RViz2 using the RobotModel display type, allowing developers to see how their robot will appear in the real world or simulation.

### Simulation Environments

URDF models are used in various simulation environments:

#### Gazebo/Harmonic
- Physics simulation with realistic dynamics
- Sensor simulation (cameras, LiDAR, IMUs)
- Environment interaction

#### Ignition Gazebo
- Modern simulation engine
- High-fidelity physics and rendering
- Realistic sensor models

#### Webots
- Web-based simulation platform
- Multi-robot simulation capabilities
- Realistic physics and sensors

## Creating and Validating URDF Models

### Best Practices

When creating URDF models for humanoid robots:

1. **Start Simple**: Begin with a basic kinematic model and gradually add complexity
2. **Use Standard Formats**: Follow URDF conventions for consistency
3. **Include Proper Inertias**: Accurate mass properties are crucial for simulation
4. **Validate Joint Limits**: Ensure joint ranges match physical capabilities
5. **Test Incrementally**: Add one limb at a time and test each addition

### Validation Tools

Several tools help validate URDF models:

- **check_urdf**: Checks syntax and basic validity
- **urdf_tutorial**: Provides examples and validation procedures
- **RViz**: Visual inspection of the robot model
- **TF2 tools**: Check transform relationships

Example validation command:
```bash
check_urdf /path/to/robot.urdf
```

### Common Pitfalls

- **Incorrect Origins**: Joint origins that don't match physical mounting points
- **Missing Inertias**: Links without proper mass/inertia properties
- **Invalid Joint Limits**: Limits that exceed physical capabilities
- **Non-unique Names**: Duplicate link or joint names

## URDF Extensions and Alternatives

While URDF is the standard in ROS, there are extensions and alternatives:

### XACRO
XACRO (XML Macros) extends URDF with:
- Parameter substitution
- Mathematical expressions
- Macro definitions
- File inclusion

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

<xacro:property name="M_PI" value="3.1415926535897931" />
<xacro:property name="arm_length" value="0.3" />

<xacro:macro name="simple_arm" params="prefix parent_link">
  <joint name="${prefix}_shoulder_joint" type="revolute">
    <parent link="${parent_link}"/>
    <child link="${prefix}_upper_arm_link"/>
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
  </joint>

  <link name="${prefix}_upper_arm_link">
    <visual>
      <geometry>
        <capsule length="${arm_length}" radius="0.05"/>
      </geometry>
    </visual>
  </link>
</xacro:macro>

<!-- Use the macro -->
<xacro:simple_arm prefix="left" parent_link="torso_link"/>

</robot>
```

### SDF (Simulation Description Format)
Used primarily in Gazebo/Ignition with more advanced simulation features.

## Practical Applications

### Motion Planning
URDF models enable motion planning algorithms to:
- Calculate inverse kinematics solutions
- Plan collision-free trajectories
- Optimize joint configurations

### Control Systems
Controllers use URDF models to:
- Implement computed torque control
- Calculate dynamic compensation
- Estimate robot state

### Perception Systems
URDF models help perception systems:
- Register sensor data to robot frame
- Segment robot parts from environment
- Track robot pose and configuration

## Summary

In this chapter, we've covered:

- The purpose of URDF in describing robot models
- Links, joints, and kinematic chains in robot structure
- URDF's role in ROS 2 and simulation environments
- Best practices for creating and validating URDF models
- Extensions like XACRO for more complex descriptions
- Practical applications in motion planning and control

URDF is fundamental to robotics development in ROS, especially for complex humanoid robots where accurate modeling of kinematic chains is essential for successful operation.

## Learning Objectives Review

After completing this chapter, you should be able to:
- Explain the purpose of URDF in robot modeling
- Describe links, joints, and kinematic chains concepts
- Understand URDF's role in ROS 2 and simulation
- Create basic URDF models for robot components
- Identify best practices for URDF development

## Previous and Next Steps

Previous: [Chapter 2: Python Agents with rclpy](./chapter-2-python-agents-rclpy.md)

Return to [Module Home](./index.md) or explore additional ROS 2 topics.