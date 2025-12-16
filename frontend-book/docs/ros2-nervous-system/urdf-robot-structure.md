---
title: Robot Structure with URDF
---

# Robot Structure with URDF

## Understanding URDF for Humanoid Robots

The Unified Robot Description Format (URDF) is an XML-based format that describes robot models in ROS. For humanoid robots, URDF is essential as it defines the physical structure, kinematic chains, and visual properties of the robot.

### What is URDF?

URDF (Unified Robot Description Format) is an XML format that defines:
- Robot kinematic structure (joints and links)
- Visual and collision properties
- Physical properties (mass, inertia, etc.)
- Materials and colors

### Why URDF Matters for Humanoid Robots

Humanoid robots have complex kinematic structures with multiple limbs, each containing several joints. URDF allows us to:
- Define the complete kinematic chain from base to end effectors
- Model complex multi-branch structures (arms, legs, head)
- Prepare robot models for simulation and real-world deployment
- Integrate with ROS tools like RViz, Gazebo, and MoveIt

## Link and Joint Definitions

### Links

Links represent rigid bodies in the robot. For a humanoid robot, links might include:
- Torso
- Head
- Upper arms, lower arms, hands
- Upper legs, lower legs, feet

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

### Joints

Joints define the connections between links and their motion properties:

```xml
<joint name="torso_head_joint" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
</joint>
```

## Visual and Collision Properties

### Visual Elements

Visual elements define how the robot appears in simulation and visualization tools:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://humanoid_description/meshes/head.dae"/>
  </geometry>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
</visual>
```

### Collision Elements

Collision elements define the physical collision boundaries:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://humanoid_description/meshes/head_collision.dae"/>
  </geometry>
</collision>
```

## Materials and Colors

Materials define the visual appearance of robot parts:

```xml
<material name="blue">
  <color rgba="0.0 0.0 1.0 1.0"/>
</material>

<material name="red">
  <color rgba="1.0 0.0 0.0 1.0"/>
</material>
```

## Complete Humanoid Robot Example

Here's a simplified example of a humanoid robot URDF structure:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="torso_head_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5.0" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="torso_left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Additional links and joints for complete humanoid structure -->
</robot>
```

## Simulation Readiness Considerations

### URDF vs. XACRO

For complex humanoid robots, consider using XACRO (XML Macros) to make URDF files more maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="simple_arm" params="prefix parent_link">
    <link name="${prefix}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
      </visual>
    </link>

    <joint name="${parent_link}_${prefix}_shoulder_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${prefix}_upper_arm"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <xacro:simple_arm prefix="left" parent_link="torso"/>
  <xacro:simple_arm prefix="right" parent_link="torso"/>
</robot>
```

### Validation

Always validate your URDF files:

```bash
# Check for XML syntax errors
xmllint --noout your_robot.urdf

# Use check_urdf tool to validate kinematic structure
check_urdf your_robot.urdf
```

## Best Practices for Humanoid Robots

1. **Start Simple**: Begin with a basic skeleton and add complexity gradually
2. **Use Standard Joints**: Follow ROS conventions for joint naming and types
3. **Consider Mass Distribution**: Properly weight your robot for realistic simulation
4. **Collision vs Visual**: Use simplified collision geometry for better performance
5. **Documentation**: Comment your URDF files to explain the structure

## Summary

URDF is fundamental for humanoid robotics in ROS. It defines the robot's physical structure, enabling:
- Visualization in RViz
- Physics simulation in Gazebo
- Motion planning with MoveIt
- Integration with other ROS tools

Understanding URDF is crucial for developing humanoid robots that can interact with the ROS ecosystem effectively.