---
title: Physics Simulation with Gazebo
---

# Physics Simulation with Gazebo

## Introduction to Gazebo Physics

Gazebo is a powerful 3D simulation environment that enables the accurate and efficient simulation of robots and their interactions with the world. It provides physics simulation, realistic rendering, and convenient programmatic interfaces that are essential for robotics development and research.

### What is Gazebo?

Gazebo is a physics-based simulation environment that provides:
- Accurate physics simulation using engines like ODE, Bullet, and DART
- High-fidelity rendering with support for various sensors
- Realistic environment modeling with lighting and atmospheric effects
- Integration with ROS/ROS2 for robot control and communication

### Why Physics Simulation Matters for Humanoid Robots

Physics simulation is crucial for humanoid robots because:
- It allows testing of control algorithms in a safe environment
- It enables rapid prototyping of robot designs
- It provides realistic sensor data for perception algorithm development
- It reduces the cost and risk associated with physical robot testing

## Collision Detection and Response

Collision detection is fundamental to physics simulation. In Gazebo, collision detection ensures that objects interact realistically with each other and their environment.

### Collision Geometries

Gazebo supports various collision geometries:
- **Box**: Rectangular prisms for simple objects
- **Sphere**: Perfect spheres for rounded objects
- **Cylinder**: Cylindrical shapes for wheels and limbs
- **Mesh**: Complex shapes defined by triangle meshes
- **Plane**: Infinite flat surfaces for floors and walls

### Collision Properties

Each collision object in Gazebo has properties that determine how it interacts:
- **Surface friction**: Determines how objects slide against each other
- **Bounce**: Controls the elasticity of collisions
- **Contact parameters**: Define how collision forces are computed

## Physics Properties (Mass, Friction, Restitution)

### Mass Properties

Mass properties define how objects respond to forces and torques:

```xml
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

- **Mass**: The total mass of the link in kilograms
- **Inertia tensor**: Describes how mass is distributed throughout the link
  - `ixx`, `iyy`, `izz`: Moments of inertia about the principal axes
  - `ixy`, `ixz`, `iyz`: Products of inertia (usually zero for symmetric objects)

### Friction Properties

Friction determines how surfaces interact when they slide against each other:

```xml
<gazebo reference="link_name">
  <mu1>1.0</mu1>  <!-- Primary friction coefficient -->
  <mu2>1.0</mu2>  <!-- Secondary friction coefficient -->
  <kp>1000000.0</kp>  <!-- Spring stiffness -->
  <kd>100.0</kd>    <!-- Damping coefficient -->
</gazebo>
```

### Restitution (Bounciness)

Restitution controls how bouncy an object is:
- Value ranges from 0.0 (no bounce) to 1.0 (perfectly elastic)
- Higher values result in more bouncy collisions
- Lower values result in more energy loss during collisions

## Environment Modeling Techniques

### Creating Realistic Environments

Effective environment modeling in Gazebo involves:
- Using appropriate textures and materials for visual realism
- Configuring lighting to match real-world conditions
- Adding environmental effects like fog or atmospheric scattering
- Including interactive objects that robots might encounter

### World Files

Gazebo worlds are defined using SDF (Simulation Description Format):

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Include models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Define lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.3 -0.9</direction>
    </light>

    <!-- Define models -->
    <model name="my_robot">
      <!-- Model definition here -->
    </model>
  </world>
</sdf>
```

## Humanoid Robot Physics Configuration

### Joint Dynamics

For humanoid robots, proper joint dynamics are essential:

```xml
<joint name="hip_joint" type="revolute">
  <parent>torso</parent>
  <child>thigh</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100</effort>
      <velocity>1</velocity>
    </limit>
    <dynamics>
      <damping>1.0</damping>
      <friction>0.1</friction>
    </dynamics>
  </axis>
</joint>
```

### Center of Mass Considerations

Humanoid robots require careful attention to center of mass:
- The center of mass should remain within the support polygon for stability
- Weight distribution affects balance and gait patterns
- Proper mass properties prevent simulation instabilities

## Official Gazebo Documentation References

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [SDF Specification](http://sdformat.org/spec)
- [Physics Engine Comparison](http://gazebosim.org/tutorials?tut=physics)
- [ROS Integration Guide](http://gazebosim.org/tutorials?tut=ros_gzplugins)

## Configuration Examples

### Basic Physics Configuration

```xml
<sdf version="1.7">
  <world name="physics_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <model name="simple_box">
      <pose>0 0 1 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1667</ixx>
            <iyy>0.1667</iyy>
            <izz>0.1667</izz>
          </inertia>
        </inertial>

        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>

        <visual name="box_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Summary

Physics simulation with Gazebo provides the foundation for realistic humanoid robot simulation. Understanding mass properties, collision detection, and environment modeling is essential for creating effective simulation environments. Proper configuration ensures stable, realistic behavior that accurately reflects real-world physics.