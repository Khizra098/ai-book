# Quickstart: Digital Twin (Gazebo & Unity)

**Feature**: Module 2: The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-16
**Status**: Complete

## Overview

This quickstart guide provides a rapid introduction to the Digital Twin module, covering the essential setup and first steps for working with Gazebo and Unity simulation environments for humanoid robotics.

## Prerequisites

Before starting with the Digital Twin module, ensure you have:

1. **System Requirements**:
   - 64-bit operating system (Ubuntu 20.04/22.04 recommended, Windows 10/11, or macOS 10.15+)
   - 8GB+ RAM (16GB+ recommended for Unity)
   - Modern CPU with SSE2 support
   - Dedicated GPU with OpenGL 3.3+ support (for Unity visualization)
   - 20GB+ available disk space

2. **Software Requirements**:
   - Git version control system
   - Node.js 18+ LTS (for Docusaurus documentation)
   - Gazebo Classic 11.x or newer
   - Unity Hub with Unity 2022.3 LTS or newer
   - ROS Noetic (for ROS/Gazebo integration) or ROS2 Humble Hawksbill

## Setup Process

### 1. Clone and Navigate to Documentation

```bash
git clone [repository-url]
cd [repository-name]
```

### 2. Install Documentation Dependencies

```bash
cd frontend-book
npm install
```

### 3. Start Documentation Server

```bash
npm start
```

The documentation will be available at `http://localhost:3000`

## First Steps with Gazebo Simulation

### 1. Basic Gazebo Environment

Start with the simplest Gazebo simulation to understand physics concepts:

1. Launch Gazebo:
   ```bash
   gazebo
   ```

2. Load a basic world file to experiment with physics properties

3. Add a simple robot model to observe collision detection and dynamics

### 2. Physics Properties Configuration

Learn to configure mass, friction, and restitution properties:
- Mass: Controls how objects respond to forces
- Friction: Controls surface interaction resistance
- Restitution: Controls bounciness (0.0 = no bounce, 1.0 = perfect bounce)

## First Steps with Unity Digital Twins

### 1. Unity Environment Setup

1. Launch Unity Hub
2. Create a new 3D project or open the provided template
3. Import required packages for robotics simulation

### 2. Basic Digital Twin Scene

1. Create a simple environment with basic shapes
2. Add lighting and materials for realistic rendering
3. Implement basic camera controls for navigation

## First Steps with Sensor Simulation

### 1. LiDAR Simulation in Gazebo

1. Add a LiDAR sensor to your robot model
2. Configure scan parameters (range, resolution, field of view)
3. Visualize the point cloud data in RViz

### 2. Depth Camera Simulation

1. Add a depth camera sensor to your robot
2. Configure image parameters (resolution, field of view)
3. Access both RGB and depth data streams

### 3. IMU Simulation

1. Add an IMU sensor to capture orientation and acceleration
2. Configure noise parameters to match real sensor characteristics
3. Integrate with robot state publisher

## Running the Examples

### 1. Physics Simulation Example

```bash
# Launch the physics simulation example
roslaunch digital_twin_examples physics_simulation.launch
```

### 2. Unity Digital Twin Example

1. Open the Unity project in the `examples/unity/` directory
2. Load the "DigitalTwinDemo" scene
3. Press Play to start the simulation

### 3. Sensor Fusion Example

```bash
# Launch the complete sensor simulation
roslaunch digital_twin_examples sensor_fusion.launch
```

## Validation and Testing

### 1. Physics Validation

- Verify that objects behave with realistic physics
- Check collision detection accuracy
- Test stability with complex multi-body interactions

### 2. Sensor Validation

- Compare simulated sensor data with expected values
- Validate point cloud density and accuracy
- Test IMU data consistency

### 3. Performance Testing

- Monitor simulation frame rate
- Check for physics instabilities
- Validate real-time performance requirements

## Troubleshooting Common Issues

### Gazebo Issues
- **Slow performance**: Reduce visual complexity or physics update rate
- **Physics instability**: Check mass/inertia values and collision geometries
- **Model not loading**: Verify model paths and URDF/Xacro syntax

### Unity Issues
- **Rendering problems**: Update graphics drivers and check hardware compatibility
- **Performance issues**: Reduce rendering quality or scene complexity
- **Import errors**: Verify Unity version compatibility

### Sensor Simulation Issues
- **No sensor data**: Check sensor configuration and topic connections
- **Inaccurate data**: Verify sensor parameters and noise settings
- **Synchronization problems**: Check timestamps and coordinate frames

## Next Steps

After completing this quickstart:

1. Proceed to Chapter 1: Physics Simulation with Gazebo for in-depth physics concepts
2. Move to Chapter 2: Digital Twins & HRI in Unity for advanced visualization
3. Complete Chapter 3: Sensor Simulation & Validation for perception systems
4. Experiment with the complete examples that combine all concepts
5. Try creating your own digital twin environment with custom objects and sensors

## Resources

- Official Gazebo documentation: http://gazebosim.org/tutorials
- Unity documentation: https://docs.unity3d.com/
- ROS simulation tutorials: http://wiki.ros.org/Simulation
- Module-specific examples and code in the repository