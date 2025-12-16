# Data Model: Digital Twin (Gazebo & Unity)

**Feature**: Module 2: The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-16
**Status**: Complete

## Overview

This document defines the key entities and their relationships for the Digital Twin module. Since this is an educational documentation module, the "data model" represents the conceptual entities that students will learn about and work with in simulation environments.

## Core Entities

### Physics Simulation Models

**Description**: Mathematical representations of physical properties used in Gazebo simulation
**Fields**:
- `entity_id`: Unique identifier for the physics entity
- `mass`: Mass of the object in kilograms
- `friction_coefficient`: Coefficient of friction for surface interactions
- `restitution`: Bounciness coefficient (0.0 to 1.0)
- `collision_geometry`: Type and parameters of collision shape (box, sphere, cylinder, mesh)
- `inertia_tensor`: 3x3 matrix representing rotational inertia properties
- `dynamics_model`: Type of physics model (rigid body, soft body, etc.)

**Validation Rules**:
- Mass must be positive (> 0)
- Friction coefficient must be between 0.0 and 10.0
- Restitution must be between 0.0 and 1.0
- Collision geometry parameters must be valid for the geometry type

### Digital Twin Environments

**Description**: High-fidelity 3D representations of real-world spaces for simulation
**Fields**:
- `environment_id`: Unique identifier for the digital twin environment
- `name`: Descriptive name of the environment
- `description`: Detailed description of the environment
- `dimensions`: 3D bounding box dimensions (x, y, z)
- `lighting_conditions`: Types of lighting present (natural, artificial, mixed)
- `material_properties`: Surface properties of objects in the environment
- `objects`: Collection of static and dynamic objects in the environment
- `interaction_points`: Locations where HRI can occur

**Validation Rules**:
- Dimensions must be positive values
- Environment must contain at least one object
- Lighting conditions must be a valid enum value

### Sensor Simulation Data

**Description**: Synthetic sensor outputs that replicate real sensor behavior
**Fields**:
- `sensor_type`: Type of sensor (LiDAR, depth_camera, IMU, etc.)
- `sensor_id`: Unique identifier for the sensor instance
- `timestamp`: Time of sensor reading
- `data_format`: Format of the sensor data (point_cloud, depth_map, imu_readings, etc.)
- `data_payload`: The actual sensor data (array of values)
- `confidence_level`: Confidence level of the sensor reading (0.0 to 1.0)
- `error_bounds`: Expected error margins for the sensor data
- `frame_id`: Coordinate frame in which the data is expressed

**Validation Rules**:
- Timestamp must be in valid format
- Data payload must match the expected format for sensor type
- Confidence level must be between 0.0 and 1.0
- Frame ID must reference a valid coordinate frame

### HRI Interfaces

**Description**: Human-Robot Interaction elements that enable communication and control
**Fields**:
- `interface_id`: Unique identifier for the HRI interface
- `interface_type`: Type of interface (GUI, voice, gesture, haptic, etc.)
- `input_methods`: Supported input methods for the interface
- `output_methods`: Supported output methods for the interface
- `interaction_modes`: Available interaction modes (supervisory, direct control, etc.)
- `safety_constraints`: Safety constraints applied to the interface
- `user_feedback`: Types of feedback provided to the user

**Validation Rules**:
- Interface type must be a valid enum value
- Input and output methods must be compatible
- Safety constraints must be properly defined

## Entity Relationships

### Physics Simulation Models ↔ Digital Twin Environments
- One environment contains many physics simulation models
- Physics models are positioned within the environment space
- Relationship: Digital Twin Environments "contain" Physics Simulation Models

### Sensor Simulation Data → Physics Simulation Models
- Sensor data is generated based on physics models and environment
- Sensors observe physics models in the environment
- Relationship: Sensor Simulation Data "observes" Physics Simulation Models

### HRI Interfaces → Digital Twin Environments
- HRI interfaces operate within digital twin environments
- Interfaces allow users to interact with the environment and its contents
- Relationship: HRI Interfaces "operate_in" Digital Twin Environments

### Sensor Simulation Data → HRI Interfaces
- Sensor data may be presented through HRI interfaces
- Interfaces can control sensor parameters
- Relationship: HRI Interfaces "access" Sensor Simulation Data

## State Transitions

### Physics Simulation Model States
- `initialized`: Model has been loaded but not yet simulated
- `active`: Model is participating in physics simulation
- `paused`: Model is temporarily excluded from simulation
- `destroyed`: Model has been removed from simulation

### Digital Twin Environment States
- `created`: Environment has been defined but not loaded
- `loaded`: Environment assets have been loaded
- `running`: Environment is actively simulating
- `paused`: Environment simulation is temporarily stopped
- `destroyed`: Environment resources have been released

### Sensor Simulation States
- `configured`: Sensor parameters have been set
- `active`: Sensor is generating data
- `calibrating`: Sensor is in calibration mode
- `error`: Sensor has encountered an error condition

## Validation Rules Summary

1. **Consistency**: All coordinate frames referenced must exist
2. **Completeness**: Required fields must be present for each entity
3. **Range**: All values must be within specified ranges
4. **Relationships**: Referenced entities must exist when relationships are defined
5. **Timeliness**: Timestamps must be in chronological order for time-series data

## Constraints

1. **Performance**: Simulation must maintain real-time performance for interactive applications
2. **Accuracy**: Sensor simulation must match real-world behavior within defined tolerances
3. **Safety**: All HRI interfaces must include appropriate safety constraints
4. **Reproducibility**: Simulation results must be reproducible with the same initial conditions
5. **Scalability**: Environments must support realistic numbers of objects and sensors