# Data Model: Digital Twin Simulation Module

**Created**: 2025-12-11
**Feature**: Module 2 — The Digital Twin (Gazebo & Unity)
**Status**: Draft

## Entity Descriptions

### Gazebo Simulation Environment
- **Description**: Physics-based simulation environment that models real-world physics properties
- **Attributes**:
  - World file (SDF format) defining the simulation environment
  - Physics engine parameters (gravity, solver type, step size)
  - Material properties (friction coefficients, restitution)
  - Light sources and environmental properties
- **Relationships**: Contains multiple robot models and environment objects

### Unity Rendering System
- **Description**: High-fidelity visualization system for photorealistic rendering
- **Attributes**:
  - Scene configuration with lighting and materials
  - Rendering pipeline settings (quality, resolution, effects)
  - Camera properties and viewpoints
  - Asset references for 3D models and materials
- **Relationships**: Interfaces with robot models and environment objects for visualization

### Humanoid Robot Models
- **Description**: Digital representations of robots with proper physics and sensor configurations
- **Attributes**:
  - URDF/SDF definition files
  - Physical properties (mass, dimensions, joint limits)
  - Sensor configurations (LiDAR, cameras, IMU placement)
  - Control interfaces (joint controllers, actuator models)
- **Relationships**: Contains multiple joints and links, connects to sensors

### Sensor Simulation Data
- **Description**: Simulated sensor outputs that mimic real sensors
- **Types**:
  - LiDAR: Point cloud data (3D coordinates, intensity)
  - Depth Camera: RGB-D data (color image + depth map)
  - IMU: Acceleration and angular velocity data
  - Camera: Visual sensor data (RGB images)
- **Attributes**:
  - Data format and frequency
  - Noise characteristics and accuracy parameters
  - Field of view and resolution
  - Frame of reference and coordinate system
- **Relationships**: Associated with specific robot models and mounting positions

### Environment Objects
- **Description**: 3D models and physics objects that populate simulation scenes
- **Attributes**:
  - Shape and geometry (mesh files, primitive shapes)
  - Physical properties (mass, friction, restitution)
  - Visual properties (textures, materials, colors)
  - Static vs dynamic classification
- **Relationships**: Part of Gazebo simulation environment, interact with robot models

## State Transitions

### Simulation States
- **Initialization**: Environment and robot models loaded, physics engine prepared
- **Ready**: Simulation paused, ready to start, all systems initialized
- **Running**: Active simulation with physics, rendering, and sensor updates
- **Paused**: Simulation temporarily stopped, can be resumed
- **Stopped**: Simulation ended, resources released

### Sensor States
- **Disabled**: Sensor not actively publishing data
- **Enabled**: Sensor actively publishing data
- **Error**: Sensor experiencing issues, data may be unreliable

## Relationships

### Simulation Environment Relationships
- Gazebo Simulation Environment **contains** multiple Environment Objects
- Gazebo Simulation Environment **hosts** multiple Humanoid Robot Models
- Unity Rendering System **visualizes** the same environment as Gazebo Simulation Environment

### Robot and Sensor Relationships
- Humanoid Robot Model **has** multiple Sensor Simulation Data streams
- Humanoid Robot Model **consists of** multiple joints and links
- Sensor Simulation Data **originates from** specific mounting points on Humanoid Robot Model

### Visualization Relationships
- Unity Rendering System **renders** Humanoid Robot Models
- Unity Rendering System **displays** Environment Objects
- Unity Rendering System **simulates** Human-Robot Interaction scenarios