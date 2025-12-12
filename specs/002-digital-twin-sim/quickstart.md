# Quickstart Guide: Digital Twin Simulation Setup

**Created**: 2025-12-11
**Feature**: Module 2 — The Digital Twin (Gazebo & Unity)
**Status**: Draft

## Gazebo Setup Instructions

### Prerequisites
- Ubuntu 22.04 LTS (recommended for Gazebo compatibility)
- At least 8GB RAM and 20GB free disk space
- Graphics card with OpenGL 3.3+ support

### Installing Gazebo (Harmonic)

1. **Set up the repository**:
   ```bash
   sudo apt update
   sudo apt install wget lsb-release gnupg
   sudo sh -c 'echo "deb [arch=amd64] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   sudo apt update
   ```

2. **Install Gazebo Harmonic**:
   ```bash
   sudo apt install gazebo
   ```

3. **Verify Installation**:
   ```bash
   gazebo --version
   ```

### Installing ROS 2 Humble Dependencies
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev
```

## Unity Setup Instructions

### Prerequisites
- Windows 10/11, macOS 10.14+, or Ubuntu 18.04+
- 8GB+ RAM, 4GB+ free disk space
- Graphics card with DX10 support (Windows) or Metal support (macOS)

### Installing Unity Hub and Editor

1. **Download Unity Hub** from https://unity.com/download
2. **Install Unity Hub** following the installer instructions
3. **Open Unity Hub** and sign in with Unity ID
4. **Go to Installs tab** and click "Add"
5. **Select Unity LTS version** (2021.3.x recommended)
6. **Select modules**: Android Build Support (if needed), iOS Build Support (if needed), etc.
7. **Click Done** to install the Unity Editor

### Installing Unity Robotics Packages
1. **Open Unity Editor**
2. **Go to Window → Package Manager**
3. **Click + icon → Add package from git URL**
4. **Add these packages**:
   - `com.unity.robotics.ros-tcp-connector`
   - `com.unity.robotics.urdf-importer`
   - `com.unity.robotics.simulation`

## Basic Simulation Setup

### Gazebo Environment Test
```bash
# Launch an empty world
gazebo

# Launch with a simple robot model
gazebo ~/gazebo_models/robots/pr2.model
```

### Unity Scene Setup
1. **Create new 3D project** in Unity Hub
2. **Import robotics packages** as above
3. **Create new scene** with basic lighting
4. **Test basic physics** with primitive objects

## Sample Simulation Workflow

### 1. Physics Simulation in Gazebo
- Create a world file with basic environment
- Add robot model with URDF/SDF
- Configure physics properties (gravity, friction, etc.)
- Test basic movement and collisions

### 2. Environment Building
- Create world file with obstacles and objects
- Configure object properties (static/dynamic, friction, etc.)
- Test robot navigation through environment

### 3. Unity Rendering
- Import robot model into Unity
- Configure materials and lighting for realism
- Test basic rendering of the robot

### 4. Sensor Simulation
- Configure LiDAR, depth camera, and IMU plugins in Gazebo
- Verify sensor data output
- Test integration with ROS 2 nodes

## Troubleshooting

### Common Issues

1. **Gazebo won't start**:
   - Check graphics drivers and OpenGL support
   - Try running with `gazebo --verbose` for error details

2. **Unity crashes on startup**:
   - Update graphics drivers
   - Run Unity in safe mode to diagnose plugin issues

3. **ROS 2 integration problems**:
   - Verify ROS_DISTRO is set correctly
   - Check network connectivity between ROS nodes

4. **Performance issues**:
   - Reduce physics update rate in Gazebo
   - Lower rendering quality settings temporarily

## Next Steps

After completing the setup:
1. Review the Gazebo tutorials: http://gazebosim.org/tutorials
2. Explore Unity Robotics tutorials: https://github.com/Unity-Technologies/Unity-Robotics-Hub
3. Practice creating basic simulation scenarios
4. Test sensor simulation with simple robot models