---
title: Hardware & Lab Setup
sidebar_position: 3
---

# Hardware & Lab Setup

## Overview

Setting up an appropriate hardware environment is crucial for working with Physical AI and humanoid robotics. This chapter provides guidance on configuring digital twin workstations, edge AI hardware, and robot lab environments with appropriate specifications for different use cases and budgets.

## Digital Twin Workstation Specifications

For effective simulation and development work, your workstation should meet these specifications:

### Minimum Requirements
- **CPU**: Intel i5-10400 or AMD Ryzen 5 3600
- **RAM**: 16 GB DDR4 (32 GB recommended)
- **GPU**: NVIDIA GTX 1060 6GB or equivalent
- **Storage**: 500 GB SSD
- **OS**: Ubuntu 22.04 LTS or Windows 10/11 with WSL2

### Recommended Configuration
- **CPU**: Intel i7-12700K or AMD Ryzen 7 5800X
- **RAM**: 32 GB DDR4 3200MHz
- **GPU**: NVIDIA RTX 3080 or better
- **Storage**: 1 TB NVMe SSD
- **OS**: Ubuntu 22.04 LTS (preferred for robotics development)

### High-Performance Configuration
- **CPU**: Intel i9-12900K or AMD Ryzen 9 5900X
- **RAM**: 64 GB DDR4 3600MHz
- **GPU**: NVIDIA RTX 4090 or dual RTX 4080
- **Storage**: 2 TB+ NVMe SSD + additional storage array
- **OS**: Ubuntu 22.04 LTS

## Edge AI Hardware for Robotics

### NVIDIA Jetson Orin
The NVIDIA Jetson Orin platform is recommended as the primary edge AI computing platform for robotics applications:

#### Jetson Orin NX
- **GPU**: 2048-core NVIDIA Ampere architecture GPU
- **CPU**: 6-core ARM Cortex-A78AE v8.2 64-bit CPU
- **DL Accelerator**: 2x INT8 Tensor Cores
- **Memory**: 8 GB LPDDR5
- **Power**: 10-25W
- **Use case**: Mobile robots, perception tasks

#### Jetson Orin AGX
- **GPU**: 2048-core NVIDIA Ampere architecture GPU
- **CPU**: 8-core ARM Cortex-A78AE v8.2 64-bit CPU
- **DL Accelerator**: 4x INT8 Tensor Cores
- **Memory**: 16/32 GB LPDDR5
- **Power**: 15-60W
- **Use case**: Complex humanoid robots, simultaneous localization and mapping

### Intel RealSense
Intel RealSense cameras provide excellent depth sensing and 3D perception capabilities:

#### D400 Series (D415, D435, D455)
- **Depth Technology**: Stereo vision
- **Depth Accuracy**: ±2% at 1m
- **Depth Range**: 0.2m to 10m (varies by model)
- **Connectivity**: USB 3.0
- **Use case**: Robot navigation, object detection, hand tracking

#### L500 Series (L515)
- **Depth Technology**: LiDAR
- **Depth Accuracy**: ±1% at 1m
- **Depth Range**: 0.25m to 9m
- **Connectivity**: USB-C
- **Use case**: High-accuracy indoor mapping and navigation

### ReSpeaker for Voice Interaction
ReSpeaker provides excellent audio processing capabilities for voice-enabled robots:

#### ReSpeaker 2-Mic Array
- **Microphones**: 2 microphones with beamforming
- **Audio Processing**: Voice activity detection, acoustic echo cancellation
- **Connectivity**: USB
- **Use case**: Basic voice commands and interaction

#### ReSpeaker 4-Mic Array
- **Microphones**: 4 microphones with advanced beamforming
- **Audio Processing**: Direction-of-arrival (DOA), automatic speech recognition
- **Connectivity**: USB
- **Use case**: Complex voice interaction, noise cancellation

## Robot Lab Tiers

### Proxy Tier (Simulation Only)
Perfect for initial learning and development without physical hardware:

#### Components
- Digital twin workstation (minimum specifications above)
- Software: ROS 2 Humble Hawksbill, Gazebo, Unity
- Development tools: VS Code, Git, Docker
- Internet connection for cloud resources

#### Advantages
- Low cost entry point
- No physical hardware maintenance
- Safe environment for algorithm development
- Easy to reset and iterate

#### Limitations
- No real-world physics validation
- Limited sensor accuracy modeling
- Cannot test physical interactions

### Miniature Tier (Small Robots)
For hands-on experience with physical robots:

#### Components
- Digital twin workstation (recommended specifications)
- NVIDIA Jetson Orin NX for edge AI
- Intel RealSense D435 for depth sensing
- Small mobile robot platform (e.g., TurtleBot 4, Clearpath Jackal)
- Network infrastructure for robot communication

#### Advantages
- Real sensor data and physics
- Practical implementation experience
- Cost-effective for multiple robots
- Suitable for classroom settings

#### Limitations
- Limited complexity of robot platforms
- Smaller scale than humanoid robots

### Premium Tier (Full Humanoid)
For advanced humanoid robotics development:

#### Components
- High-performance workstation (high-performance configuration)
- NVIDIA Jetson Orin AGX for edge AI
- Intel RealSense L515 for high-accuracy depth sensing
- Full humanoid robot platform (e.g., NAO, Pepper, custom biped)
- Complete sensor and actuator suite
- Motion capture system for validation (optional)
- Safety equipment and infrastructure

#### Advantages
- Full humanoid development experience
- Advanced perception and interaction capabilities
- Research-grade validation possibilities
- Closest to real-world deployment

#### Limitations
- High cost
- Complex maintenance requirements
- Safety considerations
- Space requirements

## Cloud vs Local Compute

### When to Use Cloud Compute
- Large-scale simulation environments
- Training complex models
- Data processing and analysis
- Collaborative development
- Resource-intensive computations

### When to Use Local Compute
- Real-time robot control
- Low-latency sensor processing
- Offline development
- Privacy-sensitive applications
- Edge AI inference

### Hybrid Approach
Most effective robotics projects use a combination of cloud and local compute:

1. **Development**: Local workstation for coding and simulation
2. **Training**: Cloud resources for model training
3. **Deployment**: Local edge devices for real-time operation
4. **Monitoring**: Cloud for data analysis and logging

## Latency Issues and Mitigation Strategies

### Common Latency Sources
- Network delays in cloud communication
- Processing time for complex algorithms
- Sensor-to-action pipeline delays
- Communication overhead between system components

### Mitigation Strategies
1. **Edge Processing**: Move computation closer to the robot
2. **Pipeline Optimization**: Optimize sensor processing and action execution
3. **Predictive Control**: Use predictive algorithms to compensate for delays
4. **Asynchronous Processing**: Process different sensor modalities in parallel
5. **Model Optimization**: Use quantized or compressed models for faster inference

## Troubleshooting Common Hardware Setup Issues

### ROS 2 Connection Issues
- Ensure network configuration is correct
- Check firewall settings
- Verify ROS_DOMAIN_ID consistency across systems
- Use `ros2 topic list` to verify connectivity

### GPU Acceleration Problems
- Verify NVIDIA drivers are properly installed
- Check CUDA version compatibility
- Ensure proper permissions for GPU access
- Use `nvidia-smi` to verify GPU status

### Sensor Calibration Issues
- Follow manufacturer calibration procedures
- Use ROS 2 calibration tools
- Verify mounting positions and orientations
- Check for environmental interference

## References

- NVIDIA Jetson Orin Developer Guide. (2023). https://developer.nvidia.com/embedded/jetson-orin
- Intel RealSense Documentation. (2023). https://www.intelrealsense.com/
- ROS 2 Hardware Setup Best Practices. (2023). https://docs.ros.org/en/humble/