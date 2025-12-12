# Quickstart Guide: Module 3 — The AI-Robot Brain (NVIDIA Isaac™)

**Created**: 2025-12-11
**Feature**: Module 3 — The AI-Robot Brain (NVIDIA Isaac™)
**Status**: Draft

## Prerequisites

- Ubuntu 22.04 LTS (recommended for Isaac Sim compatibility)
- NVIDIA GPU with CUDA support (RTX series or equivalent)
- CUDA 11.8+ and cuDNN 8.0+
- ROS 2 Humble Hawksbill installed
- Docker and Docker Compose
- At least 32GB RAM and 50GB free disk space
- Isaac Sim license or access to evaluation version

## Setting up Isaac Sim Environment

### Installing Isaac Sim
1. **Download Isaac Sim**:
   ```bash
   # Download from NVIDIA Developer website
   # Follow installation instructions for Linux
   ```

2. **Install dependencies**:
   ```bash
   sudo apt update
   sudo apt install nvidia-driver-535 nvidia-utils-535
   # Verify GPU: nvidia-smi
   ```

3. **Configure Isaac Sim**:
   ```bash
   # Set up environment variables
   export ISAAC_SIM_PATH=/path/to/isaac-sim
   export NVIDIA_PYINDEX=https://pyindex.nvidia.com
   ```

### Setting up Isaac ROS
1. **Install Isaac ROS dependencies**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-isaac-ros-common
   sudo apt install ros-humble-isaac-ros-visual-slam
   sudo apt install ros-humble-isaac-ros-pose-estimation
   ```

2. **Verify Isaac ROS installation**:
   ```bash
   # Test Isaac ROS packages
   ros2 launch isaac_ros_visual_slam visual_slam.launch.py
   ```

### Setting up Navigation2 for Bipedal Robots
1. **Install Nav2 packages**:
   ```bash
   sudo apt install ros-humble-navigation2
   sudo apt install ros-humble-nav2-bringup
   sudo apt install ros-humble-isaac-ros-navigation
   ```

2. **Configure Nav2 for bipedal locomotion**:
   ```bash
   # Create custom Nav2 configuration for bipedal robots
   # Configure path planner for step-by-step bipedal movement
   ```

## Basic Isaac System Test

### Testing Isaac Sim
1. **Launch Isaac Sim**:
   ```bash
   cd $ISAAC_SIM_PATH
   ./isaac-sim.sh
   ```

2. **Load humanoid robot model**:
   ```bash
   # In Isaac Sim UI:
   # 1. Load a humanoid robot model (e.g., ATRIAS or similar)
   # 2. Configure sensors (cameras, IMU, depth sensors)
   # 3. Set up physics properties for bipedal locomotion
   ```

3. **Run perception pipeline**:
   ```bash
   # Test object detection in simulation
   ros2 launch isaac_ros_apriltag apriltag.launch.py
   ```

### Testing Isaac ROS Integration
1. **Launch Visual SLAM**:
   ```bash
   ros2 launch isaac_ros_visual_slam visual_slam.launch.py
   ```

2. **Verify SLAM performance**:
   ```bash
   # Monitor map building and localization
   ros2 topic echo /map
   ros2 topic echo /visual_slam/tracking/pose
   ```

### Testing Navigation Pipeline
1. **Launch Nav2 with Isaac ROS**:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py
   ```

2. **Send navigation goal**:
   ```bash
   ros2 run nav2_msgs example_nav_to_pose.py
   ```

## Complete Isaac Integration Example

### Full System Integration
```bash
# Terminal 1: Launch Isaac Sim with humanoid robot
./isaac-sim.sh

# Terminal 2: Launch Isaac ROS perception stack
ros2 launch isaac_ros_visual_slam visual_slam.launch.py

# Terminal 3: Launch Navigation stack
ros2 launch nav2_bringup navigation_launch.py

# Terminal 4: Send navigation command for bipedal robot
ros2 run nav2_msgs example_nav_to_pose.py
```

### Educational Example: Perception to Navigation
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class IsaacIntegrationDemo(Node):
    def __init__(self):
        super().__init__('isaac_integration_demo')

        # Perception components
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

        # Navigation components
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info("Isaac Integration Demo initialized")

    def image_callback(self, msg):
        """Process image from Isaac Sim camera"""
        self.get_logger().info("Received image from Isaac Sim")
        # Process perception data here

    def navigate_to_object(self, object_position):
        """Navigate to detected object using Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = object_position.x
        goal_msg.pose.pose.position.y = object_position.y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        return future

def main(args=None):
    rclpy.init(args=args)
    demo = IsaacIntegrationDemo()

    # Example: navigate to position (2.0, 1.0)
    future = demo.navigate_to_object(type('obj', (object,), {'x': 2.0, 'y': 1.0})())

    try:
        rclpy.spin_until_future_complete(demo, future)
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting Common Issues

### Isaac Sim Problems
1. **GPU not detected**:
   - Verify NVIDIA drivers: `nvidia-smi`
   - Check CUDA installation: `nvcc --version`
   - Ensure Isaac Sim is launched with proper environment

2. **Physics simulation unstable**:
   - Adjust physics timestep in Isaac Sim settings
   - Verify robot model mass and inertia properties
   - Check collision mesh quality

### Isaac ROS Integration Issues
1. **Visual SLAM tracking lost**:
   - Ensure sufficient visual features in environment
   - Check camera calibration parameters
   - Verify lighting conditions for good visual tracking

2. **High latency in perception pipeline**:
   - Verify GPU acceleration is enabled
   - Check CUDA and TensorRT installations
   - Monitor GPU utilization: `nvidia-smi`

### Navigation Problems
1. **Path planner fails to find route**:
   - Check map quality from SLAM system
   - Verify robot footprint configuration
   - Ensure Nav2 costmap parameters are correct

2. **Bipedal robot falls during navigation**:
   - Adjust path smoothing for bipedal locomotion
   - Verify gait parameters for stable walking
   - Check balance control algorithms

## Next Steps

After completing the setup:
1. Review the Isaac module chapters:
   - Chapter 1: Advanced Perception & Training
   - Chapter 2: Isaac Sim: Simulation & Synthetic Data
   - Chapter 3: Isaac ROS: VSLAM & Navigation
   - Chapter 4: Nav2: Bipedal Path Planning

2. Practice with simple scenarios before complex tasks
3. Experiment with different humanoid robot models
4. Test perception algorithms in various lighting conditions
5. Validate navigation performance in different environments