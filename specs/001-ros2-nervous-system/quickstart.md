# Quickstart Guide: ROS 2 Nervous System Module

**Created**: 2025-12-11
**Feature**: Module 1 — The Robotic Nervous System (ROS 2)
**Status**: Draft

## ROS 2 Setup Instructions

### Prerequisites
- Ubuntu 22.04 LTS (recommended for ROS 2 Humble Hawksbill)
- Python 3.8 or higher
- At least 4GB RAM and 20GB free disk space

### Installing ROS 2 Humble Hawksbill

1. **Set locale to UTF-8**:
   ```bash
   sudo locale-gen en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. **Add ROS 2 apt repository**:
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. **Install ROS 2 packages**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install ros-humble-rclpy
   ```

4. **Install Python dependencies**:
   ```bash
   pip3 install ros2cli
   ```

5. **Source the ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

### Setting up a ROS 2 Workspace

1. **Create workspace directory**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. **Build the workspace**:
   ```bash
   colcon build
   source install/setup.bash
   ```

### Verifying Installation

Test that ROS 2 is working properly:

1. **Open a new terminal and source ROS 2**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Test basic functionality**:
   ```bash
   ros2 topic list
   ros2 node list
   ```

### Running Examples

1. **Terminal 1 - Run a publisher**:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_cpp talker
   ```

2. **Terminal 2 - Run a subscriber**:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_py listener
   ```

### Python Environment Setup for AI Agents

1. **Create virtual environment**:
   ```bash
   python3 -m venv ~/ros2_ai_env
   source ~/ros2_ai_env/bin/activate
   ```

2. **Install Python packages**:
   ```bash
   pip install rclpy
   pip install numpy  # For AI/ML operations
   pip install openai  # If using OpenAI agents
   ```

## Troubleshooting

### Common Issues

1. **ROS 2 commands not found**:
   - Ensure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`
   - Add to your `.bashrc` file to make it permanent

2. **Permission errors**:
   - Make sure you're using the correct user account
   - Check file permissions on ROS 2 installation

3. **Python import errors**:
   - Verify rclpy is installed in your Python environment
   - Check that you're using Python 3.8+

## Next Steps

After completing the setup:
1. Review the ROS 2 tutorials: https://docs.ros.org/en/humble/Tutorials.html
2. Practice creating basic ROS 2 nodes
3. Learn about topics, services, and actions
4. Explore the rclpy Python client library