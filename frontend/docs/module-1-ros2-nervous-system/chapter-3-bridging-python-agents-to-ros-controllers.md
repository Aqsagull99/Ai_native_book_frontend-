---
title: Bridging Python Agents to ROS Controllers
sidebar_position: 3
---

# Bridging Python Agents to ROS Controllers: rclpy Integration

## Overview

This chapter explores how to connect AI agents written in Python to ROS 2 controllers using the rclpy client library, focusing on the AI Agent → ROS Node → Joint Command flow in humanoid robotics.

## Introduction to rclpy

rclpy is the Python client library for ROS 2 that allows Python-based AI agents to create nodes, publish/subscribe to topics, and provide/use services.

### Basic AI Agent Node Structure

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Publishers for sending commands
        self.command_pub = self.create_publisher(
            JointTrajectory,
            'ai_joint_commands',
            10
        )

        # Subscribers for receiving sensor data
        self.sensor_sub = self.create_subscription(
            JointState,
            'robot_joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for AI decision making
        self.ai_timer = self.create_timer(0.1, self.ai_decision_loop)

    def joint_state_callback(self, msg):
        # Process sensor data and make decisions
        decision = self.make_decision(msg)
        command_msg = self.create_command_message(decision)
        self.command_pub.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()
    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        ai_agent.get_logger().info('AI Agent interrupted')
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()
```

## The AI Agent → ROS Node → Joint Command Flow

### Complete Integration Example

```python
class HumanoidAIAgent(Node):
    def __init__(self):
        super().__init__('humanoid_ai_agent')

        # Publishers and Subscribers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_commands',
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # AI control timer
        self.ai_control_timer = self.create_timer(0.1, self.ai_control_loop)

    def ai_control_loop(self):
        # 1. AI Agent: Process sensor data and make decisions
        control_decision = self.process_sensor_data_and_decide()

        # 2. ROS Node: Convert decision to ROS message
        trajectory_command = self.create_trajectory_from_decision(control_decision)

        # 3. Joint Command: Publish to robot controller
        if trajectory_command:
            self.trajectory_pub.publish(trajectory_command)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = HumanoidAIAgent()
    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        ai_agent.get_logger().info('Humanoid AI Agent shutting down')
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()
```

## Best Practices for AI-ROS Integration

### Performance Considerations
- Optimize message rates for real-time performance
- Implement proper error handling for AI model failures

### Safety and Reliability
- Implement timeouts for AI decision processes
- Provide safe fallback behaviors when AI fails
- Validate AI outputs before sending to robot controllers

## Summary

This chapter demonstrated how to bridge Python AI agents to ROS 2 controllers using rclpy, implementing the complete AI Agent → ROS Node → Joint Command flow essential for intelligent humanoid robot behaviors.

## References

1. ROS 2 Documentation Team. (2023). ROS 2 Humble Hawksbill Documentation. Open Robotics. https://docs.ros.org/en/humble/
2. Witt, J., Lawitzky, G., Wiedemeyer, T., & Beetz, M. (2020). Humanoid Robot Control and Perception Using ROS 2. Journal of Intelligent & Robotic Systems, 99(3-4), 871-887.