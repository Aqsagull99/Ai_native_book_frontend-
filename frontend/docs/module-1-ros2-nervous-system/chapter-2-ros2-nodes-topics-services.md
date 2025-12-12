---
title: ROS 2 Nodes, Topics, Services
sidebar_position: 2
---

# ROS 2 Communication Patterns: Nodes, Topics, and Services

## Overview

Understanding ROS 2 communication patterns is essential for creating effective humanoid robot applications. This chapter explores the fundamental communication mechanisms: Nodes, Topics, and Services.

## Nodes: The Building Blocks of ROS 2

A node is an executable process that works as part of a ROS 2 system. In humanoid robotics, nodes typically represent individual sensors, control algorithms, perception systems, or behavior controllers.

### Creating a Node in Python

Using the rclpy client library:

```python
import rclpy
from rclpy.node import Node

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Humanoid Controller node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Topics: Publish-Subscribe Communication

Topics implement a publish-subscribe communication pattern. For humanoid robots, topics are commonly used for joint states, sensor data, and command messages.

```python
from sensor_msgs.msg import JointState

# Publisher
joint_pub = self.create_publisher(JointState, 'joint_states', 10)

# Subscriber
joint_sub = self.create_subscription(
    JointState,
    'joint_commands',
    self.joint_command_callback,
    10
)
```

### Quality of Service (QoS) Settings

QoS settings allow fine-tuning communication behavior for reliability and performance.

## Services: Request-Reply Communication

Services implement a request-reply pattern for synchronous operations like robot activation or parameter configuration.

```python
from example_interfaces.srv import SetBool

# Service server
self.service = self.create_service(
    SetBool,
    'robot_enable',
    self.enable_robot_callback
)
```

## Actions: Goal-Oriented Communication

Actions are used for long-running tasks with feedback, ideal for humanoid behaviors like walking or manipulation.

```python
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

# Action client
self._action_client = ActionClient(
    self,
    FollowJointTrajectory,
    'follow_joint_trajectory'
)
```

## Practical Examples for Humanoid Robots

Different humanoid functions benefit from different communication patterns:
- **Sensors**: Topics (publish sensor data continuously)
- **Joint Control**: Topics (publish commands, subscribe to states)
- **Calibration**: Services (request with parameters)
- **Walking**: Actions (with feedback on progress)

## Summary

ROS 2 communication patterns provide flexible mechanisms for coordinating humanoid robot subsystems. Understanding when to use topics, services, and actions is crucial for creating robust robot applications.

## References

1. ROS 2 Documentation Team. (2023). ROS 2 Humble Hawksbill Documentation. Open Robotics. https://docs.ros.org/en/humble/
2. Macenski, S., & D'Andrea, R. (2021). Real-Time Performance Analysis of ROS 2 and ROS 1 in Multi-Robot Systems. IEEE Robotics and Automation Letters, 6(3), 5718-5725.