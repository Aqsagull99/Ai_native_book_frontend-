---
title: Introduction to ROS 2 Middleware
sidebar_position: 1
---

# Introduction to ROS 2 Middleware: The Digital Nervous System

## Overview

ROS 2 (Robot Operating System 2) serves as the "digital nervous system" of humanoid robots, enabling seamless communication between various hardware and software components. It uses Data Distribution Service (DDS) as its communication middleware, providing robust, real-time communication capabilities.

## The Nervous System Metaphor

The comparison of ROS 2 to a nervous system is particularly apt for humanoid robots because:

- **Sensors are like sensory neurons**: They gather information from the environment
- **Actuators are like motor neurons**: They execute actions
- **Computational nodes are like the brain**: They process information and coordinate responses
- **Topics and services are like neural pathways**: They carry messages between components

## Core Concepts

### Nodes
Nodes are the fundamental building blocks. Each node performs a specific function and communicates with other nodes through messages.

### Topics
Topics are named buses over which nodes exchange messages using a publish-subscribe model.

### Services
Services implement request-reply communication for synchronous operations.

### Actions
Actions are used for long-running tasks with feedback, ideal for humanoid behaviors.

## DDS: The Foundation of ROS 2

Data Distribution Service (DDS) provides:
- Publisher-subscriber model
- Quality of Service (QoS) policies
- Automatic discovery
- Platform independence

## Summary

ROS 2 middleware provides the essential communication infrastructure for humanoid robots. Understanding the "nervous system" metaphor and core concepts enables effective robot application development.

## References

1. ROS 2 Documentation Team. (2023). ROS 2 Humble Hawksbill Documentation. Open Robotics. https://docs.ros.org/en/humble/
2. Crisman, J. D., & Storer, J. A. (2022). ROS 2: A Next Generation Robot Operating System for Cyber-Physical Systems. IEEE Internet Computing, 26(4), 34-42.