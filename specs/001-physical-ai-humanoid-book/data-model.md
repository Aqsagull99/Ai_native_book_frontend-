# Data Model: Physical AI & Humanoid Robotics Book

## Overview

This document defines the key entities and conceptual structures for the Physical AI & Humanoid Robotics book. These entities represent the core concepts that will be explained and interconnected throughout the educational content.

## Core Entities

### Physical AI Content
- **Definition**: Educational material covering AI systems operating in the real physical world, including theoretical foundations and practical applications
- **Attributes**:
  - Theoretical concepts
  - Practical implementations
  - Real-world examples
  - Simulation exercises
- **Relationships**: Connected to Embodied Intelligence Concepts, Robotics Tools Curriculum
- **Validation rules**: Must include both theoretical and practical components

### Embodied Intelligence Concepts
- **Definition**: Core principles of how AI systems can be integrated with physical bodies to create intelligent behavior
- **Attributes**:
  - Sensory integration
  - Motor control
  - Environmental interaction
  - Adaptive behavior
- **Relationships**: Connected to Physical AI Content, Learning Path Structure
- **Validation rules**: Must demonstrate clear connection between AI and physical embodiment

### Robotics Tools Curriculum
- **Definition**: Structured learning content for ROS 2, Gazebo & Unity, NVIDIA Isaac, and LLM-driven VLA systems
- **Attributes**:
  - Tool-specific tutorials
  - Integration workflows
  - Best practices
  - Troubleshooting guides
- **Relationships**: Connected to Hardware Specifications, Learning Path Structure
- **Validation rules**: Must provide hands-on examples for each tool

### Hardware Specifications
- **Definition**: Detailed requirements for digital twin workstations, edge AI hardware (Jetson Orin, Intel RealSense, ReSpeaker), and robot lab configurations
- **Attributes**:
  - Minimum requirements
  - Recommended configurations
  - Budget tiers (Proxy, Miniature, Premium)
  - Integration guidelines
- **Relationships**: Connected to Robotics Tools Curriculum, Learning Path Structure
- **Validation rules**: Must provide clear specifications for each hardware component

### Learning Path Structure
- **Definition**: 13-week curriculum with weekly objectives, assignments, and progression from basics to advanced concepts
- **Attributes**:
  - Weekly topics
  - Learning objectives
  - Assignments and exercises
  - Assessment criteria
- **Relationships**: Connected to all other entities as the organizing framework
- **Validation rules**: Must follow logical progression from foundational to advanced concepts

## State Transitions

### Learning Progression States
- **Initial State**: Student with basic AI and robotics background
- **Transition 1**: Complete Physical AI foundations (Weeks 1-2) → Student understands core Physical AI concepts
- **Transition 2**: Complete ROS 2 module (Weeks 3-5) → Student can work with ROS 2 for robotics applications
- **Transition 3**: Complete Gazebo & Unity module (Weeks 6-7) → Student can simulate robotics environments
- **Transition 4**: Complete NVIDIA Isaac module (Weeks 8-10) → Student can implement perception and planning systems
- **Transition 5**: Complete Humanoid development module (Weeks 11-12) → Student can develop humanoid robot behaviors
- **Transition 6**: Complete Conversational robotics module (Week 13) → Student can integrate voice and AI systems
- **Final State**: Student completes capstone project and understands simulation-to-deployment workflow

## Relationships

### Entity Relationships
1. **Physical AI Content** ←→ **Embodied Intelligence Concepts**
   - Bidirectional: Core concepts inform practical applications and vice versa

2. **Learning Path Structure** → **All other entities**
   - Hierarchical: Organizes all content into structured learning sequence

3. **Robotics Tools Curriculum** ↔ **Hardware Specifications**
   - Bidirectional: Tools require specific hardware; hardware enables specific tool usage

4. **Robotics Tools Curriculum** → **Embodied Intelligence Concepts**
   - Dependency: Tools implement concepts in practice

## Validation Rules

### Content Validation
- Each chapter must connect to at least 2 core entities
- All practical examples must map to theoretical concepts
- Hardware specifications must align with tool requirements
- Weekly objectives must build toward capstone project goals

### Citation Validation
- Each entity must be supported by at least 2 peer-reviewed sources
- All claims must be traceable to specific citations
- Minimum 50% of sources must be peer-reviewed for each entity

### Readability Validation
- Content complexity must match Flesch-Kincaid Grade 10-12 level
- Technical terms must be defined when first introduced
- Concepts must be explained with practical examples