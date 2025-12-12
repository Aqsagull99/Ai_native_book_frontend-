# Isaac Architecture Diagram Templates

This document provides reusable diagram templates for the Isaac module documentation. These templates can be used to create consistent visualizations throughout the chapters.

## 1. Isaac Perception Pipeline

```mermaid
graph LR
    A[Sensor Data<br/>RGB, Depth, IMU] --> B(Isaac Perception System)
    B --> C{Object Detection<br/>Classification}
    C --> D[Detection Results<br/>Bounding Boxes, Labels]
    C --> E[Tracking IDs]
    B --> F[Confidence Scores]

    style A fill:#e1f5fe
    style D fill:#f3e5f5
    style E fill:#f3e5f5
    style F fill:#f3e5f5
```

## 2. Isaac Sim Environment Architecture

```mermaid
graph TB
    A[Humanoid Robot Model] --> B{Isaac Sim<br/>Environment}
    C[Physics Engine<br/>PhysX 5] --> B
    D[Lighting & Materials] --> B
    E[Sensors<br/>Cameras, IMU, etc.] --> B

    B --> F[Synthetic Sensor Data]
    B --> G[Ground Truth Data]
    B --> H[Annotation Data]

    style A fill:#e8f5e8
    style C fill:#fff3e0
    style F fill:#f3e5f5
    style G fill:#f3e5f5
    style H fill:#f3e5f5
```

## 3. Isaac ROS VSLAM Architecture

```mermaid
graph LR
    A[Stereo Camera<br/>or RGB-D] --> B{Isaac ROS<br/>Visual SLAM}
    B --> C[6DOF Pose<br/>Estimates]
    B --> D[Sparse/Dense<br/>Map]
    B --> E[Tracking Status]

    C --> F[Navigate to Pose<br/>Action]
    D --> G[Costmap<br/>Generation]
    E --> H[Recovery<br/>Behaviors]

    style A fill:#e1f5fe
    style C fill:#f3e5f5
    style D fill:#f3e5f5
    style E fill:#f3e5f5
```

## 4. Bipedal Path Planning Integration

```mermaid
graph TB
    A[Map Data] --> B{Bipedal<br/>Path Planner}
    C[Start Pose] --> B
    D[Goal Pose] --> B
    E[Robot Constraints<br/>Step Size, Balance] --> B

    B --> F[Valid Path<br/>Step by Step]
    B --> G[Gait Parameters<br/>Timing, Footsteps]
    B --> H[Balance Metrics<br/>Stability Measures]

    F --> I[Navigation<br/>Execution]

    style A fill:#e1f5fe
    style C fill:#e1f5fe
    style D fill:#e1f5fe
    style F fill:#f3e5f5
    style G fill:#f3e5f5
    style H fill:#f3e5f5
```

## 5. Complete Isaac Integration Pipeline

```mermaid
graph LR
    A[Sensor Data] --> B(Isaac Perception)
    B --> C{SLAM Localization}
    C --> D[Path Planning]
    D --> E[Navigate Execution]

    F[Isaac Sim] -.-> B
    F -.-> C
    F -.-> D

    style A fill:#e1f5fe
    style B fill:#fff9c4
    style C fill:#fff9c4
    style D fill:#fff9c4
    style E fill:#c8e6c9
    style F fill:#e1bee7
```

## 6. Simulation-to-Reality Transfer

```mermaid
graph LR
    A[Real Robot<br/>Sensors] -->|Data Format| B{Sim-to-Real<br/>Transfer}
    C[Isaac Sim<br/>Synthetic Data] -->|Training| B
    B --> D[Perception<br/>Model]
    D --> E[Navigation<br/>System]
    E --> F[Real Robot<br/>Actions]

    style A fill:#e8f5e8
    style C fill:#e1f5fe
    style D fill:#fff9c4
    style E fill:#c8e6c9
    style F fill:#e8f5e8
```

## Usage Instructions

1. Copy the desired diagram template into your chapter document
2. Customize the labels and descriptions as needed for your specific content
3. Ensure the diagram aligns with the technical concepts being explained
4. Use consistent color coding across all diagrams in the module