# ROS 2 Communication Diagram

## Diagram Description: ROS Graph and Node Connections

This diagram illustrates the ROS 2 communication architecture for humanoid robots, showing:

1. **Nodes**: Different colored boxes representing various robot subsystems
   - Sensor nodes (blue): Camera, IMU, Force/Torque sensors
   - Control nodes (green): Balance controller, Walking pattern generator
   - Perception nodes (orange): Object recognition, SLAM
   - Actuator nodes (red): Joint controllers

2. **Topics**: Arrows with labels showing publish-subscribe relationships
   - `/joint_states`: Joint position/velocity/effort data
   - `/sensor_data`: Sensor readings from various sources
   - `/joint_commands`: Commands sent to actuators
   - `/robot_state`: Overall robot state information

3. **Communication Patterns**:
   - Solid arrows: Topic-based (publish/subscribe)
   - Dashed arrows: Service-based (request/reply)
   - Dotted arrows: Action-based (goal/feedback/result)

4. **Quality of Service Settings**: Labels indicating reliability and durability settings for critical vs. non-critical data

## Intended Use

This diagram should be converted to a PNG file and placed in `frontend/docs/images/ros2-communication-diagram.png` for use in the documentation. The diagram helps visualize how different nodes communicate in a humanoid robot system using ROS 2 communication patterns.

## Technical Specifications

- Format: PNG
- Size: 800x600 pixels minimum
- Color scheme: Use distinct colors for different node types
- Legend: Include a legend explaining colors and arrow types
- Labels: Clearly label all topics, services, and actions