# Chapter 2: Simulating Physics, Gravity & Collisions in Gazebo

## Learning Objectives

By the end of this chapter, students will be able to:
- Create complex world files with custom environments in SDF format
- Configure obstacle and object properties for realistic interactions
- Set material properties including friction and restitution coefficients
- Build practical examples of environment building for humanoid robots
- Test robot navigation and interaction in various scenarios

## Introduction to Environment Building in Gazebo

Environment building is a critical aspect of robotics simulation that allows developers to create realistic test scenarios for robot navigation, manipulation, and interaction. In Gazebo, environments are constructed using Simulation Description Format (SDF) files that define the world's geometry, physics properties, and objects.

Creating effective environments requires understanding how to balance realism with computational efficiency, ensuring that simulations are both accurate and performant.

## World File Creation (SDF Format)

### SDF Fundamentals

Simulation Description Format (SDF) is an XML-based language that describes environments, robots, and objects in Gazebo. SDF provides a complete language for describing the objects and environments in a robotic world, including:

- World properties (gravity, atmosphere, lighting)
- Models (robots, objects, obstacles)
- Physics engine configuration
- Scene properties (lighting, sky, etc.)

### Basic World File Structure

A minimal SDF world file follows this structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_environment">
    <!-- World properties -->
    <gravity>0 0 -9.8</gravity>

    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Models and objects -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom models go here -->
  </world>
</sdf>
```

### Advanced World Configuration

For more complex environments, you can add atmospheric effects, spherical coordinates, and other world properties:

```xml
<world name="outdoor_environment">
  <gravity>0 0 -9.8</gravity>

  <!-- Atmospheric properties -->
  <atmosphere type="adiabatic">
    <temperature>288.15</temperature>
    <pressure>101325</pressure>
  </atmosphere>

  <!-- Spherical coordinates for large outdoor environments -->
  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <latitude_deg>0</latitude_deg>
    <longitude_deg>0</longitude_deg>
    <elevation>0</elevation>
    <heading_deg>0</heading_deg>
  </spherical_coordinates>

  <!-- Physics engine -->
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

## Obstacle and Object Configuration

### Static vs Dynamic Objects

In Gazebo, objects can be classified as:

1. **Static objects**: Fixed in space, do not respond to physics
2. **Dynamic objects**: Affected by physics, can move and interact
3. **Kinematic objects**: Movement controlled by user/program, not physics

### Creating Static Obstacles

Static obstacles are useful for creating permanent structures like walls, floors, and furniture:

```xml
<model name="wall">
  <static>true</static>
  <link name="wall_link">
    <pose>0 0 1 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box>
          <size>5 0.1 2</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>5 0.1 2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Creating Dynamic Objects

Dynamic objects interact with physics and can be moved by robot actions:

```xml
<model name="box">
  <link name="box_link">
    <pose>1 0 0.5 0 0 0</pose>
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.083</ixx>
        <iyy>0.083</iyy>
        <izz>0.083</izz>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyz>0</iyz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.5 0.5 0.5</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.5 0.5 0.5</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Material Properties (Friction, Restitution)

### Understanding Friction

Friction is a critical parameter that affects how objects interact with each other. In Gazebo, friction is modeled using the ODE physics engine parameters:

- **Static friction (mu)**: Force required to start motion
- **Dynamic friction (mu2)**: Force required to maintain motion

```xml
<collision name="collision">
  <geometry>
    <box>
      <size>1 1 1</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.5</mu>    <!-- Static friction coefficient -->
        <mu2>0.3</mu2>  <!-- Dynamic friction coefficient -->
      </ode>
    </friction>
  </surface>
</collision>
```

### Restitution (Bounciness)

Restitution determines how "bouncy" an object is, with values ranging from 0 (no bounce) to 1 (perfectly elastic):

```xml
<surface>
  <bounce>
    <restitution_coefficient>0.8</restitution_coefficient>
    <threshold>100000</threshold>
  </bounce>
</surface>
```

### Common Material Properties

| Material | Static Friction (mu) | Restitution |
|----------|---------------------|-------------|
| Rubber | 1.15 | 0.9 |
| Concrete | 0.6-0.8 | 0.1 |
| Wood | 0.25-0.5 | 0.5 |
| Ice | 0.03 | 0.05 |
| Steel | 0.47 | 0.2 |

## Practical Examples with Environment Building for Humanoid Robots

### Example 1: Indoor Navigation Environment

Let's create an indoor environment for testing humanoid robot navigation:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="indoor_navigation">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Walls -->
    <model name="wall_1">
      <static>true</static>
      <link name="link">
        <pose>0 5 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Doorway obstacle -->
    <model name="narrow_passage">
      <static>true</static>
      <link name="left_pillar">
        <pose>-1 0 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 2</size>
            </box>
          </geometry>
        </visual>
      </link>
      <link name="right_pillar">
        <pose>1 0 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 2</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Small movable object -->
    <model name="movable_box">
      <link name="link">
        <pose>0 -2 0.5 0 0 0</pose>
        <inertial>
          <mass>2.0</mass>
          <inertia>
            <ixx>0.167</ixx>
            <iyy>0.167</iyy>
            <izz>0.167</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Example 2: Stairs and Elevation Changes

For humanoid robots, simulating stairs and elevation changes is crucial:

```xml
<model name="stairs">
  <static>true</static>
  <!-- Individual steps -->
  <model name="step_1">
    <static>true</static>
    <link name="link">
      <pose>0 0 0.15 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>2 1 0.3</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>2 1 0.3</size>
          </box>
        </geometry>
        <material>
          <ambient>0.4 0.2 0 1</ambient>
          <diffuse>0.4 0.2 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>

  <model name="step_2">
    <static>true</static>
    <link name="link">
      <pose>0 0 0.45 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>2 1 0.3</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>2 1 0.3</size>
          </box>
        </geometry>
        <material>
          <ambient>0.4 0.2 0 1</ambient>
          <diffuse>0.4 0.2 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</model>
```

### Example 3: Humanoid Robot Testing Arena

A comprehensive environment for testing various humanoid robot capabilities:

```xml
<model name="testing_arena">
  <!-- Start area -->
  <model name="start_platform">
    <static>true</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>2 2 0.1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>2 2 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>

  <!-- Balance beam -->
  <model name="balance_beam">
    <static>true</static>
    <link name="link">
      <pose>3 0 0.1 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>3 0.1 0.1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>3 0.1 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0.5 0 1</ambient>
          <diffuse>1 0.5 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>

  <!-- Soft surface for impact testing -->
  <model name="soft_surface">
    <static>true</static>
    <link name="link">
      <pose>6 0 0 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>2 2 0.1</size>
          </box>
        </collision>
        <surface>
          <friction>
            <ode>
              <mu>0.9</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.3</restitution_coefficient>
          </bounce>
        </surface>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 2 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0 0.5 1</ambient>
            <diffuse>0.5 0 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
</model>
```

## Best Practices for Environment Building

### Performance Optimization

1. **Use simple collision geometry**: Prefer boxes, spheres, and cylinders over complex meshes
2. **Minimize object count**: Combine multiple small objects into single models when possible
3. **Use static objects**: Mark immovable objects as static to reduce physics calculations
4. **Optimize mesh resolution**: Use lower-resolution meshes for collision detection than for visual rendering

### Realism Considerations

1. **Match real-world properties**: Use accurate friction and restitution values
2. **Consider robot dimensions**: Design environments appropriate for humanoid robot size
3. **Test edge cases**: Include scenarios that test robot limits
4. **Validate with real data**: Compare simulation results with real-world measurements

### Humanoid Robot Specific Considerations

1. **Step height**: Ensure stairs and obstacles match robot's capabilities
2. **Surface stability**: Test on various friction surfaces
3. **Space requirements**: Provide adequate space for humanoid robot movement
4. **Falling scenarios**: Design safe areas for testing robot recovery

## Troubleshooting Common Environment Issues

### Objects Falling Through Surfaces
- Check that static objects have `<static>true</static>` tag
- Verify collision geometry is properly defined
- Increase contact surface layer in physics engine settings

### Performance Problems
- Simplify collision geometry
- Reduce number of small objects
- Use coarser physics simulation parameters
- Consider using simpler physics engine

### Unexpected Robot Behavior
- Check friction coefficients
- Verify gravity settings
- Ensure proper mass and inertia values
- Test with different physics engine parameters

## Summary

Environment building in Gazebo is essential for creating realistic test scenarios for humanoid robots. By understanding SDF format, object configuration, and material properties, you can create diverse environments that challenge and validate robot capabilities.

The key to effective environment building is balancing realism with computational efficiency while ensuring that the environments properly test the specific capabilities of humanoid robots, including navigation, manipulation, balance, and interaction with various surfaces and obstacles.

## References

Ferreira, A., et al. (2020). Simulation tools for robotics: Comparison of Gazebo and Webots. *IEEE Latin America Transactions*, 18(4), 623-630.

Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2350-2354.

Maggio, M., et al. (2017). Gazebo as a tool for software development in robotics: The case of fault-tolerance. *Annual IEEE International Systems Conference*, 1-7.

O'Flaherty, R., et al. (2019). The Open-Source ROS Package for Simultaneous Localization and Mapping. *Journal of Software Engineering in Robotics*, 10(1), 45-58.