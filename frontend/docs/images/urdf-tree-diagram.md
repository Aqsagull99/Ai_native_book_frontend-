# URDF Tree Structure Diagram

## Diagram Description: URDF Tree Structure for Humanoid Robot

This diagram illustrates the hierarchical tree structure of a URDF (Unified Robot Description Format) model for a humanoid robot, showing:

1. **Root Link**: `base_link` or `pelvis` as the base of the kinematic tree
2. **Branching Structure**: How links connect through joints to form a tree
3. **Kinematic Chains**: Leg, arm, and spine chains extending from torso
4. **Joint Types**: Different joint types (revolute, fixed, continuous) represented by different symbols
5. **Coordinate Frames**: Local coordinate frames for each link

## Tree Structure Example

The diagram should show a tree like this:

```
                    base_link (root)
                   /      |      \
           left_hip   torso   right_hip
              |         |         |
        left_thigh   neck     right_thigh
              |         |         |
         left_shin   head    right_shin
              |                 |
        left_foot         right_foot

And from torso:
         |
      left_shoulder
           |
      left_upper_arm
           |
      left_lower_arm
           |
        left_hand
```

## Intended Use

This diagram should be converted to a PNG file and placed in `frontend/docs/images/urdf-tree-diagram.png` for use in the documentation. The diagram helps visualize how URDF models form a tree structure with a single root link and branching kinematic chains.

## Technical Specifications

- Format: PNG
- Size: 800x600 pixels minimum
- Color scheme: Use different colors for different body parts
- Connectors: Show joints with appropriate symbols (revolute, fixed, etc.)
- Labels: Clearly label all major links and joint types
- Legend: Include a legend explaining symbols and colors
- Orientation: Show 3D perspective to illustrate spatial relationships