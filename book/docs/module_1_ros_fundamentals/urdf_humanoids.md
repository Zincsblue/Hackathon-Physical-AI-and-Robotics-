# URDF for Humanoid Robots: Robot Modeling

## Learning Objectives
By the end of this section, you will be able to:
- Understand the structure and components of URDF (Unified Robot Description Format)
- Create a simple humanoid robot model with visual elements only
- Identify the key elements needed for humanoid robot modeling
- Load and visualize your URDF model in RViz2

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including:

- Links: Rigid parts of the robot (like arms, legs, torso)
- Joints: Connections between links that allow relative motion
- Visual elements: How the robot appears in simulation and visualization
- Collision elements: How the robot interacts with the environment (optional)

In humanoid robotics, URDF is crucial for creating accurate models that can be used for simulation, visualization, and control.

## URDF Structure

A URDF file has a hierarchical structure with a root `<robot>` element containing:

### Links
Links represent rigid bodies in the robot. Each link can have:
- Visual elements: How the link appears visually
- Collision elements: How the link interacts with other objects
- Inertial properties: Mass, center of mass, and inertia tensor

### Joints
Joints connect links and define how they can move relative to each other:
- Joint type: Fixed, revolute (rotational), prismatic (linear), etc.
- Parent and child links: Which links the joint connects
- Joint limits: Range of motion constraints
- Joint axis: Direction of motion

## Visual-Only URDF for Humanoids

For educational purposes, our example URDF only includes visual elements (not collision or inertial properties). This is appropriate for visualization and basic understanding without requiring complex physics calculations.

### Key Visual Components:
- `<visual>`: Defines how the link appears
- `<geometry>`: Shape and size (box, cylinder, sphere, mesh)
- `<material>`: Color and appearance properties

## Simple Arm Example Breakdown

Let's examine our `simple_arm.urdf` file:

### Robot Definition
```xml
<robot name="simple_arm">
```
Defines the root element with the robot name.

### Base Link
```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
</link>
```
Creates the base of the robot arm as a blue cylinder.

### Joints
```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="shoulder_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```
Creates a revolute (rotational) joint that connects the base to the shoulder. The joint:
- Connects parent "base_link" to child "shoulder_link"
- Is positioned 0.05m above the base
- Rotates around the Y-axis
- Has limits of ±90 degrees (-1.57 to 1.57 radians)

## Humanoid Robotics Considerations

When modeling humanoid robots in URDF, consider:

### Anthropomorphic Design
- Joint ranges should reflect human-like motion capabilities
- Proportions should be realistic for the intended application
- Degrees of freedom should match the biological equivalent

### Visualization vs. Simulation
- Visual-only URDF: Good for educational purposes and basic visualization
- Full URDF: Required for physics simulation with collision detection and dynamics

### Modular Design
- Structure your URDF to allow for easy modification
- Consider creating separate files for different body parts
- Use Xacro (XML macros) for complex robots to reduce redundancy

## Best Practices

1. **Start Simple**: Begin with a basic model and add complexity gradually
2. **Consistent Naming**: Use descriptive names for links and joints
3. **Realistic Dimensions**: Use appropriate scale for your robot
4. **Joint Limits**: Set realistic limits based on physical constraints
5. **Visual Appeal**: Use colors to distinguish different parts of the robot

## Common URDF Shapes

- `<box size="x y z">`: Rectangular prism
- `<cylinder radius="r" length="l">`: Cylinder aligned along Z-axis
- `<sphere radius="r">`: Sphere
- `<mesh filename="path">`: Complex shapes from mesh files

## Exercises

1. Modify the simple arm URDF to add a second elbow joint, creating a 2-DOF arm.

2. Change the colors of the different links to create a more distinctive visual appearance.

3. Adjust the joint limits to allow for a larger range of motion.

4. Add a simple hand or gripper to the end of the arm.

## Chapter Summary

URDF is essential for describing robot models in ROS. For humanoid robotics, it provides the foundation for visualization, simulation, and control. Visual-only URDF files are excellent for educational purposes, allowing students to understand robot structure without complex physics calculations.

---

## Navigation
- **Previous**: [Service Example](service_example.md)
- **Next**: [RViz Visualization](rviz_visualization.md)

