<!--
Sync Impact Report:
Version change: 1.0.0 → 1.1.0
Modified principles: N/A
Added sections: Book Chapter Requirements section in Module 1 Scope
Removed sections: N/A
Templates requiring updates:
  ⚠ .specify/templates/plan-template.md - May need updates for book chapter requirements
  ⚠ .specify/templates/spec-template.md - May need updates for book chapter requirements
  ⚠ .specify/templates/tasks-template.md - May need updates for book chapter requirements
  ⚠ .specify/templates/phr-template.prompt.md - May need updates for book chapter requirements
No placeholders deferred.
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### Technical Accuracy
Technical accuracy aligned with official ROS 2 (Humble/Iron) documentation

### Clarity for Learning
Clarity for a beginner–intermediate robotics audience

### Hands-On Approach
Hands-on, reproducible learning with step-by-step instructions

### Embodied-AI Mindset
Embodied-AI mindset: always connect software concepts to real robot behavior

### Agent-First Writing Style
Agent-first writing style for Claude Code (clear specs → deterministic output)

## Key Standards

### Explanations
- Keep all topics grounded in the context of humanoid robotics
- Provide conceptual diagrams using Mermaid when appropriate
- All examples must run on Ubuntu 22.04 using ROS 2 Humble

### Code Standards
- Programming language: Python (rclpy)
- ROS 2 package structure MUST follow ament_python conventions
- Include launch files, parameters, and node-to-node message flow
- All code must be testable (ros2 run, ros2 topic echo, ros2 launch)

### URDF Standards
- URDF files must be valid XML
- Include links, joints, inertial tags, visuals, and collisions
- Must load successfully in rviz2

### Documentation
Before generating content, perform an internal consistency check:
- Does each chapter build on previous knowledge?
- Does each code block reflect ROS 2 best practices?

## Module 1 Scope Requirements

The output for this module must fully cover and generate a complete book chapter:

1. **ROS 2 Fundamentals (The Robotic Nervous System)**
   - What middleware means in robotics
   - DDS communication in simple words
   - Nodes, Topics, Services, Actions — with humanoid examples
   - rclpy node creation workflow

2. **Python-to-ROS Bridge**
   - Using rclpy to connect Python agents to robot controllers
   - Example: Python agent → publish joint angles → ROS control node

3. **URDF for Humanoid Robots**
   - What a URDF is and why humanoids need accurate mass/inertia
   - Basic URDF skeleton for a humanoid torso, arm, or leg
   - Loading URDF in RViz and verifying it

4. **Project-Based Learning**
   - At minimum:
     - Build a ROS 2 package
     - Create a publisher node, subscriber node
     - Add a service

5. **Book Chapter Requirements**
   - Module 1 must generate a full book chapter suitable for a Docusaurus-based textbook
   - Content must include narrative explanations, not just code documentation
   - Chapter must be pedagogically structured with learning objectives, summaries, and exercises
   - All content must be suitable for Physical AI & Humanoid Robotics education

## Governance

- All content must adhere to the core principles and key standards
- Changes to fundamental approach require constitutional amendment
- Code examples must be verified to work in the target environment

**Version**: 1.1.0 | **Ratified**: 2025-12-11 | **Last Amended**: 2025-12-18
