---
id: 1
title: Update Robotics Constitution
stage: constitution
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Claude Code
command: /sp.constitution
labels: ["constitution", "robotics", "ros2", "textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests:
  - none
---

## Prompt

```text
$ARGUMENTS
```

Physical AI & Humanoid Robotics Textbook — Module 1: The Robotic Nervous System (ROS 2)
(Docusaurus + Spec-Kit Plus + Claude Code)

Core Principles

Technical accuracy aligned with official ROS 2 (Humble/ Iron) documentation

Clarity for a beginner–intermediate robotics audience

Hands-on, reproducible learning with step-by-step instructions

Embodied-AI mindset: always connect software concepts to real robot behavior

Agent-first writing style for Claude Code (clear specs → deterministic output)

Key Standards

Explanations:

Keep all topics grounded in the context of humanoid robotics

Provide conceptual diagrams using Mermaid when appropriate

All examples must run on Ubuntu 22.04 using ROS 2 Humble

Code Standards:

Programming language: Python (rclpy)

ROS 2 package structure MUST follow ament_python conventions

Include launch files, parameters, and node-to-node message flow

All code must be testable (ros2 run, ros2 topic echo, ros2 launch)

URDF Standards:

URDF files must be valid XML

Include links, joints, inertial tags, visuals, and collisions

Must load successfully in rviz2

Documentation:

Before generating content, perform an internal consistency check:

Does each chapter build on previous knowledge?

Does each code block reflect ROS 2 best practices?

Module 1 Scope Requirements

The output for this module must fully cover:

1. ROS 2 Fundamentals (The Robotic Nervous System)

What middleware means in robotics

DDS communication in simple words

Nodes, Topics, Services, Actions — with humanoid examples

rclpy node creation workflow

2. Python-to-ROS Bridge

Using rclpy to connect Python agents to robot controllers

Example: Python agent → publish joint angles → ROS control node

3. URDF for Humanoid Robots

What a URDF is and why humanoids need accurate mass/inertia

Basic URDF skeleton for a humanoid torso, arm, or leg

Loading URDF in RViz and verifying it

4. Project-Based Learning

At minimum:

Build a ROS 2 package

Create a publisher node, subscriber node

Add a service

## Response snapshot

Updated the project constitution for the Physical AI & Humanoid Robotics textbook project. Created a new constitution file with the specified core principles, key standards, and scope requirements. Also updated the plan, spec, and tasks templates to align with the robotics/ROS 2 context.

## Outcome

- ✅ Impact: Created new constitution for robotics textbook project and updated related templates
- 🧪 Tests: none
- 📁 Files: Updated 4 files including the main constitution and 3 templates
- 🔁 Next prompts: N/A
- 🧠 Reflection: Successfully implemented the constitution with all specified principles and aligned templates

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A