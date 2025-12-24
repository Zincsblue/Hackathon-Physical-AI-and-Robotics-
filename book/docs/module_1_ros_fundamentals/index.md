---
sidebar_position: 1
sidebar_label: "Module 1: ROS 2 Fundamentals"
---


# Module 1: The Robotic Nervous System (ROS 2)

Welcome to Module 1 of the Physical AI & Humanoid Robotics textbook! This module introduces you to ROS 2 (Robot Operating System 2) as the "nervous system" of robotic applications, with a focus on humanoid robot examples.

## Table of Contents

1. [Introduction: Why ROS 2 is the Robotic Nervous System](./introduction.md)
2. [ROS 2 Basics: Nodes, Topics, Services, and DDS](./ros2_basics.md)
3. [Creating a ROS 2 Workspace](./workspace_setup.md)
4. [Publisher/Subscriber Examples: Humanoid Joint States](./publisher_subscriber_example.md)
5. [Service Example: Joint Control Service](./service_example.md)
6. [Python Agent → ROS Controller Bridge](./python_ros_bridge.md)
7. [URDF for Humanoids: Links, Joints, and Visual Elements](./urdf_humanoids.md)
8. [Loading & Testing URDF in RViz2](./rviz_visualization.md)
9. [Hands-on Exercises](./hands_on_exercises.md)
10. [Summary Checklist](./summary_checklist.md)

## Learning Objectives

By the end of this module, you will be able to:

- Explain how ROS 2 acts as middleware for robot control
- Build a working ROS 2 Python package (rclpy) with:
  - A publisher node
  - A subscriber node
  - A service server and client
- Understand message flow using real humanoid examples (e.g., joint angles, IMU data)
- Create a valid URDF file for a simple humanoid part (arm, leg, torso)
- Load the URDF successfully in RViz2
- Understand how a Python agent connects to ROS controllers
- Follow all steps reproducibly on Ubuntu 22.04 with ROS 2 Humble installed

## Prerequisites

- Basic Python programming knowledge
- Familiarity with command-line interfaces
- Ubuntu 22.04 with ROS 2 Humble installed

## Getting Started

1. Follow the [workspace setup guide](./workspace_setup.md) to create your ROS 2 environment
2. Work through each section sequentially
3. Complete the [hands-on exercises](./hands_on_exercises.md) to reinforce your learning
4. Use the [summary checklist](./summary_checklist.md) to verify your understanding

## Humanoid Robot Context

Throughout this module, we use humanoid robot examples to illustrate ROS 2 concepts. This includes:

- Joint state messages for arm and leg movements
- IMU data for balance and orientation
- URDF models of simple humanoid structures
- Service calls for joint control operations

This approach helps you understand how ROS 2 concepts apply to the complex systems found in humanoid robots while keeping examples accessible for beginners.