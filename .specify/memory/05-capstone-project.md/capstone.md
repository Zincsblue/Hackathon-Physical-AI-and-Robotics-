---
sidebar_position: 5
title: "Capstone Project"
---

# Capstone Project: The Autonomous Humanoid

## 1. Project Overview
The final assessment for this course is the creation of an **Autonomous Humanoid**. [cite_start]This project requires you to integrate every skill you have learned: ROS 2 control, Gazebo/Isaac simulation, and VLA cognitive planning[cite: 79, 111].

## 2. The Mission
Your robot (simulated or real) must perform the following sequence autonomously:

1.  **Receive a Voice Command:** The user gives a natural language order (e.g., "Find the red ball and bring it here").
2.  **Plan the Path:** The robot uses the LLM to break this down into tasks and Nav2 to plan a path through a cluttered environment.
3.  **Navigate:** The robot walks (simulated bipedal locomotion) to the target area, avoiding obstacles.
4.  **Identify:** Using Computer Vision, it identifies the target object (the red ball).
5.  **Manipulate:** The robot interacts with the object (picks it up or touches it).

## 3. Implementation Tiers
Depending on your hardware access, you will complete one of the following tiers:

### Tier 1: The Digital Twin (Software Only)
* **Environment:** Completely inside NVIDIA Isaac Sim.
* **Task:** Full simulation of the mission.
* [cite_start]**Hardware:** Requires RTX 4070 Ti Workstation[cite: 116].

### Tier 2: The Physical Proxy (Sim-to-Real)
* **Hardware:** Unitree Go2 (Quadruped) or a Robotic Arm + Jetson Orin Nano.
* [cite_start]**Task:** Deploy the "Brain" to the Jetson to control the physical robot[cite: 145].

## 4. Assessment Criteria
You will be evaluated on:
* **Autonomy:** Can the robot recover if it bumps into something?
* **Latency:** How fast does it respond to the voice command?
* **Integration:** How smoothly do ROS 2, Whisper, and the LLM work together?

## Final Deliverable
Submit your GitHub repository containing:
1.  The complete ROS 2 Workspace.
2.  The URDF/USD files for your robot.
3.  [cite_start]A video demo (max 90 seconds) showing the successful completion of the mission[cite: 36].