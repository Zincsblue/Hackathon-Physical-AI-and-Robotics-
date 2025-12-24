---
sidebar_position: 2
title: "Module 2: The Digital Twin"
---

# Module 2: The Digital Twin (Gazebo & Unity)

## 1. Introduction: The Mirror World
Before we deploy code to a physical robot that costs thousands of dollars and can break (or break things around it), we must master the **Digital Twin**. [cite_start]A Digital Twin is a virtual replica of the robot and its environment where the laws of physics—gravity, friction, and collision—are simulated mathematically [cite: 61-63].

In this module, we bridge the gap between code and reality by building environments where our robots can learn to walk before they run.

## 2. Gazebo: The Physics Engine
Gazebo is our primary tool for simulating the rigid body dynamics of the robot. It answers the question: *If the robot steps here, will it fall?*



### 2.1 Physics, Gravity, and Collisions
In the "Physical AI" stack, the simulation must account for real-world forces. Gazebo handles:
* **Gravity:** Ensuring the robot has weight and mass.
* [cite_start]**Collisions:** Calculating what happens when the robot's foot hits the ground or an obstacle[cite: 64].
* [cite_start]**Rigid Body Dynamics:** Because these calculations are mathematically intense, they place a heavy load on your **CPU** (Intel Core i7 13th Gen+ or AMD Ryzen 9 recommended) [cite: 123-125].

### 2.2 URDF vs. SDF
While we used URDF in Module 1 to describe the robot, Gazebo often uses **SDF (Simulation Description Format)**. [cite_start]You will learn to convert your URDFs to SDFs to define not just the robot, but the entire world (lighting, terrain, obstacles)[cite: 99].

## 3. Unity: High-Fidelity Rendering
While Gazebo handles the physics, we often use **Unity** for visualization.
* **Visual Fidelity:** Unity allows us to see the robot in a photorealistic environment.
* [cite_start]**HRI (Human-Robot Interaction):** We use Unity to simulate how the robot looks and behaves when interacting with humans[cite: 65].

## 4. Simulating the Senses
A robot needs to perceive its digital world just as it would the real one. [cite_start]We simulate specific sensors to train our AI agents[cite: 66, 95].

### 4.1 The Eyes: Depth Cameras & LiDAR
* **RGB-D Cameras:** We simulate cameras like the **Intel RealSense D435i** to provide both Color (RGB) and Distance (Depth) data.
* **LiDAR:** Light Detection and Ranging sensors are simulated to help the robot map its environment and avoid obstacles.

### 4.2 The Inner Ear: IMUs
* **Inertial Measurement Units (IMUs):** These sensors (like the BNO055) provide data on balance and orientation.
* **Calibration:** In this module, you will learn to interpret IMU data to keep your humanoid upright.

## Learning Outcomes
By the end of this module, you should be able to:
1.  Set up a Gazebo simulation environment.
2.  Simulate physics, gravity, and collisions.
3.  Implement virtual sensors (LiDAR, Depth Cameras) to feed data to your ROS 2 nodes.