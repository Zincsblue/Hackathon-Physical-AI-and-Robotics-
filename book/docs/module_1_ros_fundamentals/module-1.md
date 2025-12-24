---
sidebar_position: 1
title: "Module 1: The Robotic Nervous System"
---

# Module 1: The Robotic Nervous System (ROS 2)

## 1. Introduction: The Nervous System
In the anatomy of Physical AI, if the "Brain" is the AI Agent (running LLMs or VLA models), then the **Middleware** is the nervous system. [cite_start]It is responsible for transmitting high-level commands to the physical muscles (motors) and relaying sensory data back to the brain.

In this course, we use **ROS 2 (Robot Operating System 2)** as that middleware. [cite_start]It allows us to bridge the gap between abstract Python agents and real-world hardware control.

### 1.1 Why ROS 2?
Unlike standard software, robot software relies on low-latency communication between many moving parts. ROS 2 provides the "plumbing" that allows a camera node to talk to a motor node without crashing the system.

:::danger Hardware Reality Check
**Operating System Requirement:**
While simulation tools like Isaac Sim can run on Windows, **ROS 2 (Humble or Iron) is native to Linux**.
To complete this module and the course, you **must** use **Ubuntu 22.04 LTS**. [cite_start]Attempting to use Windows for the core ROS 2 development will lead to significant friction and compatibility issues [cite: 127-129].
:::

---

## 2. Core Concepts: The Architecture
[cite_start]To control a humanoid, you must understand the three fundamental building blocks of ROS 2.

### 2.1 Nodes
A **Node** is a single, executable process that performs a specific task.
* **Analogy:** A neuron in the nervous system.
* **Example:** In a humanoid robot, you might have one node for reading the *Lidar* sensor, another node for controlling the *Knee Motors*, and a third node for *Path Planning*.

### 2.2 Topics
Nodes pass messages to each other over a named bus called a **Topic**. This is a continuous stream of data.
* **Publisher:** A node that sends data (e.g., a Camera node publishing video frames).
* **Subscriber:** A node that receives data (e.g., a Vision node processing those frames).
* **Analogy:** A live radio broadcast. You tune in (subscribe) to hear the stream.

### 2.3 Services
Sometimes you don't want a continuous stream; you just want a specific task done. **Services** are a request/response communication method.
* **Example:** An AI Agent sends a request: "Calibrate IMU." The IMU node performs the action and replies: "Calibration Complete."

---

## 3. The Python Bridge: `rclpy`
The "Physical AI" revolution is built on the ability to connect modern AI (mostly written in Python) to robotics. [cite_start]We use the **`rclpy`** library to bridge Python Agents to ROS controllers[cite: 59].

Below is an example of a simple "Agent Bridge" node. This script represents how an AI agent might publish velocity commands to a robot.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AgentBridge(Node):
    def __init__(self):
        super().__init__('ai_agent_bridge')
        # Create a publisher that talks to the robot's wheel controller
        # Topic: /cmd_vel (Command Velocity)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.send_command)
        self.get_logger().info('AI Agent Bridge Started')

    def send_command(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        msg.angular.z = 0.0 # No rotation
        self.publisher_.publish(msg)
        self.get_logger().info('Agent Command: Move Forward')

def main(args=None):
    rclpy.init(args=args)
    node = AgentBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()