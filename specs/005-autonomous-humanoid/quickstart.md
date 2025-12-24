# Quickstart Guide: Autonomous Humanoid Capstone

**Feature**: 005-autonomous-humanoid
**Date**: 2025-12-19

## Overview
This guide provides step-by-step instructions to set up and run the Autonomous Humanoid Capstone project that demonstrates an end-to-end Physical AI system with voice commands, LLM-based planning, vision perception, Nav2 navigation, and ROS 2 action execution in simulation.

## Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Python 3.10+
- Gazebo simulation environment
- pip package manager

## Installation

### 1. System Dependencies
```bash
# Install ROS 2 Humble dependencies
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
sudo apt install python3-pip
```

### 2. Python Dependencies
```bash
pip3 install openai-whisper transformers torch torchaudio opencv-python numpy
```

### 3. Project Setup
```bash
# Create ROS 2 workspace
mkdir -p ~/autonomous_humanoid_ws/src
cd ~/autonomous_humanoid_ws

# Clone the project (if available) or create the structure
# For this capstone, we'll create the package structure manually
cd src
mkdir -p capstone_autonomous_humanoid/{src,launch,config,docs,test}

# Build the workspace
cd ~/autonomous_humanoid_ws
colcon build --packages-select capstone_autonomous_humanoid
source install/setup.bash
```

## Running the Simulation

### 1. Launch the Simulation Environment
```bash
# Source the workspace
source ~/autonomous_humanoid_ws/install/setup.bash

# Launch the capstone simulation
ros2 launch capstone_autonomous_humanoid capstone.launch.py
```

### 2. Launch the Autonomous System
In a new terminal:
```bash
# Source the workspace
source ~/autonomous_humanoid_ws/install/setup.bash

# Run the main orchestrator
ros2 run capstone_autonomous_humanoid vla_orchestrator
```

### 3. Interact with the Robot
Once the system is running:
1. The robot will be listening for voice commands
2. Speak a command like "Go to the kitchen" or "Pick up the red cup"
3. Observe the robot processing the command and executing the appropriate actions

## Key Components

### Voice Processing Module
- Processes speech input using Whisper ASR
- Converts voice to text for planning
- Located in: `src/voice_processor/`

### LLM Planning Module
- Uses transformer models for task decomposition
- Converts natural language to action sequences
- Located in: `src/llm_planner/`

### Perception Module
- Processes visual input from simulation
- Detects objects and understands scene
- Located in: `src/perception/`

### Navigation Module
- Interfaces with Nav2 for path planning
- Handles obstacle avoidance and movement
- Located in: `src/navigation/`

### Action Execution Module
- Executes ROS 2 actions for robot control
- Handles manipulation and movement
- Located in: `src/action_executor/`

### System Orchestrator
- Coordinates all modules
- Manages state and workflow
- Located in: `src/orchestrator/`

## Testing the System

### Unit Tests
```bash
# Run unit tests for individual components
cd ~/autonomous_humanoid_ws
colcon test --packages-select capstone_autonomous_humanoid --ctest-args -R unit
```

### Integration Tests
```bash
# Run integration tests
colcon test --packages-select capstone_autonomous_humanoid --ctest-args -R integration
```

### Simulation Tests
```bash
# Run simulation-based tests
colcon test --packages-select capstone_autonomous_humanoid --ctest-args -R simulation
```

## Expected Behavior
1. The robot should respond to voice commands within 5 seconds
2. The end-to-end flow (voice → plan → navigate → act) should complete successfully in 80% of scenarios
3. The system should handle edge cases gracefully (unknown objects, blocked paths, etc.)

## Troubleshooting
- If voice commands aren't recognized: Check audio input and ensure Whisper model is loaded
- If navigation fails: Verify Gazebo simulation is running and Nav2 is properly configured
- If LLM planning is slow: Ensure transformer models are properly cached

## Next Steps
- Explore the documentation in the `docs/` directory
- Review the system architecture diagram
- Try custom voice commands to test the system's flexibility