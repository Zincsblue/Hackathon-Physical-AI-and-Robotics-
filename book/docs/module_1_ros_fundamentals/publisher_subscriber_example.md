# Publisher-Subscriber Example: Joint State Communication

## Learning Objectives
By the end of this section, you will be able to:
- Implement a publisher node that sends joint state information
- Create a subscriber node that receives and processes joint state data
- Launch both nodes together using a launch file
- Verify that the publisher-subscriber communication is working correctly

## Introduction

The publisher-subscriber pattern is one of the most fundamental communication patterns in ROS 2. It enables asynchronous, decoupled communication between nodes. In humanoid robotics, this pattern is commonly used for:

- Sensor data distribution (e.g., joint encoders, IMU data)
- Robot state broadcasting (e.g., current joint positions, robot pose)
- Sensor data processing pipelines

In this example, we'll create a publisher that simulates joint state data from a humanoid robot and a subscriber that receives and processes this data.

## Publisher Node Implementation

The publisher node (`joint_state_publisher.py`) creates and publishes `sensor_msgs/JointState` messages at a regular interval. Here's what it does:

1. **Initialization**: Creates a publisher for the `joint_states` topic
2. **Timer Setup**: Configures a timer to publish messages at 50Hz
3. **Message Creation**: Creates JointState messages with simulated joint positions
4. **Publishing**: Publishes the messages to the topic

### Key Components:
- Topic: `joint_states` (standard topic name for joint state information)
- Message Type: `sensor_msgs/JointState`
- Publishing Rate: 50Hz (every 0.02 seconds)

## Subscriber Node Implementation

The subscriber node (`joint_state_subscriber.py`) listens for joint state messages and processes them. Here's what it does:

1. **Subscription**: Subscribes to the `joint_states` topic
2. **Callback**: Processes received messages in the callback function
3. **Data Extraction**: Extracts joint names, positions, velocities, and efforts
4. **Processing**: Logs the data and performs basic safety checks

### Key Components:
- Topic: `joint_states` (same as publisher)
- Message Type: `sensor_msgs/JointState`
- Callback: Processes each received message

## Running the Example

### Method 1: Individual Terminal Commands

1. **Source ROS 2 Environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Build Your Package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_tutorial
   source install/setup.bash
   ```

3. **Run the Publisher** (in Terminal 1):
   ```bash
   ros2 run my_robot_tutorial joint_state_publisher
   ```

4. **Run the Subscriber** (in Terminal 2):
   ```bash
   ros2 run my_robot_tutorial joint_state_subscriber
   ```

### Method 2: Using Launch File

1. **Source ROS 2 Environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Build Your Package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_tutorial
   source install/setup.bash
   ```

3. **Run Both Nodes with Launch File**:
   ```bash
   ros2 launch my_robot_tutorial publisher_subscriber_launch.py
   ```

## Verifying the Communication

You can verify that the publisher-subscriber communication is working using these methods:

### 1. Check Active Topics
```bash
ros2 topic list
```
You should see `/joint_states` in the list.

### 2. Echo Topic Data
```bash
ros2 topic echo /joint_states
```
This will show you the joint state messages being published.

### 3. Check Topic Information
```bash
ros2 topic info /joint_states
```
This shows publisher and subscriber counts.

## Humanoid Robotics Context

In humanoid robotics, the publisher-subscriber pattern is essential for:

- **Sensor Integration**: Multiple sensor nodes publish data to topics that multiple processing nodes can subscribe to
- **State Broadcasting**: The current robot state is published so visualization, logging, and control nodes can access it
- **Distributed Processing**: Perception nodes can process sensor data independently and publish results for other nodes to use

## Best Practices

1. **Topic Naming**: Use descriptive, consistent topic names (e.g., `/arm/joint_states` for arm-specific data)
2. **Message Types**: Use standard message types when possible for interoperability
3. **Publishing Rate**: Choose appropriate rates based on your application needs
4. **Error Handling**: Always include error handling in callbacks
5. **QoS Settings**: Consider Quality of Service settings for your specific use case

## Exercises

1. Modify the publisher to publish at a different rate (e.g., 10Hz instead of 50Hz) and observe the difference.

2. Add more joints to the publisher and verify that the subscriber can handle the increased data.

3. Create a third node that subscribes to joint states and publishes a derived value (e.g., average joint position).

4. Experiment with different QoS settings and observe how they affect communication.

## Chapter Summary

The publisher-subscriber pattern is fundamental to ROS 2 communication. It enables decoupled, asynchronous communication between nodes, which is essential for robust robotic systems. In humanoid robotics, this pattern allows for flexible sensor integration, state broadcasting, and distributed processing.

---

## Navigation
- **Previous**: [Workspace Setup](workspace_setup.md)
- **Next**: [Service Example](service_example.md)

