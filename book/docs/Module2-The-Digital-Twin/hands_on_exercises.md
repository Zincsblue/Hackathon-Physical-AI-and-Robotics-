# Hands-on Exercises: Digital Twin Implementation

## Learning Objectives
By completing these exercises, you will be able to:
- Implement complete digital twin systems with Python agents and ROS 2 controllers
- Integrate multiple communication patterns in a single system
- Troubleshoot common issues in digital twin implementations
- Validate and test digital twin systems for correctness and performance

## Exercise 1: Basic Digital Twin Setup

### Objective
Create a basic digital twin system that connects a Python agent to a simulated robot.

### Steps
1. Create a new ROS 2 package for your exercise
2. Implement a simple Python agent that publishes joint commands
3. Create a basic controller that subscribes to commands and publishes state
4. Launch the system and verify communication

### Solution Template
```python
# agent_ex1.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class BasicDigitalTwinAgent(Node):
    def __init__(self):
        super().__init__('basic_agent')

        self.publisher = self.create_publisher(JointState, 'robot_commands', 10)
        self.timer = self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        msg = JointState()
        # Implement your solution here
        pass

def main():
    rclpy.init()
    node = BasicDigitalTwinAgent()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Expected Outcome
The agent should successfully send commands to the controller and receive state feedback.

---

## Exercise 2: Advanced Communication Patterns

### Objective
Implement multiple communication patterns in a single digital twin system.

### Challenge
Design a system that uses:
- Publish-subscribe for state synchronization
- Services for configuration updates
- Actions for long-running operations

### Implementation Requirements
1. Create a service server for changing robot parameters
2. Implement an action server for trajectory execution
3. Use QoS settings appropriate for each communication type

### Solution Approach
```python
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.service import Service
import rclpy

class AdvancedCommunicationNode(Node):
    def __init__(self):
        super().__init__('advanced_comm_node')

        # Service server
        self.srv = self.create_service(
            SetParameters,
            'set_robot_parameters',
            self.set_parameters_callback
        )

        # Action server
        self._action_server = ActionServer(
            self,
            ExecuteTrajectory,
            'execute_trajectory',
            self.execute_trajectory_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def set_parameters_callback(self, request, response):
        # Implement parameter setting
        pass

    def execute_trajectory_callback(self, goal_handle):
        # Implement trajectory execution
        pass
```

---

## Exercise 3: Digital Twin Bridge Implementation

### Objective
Create a bridge node that synchronizes state between physical and virtual systems.

### Challenge
Implement a bridge that:
- Handles communication failures gracefully
- Maintains state consistency during network interruptions
- Provides latency compensation

### Implementation Guide
1. Create a bridge node with dual communication interfaces
2. Implement state buffering and recovery mechanisms
3. Add error handling for network failures

### Solution Structure
```python
class RobustDigitalTwinBridge(Node):
    def __init__(self):
        super().__init__('robust_bridge')

        # Physical system interface
        self.physical_state_sub = self.create_subscription(
            JointState, 'physical_state', self.physical_state_cb, 10
        )

        # Virtual system interface
        self.virtual_cmd_pub = self.create_publisher(
            JointState, 'virtual_commands', 10
        )

        # State management
        self.physical_state_buffer = []
        self.virtual_state_buffer = []

    def physical_state_cb(self, msg):
        # Add robust state handling
        pass
```

---

## Exercise 4: URDF Extension and Validation

### Objective
Extend the existing URDF with additional sensors and validate the extended model.

### Challenge
1. Add a camera sensor to the robot URDF
2. Add IMU sensor to the robot torso
3. Create validation tools to check the extended URDF

### Implementation Steps
1. Modify the URDF to include sensor definitions
2. Update the URDF analyzer to handle sensor elements
3. Test the extended model with robot_state_publisher

### Example Extension
```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <camera name="head_camera">
      <horizontal_fov>1.3962634015954636</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.01</near>
        <far>100</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

---

## Exercise 5: Performance Optimization

### Objective
Optimize the digital twin system for real-time performance.

### Challenge
Implement optimizations to achieve:
- 100Hz state update frequency
- Sub-10ms communication latency
- Minimal CPU usage

### Optimization Techniques
1. Implement message batching
2. Use efficient data structures
3. Optimize QoS settings
4. Implement state prediction

### Implementation Approach
```python
class OptimizedDigitalTwinNode(Node):
    def __init__(self):
        super().__init__('optimized_node')

        # Use custom QoS for performance
        perf_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Batch messages to reduce overhead
        self.message_batch = []
        self.batch_timer = self.create_timer(0.01, self.send_batch)

    def send_batch(self):
        # Send batched messages efficiently
        pass
```

---

## Exercise 6: Fault Tolerance and Recovery

### Objective
Implement fault tolerance mechanisms in the digital twin system.

### Challenge
Design systems that:
- Detect communication failures
- Switch to safe modes automatically
- Recover gracefully when connections are restored

### Implementation Requirements
1. Create heartbeat mechanisms
2. Implement automatic reconnection
3. Add safe state management

### Solution Framework
```python
class FaultTolerantNode(Node):
    def __init__(self):
        super().__init__('fault_tolerant_node')

        # Heartbeat monitoring
        self.heartbeat_timer = self.create_timer(1.0, self.check_heartbeat)
        self.last_heartbeat = self.get_clock().now()

        # Safe state publisher
        self.safe_state_publisher = self.create_publisher(
            JointState, 'safe_state', 1
        )

    def check_heartbeat(self):
        time_since_heartbeat = (
            self.get_clock().now() - self.last_heartbeat
        ).nanoseconds / 1e9

        if time_since_heartbeat > 5.0:  # 5 seconds timeout
            self.enter_safe_mode()
```

---

## Exercise 7: Integration Testing

### Objective
Create comprehensive tests for the digital twin system.

### Challenge
Develop tests that verify:
- End-to-end communication
- State consistency
- Error handling
- Performance requirements

### Test Categories
1. Unit tests for individual components
2. Integration tests for communication patterns
3. System tests for complete functionality
4. Performance tests for timing requirements

### Example Test
```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from digital_twin_bridge import DigitalTwinBridge

class TestDigitalTwinIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.bridge = DigitalTwinBridge()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge)

    def test_state_synchronization(self):
        # Test that state is properly synchronized
        pass

    def test_command_execution(self):
        # Test that commands are executed correctly
        pass

    def tearDown(self):
        self.executor.shutdown()
        self.bridge.destroy_node()
        rclpy.shutdown()
```

---

## Exercise 8: Real-world Application

### Objective
Apply digital twin concepts to a practical scenario.

### Challenge Scenario
Create a digital twin for a warehouse robot that:
- Tracks inventory locations
- Plans optimal paths
- Coordinates with other robots
- Reports status to a central system

### Implementation Steps
1. Design the robot's URDF with appropriate sensors
2. Implement path planning algorithms
3. Create coordination protocols
4. Add status reporting mechanisms

### System Architecture
```
mermaid
graph TB
    subgraph "Central System"
        A[Inventory Management]
        B[Path Coordinator]
    end

    subgraph "Robot 1 Digital Twin"
        C[Python Agent]
        D[ROS 2 Controller]
        E[State Publisher]
    end

    subgraph "Robot 2 Digital Twin"
        F[Python Agent]
        G[ROS 2 Controller]
        H[State Publisher]
    end

    A --> C
    A --> F
    B --> C
    B --> F
    C --> D
    F --> G
    D --> E
    G --> H
```

---

## Solutions and Best Practices

### Common Pitfalls to Avoid
1. **Tight Coupling**: Keep agent and controller loosely coupled
2. **Inadequate Error Handling**: Always implement graceful failure modes
3. **Poor State Management**: Maintain consistent state across components
4. **Insufficient Testing**: Test both individual components and integrated systems

### Performance Tips
1. Use appropriate QoS settings for your use case
2. Implement efficient data structures for state management
3. Batch messages when possible to reduce overhead
4. Use threading for CPU-intensive operations

### Debugging Strategies
1. Use ROS 2 tools like `ros2 topic echo` and `ros2 node info`
2. Implement comprehensive logging
3. Use RViz for visual debugging
4. Create diagnostic nodes for system monitoring

## Chapter Summary

These exercises provide hands-on experience with implementing complete digital twin systems. By working through these challenges, you'll gain practical experience with:
- Communication pattern implementation
- System integration
- Performance optimization
- Fault tolerance
- Testing strategies

Each exercise builds on the previous ones, creating a comprehensive understanding of digital twin development for humanoid robotics applications.

---

## Navigation
- **Previous**: [Agent-Controller Integration](agent_controller_integration.md)
- **Next**: [Summary Checklist](summary_checklist.md)