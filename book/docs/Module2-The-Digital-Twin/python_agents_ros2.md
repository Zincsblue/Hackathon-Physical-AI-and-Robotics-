# Python Agents and ROS 2 Controller Connection

## Learning Objectives
By the end of this section, you will be able to:
- Create Python agents that connect to ROS 2 controllers using rclpy
- Implement proper communication patterns between agents and controllers
- Understand the message flow in agent-controller systems
- Design bidirectional communication for digital twin applications
- Apply best practices for agent-controller integration

## Introduction to Python Agents in ROS 2

Python agents in the context of ROS 2 are software components written in Python that interact with ROS 2 systems. These agents can act as high-level decision makers, data processors, or user interfaces that communicate with ROS 2 nodes running controllers or other robot systems.

In digital twin applications, Python agents often serve as the interface between the virtual model and external systems, processing data, making decisions, and sending commands that are then executed by ROS 2 controllers on the physical robot.

## Architecture of Agent-Controller Communication

The communication between Python agents and ROS 2 controllers typically follows this pattern:

```
mermaid
graph LR
    subgraph "Python Agent"
        A[Agent Logic]
        B[ROS 2 Client Library]
    end

    subgraph "ROS 2 System"
        C[ROS 2 Controller]
        D[DDS Middleware]
        E[Robot Hardware Interface]
    end

    A --> B
    B <--> D
    D --> C
    C --> E
    E --> D
    D --> B
    B --> A
```

## Creating a Python Agent

A Python agent in ROS 2 is implemented as a ROS 2 node using the rclpy library. Here's the basic structure:

### Basic Agent Structure

```python
import rclpy
from rclpy.node import Node

class PythonAgent(Node):
    def __init__(self):
        super().__init__('python_agent')

        # Create publishers and subscribers
        # Initialize timers
        # Set up any other components
```

### Publishers and Subscribers

Python agents typically both send commands to controllers and receive state information back:

```python
# Publisher for sending commands
self.command_publisher = self.create_publisher(
    JointState,
    'digital_twin_commands',
    10
)

# Subscriber for receiving robot state
self.state_subscription = self.create_subscription(
    JointState,
    'robot_state',
    self.state_callback,
    10
)
```

### Message Callbacks

The agent processes incoming messages through callback functions:

```python
def state_callback(self, msg):
    # Process the received state message
    self.current_robot_state = msg
    # Perform any necessary actions based on the state
```

## Creating a ROS 2 Controller

ROS 2 controllers are also ROS 2 nodes, but they typically interface more directly with robot hardware or simulation. The controller receives commands from agents and provides feedback about robot state.

### Basic Controller Structure

```python
import rclpy
from rclpy.node import Node

class ROS2Controller(Node):
    def __init__(self):
        super().__init__('ros2_controller')

        # Create subscribers for commands
        # Create publishers for state feedback
        # Initialize hardware interfaces
```

## Communication Patterns

### 1. Request-Response Pattern
The agent sends a command and waits for a response from the controller.

### 2. Publish-Subscribe Pattern (Most Common)
The agent publishes commands to a topic, and the controller subscribes to receive them. Similarly, the controller publishes state information that the agent subscribes to.

### 3. Service-Based Pattern
For more synchronous interactions, agents can call services provided by controllers.

## Digital Twin Specific Considerations

### Real-time Synchronization
In digital twin applications, timing is critical:
- Agents and controllers must maintain consistent timestamps
- Communication frequency should be high enough for real-time operation
- Latency between physical and virtual systems should be minimized

### Data Consistency
- Ensure that the virtual model accurately reflects the physical robot
- Handle network interruptions gracefully
- Implement state reconciliation mechanisms

### Bidirectional Flow
- Commands flow from agent to controller
- State information flows from controller to agent
- Both flows must be reliable and timely

## Implementation Example

Let's look at a complete example of an agent-controller pair for digital twin applications:

### Python Agent Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random

class PythonAgent(Node):
    def __init__(self):
        super().__init__('python_agent')

        # Publisher for commands
        self.command_publisher = self.create_publisher(
            JointState,
            'digital_twin_commands',
            10
        )

        # Subscriber for state
        self.state_subscription = self.create_subscription(
            JointState,
            'robot_state',
            self.state_callback,
            10
        )

        # Timer for periodic command sending
        self.timer = self.create_timer(1.0, self.timer_callback)

    def state_callback(self, msg):
        # Process state information
        self.get_logger().info(f'Received state: {len(msg.name)} joints')

    def timer_callback(self):
        # Generate and send commands
        cmd_msg = JointState()
        cmd_msg.name = ['joint1', 'joint2', 'joint3']
        cmd_msg.position = [random.uniform(-1.0, 1.0) for _ in cmd_msg.name]
        self.command_publisher.publish(cmd_msg)
```

### ROS 2 Controller Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ROS2Controller(Node):
    def __init__(self):
        super().__init__('ros2_controller')

        # Subscriber for commands
        self.command_subscription = self.create_subscription(
            JointState,
            'digital_twin_commands',
            self.command_callback,
            10
        )

        # Publisher for state
        self.state_publisher = self.create_publisher(
            JointState,
            'robot_state',
            10
        )

        # Timer for state publishing
        self.timer = self.create_timer(0.1, self.state_timer_callback)

    def command_callback(self, msg):
        # Process incoming commands
        self.get_logger().info(f'Received command: {dict(zip(msg.name, msg.position))}')

    def state_timer_callback(self):
        # Publish current robot state
        state_msg = JointState()
        # Populate with current robot state
        self.state_publisher.publish(state_msg)
```

## Best Practices

### 1. Error Handling
Always implement proper error handling in both agents and controllers:

```python
def command_callback(self, msg):
    try:
        # Process command
        self.execute_command(msg)
    except Exception as e:
        self.get_logger().error(f'Error processing command: {str(e)}')
```

### 2. Graceful Degradation
Handle network interruptions and other failures gracefully:

```python
def state_callback(self, msg):
    if msg is not None:
        self.current_robot_state = msg
    else:
        self.get_logger().warning('Received null state message')
```

### 3. Resource Management
Properly clean up resources when the node shuts down:

```python
def destroy_node(self):
    # Clean up any resources
    super().destroy_node()
```

## Chapter Summary

Python agents and ROS 2 controllers form the backbone of digital twin communication systems. By implementing proper communication patterns, you can create robust systems that enable real-time synchronization between physical and virtual robot systems. The key is to maintain consistent, reliable communication with appropriate error handling and resource management.

## Exercises

1. Modify the provided Python agent to implement a specific behavior (e.g., moving a joint in a sine wave pattern).

2. Create a new message type for a custom digital twin application and implement both agent and controller to use it.

3. Add quality of service (QoS) settings to the publishers and subscribers to ensure reliable communication.

4. Implement a simple state machine in the Python agent that changes behavior based on robot state.

---

## Navigation
- **Previous**: [Digital Twin Concepts](digital_twin_concepts.md)
- **Next**: [URDF Understanding for Robot Structure](urdf_robot_modeling.md)