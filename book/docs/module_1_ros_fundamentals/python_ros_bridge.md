# Python Agent → ROS Controller Bridge

In robotics, the bridge between high-level Python agents and ROS controllers is crucial for connecting intelligent decision-making algorithms to the physical robot. This section explains how Python agents can communicate with ROS-based robot controllers using the rclpy library.

## Understanding the Bridge Concept

The Python agent → ROS controller bridge refers to the communication pathway that allows:

- **Python agents**: High-level decision-making algorithms written in Python
- **ROS controllers**: Low-level control systems that interface with robot hardware
- **Bidirectional communication**: Information flow in both directions

This architecture enables AI and machine learning algorithms to control robots through the ROS ecosystem.

## Basic Bridge Implementation

Here's a simple example of how a Python agent might interact with ROS controllers:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from example_interfaces.srv import Trigger
import numpy as np
import time

class PythonAgentBridge(Node):
    def __init__(self):
        super().__init__('python_agent_bridge')

        # Publishers for sending commands to controllers
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Subscribers for receiving sensor data from controllers
        self.sensor_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Services for requesting specific actions from controllers
        self.control_service_client = self.create_client(
            Trigger,
            '/move_to_home_position'
        )

        # Store current robot state
        self.current_joint_positions = {}
        self.target_joint_positions = {}

        self.get_logger().info('Python Agent Bridge initialized')

    def joint_state_callback(self, msg):
        """Callback to process joint state updates from controllers"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

        self.get_logger().info(f'Updated joint states: {len(self.current_joint_positions)} joints')

    def send_joint_commands(self, joint_commands):
        """Send joint position commands to the controller"""
        msg = Float64MultiArray()
        msg.data = list(joint_commands.values())  # Assuming joint_commands is a dict

        self.command_publisher.publish(msg)
        self.get_logger().info(f'Sent joint commands: {list(joint_commands.values())}')

    def request_home_position(self):
        """Request the robot to move to home position"""
        if not self.control_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Control service not available')
            return False

        request = Trigger.Request()
        future = self.control_service_client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Home position request: {response.success}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False

def main(args=None):
    rclpy.init(args=args)
    bridge = PythonAgentBridge()

    # Example usage: Send some commands and request home position
    try:
        # Wait a bit for connections to establish
        time.sleep(1.0)

        # Example: Send joint commands (in a real system, these would come from the Python agent)
        commands = {
            'shoulder_joint': 0.5,
            'elbow_joint': 0.3,
            'wrist_joint': 0.1
        }
        bridge.send_joint_commands(commands)

        # Wait a bit before requesting home position
        time.sleep(2.0)

        # Request to move to home position
        success = bridge.request_home_position()
        if success:
            bridge.get_logger().info('Successfully moved to home position')
        else:
            bridge.get_logger().info('Failed to move to home position')

        # Keep the bridge running to receive sensor data
        rclpy.spin(bridge)

    except KeyboardInterrupt:
        bridge.get_logger().info('Shutting down Python Agent Bridge...')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Bridge with AI Agent Integration

Here's a more sophisticated example that shows how an AI agent might use the bridge:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
import time
import random

class AIAgentBridge(Node):
    def __init__(self):
        super().__init__('ai_agent_bridge')

        # Publishers for different types of commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        self.velocity_command_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscribers for sensor data
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.status_subscriber = self.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            10
        )

        # Internal state
        self.current_state = {
            'joint_positions': {},
            'robot_status': 'idle',
            'timestamp': None
        }

        # Timer for AI decision making
        self.ai_timer = self.create_timer(0.5, self.ai_decision_loop)  # 2Hz AI decisions

        self.get_logger().info('AI Agent Bridge initialized')

    def joint_state_callback(self, msg):
        """Process joint state updates"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_state['joint_positions'][name] = msg.position[i]
        self.current_state['timestamp'] = self.get_clock().now()

    def status_callback(self, msg):
        """Process robot status updates"""
        self.current_state['robot_status'] = msg.data

    def ai_decision_loop(self):
        """Main AI decision-making loop"""
        # This is where your AI algorithm would run
        # For this example, we'll implement a simple movement pattern

        if self.current_state['robot_status'] == 'ready':
            # Generate a decision based on current state
            action = self.generate_action()
            self.execute_action(action)

    def generate_action(self):
        """Generate an action based on current state (simplified AI)"""
        # In a real AI agent, this would use ML models, planning algorithms, etc.

        # Simple example: random joint movement
        if len(self.current_state['joint_positions']) > 0:
            # Create target positions slightly different from current
            targets = {}
            for joint_name, current_pos in self.current_state['joint_positions'].items():
                # Add small random offset
                offset = random.uniform(-0.1, 0.1)
                targets[joint_name] = current_pos + offset

            return {
                'type': 'joint_move',
                'targets': targets
            }
        else:
            # Default action
            return {
                'type': 'status_check',
                'message': 'No joint data available'
            }

    def execute_action(self, action):
        """Execute the AI-generated action"""
        if action['type'] == 'joint_move':
            # Publish joint commands
            msg = Float64MultiArray()
            msg.data = list(action['targets'].values())
            self.joint_command_publisher.publish(msg)

            self.get_logger().info(f'AI action: Moving joints to {action["targets"]}')
        elif action['type'] == 'status_check':
            self.get_logger().info(f'AI status: {action["message"]}')

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AIAgentBridge()

    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        ai_bridge.get_logger().info('Shutting down AI Agent Bridge...')
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bridge Patterns and Best Practices

### 1. Publisher-Subscriber Pattern for Continuous Data

For continuous sensor data and commands:

```python
# In the bridge node
def publish_sensor_data(self, sensor_data):
    """Publish processed sensor data for the AI agent"""
    msg = # Create appropriate message type
    # Populate message with sensor_data
    self.sensor_publisher.publish(msg)
```

### 2. Service Pattern for Discrete Actions

For discrete actions that require confirmation:

```python
# In the bridge node
def call_controller_service(self, command):
    """Call controller service and wait for response"""
    if self.controller_service_client.wait_for_service(timeout_sec=1.0):
        request = # Create appropriate request
        future = self.controller_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    else:
        return None
```

### 3. Action Pattern for Long-Running Tasks

For tasks that provide feedback during execution:

```python
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

class ActionBridge(Node):
    def __init__(self):
        super().__init__('action_bridge')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory'
        )
```

## Error Handling and Safety

A robust bridge should include error handling:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Time
import traceback

class RobustBridge(Node):
    def __init__(self):
        super().__init__('robust_bridge')

        # Use appropriate QoS settings for reliability
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/safe_joint_commands',
            qos_profile
        )

        # Safety timer to monitor communication
        self.safety_timer = self.create_timer(1.0, self.safety_check)
        self.last_communication_time = self.get_clock().now()

        self.get_logger().info('Robust Bridge initialized with safety checks')

    def safety_check(self):
        """Check if communication is still active"""
        current_time = self.get_clock().now()
        time_since_comm = (current_time - self.last_communication_time).nanoseconds / 1e9

        if time_since_comm > 5.0:  # 5 seconds without communication
            self.get_logger().warn('No communication for 5 seconds - safety stop!')
            self.emergency_stop()

    def emergency_stop(self):
        """Send emergency stop command"""
        # Send zero commands to all joints
        msg = Float64MultiArray()
        msg.data = [0.0] * 6  # Assuming 6 joints
        self.command_publisher.publish(msg)
        self.get_logger().info('Emergency stop command sent')

def main(args=None):
    rclpy.init(args=args)
    bridge = RobustBridge()

    try:
        rclpy.spin(bridge)
    except Exception as e:
        bridge.get_logger().error(f'Bridge error: {str(e)}')
        traceback.print_exc()
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid Robot Context

In humanoid robotics, the Python agent → ROS controller bridge is particularly important because:

- **Complex Control**: Humanoid robots have many degrees of freedom requiring sophisticated control
- **AI Integration**: Machine learning and AI algorithms need to interface with robot controllers
- **Safety Critical**: The bridge must ensure safe operation of the robot
- **Real-time Requirements**: The bridge must handle real-time communication needs

The bridge enables high-level AI algorithms to control the complex kinematics and dynamics of humanoid robots while ensuring safety and reliability through the ROS ecosystem.