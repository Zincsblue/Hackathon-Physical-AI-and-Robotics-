import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import time
import random
import math
from collections import deque


class PythonAgent(Node):
    """
    A Python agent that connects to ROS 2 controllers for digital twin applications.
    This agent demonstrates how Python software can interact with ROS 2 systems
    to control humanoid robots in a digital twin context with advanced communication patterns.
    """

    def __init__(self):
        super().__init__('python_agent')

        # Define QoS profiles for different communication needs
        reliable_qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        best_effort_qos = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Create publisher for sending commands to the robot (reliable)
        self.command_publisher = self.create_publisher(
            JointState,
            'digital_twin_commands',
            reliable_qos
        )

        # Create publisher for status updates
        self.status_publisher = self.create_publisher(
            String,
            'agent_status',
            best_effort_qos
        )

        # Create publisher for emergency stops
        self.emergency_stop_publisher = self.create_publisher(
            Bool,
            'emergency_stop',
            reliable_qos
        )

        # Create subscriber for receiving robot state
        self.state_subscription = self.create_subscription(
            JointState,
            'robot_state',
            self.state_callback,
            reliable_qos
        )

        # Create subscriber for receiving controller status
        self.controller_status_subscription = self.create_subscription(
            String,
            'controller_status',
            self.controller_status_callback,
            best_effort_qos
        )

        # Timer for sending periodic commands
        self.command_timer_period = 0.1  # seconds (10 Hz)
        self.command_timer = self.create_timer(self.command_timer_period, self.command_timer_callback)

        # Timer for sending status updates
        self.status_timer_period = 1.0  # seconds (1 Hz)
        self.status_timer = self.create_timer(self.status_timer_period, self.status_timer_callback)

        # Track the robot's current state
        self.current_robot_state = None

        # Store historical state for advanced processing
        self.state_history = deque(maxlen=20)  # Keep last 20 states
        self.last_state_time = time.time()

        # Advanced behavior flags
        self.is_active = True
        self.behavior_mode = "random"  # "random", "wave", "sine", "idle", "tracking"
        self.command_sequence = []  # For sequence-based commands

        self.get_logger().info('Python Agent initialized with advanced communication patterns for digital twin')

    def state_callback(self, msg):
        """
        Callback function to handle incoming robot state messages.
        """
        self.current_robot_state = msg
        self.get_logger().info(f'Received robot state: joints={len(msg.name)}, positions={msg.position}')

        # Store state in history for advanced processing
        self.state_history.append({
            'timestamp': time.time(),
            'names': msg.name.copy(),
            'positions': msg.position.copy()
        })

        # Calculate time since last state for frequency analysis
        current_time = time.time()
        time_diff = current_time - self.last_state_time
        self.last_state_time = current_time

        if time_diff > 0:
            frequency = 1.0 / time_diff
            self.get_logger().info(f'State update frequency: {frequency:.2f} Hz')

    def controller_status_callback(self, msg):
        """
        Callback for controller status updates.
        """
        self.get_logger().info(f'Controller status: {msg.data}')

        # React to controller status if needed
        if "error" in msg.data.lower():
            self.handle_controller_error()

    def command_timer_callback(self):
        """
        Timer callback to send commands to the robot periodically with advanced patterns.
        """
        if not self.is_active:
            return

        # Create a command message
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'

        # Define joint names for the digital twin robot
        cmd_msg.name = ['left_shoulder_pitch', 'left_elbow', 'right_shoulder_pitch',
                       'right_elbow', 'left_hip_pitch', 'left_knee', 'right_hip_pitch', 'right_knee']

        # Apply different behavior patterns based on mode
        if self.behavior_mode == "random":
            cmd_msg.position = [random.uniform(-1.0, 1.0) for _ in cmd_msg.name]
        elif self.behavior_mode == "wave":
            # Create a wave-like motion
            current_time = time.time()
            cmd_msg.position = [
                0.5 * math.sin(current_time * 0.5 + i * 0.5)
                for i in range(len(cmd_msg.name))
            ]
        elif self.behavior_mode == "sine":
            # Create sine wave motion with different frequencies
            current_time = time.time()
            cmd_msg.position = [
                0.8 * math.sin(current_time * (0.3 + 0.1 * i))
                for i in range(len(cmd_msg.name))
            ]
        elif self.behavior_mode == "tracking":
            # Track current state with slight modification
            if self.current_robot_state is not None:
                cmd_msg.position = [
                    pos + random.uniform(-0.1, 0.1)
                    for pos in self.current_robot_state.position
                ]
            else:
                cmd_msg.position = [0.0 for _ in cmd_msg.name]
        else:  # idle
            cmd_msg.position = [0.0 for _ in cmd_msg.name]

        cmd_msg.velocity = [0.0] * len(cmd_msg.name)
        cmd_msg.effort = [0.0] * len(cmd_msg.name)

        # Publish the command
        self.command_publisher.publish(cmd_msg)
        self.get_logger().info(f'Sent command: {dict(zip(cmd_msg.name, cmd_msg.position))}')

    def status_timer_callback(self):
        """
        Timer callback to send status updates about agent state.
        """
        status_msg = String()
        status_msg.data = f"Agent active, mode: {self.behavior_mode}, history_size: {len(self.state_history)}, active: {self.is_active}"
        self.status_publisher.publish(status_msg)

    def handle_controller_error(self):
        """
        Handle controller errors appropriately.
        """
        self.get_logger().warn('Controller reported error, switching to safe mode')
        self.behavior_mode = "idle"
        self.publish_emergency_stop(False)  # Disable emergency stop, just pause

    def publish_emergency_stop(self, stop=True):
        """
        Publish emergency stop command.
        """
        stop_msg = Bool()
        stop_msg.data = stop
        self.emergency_stop_publisher.publish(stop_msg)
        self.get_logger().info(f'Emergency stop {"activated" if stop else "deactivated"}')

    def change_behavior_mode(self, new_mode):
        """
        Change the behavior mode of the agent.
        """
        if new_mode in ["random", "wave", "sine", "idle", "tracking"]:
            self.get_logger().info(f'Changing behavior mode from {self.behavior_mode} to {new_mode}')
            self.behavior_mode = new_mode
        else:
            self.get_logger().warn(f'Invalid behavior mode: {new_mode}')


def main(args=None):
    """
    Main function to initialize and run the Python agent.
    """
    rclpy.init(args=args)

    try:
        agent = PythonAgent()
        rclpy.spin(agent)
    except KeyboardInterrupt:
        print('Python agent stopped by user')
    except Exception as e:
        print(f'Error in Python agent: {str(e)}')
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()