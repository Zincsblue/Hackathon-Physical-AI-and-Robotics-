import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import time


class ROS2Controller(Node):
    """
    A ROS 2 controller that interfaces with Python agents for digital twin applications.
    This controller demonstrates how ROS 2 systems can receive commands from Python agents
    and provide feedback about robot state in a digital twin context.
    """

    def __init__(self):
        super().__init__('ros2_controller')

        # Create subscriber for receiving commands from the Python agent
        self.command_subscription = self.create_subscription(
            JointState,
            'digital_twin_commands',
            self.command_callback,
            10
        )

        # Create publisher for sending robot state back to the agent
        self.state_publisher = self.create_publisher(
            JointState,
            'robot_state',
            10
        )

        # Timer for publishing robot state periodically
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.state_timer_callback)

        # Initialize robot state (simulated)
        self.robot_state = JointState()
        self.robot_state.name = ['hip_pitch', 'hip_roll', 'knee', 'ankle_pitch', 'ankle_roll']
        self.robot_state.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.robot_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.robot_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Store the latest command received
        self.last_command = None

        self.get_logger().info('ROS 2 Controller initialized for digital twin communication')

    def command_callback(self, msg):
        """
        Callback function to handle incoming commands from the Python agent.
        """
        self.last_command = msg
        self.get_logger().info(f'Received command: {dict(zip(msg.name, msg.position))}')

        # Update robot state based on the command (simulated execution)
        for i, joint_name in enumerate(self.robot_state.name):
            # Find the corresponding command
            try:
                cmd_idx = msg.name.index(joint_name)
                # Simple simulation: move toward the commanded position
                current_pos = self.robot_state.position[i]
                target_pos = msg.position[cmd_idx]
                # Move 10% of the way toward the target
                new_pos = current_pos + 0.1 * (target_pos - current_pos)
                self.robot_state.position[i] = new_pos
            except ValueError:
                # Joint not in command, keep current position
                pass

    def state_timer_callback(self):
        """
        Timer callback to publish robot state periodically.
        """
        # Update timestamp
        self.robot_state.header.stamp = self.get_clock().now().to_msg()
        self.robot_state.header.frame_id = 'base_link'

        # Publish the current robot state
        self.state_publisher.publish(self.robot_state)
        self.get_logger().debug(f'Published robot state: {dict(zip(self.robot_state.name, self.robot_state.position))}')


def main(args=None):
    """
    Main function to initialize and run the ROS 2 controller.
    """
    rclpy.init(args=args)

    try:
        controller = ROS2Controller()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('ROS 2 controller stopped by user')
    except Exception as e:
        print(f'Error in ROS 2 controller: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()