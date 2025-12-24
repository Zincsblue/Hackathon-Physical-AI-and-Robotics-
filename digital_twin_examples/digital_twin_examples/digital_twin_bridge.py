import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time


class DigitalTwinBridge(Node):
    """
    A bridge node that facilitates communication between the physical robot
    and its digital twin representation. This node synchronizes state between
    the real and virtual systems, ensuring bidirectional communication.
    """

    def __init__(self):
        super().__init__('digital_twin_bridge')

        # Publisher for sending commands to the physical robot
        self.physical_command_publisher = self.create_publisher(
            JointState,
            'physical_robot_commands',
            10
        )

        # Publisher for sending state to the digital twin
        self.digital_twin_state_publisher = self.create_publisher(
            JointState,
            'digital_twin_robot_state',
            10
        )

        # Subscriber for receiving physical robot state
        self.physical_state_subscription = self.create_subscription(
            JointState,
            'physical_robot_state',
            self.physical_state_callback,
            10
        )

        # Subscriber for receiving digital twin commands
        self.digital_twin_command_subscription = self.create_subscription(
            JointState,
            'digital_twin_commands',
            self.digital_twin_command_callback,
            10
        )

        # Timer for synchronization
        self.timer = self.create_timer(0.05, self.synchronization_callback)  # 20 Hz

        # Store the latest states
        self.latest_physical_state = None
        self.latest_digital_twin_state = None

        self.get_logger().info('Digital Twin Bridge initialized')

    def physical_state_callback(self, msg):
        """
        Callback for receiving physical robot state.
        Updates the digital twin with the physical robot's current state.
        """
        self.latest_physical_state = msg
        self.get_logger().debug(f'Received physical state: {len(msg.name)} joints')

        # Forward physical state to digital twin visualization
        self.digital_twin_state_publisher.publish(msg)

    def digital_twin_command_callback(self, msg):
        """
        Callback for receiving commands from the digital twin.
        Sends these commands to the physical robot.
        """
        self.get_logger().debug(f'Received digital twin commands: {dict(zip(msg.name, msg.position))}')

        # Forward commands to physical robot
        self.physical_command_publisher.publish(msg)

    def synchronization_callback(self):
        """
        Synchronization callback that ensures state consistency
        between physical and digital systems.
        """
        # In a real implementation, this would handle synchronization logic
        # such as state reconciliation, latency compensation, etc.
        pass

    def get_current_state(self):
        """
        Get the current state of the robot (physical or fallback to digital twin).
        """
        if self.latest_physical_state is not None:
            return self.latest_physical_state
        elif self.latest_digital_twin_state is not None:
            return self.latest_digital_twin_state
        else:
            # Return an empty state if no state is available
            empty_state = JointState()
            empty_state.header = Header()
            empty_state.header.stamp = self.get_clock().now().to_msg()
            return empty_state


def main(args=None):
    """
    Main function to run the digital twin bridge.
    """
    rclpy.init(args=args)

    try:
        bridge = DigitalTwinBridge()

        # Add some example joint names for demonstration
        bridge.get_logger().info('Digital Twin Bridge running...')
        bridge.get_logger().info('Listening for physical robot state and digital twin commands')

        rclpy.spin(bridge)

    except KeyboardInterrupt:
        print('Digital twin bridge stopped by user')
    except Exception as e:
        print(f'Error in digital twin bridge: {str(e)}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()