import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import random


class JointStatePublisher(Node):
    """
    A publisher node that publishes joint state information for a humanoid robot.
    This simulates the data that would come from actual robot joint encoders.
    """

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Set up a timer to publish at 50Hz
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize joint positions (simulated values for a simple humanoid arm)
        self.joint_positions = {
            'shoulder_pitch': 0.0,
            'shoulder_roll': 0.0,
            'elbow_yaw': 0.0,
            'elbow_pitch': 0.0,
            'wrist_pitch': 0.0,
            'wrist_yaw': 0.0
        }

        self.get_logger().info('Joint State Publisher node initialized')

    def timer_callback(self):
        """
        Callback function that publishes joint state messages at regular intervals.
        """
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Update joint positions with simple oscillating motion
        time_sec = self.get_clock().now().nanoseconds / 1e9
        for joint_name in self.joint_positions:
            # Create a simple oscillating motion for each joint
            self.joint_positions[joint_name] = 0.5 * math.sin(time_sec + hash(joint_name) % 10)

        # Set the joint names and positions
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())

        # Set velocities and efforts to zero (for simplicity)
        msg.velocity = [0.0] * len(msg.position)
        msg.effort = [0.0] * len(msg.position)

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published joint states: {dict(zip(msg.name, msg.position))}')


def main(args=None):
    """
    Main function to initialize and run the joint state publisher node.
    """
    rclpy.init(args=args)

    try:
        node = JointStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in joint state publisher: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()