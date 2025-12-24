import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):
    """
    A subscriber node that receives joint state information from a humanoid robot.
    This demonstrates how to subscribe to joint state messages and process them.
    """

    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Create subscriber for joint states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        self.get_logger().info('Joint State Subscriber node initialized')

    def listener_callback(self, msg):
        """
        Callback function that processes received joint state messages.
        """
        try:
            # Process the received joint states
            joint_data = {}
            for i, name in enumerate(msg.name):
                position = msg.position[i] if i < len(msg.position) else 0.0
                velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
                effort = msg.effort[i] if i < len(msg.effort) else 0.0
                joint_data[name] = {'position': position, 'velocity': velocity, 'effort': effort}

            # Log the received joint states
            self.get_logger().info(f'Received joint states: {joint_data}')

            # Example of processing: check for any joint exceeding safety limits
            for joint_name, data in joint_data.items():
                if abs(data['position']) > 3.0:  # Example safety limit (3 radians)
                    self.get_logger().warn(f'Joint {joint_name} position limit exceeded: {data["position"]}')

        except Exception as e:
            self.get_logger().error(f'Error processing joint state message: {str(e)}')


def main(args=None):
    """
    Main function to initialize and run the joint state subscriber node.
    """
    rclpy.init(args=args)

    try:
        node = JointStateSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Joint state subscriber stopped by user')
    except Exception as e:
        print(f'Error in joint state subscriber: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()