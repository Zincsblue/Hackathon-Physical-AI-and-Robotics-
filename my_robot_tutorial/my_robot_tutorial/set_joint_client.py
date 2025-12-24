import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool


class SetJointClient(Node):
    """
    A service client that sends requests to set joint angles for a humanoid robot.
    This demonstrates how to call a service in ROS 2.
    """

    def __init__(self):
        super().__init__('set_joint_client')

        # Create a client for the set_joint_angle service
        self.client = self.create_client(SetBool, 'set_joint_angle')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Set Joint Client initialized')

    def send_request(self, set_angle=True):
        """
        Send a request to the set_joint_angle service.
        """
        request = SetBool.Request()
        request.data = set_angle  # True to set angle, False to reset

        # Call the service asynchronously
        self.future = self.client.call_async(request)
        self.get_logger().info(f'Sent request to set joint angle: {set_angle}')
        return self.future


def main(args=None):
    """
    Main function to initialize and run the set joint client.
    """
    rclpy.init(args=args)

    try:
        client = SetJointClient()

        # Determine whether to set the angle (True) or reset (False) based on command line argument
        if len(sys.argv) > 1:
            if sys.argv[1].lower() in ['true', '1', 'yes', 'set']:
                set_angle = True
            elif sys.argv[1].lower() in ['false', '0', 'no', 'reset']:
                set_angle = False
            else:
                print("Usage: ros2 run my_robot_tutorial set_joint_client [true|false]")
                print("  true/1/yes/set: Set joint to new angle")
                print("  false/0/no/reset: Reset joint to home position")
                return
        else:
            # Default to setting the angle
            set_angle = True

        # Send the request
        future = client.send_request(set_angle)

        # Wait for the response
        rclpy.spin_until_future_complete(client, future)

        try:
            response = future.result()
            if response.success:
                client.get_logger().info(f'Success: {response.message}')
            else:
                client.get_logger().error(f'Failed: {response.message}')
        except Exception as e:
            client.get_logger().error(f'Exception while calling service: {str(e)}')

    except KeyboardInterrupt:
        print('Set joint client stopped by user')
    except Exception as e:
        print(f'Error in set joint client: {str(e)}')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()