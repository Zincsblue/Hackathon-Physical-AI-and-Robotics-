import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool


class SetJointService(Node):
    """
    A service server that handles requests to set joint angles for a humanoid robot.
    This demonstrates the request-response communication pattern in ROS 2.
    """

    def __init__(self):
        super().__init__('set_joint_service')

        # Create a service that listens on the 'set_joint_angle' topic
        self.srv = self.create_service(
            SetBool,
            'set_joint_angle',
            self.set_joint_callback
        )

        # Keep track of the current joint angle (simulated)
        self.current_joint_angle = 0.0

        self.get_logger().info('Set Joint Service server initialized')

    def set_joint_callback(self, request, response):
        """
        Callback function that handles service requests to set joint angles.
        """
        try:
            # Process the request - in a real robot, this would set the actual joint
            if request.data:  # If the request is True, set a new angle
                # For this example, we'll just set a fixed angle
                self.current_joint_angle = 1.57  # 90 degrees in radians
                response.success = True
                response.message = f'Joint angle set to {self.current_joint_angle} radians'
                self.get_logger().info(f'Set joint angle to {self.current_joint_angle}')
            else:  # If the request is False, perhaps return to home position
                self.current_joint_angle = 0.0
                response.success = True
                response.message = f'Joint angle reset to home position ({self.current_joint_angle} radians)'
                self.get_logger().info('Reset joint angle to home position')

        except Exception as e:
            self.get_logger().error(f'Error processing set joint request: {str(e)}')
            response.success = False
            response.message = f'Error setting joint angle: {str(e)}'

        return response


def main(args=None):
    """
    Main function to initialize and run the set joint service server.
    """
    rclpy.init(args=args)

    try:
        node = SetJointService()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Set joint service server stopped by user')
    except Exception as e:
        print(f'Error in set joint service: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()