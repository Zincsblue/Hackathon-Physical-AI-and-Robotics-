import rclpy
from rclpy.node import Node


class BaseRobotNode(Node):
    """
    Base class for robot nodes with error handling.
    This template can be extended for specific node implementations.
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name} node initialized')

    def safe_execute(self, func, *args, **kwargs):
        """
        Safely execute a function with error handling.
        """
        try:
            return func(*args, **kwargs)
        except Exception as e:
            self.get_logger().error(f'Error in {func.__name__}: {str(e)}')
            return None


def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)

    try:
        node = BaseRobotNode('base_robot_node')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error initializing node: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()