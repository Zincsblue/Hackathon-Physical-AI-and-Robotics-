#!/usr/bin/env python3

"""
Base Isaac Node Template

This is a template for Isaac integration examples in isaac_examples package.
It provides a foundation for creating Isaac ROS nodes with proper structure.
"""

import rclpy
from rclpy.node import Node


class BaseIsaacNode(Node):
    """
    Base class for Isaac integration nodes.
    Provides common functionality and structure for Isaac ROS nodes.
    """

    def __init__(self, node_name='base_isaac_node'):
        super().__init__(node_name)
        self.get_logger().info(f'Initializing {node_name}')

        # Initialize common Isaac-specific parameters here
        self.declare_parameters(
            namespace='',
            parameters=[
                ('simulation_mode', True),
                ('hardware_acceleration', True),
                ('sensor_topic_prefix', '/isaac/sensors'),
            ]
        )

        # Add common functionality initialization here
        self.setup_isaac_components()

    def setup_isaac_components(self):
        """Setup common Isaac components and connections."""
        self.get_logger().info('Setting up Isaac components...')
        # This would contain Isaac-specific initialization code
        pass

    def on_simulation_tick(self):
        """Called on each simulation tick in Isaac Sim."""
        # Override in subclasses for simulation-specific behavior
        pass


def main(args=None):
    rclpy.init(args=args)

    node = BaseIsaacNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()