#!/usr/bin/env python3

"""
Simple VLA demonstration node
This node demonstrates the basic VLA concept in simulation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

# Import our custom action and message types
# Since we haven't generated the action files yet, we'll simulate them
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class SimpleVLADemoNode(Node):
    """
    Simple VLA demonstration node
    """

    def __init__(self):
        super().__init__('simple_vla_demo_node')

        # Publishers for simulation
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speech_pub = self.create_publisher(String, '/speech_output', 10)

        # Subscribers for simulation
        self.voice_sub = self.create_subscription(
            String,
            '/voice_input',
            self.voice_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Timer for demonstration
        self.timer = self.create_timer(1.0, self.demo_timer_callback)

        self.get_logger().info('Simple VLA Demo Node started')

        # Demo state
        self.demo_step = 0
        self.command_received = None
        self.scene_understood = False

    def voice_callback(self, msg):
        """Handle voice command input"""
        self.get_logger().info(f'Received voice command: {msg.data}')
        self.command_received = msg.data
        self.process_command(msg.data)

    def image_callback(self, msg):
        """Handle image input (simulated)"""
        self.get_logger().info('Received image data (simulated)')
        self.scene_understood = True

    def process_command(self, command):
        """Process the natural language command"""
        self.get_logger().info(f'Processing command: {command}')

        # Simple command processing
        if 'move' in command.lower() or 'go' in command.lower():
            self.execute_navigation_command(command)
        elif 'stop' in command.lower():
            self.stop_robot()
        else:
            self.get_logger().info(f'Command not recognized: {command}')

    def execute_navigation_command(self, command):
        """Execute a simple navigation command"""
        self.get_logger().info(f'Executing navigation: {command}')

        # Publish a simple movement command
        twist = Twist()
        twist.linear.x = 0.5  # Move forward at 0.5 m/s
        twist.angular.z = 0.0  # No rotation

        self.cmd_vel_pub.publish(twist)

        # Output speech confirmation
        response = String()
        response.data = f"Okay, I will {command}"
        self.speech_pub.publish(response)

    def stop_robot(self):
        """Stop the robot"""
        self.get_logger().info('Stopping robot')

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

        # Output speech confirmation
        response = String()
        response.data = "Robot stopped"
        self.speech_pub.publish(response)

    def demo_timer_callback(self):
        """Timer callback for demonstration sequence"""
        self.demo_step += 1

        if self.demo_step == 1:
            self.get_logger().info("Demo Step 1: Ready to receive commands")
        elif self.demo_step == 10:  # After 10 seconds
            self.get_logger().info("Demo Step 2: Demonstrating basic movement")
            # Simulate a command
            self.execute_navigation_command("move forward")
        elif self.demo_step == 20:  # After 20 seconds
            self.get_logger().info("Demo Step 3: Stopping robot")
            self.stop_robot()
        elif self.demo_step >= 30:  # Reset after 30 seconds
            self.demo_step = 0


def main(args=None):
    rclpy.init(args=args)

    vla_demo_node = SimpleVLADemoNode()

    try:
        rclpy.spin(vla_demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        vla_demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()