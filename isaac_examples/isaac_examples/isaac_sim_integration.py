#!/usr/bin/env python3

"""
Isaac Sim Integration Node

This module provides integration between ROS 2 and Isaac Sim,
handling the bridge between simulation and the ROS ecosystem.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
import json
import os


class IsaacSimIntegrationNode(Node):
    """
    Node for integrating with Isaac Sim.
    Handles communication between ROS 2 and Isaac Sim environments.
    """

    def __init__(self):
        super().__init__('isaac_sim_integration')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('simulation_config_path', ''),
                ('simulation_mode', True),
                ('robot_name', 'humanoid_robot'),
                ('publish_sensor_data', True),
                ('subscribe_control_commands', True),
                ('simulation_step_rate', 50.0),  # Hz
            ]
        )

        # Get parameter values
        self.sim_config_path = self.get_parameter('simulation_config_path').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.robot_name = self.get_parameter('robot_name').value

        # Publishers for sensor data from simulation
        self.camera_pub = self.create_publisher(
            Image,
            f'/{self.robot_name}/sensors/camera/image_raw',
            10
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            f'/{self.robot_name}/sensors/camera/camera_info',
            10
        )

        self.imu_pub = self.create_publisher(
            Imu,
            f'/{self.robot_name}/sensors/imu',
            10
        )

        self.lidar_pub = self.create_publisher(
            PointCloud2,
            f'/{self.robot_name}/sensors/lidar/points',
            10
        )

        # Subscribers for control commands to simulation
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for simulation loop
        self.sim_step_rate = self.get_parameter('simulation_step_rate').value
        self.timer = self.create_timer(1.0 / self.sim_step_rate, self.simulation_step)

        # Load simulation configuration if available
        self.load_simulation_config()

        self.get_logger().info(f'Isaac Sim Integration Node initialized for {self.robot_name}')
        self.get_logger().info(f'Simulation mode: {self.simulation_mode}')

    def load_simulation_config(self):
        """Load simulation configuration from file."""
        if self.sim_config_path and os.path.exists(self.sim_config_path):
            try:
                with open(self.sim_config_path, 'r') as f:
                    config = json.load(f)
                self.get_logger().info(f'Loaded simulation config from {self.sim_config_path}')
                self.sim_config = config
            except Exception as e:
                self.get_logger().error(f'Failed to load simulation config: {e}')
                self.sim_config = {}
        else:
            self.get_logger().info('No simulation config file provided or file does not exist')
            self.sim_config = {}

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS 2."""
        self.get_logger().debug(f'Received velocity command: linear={msg.linear.x}, angular={msg.angular.z}')
        # In a real implementation, this would send the command to Isaac Sim
        # For this educational example, we'll just log it

    def simulation_step(self):
        """Main simulation step function."""
        # In a real implementation, this would interface with Isaac Sim
        # to get sensor data and send control commands
        # For this educational example, we'll simulate basic sensor data publishing

        if self.get_parameter('publish_sensor_data').value:
            self.publish_simulated_sensor_data()

    def publish_simulated_sensor_data(self):
        """Publish simulated sensor data from the Isaac Sim environment."""
        import time
        from builtin_interfaces.msg import Time

        # Create timestamp
        current_time = self.get_clock().now().to_msg()

        # Publish simulated camera data
        camera_msg = Image()
        camera_msg.header.stamp = current_time
        camera_msg.header.frame_id = f'{self.robot_name}_camera_frame'
        camera_msg.height = 480
        camera_msg.width = 640
        camera_msg.encoding = 'rgb8'
        camera_msg.is_bigendian = False
        camera_msg.step = 640 * 3  # width * channels
        camera_msg.data = [0] * (640 * 480 * 3)  # Simulated empty image data

        self.camera_pub.publish(camera_msg)

        # Publish simulated IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = f'{self.robot_name}_imu_frame'
        # Simulated values
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # Gravity
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        self.imu_pub.publish(imu_msg)

    def on_shutdown(self):
        """Cleanup when node shuts down."""
        self.get_logger().info('Shutting down Isaac Sim Integration Node')


def main(args=None):
    rclpy.init(args=args)

    node = IsaacSimIntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()