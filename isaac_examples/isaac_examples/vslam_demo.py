#!/usr/bin/env python3

"""
Isaac ROS VSLAM Demo

This module demonstrates Visual Simultaneous Localization and Mapping (VSLAM)
using Isaac ROS for hardware-accelerated processing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class VSLAMDemoNode(Node):
    """
    VSLAM demonstration node using Isaac ROS.
    Shows hardware-accelerated VSLAM capabilities.
    """

    def __init__(self):
        super().__init__('vslam_demo_node')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_hardware_acceleration', True),
                ('cuda_device_id', 0),
                ('enable_loop_closure', True),
                ('tracking_rate', 30.0),
                ('map_resolution', 0.05),  # meters per pixel
            ]
        )

        # Create subscribers for camera data
        self.left_image_sub = self.create_subscription(
            Image,
            '/isaac/sensors/stereo_camera/left/image_raw',
            self.left_image_callback,
            10
        )

        self.right_image_sub = self.create_subscription(
            Image,
            '/isaac/sensors/stereo_camera/right/image_raw',
            self.right_image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/isaac/sensors/stereo_camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publishers for VSLAM results
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/isaac/vslam/pose',
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/isaac/vslam/odometry',
            10
        )

        # Initialize VSLAM components
        self.setup_vslam_system()

        self.get_logger().info('VSLAM Demo Node initialized')

    def setup_vslam_system(self):
        """Setup the VSLAM system with Isaac ROS components."""
        self.get_logger().info('Setting up VSLAM system...')

        use_hardware_acceleration = self.get_parameter('use_hardware_acceleration').value
        cuda_device_id = self.get_parameter('cuda_device_id').value

        if use_hardware_acceleration:
            self.get_logger().info(f'Using hardware-accelerated VSLAM on CUDA device {cuda_device_id}')
        else:
            self.get_logger().info('Hardware acceleration disabled for VSLAM')

    def left_image_callback(self, msg):
        """Process left camera image for stereo VSLAM."""
        self.get_logger().debug(f'Processing left image: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

    def right_image_callback(self, msg):
        """Process right camera image for stereo VSLAM."""
        self.get_logger().debug(f'Processing right image: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

    def camera_info_callback(self, msg):
        """Process stereo camera calibration information."""
        self.get_logger().debug(f'Stereo camera info: {msg.width}x{msg.height}')

    def publish_pose(self, position, orientation, header):
        """Publish estimated pose from VSLAM."""
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose.position.x = position[0] if len(position) > 0 else 0.0
        pose_stamped.pose.position.y = position[1] if len(position) > 1 else 0.0
        pose_stamped.pose.position.z = position[2] if len(position) > 2 else 0.0

        if len(orientation) >= 4:
            pose_stamped.pose.orientation.x = orientation[0]
            pose_stamped.pose.orientation.y = orientation[1]
            pose_stamped.pose.orientation.z = orientation[2]
            pose_stamped.pose.orientation.w = orientation[3]

        self.pose_pub.publish(pose_stamped)

    def publish_odometry(self, pose, header):
        """Publish odometry from VSLAM."""
        odom = Odometry()
        odom.header = header
        odom.pose.pose = pose

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    node = VSLAMDemoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()