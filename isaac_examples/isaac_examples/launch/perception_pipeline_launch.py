#!/usr/bin/env python3

"""
Launch file for Isaac ROS Perception Pipeline

This launch file starts the Isaac ROS perception pipeline with proper configuration.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package and configuration paths
    pkg_share = get_package_share_directory('isaac_examples')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    hardware_acceleration = LaunchConfiguration('hardware_acceleration', default='true')

    # Perception pipeline node
    perception_pipeline_node = Node(
        package='isaac_examples',
        executable='perception_pipeline',
        name='perception_pipeline_node',
        parameters=[
            {
                'use_hardware_acceleration': hardware_acceleration,
                'cuda_device_id': 0,
                'tensorrt_engine_path': '',
                'enable_vslam': True,
                'tracking_rate': 30.0,
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )

    # Isaac ROS visual slam node (conceptual - would use actual Isaac ROS packages in real implementation)
    visual_slam_node = Node(
        package='isaac_examples',
        executable='vslam_demo',
        name='vslam_demo_node',
        parameters=[
            {
                'use_hardware_acceleration': hardware_acceleration,
                'cuda_device_id': 0,
                'enable_loop_closure': True,
                'tracking_rate': 30.0,
                'map_resolution': 0.05,
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        perception_pipeline_node,
        visual_slam_node
    ])