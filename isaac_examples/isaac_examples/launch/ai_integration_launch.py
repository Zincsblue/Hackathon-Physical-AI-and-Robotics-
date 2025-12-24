#!/usr/bin/env python3

"""
Launch file for Isaac AI Integration

This launch file starts the complete AI-robot brain integration with Isaac Sim, ROS, and Nav2.
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

    # Isaac Sim integration bridge node
    isaac_sim_bridge = Node(
        package='isaac_examples',
        executable='isaac_sim_integration',
        name='isaac_sim_bridge',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'simulation_mode': True
            }
        ],
        output='screen'
    )

    # Perception pipeline node
    perception_pipeline = Node(
        package='isaac_examples',
        executable='perception_pipeline',
        name='perception_pipeline',
        parameters=[
            {
                'use_hardware_acceleration': hardware_acceleration,
                'cuda_device_id': 0,
                'tensorrt_engine_path': '',
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )

    # VSLAM demo node
    vslam_demo = Node(
        package='isaac_examples',
        executable='vslam_demo',
        name='vslam_demo',
        parameters=[
            {
                'use_hardware_acceleration': hardware_acceleration,
                'cuda_device_id': 0,
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )

    # AI perception node
    ai_perception = Node(
        package='isaac_examples',
        executable='ai_perception_node',
        name='ai_perception_node',
        parameters=[
            {
                'use_hardware_acceleration': hardware_acceleration,
                'cuda_device_id': 0,
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )

    # Nav2 configurator node
    nav2_configurator = Node(
        package='isaac_examples',
        executable='nav2_configurator',
        name='nav2_configurator',
        parameters=[
            {
                'robot_type': 'humanoid',
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        # Start Isaac Sim bridge first
        isaac_sim_bridge,
        # Then perception nodes
        perception_pipeline,
        vslam_demo,
        ai_perception,
        # Finally navigation
        nav2_configurator
    ])