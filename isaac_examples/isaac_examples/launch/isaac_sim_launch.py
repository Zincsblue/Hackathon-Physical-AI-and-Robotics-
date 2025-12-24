#!/usr/bin/env python3

"""
Launch file for Isaac Sim Integration

This launch file starts the Isaac Sim integration components.
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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    sim_config_path = LaunchConfiguration('sim_config_path', default=os.path.join(pkg_share, 'config', 'isaac_sim_config.json'))

    # Isaac Sim integration node
    isaac_sim_integration = Node(
        package='isaac_examples',
        executable='isaac_sim_integration',
        name='isaac_sim_integration',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'simulation_config_path': sim_config_path,
                'simulation_mode': True
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        isaac_sim_integration
    ])