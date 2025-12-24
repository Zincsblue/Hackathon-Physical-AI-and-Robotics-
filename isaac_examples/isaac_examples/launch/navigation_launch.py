#!/usr/bin/env python3

"""
Launch file for Isaac Nav2 Navigation

This launch file starts the Nav2 navigation stack configured for humanoid robots.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package and configuration paths
    pkg_share = get_package_share_directory('isaac_examples')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))

    # Nav2 configurator node
    nav2_configurator_node = Node(
        package='isaac_examples',
        executable='nav2_configurator',
        name='nav2_configurator_node',
        parameters=[
            {
                'robot_type': 'humanoid',
                'footprint_radius': 0.3,
                'step_height': 0.1,
                'max_step_width': 0.4,
                'bipedal_behavior_enabled': True,
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )

    # Nav2 lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }
        ]
    )

    # Nav2 components would be launched here in a real implementation
    # For this educational example, we'll just include the configurator
    return LaunchDescription([
        nav2_configurator_node,
        lifecycle_manager
    ])