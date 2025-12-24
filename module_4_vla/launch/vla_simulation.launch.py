import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),

        # VLA Command Node
        Node(
            package='module_4_vla',
            executable='vla_command_node',
            name='vla_command_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Whisper Interface Node
        Node(
            package='module_4_vla',
            executable='whisper_interface_node',
            name='whisper_interface_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # LLM Planner Node
        Node(
            package='module_4_vla',
            executable='llm_planner_node',
            name='llm_planner_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Vision Processor Node
        Node(
            package='module_4_vla',
            executable='vision_processor_node',
            name='vision_processor_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Action Mapper Node
        Node(
            package='module_4_vla',
            executable='action_mapper_node',
            name='action_mapper_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        )
    ])