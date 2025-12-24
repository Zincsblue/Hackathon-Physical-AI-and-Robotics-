from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for running the digital twin system with Python agent and ROS 2 controller.
    This demonstrates how to launch multiple nodes that communicate for digital twin applications.
    """
    return LaunchDescription([
        # Launch the Python agent node
        Node(
            package='digital_twin_examples',
            executable='python_agent',
            name='python_agent',
            output='screen',
            parameters=[]
        ),

        # Launch the ROS 2 controller node
        Node(
            package='digital_twin_examples',
            executable='ros2_controller',
            name='ros2_controller',
            output='screen',
            parameters=[]
        )
    ])