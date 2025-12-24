from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for running the joint state publisher and subscriber together.
    This demonstrates how to launch multiple ROS 2 nodes in a single command.
    """
    return LaunchDescription([
        # Launch the joint state publisher node
        Node(
            package='my_robot_tutorial',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[]
        ),

        # Launch the joint state subscriber node
        Node(
            package='my_robot_tutorial',
            executable='joint_state_subscriber',
            name='joint_state_subscriber',
            output='screen',
            parameters=[]
        )
    ])