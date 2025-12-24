from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for running the set joint service and client together.
    This demonstrates how to launch multiple ROS 2 nodes that communicate via services.
    """
    return LaunchDescription([
        # Launch the set joint service server
        Node(
            package='my_robot_tutorial',
            executable='set_joint_service',
            name='set_joint_service',
            output='screen',
            parameters=[]
        )
    ])