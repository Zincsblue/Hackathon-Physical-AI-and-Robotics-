from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Launch file for visualizing the digital twin robot URDF in RViz.
    This demonstrates how to load and visualize a robot model in RViz.
    """
    # Get the URDF file path
    urdf_file_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'urdf',
        'digital_twin_robot.urdf'
    )

    # Read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Launch robot state publisher with the URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'publish_frequency': 50.0
            }],
            output='screen'
        ),

        # Launch joint state publisher (for manual joint control)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen'
        )
    ])