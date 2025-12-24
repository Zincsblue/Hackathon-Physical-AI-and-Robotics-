from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    Comprehensive launch file for the full digital twin system.
    This launches all components needed for a complete digital twin demonstration:
    - Python agent for high-level decision making
    - ROS 2 controller for low-level execution
    - Digital twin bridge for synchronization
    - Robot state publisher for visualization
    - Joint state publisher for manual control
    - RViz for visualization
    """

    # Get the package directory
    pkg_dir = os.path.dirname(os.path.dirname(__file__))

    # Get URDF file path
    urdf_file_path = os.path.join(pkg_dir, 'urdf', 'digital_twin_robot.urdf')

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
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Launch the Python agent
        Node(
            package='digital_twin_examples',
            executable='python_agent',
            name='python_agent',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),

        # Launch the ROS 2 controller
        Node(
            package='digital_twin_examples',
            executable='ros2_controller',
            name='ros2_controller',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),

        # Launch the digital twin bridge
        Node(
            package='digital_twin_examples',
            executable='digital_twin_bridge',
            name='digital_twin_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),

        # Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen'
        ),

        # Launch URDF analyzer for validation
        Node(
            package='digital_twin_examples',
            executable='urdf_analyzer',
            name='urdf_analyzer',
            output='screen'
        )
    ])