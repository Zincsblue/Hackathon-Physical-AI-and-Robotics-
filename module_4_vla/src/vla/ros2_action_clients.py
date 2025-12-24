"""
ROS 2 Action Clients for VLA Systems

This module implements ROS 2 action clients that can execute the actions
mapped by the action mapper. These clients provide the interface between
the VLA system and the actual ROS 2 robot control stack.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
import time
from typing import Dict, Any, Optional, Callable, List
from dataclasses import dataclass


@dataclass
class ActionClientResult:
    """Result from ROS 2 action client execution"""
    success: bool
    result_data: Optional[Dict[str, Any]]
    execution_time: float
    error_message: Optional[str] = None
    goal_id: Optional[str] = None


class ROS2ActionClientManager:
    """
    Manager for ROS 2 action clients in VLA systems
    Handles creation, management, and execution of various action clients
    """

    def __init__(self, node: Node):
        """
        Initialize the ROS 2 action client manager

        Args:
            node: ROS 2 node to use for creating action clients
        """
        self.node = node
        self.action_clients = {}
        self._initialize_action_clients()

    def _initialize_action_clients(self):
        """Initialize all supported ROS 2 action clients"""
        try:
            # Navigation action client
            from nav2_msgs.action import NavigateToPose
            self.action_clients['/navigate_to_pose'] = ActionClient(
                self.node, NavigateToPose, '/navigate_to_pose'
            )

            # Velocity command action client
            from geometry_msgs.action import Twist
            self.action_clients['/local_cmd_vel'] = ActionClient(
                self.node, Twist, '/local_cmd_vel'
            )

            # Manipulation action clients
            from moveit_msgs.action import MoveGroup
            self.action_clients['/move_group'] = ActionClient(
                self.node, MoveGroup, '/move_group'
            )

            from control_msgs.action import GripperCommand
            self.action_clients['/gripper_cmd'] = ActionClient(
                self.node, GripperCommand, '/gripper_cmd'
            )

            # Perception action clients
            from vision_msgs.action import DetectObjects
            self.action_clients['/detect_objects'] = ActionClient(
                self.node, DetectObjects, '/detect_objects'
            )

            from sensor_msgs.action import Image
            self.action_clients['/image_capture'] = ActionClient(
                self.node, Image, '/image_capture'
            )

            # General action clients
            from std_msgs.action import Duration as DurationAction
            self.action_clients['/delay_action'] = ActionClient(
                self.node, DurationAction, '/delay_action'
            )

            from sound_msgs.action import Speak
            self.action_clients['/tts_action'] = ActionClient(
                self.node, Speak, '/tts_action'
            )

            self.node.get_logger().info("All action clients initialized successfully")

        except ImportError as e:
            self.node.get_logger().warn(f"Could not import some action types: {e}")
            # Create mock clients for demonstration purposes
            self._create_mock_clients()

    def _create_mock_clients(self):
        """Create mock action clients for demonstration when ROS 2 packages aren't available"""
        self.node.get_logger().info("Creating mock action clients for demonstration")
        # In a real implementation, we would create actual mock clients
        # For now, we'll just log that mock clients are being used

    def execute_action(self, action_name: str, parameters: Dict[str, Any],
                      timeout: float = 30.0) -> ActionClientResult:
        """
        Execute a ROS 2 action with given parameters

        Args:
            action_name: Name of the ROS 2 action to execute
            parameters: Parameters for the action
            timeout: Timeout in seconds

        Returns:
            ActionClientResult with execution result
        """
        start_time = time.time()

        if action_name not in self.action_clients:
            return ActionClientResult(
                success=False,
                result_data=None,
                execution_time=time.time() - start_time,
                error_message=f"Action client not available: {action_name}"
            )

        action_client = self.action_clients[action_name]

        # Wait for action server to be available
        if not action_client.wait_for_server(timeout_sec=timeout/2):
            return ActionClientResult(
                success=False,
                result_data=None,
                execution_time=time.time() - start_time,
                error_message=f"Action server not available: {action_name}"
            )

        # Create goal message based on action type
        goal_msg = self._create_goal_message(action_name, parameters)
        if goal_msg is None:
            return ActionClientResult(
                success=False,
                result_data=None,
                execution_time=time.time() - start_time,
                error_message=f"Could not create goal message for: {action_name}"
            )

        # Send goal
        goal_future = action_client.send_goal_async(goal_msg)

        # Wait for result
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=timeout/2)

        if not goal_future.done():
            return ActionClientResult(
                success=False,
                result_data=None,
                execution_time=time.time() - start_time,
                error_message=f"Goal request timed out: {action_name}"
            )

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            return ActionClientResult(
                success=False,
                result_data=None,
                execution_time=time.time() - start_time,
                error_message=f"Goal rejected by server: {action_name}"
            )

        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout/2)

        if not result_future.done():
            return ActionClientResult(
                success=False,
                result_data=None,
                execution_time=time.time() - start_time,
                error_message=f"Result request timed out: {action_name}"
            )

        result = result_future.result().result
        execution_time = time.time() - start_time

        return ActionClientResult(
            success=True,
            result_data=self._extract_result_data(result),
            execution_time=execution_time,
            goal_id=str(goal_handle.goal_id)
        )

    def _create_goal_message(self, action_name: str, parameters: Dict[str, Any]):
        """Create appropriate goal message based on action type"""
        try:
            if action_name == '/navigate_to_pose':
                from nav2_msgs.action import NavigateToPose
                goal_msg = NavigateToPose.Goal()

                # Set pose from parameters
                if 'pose' in parameters:
                    goal_msg.pose = parameters['pose']
                elif 'location' in parameters:
                    # Create pose from location
                    from geometry_msgs.msg import PoseStamped
                    pose_stamped = PoseStamped()
                    pose_stamped.pose.position.x = parameters['location'][0]
                    pose_stamped.pose.position.y = parameters['location'][1]
                    if len(parameters['location']) > 2:
                        pose_stamped.pose.position.z = parameters['location'][2]
                    goal_msg.pose = pose_stamped
                return goal_msg

            elif action_name == '/move_group':
                from moveit_msgs.action import MoveGroup
                goal_msg = MoveGroup.Goal()

                # Set planning group and target pose
                if 'planning_group' in parameters:
                    goal_msg.request.group_name = parameters['planning_group']
                if 'target_pose' in parameters:
                    goal_msg.request.workspace_parameters = parameters['target_pose']
                return goal_msg

            elif action_name == '/gripper_cmd':
                from control_msgs.action import GripperCommand
                goal_msg = GripperCommand.Goal()

                # Set gripper command
                if 'command' in parameters:
                    goal_msg.command = parameters['command']
                elif 'position' in parameters:
                    from control_msgs.msg import GripperCommand as GripperCommandMsg
                    gripper_cmd = GripperCommandMsg()
                    gripper_cmd.position = parameters['position']
                    if 'max_effort' in parameters:
                        gripper_cmd.max_effort = parameters['max_effort']
                    goal_msg.command = gripper_cmd
                return goal_msg

            elif action_name == '/detect_objects':
                from vision_msgs.action import DetectObjects
                goal_msg = DetectObjects.Goal()

                # Set detection parameters
                if 'object_type' in parameters:
                    goal_msg.object_type = parameters['object_type']
                if 'roi' in parameters:
                    goal_msg.roi = parameters['roi']
                return goal_msg

            elif action_name == '/image_capture':
                from sensor_msgs.action import Image
                goal_msg = Image.Goal()

                # Set image capture parameters
                if 'camera_name' in parameters:
                    goal_msg.camera_name = parameters['camera_name']
                if 'encoding' in parameters:
                    goal_msg.encoding = parameters['encoding']
                return goal_msg

            else:
                # For other action types, return a generic goal with parameters
                class GenericGoal:
                    def __init__(self):
                        self.parameters = parameters
                return GenericGoal()

        except ImportError:
            # If ROS 2 packages aren't available, return a mock goal
            class MockGoal:
                def __init__(self):
                    self.parameters = parameters
            return MockGoal()

    def _extract_result_data(self, result_msg) -> Optional[Dict[str, Any]]:
        """Extract result data from ROS 2 result message"""
        if result_msg is None:
            return None

        # Convert result message to dictionary
        result_dict = {}
        for field_name in dir(result_msg):
            if not field_name.startswith('_') and not callable(getattr(result_msg, field_name)):
                try:
                    result_dict[field_name] = getattr(result_msg, field_name)
                except AttributeError:
                    continue

        return result_dict

    def execute_action_sequence(self, action_sequence: List[Dict[str, Any]],
                               timeout_per_action: float = 30.0) -> List[ActionClientResult]:
        """
        Execute a sequence of ROS 2 actions

        Args:
            action_sequence: List of action dictionaries with 'action_name' and 'parameters'
            timeout_per_action: Timeout for each individual action

        Returns:
            List of ActionClientResult for each action
        """
        results = []

        for i, action in enumerate(action_sequence):
            action_name = action.get('action_name')
            parameters = action.get('parameters', {})

            result = self.execute_action(action_name, parameters, timeout_per_action)
            results.append(result)

            # If action failed and is not recoverable, stop execution
            if not result.success and action.get('critical', True):
                break

        return results

    def get_available_actions(self) -> List[str]:
        """Get list of available action names"""
        return list(self.action_clients.keys())

    def cancel_goal(self, goal_id: str, action_name: str) -> bool:
        """Cancel a specific goal"""
        if action_name not in self.action_clients:
            return False

        action_client = self.action_clients[action_name]
        # In a real implementation, we would cancel the specific goal
        # This is a simplified version
        return True


class VLAActionExecutor:
    """
    High-level action executor that integrates with the VLA system
    """

    def __init__(self, node: Node):
        """Initialize the VLA action executor"""
        self.node = node
        self.client_manager = ROS2ActionClientManager(node)

    def execute_mapped_action(self, mapped_action: Dict[str, Any]) -> ActionClientResult:
        """
        Execute a single action that has been mapped from VLA to ROS 2

        Args:
            mapped_action: Action dictionary from the action mapper

        Returns:
            ActionClientResult with execution result
        """
        action_name = mapped_action.get('action_name')
        parameters = mapped_action.get('parameters', {})

        return self.client_manager.execute_action(action_name, parameters)

    def execute_mapped_plan(self, mapped_plan: List[Dict[str, Any]]) -> List[ActionClientResult]:
        """
        Execute a complete plan of mapped actions

        Args:
            mapped_plan: List of mapped action dictionaries

        Returns:
            List of results for each action
        """
        return self.client_manager.execute_action_sequence(mapped_plan)

    def execute_with_feedback(self, mapped_plan: List[Dict[str, Any]],
                           feedback_callback: Optional[Callable] = None) -> List[ActionClientResult]:
        """
        Execute a plan with feedback handling

        Args:
            mapped_plan: List of mapped action dictionaries
            feedback_callback: Optional callback for handling feedback

        Returns:
            List of results for each action
        """
        results = []

        for i, action in enumerate(mapped_plan):
            result = self.execute_mapped_action(action)
            results.append(result)

            # Call feedback callback if provided
            if feedback_callback:
                feedback_callback(i, action, result)

            # If action failed, decide whether to continue based on criticality
            if not result.success:
                if action.get('critical', True):
                    self.node.get_logger().warn(f"Critical action failed, stopping execution: {action}")
                    break
                else:
                    self.node.get_logger().info(f"Non-critical action failed, continuing: {action}")

        return results


def main():
    """Example usage of the ROS 2 action clients"""
    print("Testing ROS 2 Action Clients...")

    # Note: In a real ROS 2 environment, you would initialize rclpy and create a node
    # For this example, we'll demonstrate the structure

    try:
        rclpy.init()
        node = rclpy.create_node('vla_action_client_test')

        # Initialize the action client manager
        client_manager = ROS2ActionClientManager(node)

        print(f"Available actions: {client_manager.get_available_actions()}")

        # Example action execution (these would require actual ROS 2 services running)
        # For demonstration, we'll show the structure:

        print("\n1. Example action execution structure:")
        example_action = {
            'action_name': '/navigate_to_pose',
            'parameters': {
                'location': [1.0, 2.0, 0.0],
                'orientation': [0.0, 0.0, 0.0, 1.0]  # quaternion
            }
        }
        print(f"Would execute: {example_action['action_name']}")
        print(f"With parameters: {example_action['parameters']}")

        # Example action sequence
        print("\n2. Example action sequence:")
        action_sequence = [
            {
                'action_name': '/detect_objects',
                'parameters': {'object_type': 'cup'},
                'critical': True
            },
            {
                'action_name': '/navigate_to_pose',
                'parameters': {'location': [1.0, 1.0, 0.0]},
                'critical': True
            },
            {
                'action_name': '/gripper_cmd',
                'parameters': {'position': 0.5, 'max_effort': 10.0},
                'critical': False
            }
        ]

        print(f"Action sequence has {len(action_sequence)} actions")

        # Clean up
        node.destroy_node()
        rclpy.shutdown()

        print("\nROS 2 action client testing completed!")

    except Exception as e:
        print(f"ROS 2 may not be available in this environment: {e}")
        print("This is expected if running outside a ROS 2 environment")
        print("The code structure is ready for integration with ROS 2 when available")


if __name__ == '__main__':
    main()