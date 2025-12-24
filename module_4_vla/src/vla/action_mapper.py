"""
Action Mapper Interface for VLA Systems

This module maps high-level plans to ROS 2 actions, bridging the gap between
symbolic planning and low-level robot control.
"""

from typing import Dict, List, Any, Optional, Tuple, Callable
import time
from dataclasses import dataclass
import json


@dataclass
class ActionMappingResult:
    """Result of action mapping operation"""
    success: bool
    ros2_actions: List[Dict[str, Any]]  # List of ROS 2 action calls
    mapping_time: float
    error_message: Optional[str] = None
    action_sequence: Optional[List[str]] = None


@dataclass
class PlanTranslationResult:
    """Result of plan-to-action translation"""
    success: bool
    translated_actions: List[Dict[str, Any]]
    confidence: float
    execution_time: float
    warnings: List[str]


class ActionMapperInterface:
    """
    Interface for mapping high-level plans to ROS 2 actions
    """

    def __init__(self):
        """Initialize the action mapper"""
        self.is_initialized = True

        # Define mapping from high-level VLA actions to ROS 2 action names
        self.action_mappings = {
            # Navigation actions
            "navigate_to_location": {
                "ros2_action": "/navigate_to_pose",
                "action_type": "nav2_msgs/action/NavigateToPose",
                "parameter_mapping": {
                    "location": "pose/pose/position",
                    "orientation": "pose/pose/orientation"
                }
            },
            "move_forward": {
                "ros2_action": "/local_cmd_vel",
                "action_type": "geometry_msgs/action/Twist",
                "parameter_mapping": {
                    "distance": "linear/x",
                    "speed": "linear/x"
                }
            },
            "move_backward": {
                "ros2_action": "/local_cmd_vel",
                "action_type": "geometry_msgs/action/Twist",
                "parameter_mapping": {
                    "distance": "linear/x",
                    "speed": "linear/x"
                }
            },
            "turn_left": {
                "ros2_action": "/local_cmd_vel",
                "action_type": "geometry_msgs/action/Twist",
                "parameter_mapping": {
                    "angle": "angular/z",
                    "speed": "angular/z"
                }
            },
            "turn_right": {
                "ros2_action": "/local_cmd_vel",
                "action_type": "geometry_msgs/action/Twist",
                "parameter_mapping": {
                    "angle": "angular/z",
                    "speed": "angular/z"
                }
            },
            "stop": {
                "ros2_action": "/local_cmd_vel",
                "action_type": "geometry_msgs/action/Twist",
                "parameter_mapping": {
                    "linear_velocity": "linear/x",
                    "angular_velocity": "angular/z"
                }
            },

            # Manipulation actions
            "detect_object": {
                "ros2_action": "/detect_objects",
                "action_type": "vision_msgs/action/DetectObjects",
                "parameter_mapping": {
                    "object_type": "object_type",
                    "search_area": "roi"
                }
            },
            "approach_object": {
                "ros2_action": "/move_group",
                "action_type": "moveit_msgs/action/MoveGroup",
                "parameter_mapping": {
                    "object_position": "target_pose",
                    "approach_vector": "approach_direction"
                }
            },
            "grasp_object": {
                "ros2_action": "/gripper_cmd",
                "action_type": "control_msgs/action/GripperCommand",
                "parameter_mapping": {
                    "object_id": "object_id",
                    "force": "command/position"
                }
            },
            "lift_object": {
                "ros2_action": "/move_group",
                "action_type": "moveit_msgs/action/MoveGroup",
                "parameter_mapping": {
                    "height": "target_pose/position/z",
                    "object_id": "attached_object"
                }
            },
            "transport_object": {
                "ros2_action": "/navigate_to_pose",
                "action_type": "nav2_msgs/action/NavigateToPose",
                "parameter_mapping": {
                    "destination": "pose/pose/position",
                    "carrying": "attached_objects"
                }
            },
            "place_object": {
                "ros2_action": "/move_group",
                "action_type": "moveit_msgs/action/MoveGroup",
                "parameter_mapping": {
                    "destination": "target_pose",
                    "release": "detach_object"
                }
            },
            "release_object": {
                "ros2_action": "/gripper_cmd",
                "action_type": "control_msgs/action/GripperCommand",
                "parameter_mapping": {
                    "object_id": "object_id",
                    "position": "command/position"
                }
            },

            # General actions
            "wait": {
                "ros2_action": "/delay_action",
                "action_type": "std_msgs/action/Duration",
                "parameter_mapping": {
                    "duration": "data"
                }
            },
            "speak_text": {
                "ros2_action": "/tts_action",
                "action_type": "sound_msgs/action/Speak",
                "parameter_mapping": {
                    "text": "text",
                    "voice": "voice_id"
                }
            },
            "take_image": {
                "ros2_action": "/image_capture",
                "action_type": "sensor_msgs/action/Image",
                "parameter_mapping": {
                    "camera_id": "camera_name",
                    "format": "encoding"
                }
            },
            "describe_scene": {
                "ros2_action": "/scene_description",
                "action_type": "vision_msgs/action/DescribeScene",
                "parameter_mapping": {
                    "context": "description_context"
                }
            }
        }

        # Define parameter validators
        self.parameter_validators = {
            "navigate_to_location": self._validate_navigation_params,
            "move_forward": self._validate_movement_params,
            "turn_left": self._validate_rotation_params,
            "grasp_object": self._validate_grasp_params,
        }

        # Action execution feedback handlers
        self.feedback_handlers = {}

    def map_action(self, vla_action: str, parameters: Dict[str, Any]) -> ActionMappingResult:
        """
        Map a high-level VLA action to ROS 2 action call

        Args:
            vla_action: High-level VLA action name
            parameters: Parameters for the action

        Returns:
            ActionMappingResult with ROS 2 action details
        """
        start_time = time.time()

        if vla_action not in self.action_mappings:
            return ActionMappingResult(
                success=False,
                ros2_actions=[],
                mapping_time=time.time() - start_time,
                error_message=f"Unknown VLA action: {vla_action}"
            )

        action_mapping = self.action_mappings[vla_action]
        ros2_action = action_mapping["ros2_action"]
        action_type = action_mapping["action_type"]
        param_mapping = action_mapping["parameter_mapping"]

        # Validate parameters if validator exists
        if vla_action in self.parameter_validators:
            validation_result = self.parameter_validators[vla_action](parameters)
            if not validation_result[0]:  # validation failed
                return ActionMappingResult(
                    success=False,
                    ros2_actions=[],
                    mapping_time=time.time() - start_time,
                    error_message=f"Parameter validation failed: {validation_result[1]}"
                )

        # Map parameters according to mapping specification
        ros2_params = self._map_parameters(parameters, param_mapping)

        ros2_action_call = {
            "action_name": ros2_action,
            "action_type": action_type,
            "parameters": ros2_params,
            "original_vla_action": vla_action,
            "original_parameters": parameters
        }

        return ActionMappingResult(
            success=True,
            ros2_actions=[ros2_action_call],
            mapping_time=time.time() - start_time,
            action_sequence=[vla_action]
        )

    def map_plan(self, vla_plan: List[Dict[str, Any]]) -> ActionMappingResult:
        """
        Map an entire VLA plan to sequence of ROS 2 actions

        Args:
            vla_plan: List of VLA actions with parameters

        Returns:
            ActionMappingResult with complete ROS 2 action sequence
        """
        start_time = time.time()
        all_ros2_actions = []
        action_sequence = []
        errors = []

        for i, plan_item in enumerate(vla_plan):
            if 'action' not in plan_item:
                errors.append(f"Plan item {i} missing 'action' field: {plan_item}")
                continue

            vla_action = plan_item['action']
            parameters = plan_item.get('parameters', {})

            result = self.map_action(vla_action, parameters)
            if result.success:
                all_ros2_actions.extend(result.ros2_actions)
                action_sequence.append(vla_action)
            else:
                errors.append(f"Failed to map action {vla_action}: {result.error_message}")

        if errors:
            return ActionMappingResult(
                success=False,
                ros2_actions=all_ros2_actions,
                mapping_time=time.time() - start_time,
                error_message="; ".join(errors),
                action_sequence=action_sequence
            )

        return ActionMappingResult(
            success=True,
            ros2_actions=all_ros2_actions,
            mapping_time=time.time() - start_time,
            action_sequence=action_sequence
        )

    def translate_plan_to_actions(self, high_level_plan: Dict[str, Any]) -> PlanTranslationResult:
        """
        Translate a high-level plan with reasoning to executable ROS 2 actions

        Args:
            high_level_plan: Plan with reasoning and action sequence

        Returns:
            PlanTranslationResult with translated actions
        """
        start_time = time.time()

        if 'action_sequence' not in high_level_plan:
            return PlanTranslationResult(
                success=False,
                translated_actions=[],
                confidence=0.0,
                execution_time=time.time() - start_time,
                warnings=["Plan missing action_sequence"]
            )

        action_sequence = high_level_plan['action_sequence']
        translated_actions = []

        # Process each action in the sequence
        for action_item in action_sequence:
            if isinstance(action_item, dict) and 'action' in action_item:
                vla_action = action_item['action']
                parameters = action_item.get('parameters', {})
                reasoning = action_item.get('reasoning', '')

                # Map the action
                mapping_result = self.map_action(vla_action, parameters)
                if mapping_result.success:
                    # Add reasoning information to the mapped action
                    for ros2_action in mapping_result.ros2_actions:
                        ros2_action['reasoning'] = reasoning
                        translated_actions.append(ros2_action)
                else:
                    return PlanTranslationResult(
                        success=False,
                        translated_actions=translated_actions,
                        confidence=0.0,
                        execution_time=time.time() - start_time,
                        warnings=[f"Failed to translate action {vla_action}: {mapping_result.error_message}"]
                    )
            else:
                return PlanTranslationResult(
                    success=False,
                    translated_actions=translated_actions,
                    confidence=0.0,
                    execution_time=time.time() - start_time,
                    warnings=[f"Invalid action format: {action_item}"]
                )

        confidence = high_level_plan.get('confidence', 0.8)  # Use plan's confidence or default
        execution_time = time.time() - start_time

        return PlanTranslationResult(
            success=True,
            translated_actions=translated_actions,
            confidence=confidence,
            execution_time=execution_time,
            warnings=[]
        )

    def _map_parameters(self, vla_params: Dict[str, Any], mapping: Dict[str, str]) -> Dict[str, Any]:
        """Map VLA parameters to ROS 2 parameters according to mapping specification"""
        ros2_params = {}

        for vla_param, ros2_path in mapping.items():
            if vla_param in vla_params:
                # This is a simplified mapping - in reality, you'd need more complex path resolution
                ros2_params[ros2_path] = vla_params[vla_param]

        return ros2_params

    def _validate_navigation_params(self, params: Dict[str, Any]) -> Tuple[bool, str]:
        """Validate navigation action parameters"""
        if 'location' not in params:
            return False, "Navigation action requires 'location' parameter"
        return True, ""

    def _validate_movement_params(self, params: Dict[str, Any]) -> Tuple[bool, str]:
        """Validate movement action parameters"""
        if 'distance' not in params and 'duration' not in params:
            return False, "Movement action requires 'distance' or 'duration' parameter"
        return True, ""

    def _validate_rotation_params(self, params: Dict[str, Any]) -> Tuple[bool, str]:
        """Validate rotation action parameters"""
        if 'angle' not in params:
            return False, "Rotation action requires 'angle' parameter"
        return True, ""

    def _validate_grasp_params(self, params: Dict[str, Any]) -> Tuple[bool, str]:
        """Validate grasp action parameters"""
        if 'object_id' not in params:
            return False, "Grasp action requires 'object_id' parameter"
        return True, ""

    def get_supported_actions(self) -> List[str]:
        """Get list of supported VLA actions"""
        return list(self.action_mappings.keys())

    def get_ros2_action_spec(self, vla_action: str) -> Optional[Dict[str, Any]]:
        """Get ROS 2 action specification for a VLA action"""
        return self.action_mappings.get(vla_action)

    def register_feedback_handler(self, action_type: str, handler: Callable):
        """Register a feedback handler for a specific action type"""
        self.feedback_handlers[action_type] = handler

    def get_action_mappings(self) -> Dict[str, Any]:
        """Get all action mappings"""
        return self.action_mappings.copy()


class EnhancedActionMapper(ActionMapperInterface):
    """
    Enhanced action mapper with additional features like feedback integration
    and error recovery
    """

    def __init__(self):
        super().__init__()

        # Plan adjustment capabilities
        self.plan_adjustment_strategies = {
            "retry": self._strategy_retry,
            "alternative": self._strategy_alternative,
            "skip": self._strategy_skip,
            "delegate": self._strategy_delegate
        }

        # Error handling patterns
        self.error_patterns = {
            "navigation_failure": ["obstacle", "unreachable", "collision"],
            "manipulation_failure": ["grasp_failure", "object_lost", "collision"],
            "perception_failure": ["not_found", "occluded", "uncertain"]
        }

    def integrate_feedback(self, action_result: Dict[str, Any], current_plan: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Integrate feedback from action execution to adjust the plan

        Args:
            action_result: Result from executing an action
            current_plan: Current plan being executed

        Returns:
            Adjusted plan based on feedback
        """
        adjusted_plan = current_plan.copy()

        # Check if the action failed
        if not action_result.get('success', True):
            failure_type = action_result.get('failure_type', 'unknown')
            failed_action_idx = action_result.get('action_index', -1)

            if failed_action_idx >= 0 and failed_action_idx < len(adjusted_plan):
                failed_action = adjusted_plan[failed_action_idx]

                # Determine adjustment strategy based on failure type
                strategy = self._determine_adjustment_strategy(failure_type, failed_action)
                if strategy:
                    adjusted_plan = strategy(adjusted_plan, failed_action_idx, action_result)

        return adjusted_plan

    def _determine_adjustment_strategy(self, failure_type: str, failed_action: Dict[str, Any]) -> Optional[Callable]:
        """Determine appropriate adjustment strategy based on failure type"""
        action_type = failed_action.get('action', '')

        # Navigation failures
        if failure_type in self.error_patterns['navigation_failure'] or 'navigate' in action_type:
            return self._strategy_alternative  # Try alternative route

        # Manipulation failures
        elif failure_type in self.error_patterns['manipulation_failure'] or 'grasp' in action_type:
            return self._strategy_retry  # Try again with different parameters

        # Perception failures
        elif failure_type in self.error_patterns['perception_failure'] or 'detect' in action_type:
            return self._strategy_retry  # Look again

        # Default: skip the action
        else:
            return self._strategy_skip

    def _strategy_retry(self, plan: List[Dict[str, Any]], action_idx: int, result: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Retry strategy: try the same action with modified parameters"""
        new_plan = plan.copy()
        original_action = new_plan[action_idx].copy()

        # Modify parameters for retry (this is simplified)
        retry_params = original_action.get('parameters', {}).copy()
        retry_params['retry_count'] = retry_params.get('retry_count', 0) + 1

        # Add small variation to parameters for retry
        if 'position' in retry_params:
            pos = retry_params['position']
            if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                # Add small random offset
                import random
                retry_params['position'] = [
                    pos[0] + random.uniform(-0.1, 0.1),
                    pos[1] + random.uniform(-0.1, 0.1)
                ] + list(pos[2:]) if len(pos) > 2 else []

        original_action['parameters'] = retry_params
        new_plan[action_idx] = original_action

        return new_plan

    def _strategy_alternative(self, plan: List[Dict[str, Any]], action_idx: int, result: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Alternative strategy: replace with alternative action or approach"""
        new_plan = plan.copy()
        original_action = new_plan[action_idx]

        # For navigation, try a different approach
        if 'navigate' in original_action.get('action', ''):
            alt_params = original_action.get('parameters', {}).copy()
            # Add alternative routing hint
            alt_params['routing_preference'] = 'avoid_obstacles'
            new_plan[action_idx]['parameters'] = alt_params

        # For manipulation, try alternative grasp
        elif 'grasp' in original_action.get('action', ''):
            alt_params = original_action.get('parameters', {}).copy()
            # Try different grasp approach
            alt_params['grasp_type'] = 'side_grasp' if alt_params.get('grasp_type') == 'top_grasp' else 'top_grasp'
            new_plan[action_idx]['parameters'] = alt_params

        return new_plan

    def _strategy_skip(self, plan: List[Dict[str, Any]], action_idx: int, result: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Skip strategy: remove the failed action from the plan"""
        new_plan = plan.copy()
        new_plan.pop(action_idx)
        return new_plan

    def _strategy_delegate(self, plan: List[Dict[str, Any]], action_idx: int, result: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Delegate strategy: replace with human assistance or simpler action"""
        new_plan = plan.copy()

        # Replace with request for human assistance
        new_plan[action_idx] = {
            'action': 'request_assistance',
            'parameters': {
                'reason': result.get('error_message', 'Action failed'),
                'original_action': plan[action_idx]
            },
            'reasoning': 'Action failed, requesting human assistance'
        }

        return new_plan

    def create_recovery_plan(self, failure_context: Dict[str, Any]) -> Optional[List[Dict[str, Any]]]:
        """
        Create a recovery plan for handling specific failures

        Args:
            failure_context: Context of the failure

        Returns:
            Recovery plan or None if no recovery is possible
        """
        failure_type = failure_context.get('failure_type', 'unknown')
        failed_action = failure_context.get('failed_action', {})

        recovery_actions = []

        if failure_type in self.error_patterns['navigation_failure']:
            # Recovery for navigation: check for alternative routes, relocalize
            recovery_actions = [
                {
                    'action': 'localize_robot',
                    'parameters': {},
                    'reasoning': 'Recovering from navigation failure by relocalizing'
                },
                {
                    'action': 'update_map',
                    'parameters': {'area': failed_action.get('parameters', {}).get('location')},
                    'reasoning': 'Updating map with new obstacle information'
                }
            ]

        elif failure_type in self.error_patterns['manipulation_failure']:
            # Recovery for manipulation: re-identify object, adjust grasp
            recovery_actions = [
                {
                    'action': 'reidentify_object',
                    'parameters': failed_action.get('parameters', {}),
                    'reasoning': 'Re-identifying object after manipulation failure'
                },
                {
                    'action': 'adjust_grasp_strategy',
                    'parameters': failed_action.get('parameters', {}),
                    'reasoning': 'Trying different grasp strategy'
                }
            ]

        elif failure_type in self.error_patterns['perception_failure']:
            # Recovery for perception: change viewpoint, adjust parameters
            recovery_actions = [
                {
                    'action': 'change_viewpoint',
                    'parameters': {'angle_offset': 45.0},  # degrees
                    'reasoning': 'Changing viewpoint to improve perception'
                },
                {
                    'action': 'adjust_sensor_parameters',
                    'parameters': {'sensitivity': 'high'},
                    'reasoning': 'Adjusting sensor parameters for better detection'
                }
            ]

        return recovery_actions if recovery_actions else None


# Example usage and testing
def main():
    """Example usage of the action mapper"""
    print("Testing Action Mapper Interface...")

    # Initialize action mapper
    mapper = EnhancedActionMapper()

    # Test single action mapping
    print("\n1. Testing single action mapping:")
    result = mapper.map_action("navigate_to_location", {"location": [1.0, 2.0, 0.0]})
    print(f"  Success: {result.success}")
    print(f"  ROS 2 action: {result.ros2_actions[0]['action_name'] if result.ros2_actions else 'None'}")
    print(f"  Mapping time: {result.mapping_time:.4f}s")

    # Test plan mapping
    print("\n2. Testing plan mapping:")
    vla_plan = [
        {"action": "detect_object", "parameters": {"object_type": "red cup"}},
        {"action": "navigate_to_location", "parameters": {"location": [1.5, 2.0, 0.0]}},
        {"action": "grasp_object", "parameters": {"object_id": "red_cup_1"}}
    ]

    plan_result = mapper.map_plan(vla_plan)
    print(f"  Success: {plan_result.success}")
    print(f"  Mapped {len(plan_result.ros2_actions)} ROS 2 actions")
    print(f"  Action sequence: {plan_result.action_sequence}")

    # Test plan translation with reasoning
    print("\n3. Testing plan translation with reasoning:")
    high_level_plan = {
        "action_sequence": [
            {
                "action": "navigate_to_location",
                "parameters": {"location": [1.0, 2.0, 0.0]},
                "reasoning": "Moving to object location"
            },
            {
                "action": "detect_object",
                "parameters": {"object_type": "cup"},
                "reasoning": "Locating target object"
            }
        ],
        "confidence": 0.85
    }

    translation_result = mapper.translate_plan_to_actions(high_level_plan)
    print(f"  Translation success: {translation_result.success}")
    print(f"  Translated actions: {len(translation_result.translated_actions)}")
    print(f"  Confidence: {translation_result.confidence}")

    # Test feedback integration
    print("\n4. Testing feedback integration:")
    current_plan = [
        {"action": "navigate_to_location", "parameters": {"location": [1.0, 2.0, 0.0]}},
        {"action": "grasp_object", "parameters": {"object_id": "item1"}}
    ]

    action_result = {
        "success": False,
        "failure_type": "navigation_failure",
        "action_index": 0,
        "error_message": "Path blocked by obstacle"
    }

    adjusted_plan = mapper.integrate_feedback(action_result, current_plan)
    print(f"  Original plan length: {len(current_plan)}")
    print(f"  Adjusted plan length: {len(adjusted_plan)}")

    # Test recovery plan creation
    print("\n5. Testing recovery plan creation:")
    failure_context = {
        "failure_type": "navigation_failure",
        "failed_action": {"action": "navigate_to_location", "parameters": {"location": [1.0, 2.0, 0.0]}}
    }

    recovery_plan = mapper.create_recovery_plan(failure_context)
    if recovery_plan:
        print(f"  Recovery plan created with {len(recovery_plan)} actions")
        for action in recovery_plan:
            print(f"    - {action['action']}: {action['reasoning']}")
    else:
        print("  No recovery plan available")

    # Show supported actions
    print("\n6. Supported VLA actions:")
    supported_actions = mapper.get_supported_actions()
    print(f"  Total: {len(supported_actions)} actions")
    for action in supported_actions[:10]:  # Show first 10
        print(f"    - {action}")

    print("\nAction mapper testing completed!")


if __name__ == "__main__":
    main()