"""
LLM-based task planning interface for VLA systems
"""

import json
import asyncio
import time
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass
import re

try:
    from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM
    TRANSFORMERS_AVAILABLE = True
except ImportError:
    TRANSFORMERS_AVAILABLE = False
    print("Warning: transformers package not available. Using simulated interface.")


@dataclass
class PlanResult:
    """Result of LLM planning operation"""
    success: bool
    action_sequence: List[Dict[str, Any]]
    reasoning: str
    confidence: float
    execution_time: float
    error_message: Optional[str] = None
    raw_response: Optional[str] = None


@dataclass
class TaskPlan:
    """Structured task plan"""
    plan_id: str
    original_command: str
    action_sequence: List[Dict[str, Any]]
    reasoning: str
    constraints: List[str]
    safety_considerations: List[str]
    estimated_duration: float
    confidence: float


class LLMPlannerInterface:
    """
    Interface to LLM-based task planning for VLA systems
    """

    def __init__(self, model_name: str = "microsoft/DialoGPT-medium", device: str = "cpu"):
        """
        Initialize the LLM planner interface

        Args:
            model_name: Name of the LLM model to use
            device: Device to run model on ('cpu' or 'cuda')
        """
        self.model_name = model_name
        self.device = device
        self.model = None
        self.tokenizer = None
        self.is_initialized = False

        # Initialize model if available
        if TRANSFORMERS_AVAILABLE:
            try:
                print(f"Loading LLM model: {model_name}")
                self.tokenizer = AutoTokenizer.from_pretrained(model_name)
                self.model = AutoModelForCausalLM.from_pretrained(model_name)

                # Add padding token if it doesn't exist
                if self.tokenizer.pad_token is None:
                    self.tokenizer.pad_token = self.tokenizer.eos_token

                self.is_initialized = True
                print("LLM model loaded successfully")
            except Exception as e:
                print(f"Failed to load LLM model: {e}")
                self.is_initialized = False
        else:
            print("Transformers not available - using simulated interface")
            self.is_initialized = True  # For simulation purposes

        # Define available robot actions
        self.available_actions = [
            "navigate_to_location",
            "detect_object",
            "approach_object",
            "grasp_object",
            "lift_object",
            "transport_object",
            "place_object",
            "release_object",
            "turn_left",
            "turn_right",
            "move_forward",
            "move_backward",
            "stop",
            "wait",
            "speak_text",
            "take_image",
            "describe_scene"
        ]

    def generate_plan_prompt(self, command: str, context: Dict[str, Any] = None) -> str:
        """
        Generate a prompt for the LLM planner

        Args:
            command: Natural language command from user
            context: Environmental context information

        Returns:
            Formatted prompt for the LLM
        """
        context_str = json.dumps(context or {}, indent=2)

        prompt = f"""You are a robot task planner. Convert the following natural language command into a detailed sequence of executable actions.

Command: {command}

Environmental Context:
{context_str}

Available Actions: {', '.join(self.available_actions)}

Please generate a step-by-step plan that includes:
1. A sequence of specific actions to execute
2. Reasoning for each step
3. Safety considerations
4. Estimated time for each action

Respond in the following JSON format:
{{
    "action_sequence": [
        {{
            "action": "action_name",
            "parameters": {{"param1": "value1", "param2": "value2"}},
            "reasoning": "Why this action is needed",
            "estimated_duration": 2.5
        }}
    ],
    "reasoning": "Overall reasoning for the plan",
    "safety_considerations": ["consideration1", "consideration2"],
    "estimated_total_duration": 15.0,
    "confidence": 0.85
}}

Plan:"""

        return prompt

    def plan_task(self, command: str, context: Dict[str, Any] = None) -> PlanResult:
        """
        Plan a task using the LLM

        Args:
            command: Natural language command to plan
            context: Environmental context information

        Returns:
            PlanResult with action sequence and metadata
        """
        start_time = time.time()

        if TRANSFORMERS_AVAILABLE and self.is_initialized:
            try:
                # Generate the prompt
                prompt = self.generate_plan_prompt(command, context)

                # Tokenize the prompt
                inputs = self.tokenizer.encode(prompt, return_tensors="pt", truncation=True, max_length=1024)

                # Generate response
                with self.tokenizer.as_target_tokenizer():
                    outputs = self.model.generate(
                        inputs,
                        max_length=len(inputs[0]) + 256,
                        num_return_sequences=1,
                        temperature=0.7,
                        do_sample=True,
                        pad_token_id=self.tokenizer.eos_token_id
                    )

                # Decode the response
                response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)

                # Extract the plan part from the response
                plan_start = response.find('{')
                plan_end = response.rfind('}') + 1

                if plan_start != -1 and plan_end != 0:
                    plan_json_str = response[plan_start:plan_end]
                    try:
                        plan_data = json.loads(plan_json_str)

                        # Validate the plan structure
                        if 'action_sequence' in plan_data:
                            execution_time = time.time() - start_time
                            return PlanResult(
                                success=True,
                                action_sequence=plan_data['action_sequence'],
                                reasoning=plan_data.get('reasoning', ''),
                                confidence=plan_data.get('confidence', 0.7),
                                execution_time=execution_time,
                                raw_response=response
                            )
                        else:
                            execution_time = time.time() - start_time
                            return PlanResult(
                                success=False,
                                action_sequence=[],
                                reasoning="",
                                confidence=0.0,
                                execution_time=execution_time,
                                error_message="Invalid plan format returned by LLM",
                                raw_response=response
                            )
                    except json.JSONDecodeError as e:
                        execution_time = time.time() - start_time
                        return PlanResult(
                            success=False,
                            action_sequence=[],
                            reasoning="",
                            confidence=0.0,
                            execution_time=execution_time,
                            error_message=f"Failed to parse LLM response as JSON: {e}",
                            raw_response=response
                        )
                else:
                    execution_time = time.time() - start_time
                    return PlanResult(
                        success=False,
                        action_sequence=[],
                        reasoning="",
                        confidence=0.0,
                        execution_time=execution_time,
                        error_message="LLM did not return valid JSON plan",
                        raw_response=response
                    )

            except Exception as e:
                execution_time = time.time() - start_time
                return PlanResult(
                    success=False,
                    action_sequence=[],
                    reasoning="",
                    confidence=0.0,
                    execution_time=execution_time,
                    error_message=str(e)
                )
        else:
            # Simulated planning for educational purposes
            execution_time = time.time() - start_time
            return self._simulate_plan(command, context, execution_time)

    def _simulate_plan(self, command: str, context: Dict[str, Any], execution_time: float) -> PlanResult:
        """
        Simulate planning when LLM is not available

        Args:
            command: Command to simulate planning for
            context: Environmental context
            execution_time: Time taken for simulation

        Returns:
            Simulated PlanResult
        """
        command_lower = command.lower()

        # Generate simulated action sequence based on command
        action_sequence = []

        if any(word in command_lower for word in ["move", "go", "navigate"]):
            action_sequence = [
                {"action": "navigate_to_location", "parameters": {"target_location": "unknown"}, "reasoning": "Moving to target location", "estimated_duration": 3.0},
                {"action": "arrive_at_destination", "parameters": {}, "reasoning": "Arrived at destination", "estimated_duration": 0.5}
            ]
        elif any(word in command_lower for word in ["pick", "grasp", "take"]):
            action_sequence = [
                {"action": "detect_object", "parameters": {"object_type": "unknown"}, "reasoning": "Detecting target object", "estimated_duration": 2.0},
                {"action": "approach_object", "parameters": {"object_id": "unknown"}, "reasoning": "Approaching target object", "estimated_duration": 2.5},
                {"action": "grasp_object", "parameters": {"object_id": "unknown"}, "reasoning": "Grasping target object", "estimated_duration": 1.5}
            ]
        elif any(word in command_lower for word in ["place", "put", "set"]):
            action_sequence = [
                {"action": "navigate_to_location", "parameters": {"target_location": "placement_area"}, "reasoning": "Moving to placement location", "estimated_duration": 3.0},
                {"action": "place_object", "parameters": {"object_id": "held_object"}, "reasoning": "Placing object at destination", "estimated_duration": 1.5}
            ]
        elif any(word in command_lower for word in ["stop", "wait", "pause"]):
            action_sequence = [
                {"action": "stop", "parameters": {}, "reasoning": "Stopping current actions", "estimated_duration": 0.5},
                {"action": "wait", "parameters": {"duration": 1.0}, "reasoning": "Waiting as requested", "estimated_duration": 1.0}
            ]
        else:
            action_sequence = [
                {"action": "interpret_command", "parameters": {"command": command}, "reasoning": "Interpreting user command", "estimated_duration": 1.0},
                {"action": "plan_action", "parameters": {}, "reasoning": "Planning appropriate action", "estimated_duration": 1.5}
            ]

        return PlanResult(
            success=True,
            action_sequence=action_sequence,
            reasoning=f"Simulated plan for command: {command}",
            confidence=0.75,
            execution_time=execution_time,
            raw_response=f"Simulated response for: {command}"
        )

    def validate_plan(self, plan_result: PlanResult) -> Tuple[bool, List[str]]:
        """
        Validate a generated plan for safety and feasibility

        Args:
            plan_result: Plan to validate

        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []

        # Check if plan is empty
        if not plan_result.action_sequence:
            issues.append("Plan contains no actions")
            return False, issues

        # Check each action in the sequence
        for i, action in enumerate(plan_result.action_sequence):
            if not isinstance(action, dict):
                issues.append(f"Action at index {i} is not a valid dictionary")
                continue

            if 'action' not in action:
                issues.append(f"Action at index {i} missing 'action' field")
                continue

            action_name = action['action']
            if action_name not in self.available_actions:
                issues.append(f"Action '{action_name}' at index {i} is not supported")

            if 'reasoning' not in action:
                issues.append(f"Action at index {i} missing 'reasoning' field")

        # Check confidence threshold
        if plan_result.confidence < 0.5:
            issues.append(f"Plan confidence too low: {plan_result.confidence}")

        is_valid = len(issues) == 0
        return is_valid, issues

    def create_task_plan(self, command: str, context: Dict[str, Any] = None) -> Optional[TaskPlan]:
        """
        Create a structured task plan from a command

        Args:
            command: Natural language command
            context: Environmental context

        Returns:
            TaskPlan object or None if planning failed
        """
        plan_result = self.plan_task(command, context)

        if not plan_result.success:
            return None

        is_valid, issues = self.validate_plan(plan_result)
        if not is_valid:
            print(f"Plan validation failed: {issues}")
            return None

        # Calculate estimated duration
        estimated_duration = sum(
            action.get('estimated_duration', 1.0) for action in plan_result.action_sequence
        )

        return TaskPlan(
            plan_id=f"plan_{int(time.time())}",
            original_command=command,
            action_sequence=plan_result.action_sequence,
            reasoning=plan_result.reasoning,
            constraints=[],
            safety_considerations=["Check for obstacles", "Verify object stability"],
            estimated_duration=estimated_duration,
            confidence=plan_result.confidence
        )

    def get_available_actions(self) -> List[str]:
        """
        Get list of available robot actions

        Returns:
            List of action names
        """
        return self.available_actions.copy()

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the loaded LLM model

        Returns:
            Dictionary with model information
        """
        if self.is_initialized and TRANSFORMERS_AVAILABLE:
            return {
                "model_name": self.model_name,
                "device": self.device,
                "available": True,
                "actions": self.available_actions
            }
        else:
            return {
                "model_name": self.model_name,
                "device": self.device,
                "available": False,
                "actions": self.available_actions,
                "simulated": True
            }

    async def plan_task_async(self, command: str, context: Dict[str, Any] = None) -> PlanResult:
        """
        Asynchronously plan a task

        Args:
            command: Natural language command
            context: Environmental context

        Returns:
            PlanResult with action sequence and metadata
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.plan_task, command, context)


# Import the impossible request handler
from .impossible_request_handler import ImpossibleRequestHandler


class EnhancedLLMPlanner(LLMPlannerInterface):
    """
    Enhanced planner with additional features like chain-of-thought reasoning and impossible request handling
    """

    def __init__(self, model_name: str = "microsoft/DialoGPT-medium", device: str = "cpu"):
        super().__init__(model_name, device)

        # Initialize impossible request handler
        self.impossible_request_handler = ImpossibleRequestHandler()

        # Additional context information
        self.robot_capabilities = {
            "navigation": {"max_speed": 1.0, "precision": 0.1},
            "manipulation": {"max_payload": 2.0, "reach": 1.2},
            "perception": {"detection_range": 3.0, "accuracy": 0.95}
        }

    def generate_cot_prompt(self, command: str, context: Dict[str, Any] = None) -> str:
        """
        Generate a prompt that encourages chain-of-thought reasoning

        Args:
            command: Natural language command
            context: Environmental context

        Returns:
            Chain-of-thought prompt
        """
        context_str = json.dumps(context or {}, indent=2)

        prompt = f"""You are a robot task planner. Think step-by-step to convert the following command into a detailed plan.

Command: {command}

Environmental Context:
{context_str}

Robot Capabilities:
{json.dumps(self.robot_capabilities, indent=2)}

Available Actions: {', '.join(self.available_actions)}

Let's think step-by-step:
1. What does the user want to accomplish?
2. What objects and locations are involved?
3. What sequence of actions is needed?
4. What safety considerations apply?
5. How should the plan be structured?

Then generate the plan in JSON format:
{{
    "thought_process": [
        {{"step": 1, "thought": "analysis of command"}},
        {{"step": 2, "thought": "identification of objects/locations"}},
        {{"step": 3, "thought": "action sequence design"}},
        {{"step": 4, "thought": "safety considerations"}}
    ],
    "action_sequence": [
        {{
            "action": "action_name",
            "parameters": {{"param1": "value1"}},
            "reasoning": "reason for this action",
            "estimated_duration": 2.5
        }}
    ],
    "reasoning": "Overall reasoning for the plan",
    "safety_considerations": ["consideration1"],
    "estimated_total_duration": 15.0,
    "confidence": 0.85
}}

Chain-of-thought and Plan:"""

        return prompt

    def plan_with_cot(self, command: str, context: Dict[str, Any] = None) -> PlanResult:
        """
        Plan using chain-of-thought reasoning

        Args:
            command: Natural language command
            context: Environmental context

        Returns:
            PlanResult with action sequence and reasoning
        """
        start_time = time.time()

        if TRANSFORMERS_AVAILABLE and self.is_initialized:
            try:
                # Generate the chain-of-thought prompt
                prompt = self.generate_cot_prompt(command, context)

                # Tokenize the prompt
                inputs = self.tokenizer.encode(prompt, return_tensors="pt", truncation=True, max_length=1024)

                # Generate response
                with self.tokenizer.as_target_tokenizer():
                    outputs = self.model.generate(
                        inputs,
                        max_length=len(inputs[0]) + 512,  # Longer for CoT
                        num_return_sequences=1,
                        temperature=0.7,
                        do_sample=True,
                        pad_token_id=self.tokenizer.eos_token_id
                    )

                # Decode the response
                response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)

                # Extract the plan part from the response
                plan_start = response.find('{')
                plan_end = response.rfind('}') + 1

                if plan_start != -1 and plan_end != 0:
                    plan_json_str = response[plan_start:plan_end]
                    try:
                        plan_data = json.loads(plan_json_str)

                        # Validate the plan structure
                        if 'action_sequence' in plan_data:
                            execution_time = time.time() - start_time
                            return PlanResult(
                                success=True,
                                action_sequence=plan_data['action_sequence'],
                                reasoning=plan_data.get('reasoning', ''),
                                confidence=plan_data.get('confidence', 0.7),
                                execution_time=execution_time,
                                raw_response=response
                            )
                        else:
                            execution_time = time.time() - start_time
                            return PlanResult(
                                success=False,
                                action_sequence=[],
                                reasoning="",
                                confidence=0.0,
                                execution_time=execution_time,
                                error_message="Invalid plan format returned by LLM",
                                raw_response=response
                            )
                    except json.JSONDecodeError as e:
                        execution_time = time.time() - start_time
                        return PlanResult(
                            success=False,
                            action_sequence=[],
                            reasoning="",
                            confidence=0.0,
                            execution_time=execution_time,
                            error_message=f"Failed to parse LLM response as JSON: {e}",
                            raw_response=response
                        )
                else:
                    execution_time = time.time() - start_time
                    return PlanResult(
                        success=False,
                        action_sequence=[],
                        reasoning="",
                        confidence=0.0,
                        execution_time=execution_time,
                        error_message="LLM did not return valid JSON plan",
                        raw_response=response
                    )

            except Exception as e:
                execution_time = time.time() - start_time
                return PlanResult(
                    success=False,
                    action_sequence=[],
                    reasoning="",
                    confidence=0.0,
                    execution_time=execution_time,
                    error_message=str(e)
                )
        else:
            # Use base class simulation
            return self._simulate_plan(command, context, time.time() - start_time)

    def check_impossible_request(self, command: str, context: Dict[str, Any] = None) -> Tuple[bool, str, List[str]]:
        """
        Check if a command represents an impossible request before planning

        Args:
            command: Natural language command to check
            context: Environmental context information

        Returns:
            Tuple of (is_impossible, reason, alternatives)
        """
        return self.impossible_request_handler.handle_impossible_request(command, context)


# Example usage and testing
def main():
    """
    Example usage of the LLM planner
    """
    print("Testing LLM Planner Interface...")

    # Initialize planner
    planner = EnhancedLLMPlanner()

    # Check if LLM is available
    model_info = planner.get_model_info()
    print(f"Model info: {model_info}")

    # Test planning
    test_commands = [
        "Move to the kitchen",
        "Pick up the red cup from the table",
        "Go to the living room and stop",
        "Bring me the book"
    ]

    for command in test_commands:
        print(f"\nPlanning command: '{command}'")

        # Plan with regular method
        result = planner.plan_task(command)
        print(f"  Success: {result.success}")
        print(f"  Confidence: {result.confidence:.2f}")
        print(f"  Actions: {len(result.action_sequence)} steps")
        print(f"  Execution time: {result.execution_time:.2f}s")

        if result.success:
            # Validate the plan
            is_valid, issues = planner.validate_plan(result)
            print(f"  Valid: {is_valid}")
            if issues:
                print(f"  Issues: {issues}")

            # Create structured task plan
            task_plan = planner.create_task_plan(command)
            if task_plan:
                print(f"  Plan ID: {task_plan.plan_id}")
                print(f"  Estimated duration: {task_plan.estimated_duration:.2f}s")

    print("\nLLM planner test completed!")


if __name__ == "__main__":
    main()