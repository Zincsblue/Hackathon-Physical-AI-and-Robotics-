"""
Hierarchical Task Network (HTN) Implementation for VLA Systems

This module implements a hierarchical task network for decomposing high-level
commands into executable actions in VLA systems.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional, Callable
import json
import time


@dataclass
class Task:
    """Represents a task in the hierarchical task network"""
    name: str
    method: str  # 'primitive' for basic actions, 'compound' for composed tasks
    parameters: Dict[str, Any]
    subtasks: Optional[List['Task']] = None
    status: str = 'pending'  # pending, executing, completed, failed
    estimated_duration: float = 0.0


@dataclass
class HTNPlan:
    """Represents a hierarchical task network plan"""
    plan_id: str
    root_task: Task
    decomposition_depth: int = 0
    created_at: float = 0.0


class HierarchicalTaskNetwork:
    """
    Hierarchical Task Network for VLA systems
    Decomposes high-level commands into executable primitive actions
    """

    def __init__(self):
        """Initialize the HTN planner"""
        self.task_methods = {}
        self.primitive_actions = [
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

        # Register default task decomposition methods
        self._register_default_methods()

    def _register_default_methods(self):
        """Register default task decomposition methods"""
        self.register_method("fetch_object", self._decompose_fetch_object)
        self.register_method("move_object", self._decompose_move_object)
        self.register_method("navigate_and_perform", self._decompose_navigate_and_perform)
        self.register_method("clean_area", self._decompose_clean_area)

    def register_method(self, task_name: str, method: Callable):
        """Register a method for decomposing a task"""
        self.task_methods[task_name] = method

    def decompose_task(self, task: Task, context: Dict[str, Any] = None) -> HTNPlan:
        """
        Decompose a high-level task into a hierarchical plan

        Args:
            task: The high-level task to decompose
            context: Environmental context information

        Returns:
            HTNPlan with the hierarchical decomposition
        """
        context = context or {}
        start_time = time.time()

        # Decompose the root task
        root_task = self._decompose_recursive(task, context)

        plan = HTNPlan(
            plan_id=f"htn_{int(start_time)}",
            root_task=root_task,
            decomposition_depth=self._calculate_depth(root_task),
            created_at=start_time
        )

        return plan

    def _decompose_recursive(self, task: Task, context: Dict[str, Any]) -> Task:
        """Recursively decompose a task into subtasks"""
        if task.method == 'primitive':
            # Already a primitive action, no further decomposition needed
            return task

        # Check if we have a method for this task
        if task.name in self.task_methods:
            try:
                subtasks = self.task_methods[task.name](task.parameters, context)
                task.subtasks = subtasks
                task.method = 'compound'

                # Recursively decompose subtasks
                for i, subtask in enumerate(subtasks):
                    subtasks[i] = self._decompose_recursive(subtask, context)
            except Exception as e:
                print(f"Error decomposing task {task.name}: {e}")
                task.status = 'failed'
        else:
            # If no specific method, try to match based on keywords
            subtasks = self._infer_decomposition(task, context)
            if subtasks:
                task.subtasks = subtasks
                task.method = 'compound'
                for i, subtask in enumerate(subtasks):
                    subtasks[i] = self._decompose_recursive(subtask, context)
            else:
                # If we can't decompose further, mark as primitive or fail
                print(f"No decomposition method found for task: {task.name}")
                task.status = 'failed'

        return task

    def _infer_decomposition(self, task: Task, context: Dict[str, Any]) -> Optional[List[Task]]:
        """Infer decomposition based on task name and parameters"""
        task_name_lower = task.name.lower()

        if any(keyword in task_name_lower for keyword in ['fetch', 'get', 'bring']):
            return self._decompose_fetch_object(task.parameters, context)
        elif any(keyword in task_name_lower for keyword in ['move', 'transport']):
            return self._decompose_move_object(task.parameters, context)
        elif any(keyword in task_name_lower for keyword in ['go', 'navigate']):
            return self._decompose_navigate_and_perform(task.parameters, context)
        elif any(keyword in task_name_lower for keyword in ['clean', 'tidy']):
            return self._decompose_clean_area(task.parameters, context)
        else:
            # If we can't infer, return None to indicate failure
            return None

    def _decompose_fetch_object(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> List[Task]:
        """Decompose a fetch object task"""
        object_type = parameters.get('object_type', 'unknown')
        target_location = parameters.get('location', 'unknown')

        subtasks = [
            Task(
                name='detect_object',
                method='primitive',
                parameters={'object_type': object_type, 'search_location': target_location},
                estimated_duration=2.0
            ),
            Task(
                name='navigate_to_location',
                method='primitive',
                parameters={'location': target_location},
                estimated_duration=3.0
            ),
            Task(
                name='approach_object',
                method='primitive',
                parameters={'object_type': object_type},
                estimated_duration=1.5
            ),
            Task(
                name='grasp_object',
                method='primitive',
                parameters={'object_type': object_type},
                estimated_duration=1.0
            ),
            Task(
                name='transport_object',
                method='primitive',
                parameters={'destination': parameters.get('destination', 'current_location')},
                estimated_duration=4.0
            )
        ]

        # Add placement if destination is specified
        if 'destination' in parameters:
            subtasks.append(
                Task(
                    name='place_object',
                    method='primitive',
                    parameters={'destination': parameters['destination']},
                    estimated_duration=1.0
                )
            )

        return subtasks

    def _decompose_move_object(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> List[Task]:
        """Decompose a move object task"""
        object_type = parameters.get('object_type', 'unknown')
        source_location = parameters.get('from', 'unknown')
        target_location = parameters.get('to', 'unknown')

        subtasks = [
            Task(
                name='navigate_to_location',
                method='primitive',
                parameters={'location': source_location},
                estimated_duration=3.0
            ),
            Task(
                name='detect_object',
                method='primitive',
                parameters={'object_type': object_type, 'search_location': source_location},
                estimated_duration=2.0
            ),
            Task(
                name='approach_object',
                method='primitive',
                parameters={'object_type': object_type},
                estimated_duration=1.5
            ),
            Task(
                name='grasp_object',
                method='primitive',
                parameters={'object_type': object_type},
                estimated_duration=1.0
            ),
            Task(
                name='navigate_to_location',
                method='primitive',
                parameters={'location': target_location},
                estimated_duration=3.0
            ),
            Task(
                name='place_object',
                method='primitive',
                parameters={'destination': target_location},
                estimated_duration=1.0
            )
        ]

        return subtasks

    def _decompose_navigate_and_perform(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> List[Task]:
        """Decompose a navigate and perform task"""
        target_location = parameters.get('location', 'unknown')
        action = parameters.get('action', 'wait')

        subtasks = [
            Task(
                name='navigate_to_location',
                method='primitive',
                parameters={'location': target_location},
                estimated_duration=3.0
            )
        ]

        if action == 'wait':
            subtasks.append(
                Task(
                    name='wait',
                    method='primitive',
                    parameters={'duration': parameters.get('duration', 1.0)},
                    estimated_duration=parameters.get('duration', 1.0)
                )
            )
        elif action == 'describe':
            subtasks.append(
                Task(
                    name='describe_scene',
                    method='primitive',
                    parameters={},
                    estimated_duration=2.0
                )
            )
        elif action == 'take_picture':
            subtasks.append(
                Task(
                    name='take_image',
                    method='primitive',
                    parameters={},
                    estimated_duration=0.5
                )
            )

        return subtasks

    def _decompose_clean_area(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> List[Task]:
        """Decompose a clean area task"""
        area = parameters.get('area', 'unknown')

        subtasks = [
            Task(
                name='navigate_to_location',
                method='primitive',
                parameters={'location': area},
                estimated_duration=2.0
            ),
            Task(
                name='detect_objects',
                method='primitive',
                parameters={'search_area': area},
                estimated_duration=3.0
            ),
            Task(
                name='classify_objects',
                method='primitive',
                parameters={'objects': parameters.get('objects', [])},
                estimated_duration=1.0
            )
        ]

        # For each object that needs to be moved/cleaned
        objects_to_handle = parameters.get('objects', [])
        for obj in objects_to_handle:
            if obj.get('needs_moving', False):
                subtasks.extend([
                    Task(
                        name='approach_object',
                        method='primitive',
                        parameters={'object_type': obj['type']},
                        estimated_duration=1.0
                    ),
                    Task(
                        name='grasp_object',
                        method='primitive',
                        parameters={'object_type': obj['type']},
                        estimated_duration=1.0
                    ),
                    Task(
                        name='transport_object',
                        method='primitive',
                        parameters={'destination': obj.get('destination', 'storage_area')},
                        estimated_duration=3.0
                    ),
                    Task(
                        name='place_object',
                        method='primitive',
                        parameters={'destination': obj['destination']},
                        estimated_duration=1.0
                    )
                ])

        return subtasks

    def _calculate_depth(self, task: Task) -> int:
        """Calculate the maximum depth of the task hierarchy"""
        if not task.subtasks:
            return 0
        return 1 + max(self._calculate_depth(subtask) for subtask in task.subtasks)

    def execute_plan(self, plan: HTNPlan, executor_callback: Callable = None) -> Dict[str, Any]:
        """
        Execute a hierarchical plan

        Args:
            plan: The HTN plan to execute
            executor_callback: Optional callback for executing primitive actions

        Returns:
            Execution results and statistics
        """
        start_time = time.time()

        results = {
            'plan_id': plan.plan_id,
            'executed_tasks': [],
            'failed_tasks': [],
            'execution_time': 0.0,
            'success': True
        }

        success = self._execute_task_recursive(plan.root_task, results, executor_callback)
        results['success'] = success
        results['execution_time'] = time.time() - start_time

        return results

    def _execute_task_recursive(self, task: Task, results: Dict[str, Any], executor_callback: Callable) -> bool:
        """Recursively execute a task and its subtasks"""
        task.status = 'executing'

        if task.method == 'primitive':
            # Execute primitive action
            if executor_callback:
                try:
                    result = executor_callback(task.name, task.parameters)
                    task.status = 'completed' if result.get('success', True) else 'failed'
                    results['executed_tasks'].append({
                        'task': task.name,
                        'parameters': task.parameters,
                        'result': result,
                        'status': task.status
                    })
                    return result.get('success', True)
                except Exception as e:
                    task.status = 'failed'
                    results['failed_tasks'].append({
                        'task': task.name,
                        'parameters': task.parameters,
                        'error': str(e)
                    })
                    return False
            else:
                # Simulate execution
                task.status = 'completed'
                results['executed_tasks'].append({
                    'task': task.name,
                    'parameters': task.parameters,
                    'result': {'success': True, 'message': 'Simulated execution'},
                    'status': task.status
                })
                return True
        else:
            # Execute compound task by executing all subtasks
            if task.subtasks:
                for subtask in task.subtasks:
                    success = self._execute_task_recursive(subtask, results, executor_callback)
                    if not success:
                        # If any subtask fails, the compound task fails
                        task.status = 'failed'
                        return False
            task.status = 'completed'
            return True

    def get_primitive_actions(self) -> List[str]:
        """Get list of available primitive actions"""
        return self.primitive_actions.copy()

    def validate_plan(self, plan: HTNPlan) -> List[str]:
        """Validate a hierarchical plan for correctness"""
        issues = []

        def check_task(task: Task, depth: int = 0):
            # Check if task name is valid
            if task.method == 'primitive' and task.name not in self.primitive_actions:
                issues.append(f"Unknown primitive action: {task.name}")

            # Check if parameters are valid
            if not isinstance(task.parameters, dict):
                issues.append(f"Invalid parameters for task {task.name}: must be dict")

            # Recursively check subtasks
            if task.subtasks:
                for subtask in task.subtasks:
                    check_task(subtask, depth + 1)

        check_task(plan.root_task)
        return issues


class EnhancedHTNPlanner(HierarchicalTaskNetwork):
    """
    Enhanced HTN planner with additional features like safety checks
    and plan refinement
    """

    def __init__(self):
        super().__init__()
        self.safety_constraints = []
        self.planning_context = {}

    def add_safety_constraint(self, constraint: Callable[[Task], bool]):
        """Add a safety constraint function"""
        self.safety_constraints.append(constraint)

    def decompose_task_with_safety(self, task: Task, context: Dict[str, Any] = None) -> HTNPlan:
        """Decompose task with safety validation"""
        plan = self.decompose_task(task, context)

        # Apply safety constraints
        issues = self.validate_safety(plan)
        if issues:
            print(f"Safety issues found: {issues}")
            # Attempt to refine the plan to address safety issues
            plan = self.refine_plan_for_safety(plan, issues)

        return plan

    def validate_safety(self, plan: HTNPlan) -> List[str]:
        """Validate plan for safety issues"""
        issues = []

        def check_task_safety(task: Task):
            for constraint in self.safety_constraints:
                try:
                    if not constraint(task):
                        issues.append(f"Safety constraint violated for task: {task.name}")
                except Exception as e:
                    issues.append(f"Error checking safety constraint: {e}")

            if task.subtasks:
                for subtask in task.subtasks:
                    check_task_safety(subtask)

        check_task_safety(plan.root_task)
        return issues

    def refine_plan_for_safety(self, plan: HTNPlan, issues: List[str]) -> HTNPlan:
        """Refine plan to address safety issues"""
        # This is a basic implementation - in practice, this would be more sophisticated
        print("Refining plan for safety...")
        return plan  # Return the original plan for now


# Example usage and testing
def main():
    """Example usage of the HTN planner"""
    print("Testing Hierarchical Task Network...")

    # Initialize HTN planner
    htn = EnhancedHTNPlanner()

    # Example 1: Fetch object task
    print("\n1. Testing fetch object task decomposition:")
    fetch_task = Task(
        name='fetch_object',
        method='compound',
        parameters={
            'object_type': 'red cup',
            'location': 'kitchen table',
            'destination': 'living room'
        }
    )

    context = {
        'robot_location': 'charging_dock',
        'object_locations': {'red cup': 'kitchen table'},
        'environment_map': 'available'
    }

    plan = htn.decompose_task(fetch_task, context)
    print(f"Plan ID: {plan.plan_id}")
    print(f"Decomposition depth: {plan.decomposition_depth}")

    # Print the task hierarchy
    def print_task(task, indent=0):
        prefix = "  " * indent
        print(f"{prefix}- {task.name} ({task.method}) - {task.status}")
        if task.subtasks:
            for subtask in task.subtasks:
                print_task(subtask, indent + 1)

    print("Task hierarchy:")
    print_task(plan.root_task)

    # Example 2: Move object task
    print("\n2. Testing move object task decomposition:")
    move_task = Task(
        name='move_object',
        method='compound',
        parameters={
            'object_type': 'book',
            'from': 'coffee table',
            'to': 'bookshelf'
        }
    )

    move_plan = htn.decompose_task(move_task, context)
    print(f"Move plan decomposition depth: {move_plan.decomposition_depth}")

    # Example 3: Add a safety constraint
    print("\n3. Testing safety constraints:")
    def avoid_human_safety(task: Task) -> bool:
        """Safety constraint: avoid actions near humans"""
        if task.name in ['move_forward', 'navigate_to_location']:
            # In a real system, we'd check if the path is clear of humans
            return True  # For simulation, always allow
        return True

    htn.add_safety_constraint(avoid_human_safety)

    # Validate the plan with safety constraints
    safety_issues = htn.validate_safety(plan)
    print(f"Safety issues: {safety_issues}")

    # Example 4: Execute the plan (simulated)
    print("\n4. Testing plan execution:")
    def mock_executor(action_name: str, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Mock executor for primitive actions"""
        print(f"  Executing: {action_name} with {parameters}")
        return {'success': True, 'result': f"Executed {action_name}"}

    execution_results = htn.execute_plan(plan, mock_executor)
    print(f"Execution success: {execution_results['success']}")
    print(f"Executed tasks: {len(execution_results['executed_tasks'])}")
    print(f"Failed tasks: {len(execution_results['failed_tasks'])}")

    print("\nHTN testing completed!")


if __name__ == "__main__":
    main()