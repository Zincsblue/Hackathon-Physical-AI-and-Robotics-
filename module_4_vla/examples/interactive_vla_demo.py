"""
Interactive VLA (Vision-Language-Action) demonstration
This example simulates the core components of a VLA system
"""

import time
import random
from typing import List, Dict, Any
from dataclasses import dataclass


@dataclass
class VLAResult:
    """Result of a VLA operation"""
    success: bool
    message: str
    execution_time: float
    action_sequence: List[str]


class VisionSystem:
    """Simulates the vision component of VLA"""

    def __init__(self):
        self.objects_in_scene = [
            "red cup", "blue ball", "green box", "yellow book",
            "white chair", "black table", "orange pen", "purple vase"
        ]

    def perceive_scene(self) -> List[str]:
        """Simulate perceiving objects in the environment"""
        print("🔍 Vision System: Analyzing the scene...")
        time.sleep(0.5)  # Simulate processing time

        # Randomly select 4-6 objects from the scene
        num_objects = random.randint(4, 6)
        scene_objects = random.sample(self.objects_in_scene, num_objects)

        print(f"   Detected objects: {', '.join(scene_objects)}")
        return scene_objects


class LanguageSystem:
    """Simulates the language component of VLA"""

    def __init__(self):
        self.known_actions = ["fetch", "move", "pick", "place", "go to", "bring"]
        self.known_objects = [
            "cup", "ball", "box", "book", "chair", "table",
            "pen", "vase", "bottle", "phone", "keys", "wallet"
        ]

    def interpret_command(self, command: str) -> Dict[str, Any]:
        """Interpret a natural language command"""
        print(f"💬 Language System: Processing command: '{command}'")
        time.sleep(0.3)  # Simulate processing time

        command_lower = command.lower()

        # Extract action
        action = None
        for known_action in self.known_actions:
            if known_action in command_lower:
                action = known_action
                break

        # Extract object
        target_object = None
        for known_object in self.known_objects:
            if known_object in command_lower:
                target_object = known_object
                break

        # Extract color (if any)
        colors = ["red", "blue", "green", "yellow", "white", "black", "orange", "purple"]
        target_color = None
        for color in colors:
            if color in command_lower:
                target_color = color
                break

        result = {
            "command": command,
            "action": action,
            "target_object": target_object,
            "target_color": target_color,
            "confidence": 0.85  # Simulated confidence
        }

        print(f"   Interpreted: action='{action}', object='{target_object}', color='{target_color}'")
        return result


class ActionSystem:
    """Simulates the action component of VLA"""

    def __init__(self):
        self.simulated_robot_actions = [
            "navigate_to_location",
            "detect_object",
            "approach_object",
            "grasp_object",
            "lift_object",
            "transport_object",
            "place_object",
            "return_to_home"
        ]

    def plan_actions(self, language_result: Dict[str, Any], scene_objects: List[str]) -> List[str]:
        """Plan actions based on language interpretation and scene"""
        print("⚙️  Action System: Planning actions...")
        time.sleep(0.4)  # Simulate planning time

        action_plan = []

        if language_result["action"] in ["fetch", "pick", "bring"]:
            # Find the target object in the scene
            target = None
            if language_result["target_color"] and language_result["target_object"]:
                target = f"{language_result['target_color']} {language_result['target_object']}"
            elif language_result["target_object"]:
                target = language_result["target_object"]

            if target:
                # Check if target exists in scene
                target_found = any(target in obj for obj in scene_objects)

                if target_found:
                    action_plan = [
                        "navigate_to_location",
                        "detect_object",
                        "approach_object",
                        "grasp_object",
                        "lift_object",
                        "transport_object",
                        "place_object",
                        "return_to_home"
                    ]
                else:
                    action_plan = ["search_for_object", "report_failure"]
            else:
                action_plan = ["interpret_command", "plan_action", "execute_action"]

        elif language_result["action"] in ["go to"]:
            action_plan = ["navigate_to_location", "arrive_at_destination"]

        else:
            action_plan = ["interpret_command", "plan_action", "execute_action"]

        print(f"   Planned actions: {action_plan}")
        return action_plan

    def execute_action_sequence(self, action_plan: List[str]) -> VLAResult:
        """Execute the planned action sequence"""
        print("🤖 Action System: Executing action sequence...")

        start_time = time.time()

        for i, action in enumerate(action_plan):
            print(f"   Step {i+1}/{len(action_plan)}: {action}")
            time.sleep(0.3)  # Simulate action execution time

        execution_time = time.time() - start_time

        # Simulate success/failure
        success = random.choice([True, True, True, True, True])  # 80% success rate
        message = "Action sequence completed successfully" if success else "Action sequence failed"

        return VLAResult(
            success=success,
            message=message,
            execution_time=round(execution_time, 2),
            action_sequence=action_plan
        )


class VLADemo:
    """Main VLA demonstration system"""

    def __init__(self):
        self.vision = VisionSystem()
        self.language = LanguageSystem()
        self.action = ActionSystem()

    def process_command(self, command: str) -> VLAResult:
        """Process a complete VLA command"""
        print(f"\n🚀 Processing VLA Command: '{command}'")
        print("=" * 50)

        # Step 1: Vision processing
        scene_objects = self.vision.perceive_scene()

        # Step 2: Language processing
        language_result = self.language.interpret_command(command)

        # Step 3: Action planning and execution
        action_plan = self.action.plan_actions(language_result, scene_objects)
        result = self.action.execute_action_sequence(action_plan)

        print(f"\n✅ Result: {result.message}")
        print(f"   Execution time: {result.execution_time}s")
        print(f"   Success: {result.success}")
        print("=" * 50)

        return result


def main():
    """Interactive VLA demonstration"""
    print("🤖 Vision-Language-Action (VLA) Interactive Demo")
    print("This demo simulates how a VLA system processes natural language commands")
    print("and executes them using vision and action planning.\n")

    # Create VLA demo instance
    vla_demo = VLADemo()

    # Sample commands to demonstrate VLA capabilities
    sample_commands = [
        "Fetch the red cup",
        "Bring me the blue ball",
        "Go to the table",
        "Pick up the green box",
        "Get the yellow book"
    ]

    print("Running sample commands:\n")

    for command in sample_commands:
        result = vla_demo.process_command(command)
        print()  # Add spacing

    # Interactive mode
    print("\n🎮 Interactive Mode")
    print("Enter your own commands (or 'quit' to exit):")

    while True:
        user_command = input("\nEnter command: ").strip()

        if user_command.lower() in ['quit', 'exit', 'q']:
            print("👋 Thanks for trying the VLA demo!")
            break

        if user_command:
            vla_demo.process_command(user_command)


if __name__ == "__main__":
    main()