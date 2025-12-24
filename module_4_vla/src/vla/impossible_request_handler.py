"""
Error handling for impossible requests in VLA systems

This module implements detection and handling of impossible or invalid requests
in the VLA system, providing appropriate responses and alternatives.
"""

from typing import Dict, List, Any, Tuple, Optional
import re
from dataclasses import dataclass


@dataclass
class ImpossibleRequestResult:
    """Result when an impossible request is detected"""
    is_impossible: bool
    reason: str
    confidence: float  # 0.0 to 1.0
    alternatives: List[str]
    suggested_reformulation: Optional[str] = None


class ImpossibleRequestHandler:
    """
    Handler for detecting and managing impossible requests in VLA systems
    """

    def __init__(self):
        """Initialize the impossible request handler"""
        self.impossible_patterns = [
            # Physical impossibilities
            r"lift.*more than.*kg",
            r"lift.*heavier than",
            r"fly",
            r"become.*incorporeal",
            r"go through walls",
            r"pass through solid objects",

            # Logical impossibilities
            r"be in two places at once",
            r"clone yourself",
            r"stop time",
            r"break laws of physics",

            # Capability limitations
            r"cook.*without.*oven",
            r"drive.*without.*license",
            r"perform surgery",
            r"handle.*hazardous materials",
        ]

        self.physical_constraints = {
            "max_lift_weight": 2.0,  # kg
            "max_reach": 1.5,        # meters
            "max_speed": 1.0,        # m/s
            "max_operating_time": 8.0 * 3600,  # seconds (8 hours)
        }

        self.known_objects = set([
            "cup", "book", "bottle", "plate", "fork", "spoon", "knife",
            "glass", "phone", "keys", "wallet", "shoes", "hat", "coat"
        ])

        self.known_locations = set([
            "kitchen", "living room", "bedroom", "bathroom", "office",
            "dining room", "hallway", "garage", "garden", "entrance"
        ])

    def detect_impossible_request(self, command: str, context: Dict[str, Any] = None) -> ImpossibleRequestResult:
        """
        Detect if a request is impossible based on various factors

        Args:
            command: The natural language command to evaluate
            context: Environmental context information

        Returns:
            ImpossibleRequestResult with detection results
        """
        context = context or {}
        command_lower = command.lower().strip()

        # Check against impossible patterns
        for pattern in self.impossible_patterns:
            if re.search(pattern, command_lower):
                return ImpossibleRequestResult(
                    is_impossible=True,
                    reason=f"Request matches impossible pattern: {pattern}",
                    confidence=0.95,
                    alternatives=self._generate_alternatives(command, context)
                )

        # Check physical constraints
        physical_issues = self._check_physical_constraints(command, context)
        if physical_issues:
            return ImpossibleRequestResult(
                is_impossible=True,
                reason=f"Physical constraints violated: {', '.join(physical_issues)}",
                confidence=0.85,
                alternatives=self._generate_alternatives(command, context)
            )

        # Check capability limitations
        capability_issues = self._check_capability_limitations(command, context)
        if capability_issues:
            return ImpossibleRequestResult(
                is_impossible=True,
                reason=f"Capability limitations: {', '.join(capability_issues)}",
                confidence=0.80,
                alternatives=self._generate_alternatives(command, context)
            )

        # Check object/location existence if specified
        existence_issues = self._check_existence_requirements(command, context)
        if existence_issues:
            return ImpossibleRequestResult(
                is_impossible=True,
                reason=f"Missing requirements: {', '.join(existence_issues)}",
                confidence=0.70,
                alternatives=self._generate_alternatives(command, context),
                suggested_reformulation=self._suggest_reformulation(command, existence_issues)
            )

        # If no issues found, the request is possible
        return ImpossibleRequestResult(
            is_impossible=False,
            reason="Request appears to be possible",
            confidence=1.0,
            alternatives=[]
        )

    def _check_physical_constraints(self, command: str, context: Dict[str, Any]) -> List[str]:
        """Check if the command violates physical constraints"""
        issues = []
        command_lower = command.lower()

        # Check for lifting requests
        weight_match = re.search(r"lift|carry|move.*(\d+(?:\.\d+)?)\s*kg|kilograms?", command_lower)
        if weight_match:
            try:
                requested_weight = float(weight_match.group(1))
                if requested_weight > self.physical_constraints["max_lift_weight"]:
                    issues.append(f"Requested weight ({requested_weight}kg) exceeds max capacity ({self.physical_constraints['max_lift_weight']}kg)")
            except ValueError:
                pass

        # Check for reach constraints
        reach_match = re.search(r"get.*high|reach.*above|access.*top shelf", command_lower)
        if reach_match:
            # If context specifies height, check it
            if 'height' in context:
                try:
                    requested_height = float(context['height'])
                    if requested_height > self.physical_constraints["max_reach"]:
                        issues.append(f"Requested height ({requested_height}m) exceeds max reach ({self.physical_constraints['max_reach']}m)")
                except ValueError:
                    pass

        return issues

    def _check_capability_limitations(self, command: str, context: Dict[str, Any]) -> List[str]:
        """Check if the command requires capabilities beyond the robot's"""
        issues = []
        command_lower = command.lower()

        # Check for specialized skills
        if any(skill in command_lower for skill in ["surgery", "medical procedure", "hazardous material"]):
            issues.append(f"Robot lacks {command.split()[0] if command.split() else 'required'} specialization")

        # Check for transportation without proper setup
        if "drive" in command_lower and "car" in command_lower:
            issues.append("Robot cannot operate vehicle without proper interface and licensing")

        return issues

    def _check_existence_requirements(self, command: str, context: Dict[str, Any]) -> List[str]:
        """Check if required objects or locations exist"""
        issues = []
        command_lower = command.lower()

        # Extract object types from command
        object_matches = re.findall(r"(?:the|a|an)\s+([a-zA-Z]+)", command_lower)
        location_matches = re.findall(r"(?:to|from|at|in|on)\s+([a-zA-Z\s]+)", command_lower)

        # Clean up location matches
        cleaned_locations = []
        for loc in location_matches:
            # Remove common words that aren't locations
            words = loc.strip().split()
            if len(words) > 0:
                potential_location = words[0]  # Take first word as potential location
                if potential_location in self.known_locations:
                    cleaned_locations.append(potential_location)

        # Check if requested objects are known
        for obj in object_matches:
            if obj not in self.known_objects and obj not in ["the", "a", "an", "and", "or", "with", "for", "to", "from", "at", "in", "on"]:
                # Check if it's a general category that might be valid
                if obj not in ["object", "item", "thing", "something"]:
                    issues.append(f"Unknown object type: {obj}")

        # Check if requested locations are known
        for loc in cleaned_locations:
            if loc not in self.known_locations:
                issues.append(f"Unknown location: {loc}")

        return issues

    def _generate_alternatives(self, command: str, context: Dict[str, Any]) -> List[str]:
        """Generate alternative ways to fulfill the user's intent"""
        alternatives = []
        command_lower = command.lower()

        if "fly" in command_lower:
            alternatives.append("Navigate using ground-based movement")
            alternatives.append("Request assistance from a drone if available")

        elif any(word in command_lower for word in ["lift", "carry", "move"]):
            # Check if it's about heavy objects
            if any(word in command_lower for word in ["heavy", "heavyweight", "large"]):
                alternatives.append("Break the task into smaller parts")
                alternatives.append("Request human assistance for heavy items")
                alternatives.append("Use specialized equipment if available")

        elif any(word in command_lower for word in ["kitchen", "bathroom", "bedroom"]):
            # Location-based alternatives
            alternatives.append(f"Navigate to the closest accessible location: {list(self.known_locations)[0]}")
            alternatives.append("Provide information about the requested location instead")

        # Default alternatives
        if not alternatives:
            alternatives.extend([
                "Ask for clarification on the request",
                "Break down the request into smaller tasks",
                "Suggest a similar achievable task",
                "Request human assistance"
            ])

        return alternatives

    def _suggest_reformulation(self, command: str, issues: List[str]) -> Optional[str]:
        """Suggest a reformulated command that might be achievable"""
        command_lower = command.lower()

        for issue in issues:
            if "unknown object" in issue.lower():
                obj_name = issue.split(": ")[1] if ": " in issue else "object"
                return f"Could you clarify what a '{obj_name}' is or specify a known object like 'cup' or 'book'?"

            elif "unknown location" in issue.lower():
                loc_name = issue.split(": ")[1] if ": " in issue else "location"
                return f"Could you specify a known location like 'kitchen' or 'living room' instead of '{loc_name}'?"

            elif "weight" in issue.lower():
                # Extract the weight from the issue
                import re
                weight_match = re.search(r"(\d+(?:\.\d+)?)kg", issue)
                if weight_match:
                    weight = weight_match.group(1)
                    return f"Could you request an object under {self.physical_constraints['max_lift_weight']}kg instead of {weight}kg?"

        return "Could you please rephrase your request with known objects and locations?"

    def handle_impossible_request(self, command: str, context: Dict[str, Any] = None) -> Tuple[bool, str, List[str]]:
        """
        Handle an impossible request by providing feedback and alternatives

        Args:
            command: The impossible command to handle
            context: Environmental context information

        Returns:
            Tuple of (handled_successfully, response_message, alternative_suggestions)
        """
        result = self.detect_impossible_request(command, context)

        if not result.is_impossible:
            return True, "Request is possible and can be processed normally", []

        # Generate appropriate response
        response_parts = [
            f"I cannot fulfill the request '{command}' because {result.reason}.",
            f"Here are some alternatives:"
        ]

        for i, alt in enumerate(result.alternatives[:3]):  # Limit to 3 alternatives
            response_parts.append(f"{i+1}. {alt}")

        if result.suggested_reformulation:
            response_parts.append(f"\nSuggested reformulation: {result.suggested_reformulation}")

        response_message = " ".join(response_parts)
        return False, response_message, result.alternatives


class EnhancedImpossibleRequestHandler(ImpossibleRequestHandler):
    """
    Enhanced handler with additional features like learning from feedback
    """

    def __init__(self):
        super().__init__()
        self.feedback_history = []
        self.learned_patterns = []

    def learn_from_feedback(self, command: str, original_result: ImpossibleRequestResult,
                          user_feedback: str) -> None:
        """
        Learn from user feedback about impossible request detection
        """
        feedback_entry = {
            "command": command,
            "original_result": original_result,
            "user_feedback": user_feedback,
            "timestamp": __import__('time').time()
        }

        self.feedback_history.append(feedback_entry)

        # Simple learning: if user feedback indicates a false positive,
        # add a pattern to avoid similar false detections
        if "actually possible" in user_feedback.lower() or "you can do" in user_feedback.lower():
            # Add to learned patterns to avoid future false positives
            self.learned_patterns.append(command.lower())

    def detect_impossible_request(self, command: str, context: Dict[str, Any] = None) -> ImpossibleRequestResult:
        """
        Enhanced detection that considers learned patterns
        """
        # First check learned patterns (to avoid false positives)
        for pattern in self.learned_patterns:
            if pattern in command.lower():
                # This was previously marked as impossible but user said it's possible
                return ImpossibleRequestResult(
                    is_impossible=False,
                    reason="Learned from feedback that this is possible",
                    confidence=0.8,
                    alternatives=[]
                )

        # Use parent method for actual detection
        return super().detect_impossible_request(command, context)


# Example usage and testing
def main():
    """Example usage of the impossible request handler"""
    print("Testing Impossible Request Handler...")

    handler = EnhancedImpossibleRequestHandler()

    # Test cases for impossible requests
    test_cases = [
        "Fly to the moon",
        "Lift the 50kg refrigerator",
        "Perform surgery on the patient",
        "Go through the wall to the other room",
        "Be in the kitchen and bedroom at the same time",
        "Bring me the purple elephant",
        "Drive the car to the store",
        "Get the book from the living room",  # This should be possible
        "Lift the 1kg box",  # This should be possible
    ]

    for command in test_cases:
        print(f"\nTesting command: '{command}'")
        result = handler.detect_impossible_request(command)

        print(f"  Impossible: {result.is_impossible}")
        print(f"  Reason: {result.reason}")
        print(f"  Confidence: {result.confidence:.2f}")

        if result.alternatives:
            print(f"  Alternatives: {result.alternatives[:2]}")  # Show first 2 alternatives

        if result.suggested_reformulation:
            print(f"  Reformulation: {result.suggested_reformulation}")

    # Test handling impossible requests
    print("\n" + "="*50)
    print("Testing request handling:")

    impossible_command = "Fly to the moon and bring back moon rocks"
    success, response, alternatives = handler.handle_impossible_request(impossible_command)

    print(f"Command: {impossible_command}")
    print(f"Handled successfully: {success}")
    print(f"Response: {response}")

    # Simulate learning from feedback
    print("\n" + "="*30)
    print("Testing learning from feedback:")

    handler.learn_from_feedback(
        "Open the door",
        ImpossibleRequestResult(True, "Unknown action", 0.7, ["Ask for help"]),
        "Actually possible, you have a door opening mechanism"
    )

    # Now check if it recognizes the learned command
    result = handler.detect_impossible_request("Open the door")
    print(f"'Open the door' now considered impossible: {result.is_impossible}")

    print("\nImpossible request handler testing completed!")


if __name__ == "__main__":
    main()