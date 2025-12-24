"""
Command parsing and validation for voice inputs in VLA systems
"""

import re
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
import json


@dataclass
class ParsedCommand:
    """Represents a parsed voice command"""
    command_type: str  # navigation, manipulation, control, etc.
    action: str        # specific action to perform
    objects: List[str] # objects involved in the command
    locations: List[str] # locations mentioned
    quantities: List[str] # quantities mentioned
    raw_text: str      # original command text
    confidence: float  # confidence in parsing
    entities: Dict[str, List[str]]  # all extracted entities
    is_valid: bool     # whether command is valid


class CommandParser:
    """
    Parses and validates voice commands for VLA systems
    """

    def __init__(self):
        # Define command patterns and actions
        self.command_patterns = {
            # Navigation commands
            'navigation': [
                r'.*move\s+(forward|backward|left|right).*',
                r'.*(go|move|navigate)\s+to.*',
                r'.*(go|move|navigate)\s+(up|down|ahead).*',
                r'.*turn\s+(left|right|around).*',
                r'.*face\s+(left|right|around).*',
            ],

            # Manipulation commands
            'manipulation': [
                r'.*(pick|grasp|take|grab|get|lift)\s+up.*',
                r'.*(put|place|drop|set)\s+(down|on|in).*',
                r'.*(pick|grasp|take|grab|get)\s+(the\s+|a\s+)?(\w+)\s+(up|from).*',
                r'.*(put|place|drop|set)\s+(the\s+|a\s+)?(\w+)\s+(down|on|in|at).*',
                r'.*(bring|fetch|carry)\s+(me|to).*',
                r'.*(open|close)\s+(\w+).*',
            ],

            # Control commands
            'control': [
                r'.*(stop|halt|pause|wait|stand|stay|idle).*',
                r'.*(start|continue|resume|go|proceed).*',
                r'.*(help|assist|repeat|again|more).*',
            ],

            # Interaction commands
            'interaction': [
                r'.*(say|speak|tell|greet|hello|hi)\s+.*',
                r'.*(look|see|find|search|where)\s+.*',
                r'.*(describe|explain|what\s+is|what\s+are)\s+.*',
            ]
        }

        # Define common objects that robots might interact with
        self.common_objects = {
            'furniture': ['table', 'chair', 'desk', 'bed', 'couch', 'sofa', 'shelf', 'cupboard'],
            'kitchen': ['cup', 'glass', 'plate', 'bowl', 'fork', 'spoon', 'knife', 'bottle', 'mug', 'pan'],
            'electronics': ['phone', 'tablet', 'laptop', 'tv', 'remote', 'computer'],
            'personal_items': ['keys', 'wallet', 'book', 'pen', 'pencil', 'bag', 'hat', 'glasses'],
            'other': ['ball', 'box', 'toy', 'object', 'item', 'thing', 'stuff']
        }

        # Define locations
        self.common_locations = {
            'rooms': ['kitchen', 'living room', 'bedroom', 'bathroom', 'office', 'dining room', 'hallway'],
            'directions': ['left', 'right', 'front', 'back', 'behind', 'in front', 'next to', 'near', 'far'],
            'furniture_locs': ['on', 'under', 'above', 'beside', 'at', 'by', 'in', 'inside', 'outside']
        }

        # Flatten object lists for easy lookup
        self.all_objects = []
        for category in self.common_objects.values():
            self.all_objects.extend(category)

        # Flatten location lists for easy lookup
        self.all_locations = []
        for category in self.common_locations.values():
            self.all_locations.extend(category)

    def parse_command(self, text: str) -> ParsedCommand:
        """
        Parse a voice command and extract its components

        Args:
            text: The raw text command from voice recognition

        Returns:
            ParsedCommand object with command components
        """
        if not text or not isinstance(text, str):
            return ParsedCommand(
                command_type='invalid',
                action='none',
                objects=[],
                locations=[],
                quantities=[],
                raw_text=text or '',
                confidence=0.0,
                entities={},
                is_valid=False
            )

        # Normalize the text
        normalized_text = self._normalize_text(text)

        # Determine command type
        command_type = self._determine_command_type(normalized_text)

        # Extract entities
        entities = self._extract_entities(normalized_text)

        # Determine specific action
        action = self._determine_action(normalized_text, command_type)

        # Calculate confidence in parsing
        confidence = self._calculate_parsing_confidence(normalized_text, entities)

        # Check validity
        is_valid = self._is_command_valid(command_type, action, entities)

        return ParsedCommand(
            command_type=command_type,
            action=action,
            objects=entities.get('objects', []),
            locations=entities.get('locations', []),
            quantities=entities.get('quantities', []),
            raw_text=text,
            confidence=confidence,
            entities=entities,
            is_valid=is_valid
        )

    def _normalize_text(self, text: str) -> str:
        """Normalize text for processing"""
        # Convert to lowercase
        normalized = text.lower()

        # Remove extra whitespace
        normalized = ' '.join(normalized.split())

        # Replace common contractions
        contractions = {
            "i'm": "i am",
            "you're": "you are",
            "it's": "it is",
            "don't": "do not",
            "doesn't": "does not",
            "didn't": "did not",
            "can't": "cannot",
            "won't": "will not",
            "shouldn't": "should not",
            "wouldn't": "would not",
            "couldn't": "could not"
        }

        for contraction, expanded in contractions.items():
            normalized = normalized.replace(contraction, expanded)

        return normalized

    def _determine_command_type(self, text: str) -> str:
        """Determine the type of command based on patterns"""
        for cmd_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text, re.IGNORECASE):
                    return cmd_type

        # If no pattern matches, return 'general'
        return 'general'

    def _determine_action(self, text: str, command_type: str) -> str:
        """Determine the specific action from the text"""
        # Common actions for each command type
        actions = {
            'navigation': [
                ('move forward', r'.*(move\s+forward|go\s+forward|forward).*'),
                ('move backward', r'.*(move\s+backward|go\s+backward|backward|back).*'),
                ('turn left', r'.*(turn\s+left|left\s+turn|go\s+left|left).*'),
                ('turn right', r'.*(turn\s+right|right\s+turn|go\s+right|right).*'),
                ('go to location', r'.*(go\s+to|move\s+to|navigate\s+to|head\s+to).*'),
                ('stop', r'.*(stop|halt|pause|stand|stay|idle|freeze).*'),
            ],
            'manipulation': [
                ('pick up object', r'.*(pick|grasp|take|grab|get|lift)\s+up.*'),
                ('place object', r'.*(put|place|drop|set)\s+(down|on|in).*'),
                ('bring object', r'.*(bring|fetch|carry)\s+(me|to).*'),
                ('open object', r'.*(open|uncover|unlock)\s+(\w+).*'),
                ('close object', r'.*(close|cover|lock)\s+(\w+).*'),
            ],
            'control': [
                ('stop', r'.*(stop|halt|pause|wait|stand|stay|idle).*'),
                ('continue', r'.*(start|continue|resume|go|proceed|begin).*'),
                ('repeat', r'.*(repeat|again|more|replay|redo).*'),
            ],
            'interaction': [
                ('speak', r'.*(say|speak|tell|greet|hello|hi)\s+.*'),
                ('find object', r'.*(look|see|find|search|where)\s+.*'),
                ('describe', r'.*(describe|explain|what\s+is|what\s+are)\s+.*'),
            ]
        }

        if command_type in actions:
            for action_name, pattern in actions[command_type]:
                if re.search(pattern, text, re.IGNORECASE):
                    return action_name

        # Return a default action if no specific pattern matches
        return f'perform {command_type} action'

    def _extract_entities(self, text: str) -> Dict[str, List[str]]:
        """Extract entities (objects, locations, etc.) from the text"""
        entities = {
            'objects': [],
            'locations': [],
            'quantities': [],
            'other': []
        }

        # Extract objects
        for obj in self.all_objects:
            if re.search(rf'\b{obj}\b', text, re.IGNORECASE):
                if obj not in entities['objects']:
                    entities['objects'].append(obj)

        # Extract locations
        for loc in self.all_locations:
            if re.search(rf'\b{loc}\b', text, re.IGNORECASE):
                if loc not in entities['locations']:
                    entities['locations'].append(loc)

        # Extract quantities (numbers, amounts)
        quantity_pattern = r'\b(\d+|one|two|three|four|five|six|seven|eight|nine|ten|a|an|some|several|few|many|all)\b'
        quantities = re.findall(quantity_pattern, text, re.IGNORECASE)
        entities['quantities'] = list(set(quantities))

        # Extract other potential entities (words not captured above)
        all_words = set(re.findall(r'\b\w+\b', text))
        used_entities = set(entities['objects'] + entities['locations'])
        other_entities = [word for word in all_words
                         if word not in used_entities and
                         word not in ['the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with', 'by']]
        entities['other'] = other_entities

        return entities

    def _calculate_parsing_confidence(self, text: str, entities: Dict[str, List[str]]) -> float:
        """Calculate confidence in the parsing result"""
        confidence = 0.5  # Base confidence

        # Increase confidence based on entity extraction
        if entities['objects']:
            confidence += 0.1 * len(entities['objects'])
        if entities['locations']:
            confidence += 0.1 * len(entities['locations'])

        # Increase confidence if we have a specific action
        if text and any(word in text for word in ['pick', 'go', 'move', 'place', 'stop', 'turn']):
            confidence += 0.15

        # Ensure confidence is within bounds
        return min(confidence, 1.0)

    def _is_command_valid(self, command_type: str, action: str, entities: Dict[str, List[str]]) -> bool:
        """Check if the parsed command is valid"""
        # Invalid if command type is invalid
        if command_type == 'invalid':
            return False

        # Invalid if action is 'none' and no specific action could be determined
        if action == 'none' or 'perform general action' in action:
            return False

        # Otherwise, consider it valid if we could parse something meaningful
        return len(action) > 2 and command_type != 'general'

    def validate_command(self, parsed_command: ParsedCommand) -> Tuple[bool, List[str]]:
        """
        Validate a parsed command and return validation results

        Args:
            parsed_command: ParsedCommand object to validate

        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []

        # Check if command type is recognized
        if parsed_command.command_type == 'invalid':
            issues.append("Unknown command type")

        # Check if action is recognized
        if not parsed_command.action or parsed_command.action == 'none':
            issues.append("No valid action detected")

        # Check if command is too generic
        if parsed_command.command_type == 'general' and not parsed_command.entities.get('objects') and not parsed_command.entities.get('locations'):
            issues.append("Command is too generic, lacks specific objects or locations")

        # Check confidence threshold
        if parsed_command.confidence < 0.3:
            issues.append(f"Low parsing confidence: {parsed_command.confidence:.2f}")

        is_valid = len(issues) == 0
        return is_valid, issues

    def suggest_command_corrections(self, text: str) -> List[str]:
        """
        Suggest possible corrections for potentially misunderstood commands

        Args:
            text: Original command text

        Returns:
            List of suggested corrections
        """
        suggestions = []
        normalized = self._normalize_text(text)

        # Check for common issues
        if 'what' in normalized and 'where' in normalized:
            suggestions.append("Did you mean to ask 'where' instead of 'what'?")

        if 'and' in normalized and len(normalized.split()) > 8:
            suggestions.append("Try breaking complex commands into simpler parts")

        # Check for potential object identification issues
        if any(word in normalized for word in ['it', 'that', 'this', 'there']):
            suggestions.append("Be more specific about which object you mean")

        # Check for location ambiguity
        if any(word in normalized for word in ['there', 'here', 'over there']):
            suggestions.append("Specify exact locations or point to objects")

        return suggestions


class VoiceCommandValidator:
    """
    Higher-level validator for voice commands in VLA systems
    """

    def __init__(self):
        self.parser = CommandParser()
        self.confidence_threshold = 0.6

    def validate_voice_command(self, text: str, whisper_confidence: float = 0.8) -> Tuple[bool, ParsedCommand, List[str]]:
        """
        Validate a voice command with both ASR and parsing validation

        Args:
            text: Text from ASR system
            whisper_confidence: Confidence from ASR system

        Returns:
            Tuple of (is_valid, parsed_command, issues_list)
        """
        # Parse the command
        parsed_command = self.parser.parse_command(text)

        # Validate the parsed command
        is_parsed_valid, parsing_issues = self.parser.validate_command(parsed_command)

        # Combine ASR and parsing validation
        overall_valid = is_parsed_valid and whisper_confidence >= self.confidence_threshold

        # Collect all issues
        all_issues = parsing_issues[:]
        if whisper_confidence < self.confidence_threshold:
            all_issues.append(f"ASR confidence too low: {whisper_confidence:.2f} < {self.confidence_threshold}")

        return overall_valid, parsed_command, all_issues

    def get_command_feedback(self, text: str, whisper_confidence: float = 0.8) -> Dict[str, Any]:
        """
        Get comprehensive feedback about a voice command

        Args:
            text: Text from ASR system
            whisper_confidence: Confidence from ASR system

        Returns:
            Dictionary with comprehensive command analysis
        """
        is_valid, parsed_command, issues = self.validate_voice_command(text, whisper_confidence)

        feedback = {
            'raw_command': text,
            'is_valid': is_valid,
            'issues': issues,
            'suggestions': self.parser.suggest_command_corrections(text),
            'parsed_command': {
                'type': parsed_command.command_type,
                'action': parsed_command.action,
                'objects': parsed_command.objects,
                'locations': parsed_command.locations,
                'confidence': parsed_command.confidence,
                'entities': parsed_command.entities
            },
            'overall_confidence': min(whisper_confidence, parsed_command.confidence)
        }

        return feedback


# Example usage and testing
def main():
    """
    Example usage of the command parser
    """
    print("Testing Command Parser...")

    # Initialize parser
    parser = CommandParser()
    validator = VoiceCommandValidator()

    # Test commands
    test_commands = [
        "Pick up the red cup from the table",
        "Go to the kitchen and bring me water",
        "Move the book from the chair to the shelf",
        "Turn left and then stop",
        "Open the door",
        "Describe what you see",
        "Unclear command with no meaning",
        "Stop moving immediately"
    ]

    for cmd in test_commands:
        print(f"\nTesting command: '{cmd}'")

        # Parse the command
        parsed = parser.parse_command(cmd)
        print(f"  Type: {parsed.command_type}")
        print(f"  Action: {parsed.action}")
        print(f"  Objects: {parsed.objects}")
        print(f"  Locations: {parsed.locations}")
        print(f"  Confidence: {parsed.confidence:.2f}")
        print(f"  Valid: {parsed.is_valid}")

        # Validate the command
        is_valid, issues = parser.validate_command(parsed)
        print(f"  Validation issues: {issues}")

        # Test with validator
        validator_result = validator.get_command_feedback(cmd, whisper_confidence=0.85)
        print(f"  Overall validity: {validator_result['is_valid']}")
        print(f"  Suggestions: {validator_result['suggestions']}")

    print("\nCommand parser test completed successfully!")


if __name__ == "__main__":
    main()