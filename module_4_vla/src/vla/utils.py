"""
Utility functions for VLA system
"""

import json
import yaml
import os
from typing import Any, Dict, List, Optional
import numpy as np


def load_config(config_path: str) -> Dict[str, Any]:
    """
    Load configuration from YAML file
    """
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config


def save_config(config: Dict[str, Any], config_path: str) -> None:
    """
    Save configuration to YAML file
    """
    with open(config_path, 'w') as file:
        yaml.dump(config, file, default_flow_style=False)


def normalize_text(text: str) -> str:
    """
    Normalize text for processing
    """
    # Convert to lowercase
    text = text.lower()
    # Remove extra whitespace
    text = ' '.join(text.split())
    return text


def calculate_similarity(text1: str, text2: str) -> float:
    """
    Calculate similarity between two texts (simple implementation)
    """
    if not text1 or not text2:
        return 0.0

    # Simple word overlap similarity
    words1 = set(normalize_text(text1).split())
    words2 = set(normalize_text(text2).split())

    if not words1 and not words2:
        return 1.0
    if not words1 or not words2:
        return 0.0

    intersection = words1.intersection(words2)
    union = words1.union(words2)

    return len(intersection) / len(union)


def format_command_output(command: str, success: bool, message: str = "") -> Dict[str, Any]:
    """
    Format command execution output
    """
    return {
        "command": command,
        "success": success,
        "message": message,
        "timestamp": get_current_timestamp()
    }


def get_current_timestamp() -> str:
    """
    Get current timestamp in ISO format
    """
    from datetime import datetime
    return datetime.now().isoformat()


def validate_command(command: str) -> bool:
    """
    Basic validation of command
    """
    if not command or not isinstance(command, str):
        return False

    # Basic checks
    command = command.strip()
    if len(command) < 3:  # Minimum command length
        return False

    # Check for basic command structure
    # This is a simplified validation
    if any(char.isalpha() for char in command):
        return True

    return False


def extract_entities(text: str, known_entities: List[str]) -> List[str]:
    """
    Extract known entities from text
    """
    text_lower = text.lower()
    found_entities = []

    for entity in known_entities:
        if entity.lower() in text_lower:
            found_entities.append(entity)

    return list(set(found_entities))  # Remove duplicates


def create_action_sequence(command: str) -> List[str]:
    """
    Create a basic action sequence from a command
    """
    # This is a simplified implementation
    # In a real system, this would involve complex NLP and planning
    command_lower = command.lower()

    # Basic action extraction
    if "move" in command_lower or "go" in command_lower:
        return ["navigate_to_location"]
    elif "pick" in command_lower or "grasp" in command_lower:
        return ["detect_object", "approach_object", "grasp_object"]
    elif "place" in command_lower or "put" in command_lower:
        return ["navigate_to_location", "release_object"]
    else:
        return ["interpret_command", "plan_action", "execute_action"]


def confidence_score(text: str, threshold: float = 0.7) -> float:
    """
    Calculate a basic confidence score for text processing
    """
    # This is a simplified confidence calculation
    # In a real system, this would use more sophisticated methods
    if not text:
        return 0.0

    # Length-based confidence (longer text might be more meaningful)
    length_factor = min(len(text) / 100, 0.5)  # Max 0.5 from length

    # Keyword-based confidence
    action_keywords = ["move", "go", "pick", "place", "grasp", "navigate", "turn", "stop"]
    keyword_count = sum(1 for keyword in action_keywords if keyword in text.lower())
    keyword_factor = min(keyword_count * 0.1, 0.3)  # Max 0.3 from keywords

    # Basic confidence calculation
    base_confidence = 0.2  # Base confidence
    confidence = base_confidence + length_factor + keyword_factor

    return min(confidence, 1.0)  # Cap at 1.0