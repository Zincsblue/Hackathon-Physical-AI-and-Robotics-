"""
Error handling and confidence scoring for voice commands in VLA systems
"""

import time
import logging
from typing import Dict, List, Tuple, Optional, Any, Callable
from dataclasses import dataclass
from enum import Enum
import numpy as np


class ErrorType(Enum):
    """Types of errors that can occur in voice processing"""
    ASR_ERROR = "asr_error"           # Automatic Speech Recognition errors
    PARSING_ERROR = "parsing_error"   # Command parsing errors
    VALIDATION_ERROR = "validation_error"  # Command validation errors
    EXECUTION_ERROR = "execution_error"    # Command execution errors
    CONFIDENCE_ERROR = "confidence_error"  # Low confidence errors
    TIMEOUT_ERROR = "timeout_error"        # Timeout errors
    AUDIO_ERROR = "audio_error"            # Audio processing errors


@dataclass
class VoiceCommandResult:
    """Result of voice command processing"""
    success: bool
    command_text: str
    parsed_command: Optional[Any] = None
    confidence_score: float = 0.0
    error_type: Optional[ErrorType] = None
    error_message: Optional[str] = None
    processing_time: float = 0.0
    suggestions: List[str] = None
    retry_count: int = 0


class ConfidenceScorer:
    """
    Confidence scoring for voice commands in VLA systems
    """

    def __init__(self):
        self.min_confidence_threshold = 0.6
        self.parsing_weight = 0.4
        self.asr_weight = 0.4
        self.context_weight = 0.2

    def calculate_overall_confidence(
        self,
        asr_confidence: float,
        parsing_confidence: float,
        context_confidence: float = 1.0
    ) -> float:
        """
        Calculate overall confidence score combining multiple factors

        Args:
            asr_confidence: Confidence from ASR system
            parsing_confidence: Confidence from command parsing
            context_confidence: Confidence from contextual validation

        Returns:
            Overall confidence score (0.0 to 1.0)
        """
        # Ensure all confidence values are in valid range
        asr_confidence = max(0.0, min(1.0, asr_confidence))
        parsing_confidence = max(0.0, min(1.0, parsing_confidence))
        context_confidence = max(0.0, min(1.0, context_confidence))

        # Weighted combination of confidences
        overall_confidence = (
            asr_confidence * self.asr_weight +
            parsing_confidence * self.parsing_weight +
            context_confidence * self.context_weight
        )

        return overall_confidence

    def is_confident_enough(self, confidence: float) -> bool:
        """
        Check if confidence is above threshold

        Args:
            confidence: Confidence score to check

        Returns:
            True if confidence is sufficient
        """
        return confidence >= self.min_confidence_threshold

    def get_confidence_level(self, confidence: float) -> str:
        """
        Get confidence level as a string

        Args:
            confidence: Confidence score

        Returns:
            Confidence level as string ('very_low', 'low', 'medium', 'high', 'very_high')
        """
        if confidence < 0.3:
            return 'very_low'
        elif confidence < 0.5:
            return 'low'
        elif confidence < 0.7:
            return 'medium'
        elif confidence < 0.9:
            return 'high'
        else:
            return 'very_high'

    def calculate_parsing_confidence(self, parsed_command: Any) -> float:
        """
        Calculate confidence in command parsing

        Args:
            parsed_command: Parsed command object

        Returns:
            Parsing confidence score
        """
        if not parsed_command or not hasattr(parsed_command, 'command_type'):
            return 0.0

        confidence = 0.3  # Base confidence

        # Increase confidence based on successful parsing
        if parsed_command.command_type != 'invalid':
            confidence += 0.2

        if parsed_command.action and parsed_command.action != 'none':
            confidence += 0.2

        # Increase based on number of entities found
        entity_count = 0
        if hasattr(parsed_command, 'objects'):
            entity_count += len(parsed_command.objects)
        if hasattr(parsed_command, 'locations'):
            entity_count += len(parsed_command.locations)

        confidence += min(entity_count * 0.1, 0.3)  # Max 0.3 for entities

        return min(confidence, 1.0)


class VoiceCommandErrorHandler:
    """
    Comprehensive error handling for voice commands
    """

    def __init__(self):
        self.error_counts = {}
        self.confidence_scorer = ConfidenceScorer()
        self.logger = logging.getLogger(__name__)

    def handle_asr_error(self, error: Exception, audio_data: np.ndarray) -> VoiceCommandResult:
        """
        Handle errors from ASR system

        Args:
            error: Exception that occurred
            audio_data: Audio data that caused the error

        Returns:
            VoiceCommandResult with error information
        """
        self.logger.error(f"ASR Error: {str(error)}")

        # Determine error type based on exception
        error_type = ErrorType.ASR_ERROR
        error_message = str(error)

        # Check for specific error patterns
        if "timeout" in str(error).lower():
            error_type = ErrorType.TIMEOUT_ERROR
        elif "audio" in str(error).lower() or "sample rate" in str(error).lower():
            error_type = ErrorType.AUDIO_ERROR

        return VoiceCommandResult(
            success=False,
            command_text="",
            error_type=error_type,
            error_message=error_message,
            confidence_score=0.0,
            suggestions=self._get_asr_error_suggestions(error)
        )

    def handle_parsing_error(self, command_text: str, error: Exception) -> VoiceCommandResult:
        """
        Handle errors from command parsing

        Args:
            command_text: Text that failed to parse
            error: Exception that occurred

        Returns:
            VoiceCommandResult with error information
        """
        self.logger.error(f"Parsing Error for '{command_text}': {str(error)}")

        return VoiceCommandResult(
            success=False,
            command_text=command_text,
            error_type=ErrorType.PARSING_ERROR,
            error_message=str(error),
            confidence_score=0.0,
            suggestions=self._get_parsing_error_suggestions(command_text)
        )

    def handle_validation_error(self, parsed_command: Any, issues: List[str]) -> VoiceCommandResult:
        """
        Handle errors from command validation

        Args:
            parsed_command: Parsed command that failed validation
            issues: List of validation issues

        Returns:
            VoiceCommandResult with error information
        """
        self.logger.warning(f"Validation issues: {issues}")

        return VoiceCommandResult(
            success=False,
            command_text=parsed_command.raw_text if hasattr(parsed_command, 'raw_text') else "",
            parsed_command=parsed_command,
            error_type=ErrorType.VALIDATION_ERROR,
            error_message=f"Validation failed: {', '.join(issues)}",
            confidence_score=0.0,
            suggestions=self._get_validation_suggestions(issues)
        )

    def handle_confidence_error(self, command_text: str, confidence: float) -> VoiceCommandResult:
        """
        Handle low confidence errors

        Args:
            command_text: Command text with low confidence
            confidence: Confidence score

        Returns:
            VoiceCommandResult with error information
        """
        self.logger.warning(f"Low confidence for command '{command_text}': {confidence:.2f}")

        return VoiceCommandResult(
            success=False,
            command_text=command_text,
            error_type=ErrorType.CONFIDENCE_ERROR,
            error_message=f"Confidence too low: {confidence:.2f}",
            confidence_score=confidence,
            suggestions=self._get_confidence_suggestions(command_text)
        )

    def _get_asr_error_suggestions(self, error: Exception) -> List[str]:
        """Get suggestions for ASR errors"""
        error_str = str(error).lower()
        suggestions = []

        if "timeout" in error_str:
            suggestions.append("Try speaking more clearly and closer to the microphone")
        elif "audio" in error_str:
            suggestions.append("Check microphone connection and audio settings")
        elif "model" in error_str:
            suggestions.append("Verify ASR model is properly loaded")
        else:
            suggestions.append("Ensure good audio quality and clear speech")

        return suggestions

    def _get_parsing_error_suggestions(self, command_text: str) -> List[str]:
        """Get suggestions for parsing errors"""
        suggestions = [
            "Use simpler, more direct commands",
            "Be more specific about objects and locations",
            "Try breaking complex commands into smaller parts"
        ]

        # Add context-specific suggestions
        if len(command_text.split()) > 10:
            suggestions.append("Try shortening your command")

        if any(word in command_text.lower() for word in ['it', 'that', 'this']):
            suggestions.append("Be more specific about which object you mean")

        return suggestions

    def _get_validation_suggestions(self, issues: List[str]) -> List[str]:
        """Get suggestions for validation errors"""
        suggestions = []

        for issue in issues:
            if "generic" in issue.lower() or "specific" in issue.lower():
                suggestions.append("Be more specific about objects and locations")
            elif "action" in issue.lower():
                suggestions.append("Use supported action words like 'move', 'pick', 'place', 'stop'")
            elif "confidence" in issue.lower():
                suggestions.append("Speak more clearly or repeat the command")

        if not suggestions:
            suggestions.append("Try rephrasing your command using simpler words")

        return suggestions

    def _get_confidence_suggestions(self, command_text: str) -> List[str]:
        """Get suggestions for low confidence"""
        suggestions = [
            "Speak more clearly and at a moderate pace",
            "Reduce background noise",
            "Move closer to the microphone",
            "Repeat the command"
        ]

        # Add command-specific suggestions
        if len(command_text.split()) > 8:
            suggestions.append("Try using shorter, simpler commands")

        return suggestions

    def should_retry_command(self, result: VoiceCommandResult, max_retries: int = 3) -> bool:
        """
        Determine if command should be retried

        Args:
            result: Current result
            max_retries: Maximum number of retries allowed

        Returns:
            True if command should be retried
        """
        # Don't retry on certain error types
        non_retryable_errors = [
            ErrorType.VALIDATION_ERROR,  # Validation errors likely to persist
        ]

        if result.error_type in non_retryable_errors:
            return False

        # Retry on ASR, audio, or confidence errors
        retryable_errors = [
            ErrorType.ASR_ERROR,
            ErrorType.AUDIO_ERROR,
            ErrorType.CONFIDENCE_ERROR,
            ErrorType.TIMEOUT_ERROR
        ]

        return (result.error_type in retryable_errors and
                result.retry_count < max_retries)

    def get_error_recovery_suggestions(self, result: VoiceCommandResult) -> List[str]:
        """
        Get suggestions for recovering from an error

        Args:
            result: The error result

        Returns:
            List of recovery suggestions
        """
        suggestions = []

        if result.error_type == ErrorType.ASR_ERROR:
            suggestions.extend([
                "Check audio input device",
                "Ensure clear audio environment",
                "Speak at consistent volume"
            ])
        elif result.error_type == ErrorType.CONFIDENCE_ERROR:
            suggestions.extend([
                "Speak more clearly",
                "Reduce background noise",
                "Move closer to microphone"
            ])
        elif result.error_type == ErrorType.PARSING_ERROR:
            suggestions.extend([
                "Use simpler command structure",
                "Specify exact objects and locations",
                "Try alternative phrasing"
            ])
        elif result.error_type == ErrorType.TIMEOUT_ERROR:
            suggestions.extend([
                "Speak more quickly after activation",
                "Check that the system is listening"
            ])
        elif result.error_type == ErrorType.AUDIO_ERROR:
            suggestions.extend([
                "Verify microphone is working",
                "Check audio input settings",
                "Ensure proper sample rate (16kHz)"
            ])

        # Add general suggestions
        if not suggestions:
            suggestions.append("Try repeating the command")
            suggestions.append("Use the system's supported command format")

        return suggestions


class VoiceCommandProcessor:
    """
    Main processor that combines error handling and confidence scoring
    """

    def __init__(self):
        self.error_handler = VoiceCommandErrorHandler()
        self.confidence_scorer = ConfidenceScorer()

    def process_voice_command(
        self,
        audio_data: np.ndarray,
        asr_function: Callable,
        command_parser: Any,
        asr_confidence: float = 0.8,
        retry_on_error: bool = True,
        max_retries: int = 3
    ) -> VoiceCommandResult:
        """
        Process a voice command with comprehensive error handling and confidence scoring

        Args:
            audio_data: Audio data to process
            asr_function: Function to perform ASR
            command_parser: Parser to parse the command
            asr_confidence: Confidence from ASR system
            retry_on_error: Whether to retry on certain errors
            max_retries: Maximum number of retries

        Returns:
            VoiceCommandResult with processing outcome
        """
        start_time = time.time()
        retry_count = 0

        while True:
            try:
                # Perform ASR
                transcription_result = asr_function(audio_data)
                command_text = transcription_result.text if hasattr(transcription_result, 'text') else transcription_result

                # Handle ASR errors
                if hasattr(transcription_result, 'success') and not transcription_result.success:
                    result = self.error_handler.handle_asr_error(
                        Exception(transcription_result.error_message),
                        audio_data
                    )
                    result.processing_time = time.time() - start_time
                    return result

                # Parse the command
                try:
                    parsed_command = command_parser.parse_command(command_text)
                except Exception as e:
                    result = self.error_handler.handle_parsing_error(command_text, e)
                    result.processing_time = time.time() - start_time
                    return result

                # Validate the parsed command
                is_valid, validation_issues = command_parser.validate_command(parsed_command)
                if not is_valid:
                    result = self.error_handler.handle_validation_error(parsed_command, validation_issues)
                    result.processing_time = time.time() - start_time
                    return result

                # Calculate overall confidence
                parsing_confidence = self.confidence_scorer.calculate_parsing_confidence(parsed_command)
                context_confidence = 1.0  # In a real system, this would consider context
                overall_confidence = self.confidence_scorer.calculate_overall_confidence(
                    asr_confidence,
                    parsing_confidence,
                    context_confidence
                )

                # Check if confidence is sufficient
                if not self.confidence_scorer.is_confident_enough(overall_confidence):
                    result = self.error_handler.handle_confidence_error(command_text, overall_confidence)
                    result.retry_count = retry_count

                    if retry_on_error and self.error_handler.should_retry_command(result, max_retries):
                        retry_count += 1
                        continue  # Retry the command
                    else:
                        result.processing_time = time.time() - start_time
                        return result

                # Success case
                result = VoiceCommandResult(
                    success=True,
                    command_text=command_text,
                    parsed_command=parsed_command,
                    confidence_score=overall_confidence,
                    processing_time=time.time() - start_time,
                    retry_count=retry_count
                )
                return result

            except Exception as e:
                # Handle unexpected errors
                result = self.error_handler.handle_asr_error(e, audio_data)
                result.processing_time = time.time() - start_time
                result.retry_count = retry_count

                if retry_on_error and self.error_handler.should_retry_command(result, max_retries):
                    retry_count += 1
                    continue  # Retry the command
                else:
                    return result


# Example usage and testing
def main():
    """
    Example usage of the error handling and confidence scoring
    """
    print("Testing Error Handling and Confidence Scoring...")

    # Initialize components
    processor = VoiceCommandProcessor()
    scorer = ConfidenceScorer()

    # Test confidence calculations
    print("\nTesting confidence calculations:")
    asr_conf = 0.8
    parsing_conf = 0.7
    context_conf = 0.9

    overall_conf = scorer.calculate_overall_confidence(asr_conf, parsing_conf, context_conf)
    print(f"  ASR: {asr_conf}, Parsing: {parsing_conf}, Context: {context_conf}")
    print(f"  Overall: {overall_conf:.2f}")
    print(f"  Is confident enough: {scorer.is_confident_enough(overall_conf)}")
    print(f"  Confidence level: {scorer.get_confidence_level(overall_conf)}")

    # Test error handling
    print("\nTesting error handling:")
    error_handler = VoiceCommandErrorHandler()

    # Simulate different types of errors
    asr_error_result = error_handler.handle_asr_error(Exception("ASR timeout"), np.array([1, 2, 3]))
    print(f"  ASR Error: {asr_error_result.error_type}, Suggestions: {asr_error_result.suggestions}")

    validation_result = error_handler.handle_validation_error(
        type('obj', (object,), {'raw_text': 'test command'})(),
        ["Command is too generic"]
    )
    print(f"  Validation Error Suggestions: {validation_result.suggestions}")

    confidence_result = error_handler.handle_confidence_error("Move forward", 0.3)
    print(f"  Confidence Error Suggestions: {confidence_result.suggestions}")

    # Test retry logic
    should_retry = error_handler.should_retry_command(asr_error_result)
    print(f"  Should retry: {should_retry}")

    print("\nError handling and confidence scoring test completed successfully!")


if __name__ == "__main__":
    main()