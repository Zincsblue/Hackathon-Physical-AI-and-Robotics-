"""
VLA Orchestrator - Main Integration Component

This module orchestrates the complete Vision-Language-Action pipeline,
coordinating between perception, language understanding, planning, and action execution.
"""

from typing import Dict, Any, Optional, List, Callable
import time
import asyncio
from dataclasses import dataclass
import threading
from enum import Enum
import logging
import traceback
from datetime import datetime

# Import monitoring module
from .monitoring import get_monitoring_system, LogLevel


class VLAState(Enum):
    """Current state of the VLA system"""
    IDLE = "idle"
    LISTENING = "listening"
    UNDERSTANDING = "understanding"
    PLANNING = "planning"
    EXECUTING = "executing"
    ERROR = "error"
    COMPLETED = "completed"


class VLAErrorType(Enum):
    """Types of errors that can occur in the VLA system"""
    INITIALIZATION_ERROR = "initialization_error"
    LANGUAGE_PROCESSING_ERROR = "language_processing_error"
    VISION_PROCESSING_ERROR = "vision_processing_error"
    PLANNING_ERROR = "planning_error"
    EXECUTION_ERROR = "execution_error"
    COMMUNICATION_ERROR = "communication_error"
    RESOURCE_ERROR = "resource_error"
    VALIDATION_ERROR = "validation_error"
    TIMEOUT_ERROR = "timeout_error"
    UNKNOWN_ERROR = "unknown_error"


@dataclass
class VLAErrorInfo:
    """Detailed information about an error in the VLA system"""
    error_type: VLAErrorType
    error_message: str
    module_name: str
    timestamp: datetime
    traceback_info: Optional[str] = None
    severity: str = "medium"  # low, medium, high, critical


@dataclass
class VLAExecutionResult:
    """Result of VLA system execution"""
    success: bool
    final_state: VLAState
    execution_time: float
    steps_completed: List[str]
    error_message: Optional[str] = None
    action_results: Optional[List[Dict[str, Any]]] = None
    error_info: Optional[VLAErrorInfo] = None
    warnings: List[str] = None

    def __post_init__(self):
        if self.warnings is None:
            self.warnings = []


class VLAModuleInterface:
    """Interface for VLA system modules with comprehensive error handling"""

    def __init__(self, name: str):
        self.name = name
        self.is_initialized = False
        self.logger = logging.getLogger(f"VLA.{name}")
        self.error_count = 0
        self.last_error = None

    def initialize(self) -> bool:
        """Initialize the module with error handling"""
        try:
            self.logger.info(f"Initializing module: {self.name}")
            self.is_initialized = True
            self.logger.info(f"Module {self.name} initialized successfully")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize module {self.name}: {str(e)}")
            self.error_count += 1
            self.last_error = str(e)
            return False

    def process(self, input_data: Dict[str, Any]) -> Dict[str, Any]:
        """Process input data and return results with error handling"""
        if not self.is_initialized:
            error_msg = f"Module {self.name} not initialized"
            self.logger.error(error_msg)
            raise RuntimeError(error_msg)

        try:
            self.logger.debug(f"Processing input in {self.name} module")
            result = self._process_internal(input_data)
            self.logger.debug(f"Processing completed in {self.name} module")
            return result
        except Exception as e:
            self.error_count += 1
            self.last_error = str(e)
            error_msg = f"Error in {self.name} module: {str(e)}"
            self.logger.error(error_msg)
            self.logger.debug(f"Error traceback: {traceback.format_exc()}")
            raise

    def _process_internal(self, input_data: Dict[str, Any]) -> Dict[str, Any]:
        """Internal processing method to be overridden by subclasses"""
        raise NotImplementedError

    def validate_input(self, input_data: Dict[str, Any]) -> bool:
        """Validate input data before processing"""
        return True

    def shutdown(self):
        """Clean up module resources"""
        try:
            self.logger.info(f"Shutting down module: {self.name}")
            self.is_initialized = False
            self.logger.info(f"Module {self.name} shut down successfully")
        except Exception as e:
            self.logger.error(f"Error during shutdown of {self.name}: {str(e)}")


class LanguageUnderstandingModule(VLAModuleInterface):
    """Module for language understanding and command parsing"""

    def __init__(self):
        super().__init__("LanguageUnderstanding")
        self.command_keywords = {
            "navigation": ["go to", "move to", "navigate", "walk to"],
            "manipulation": ["pick up", "grasp", "take", "lift", "place", "put", "drop"],
            "perception": ["find", "detect", "see", "look for", "identify"],
            "communication": ["speak", "say", "tell", "announce"]
        }

    def _process_internal(self, input_data: Dict[str, Any]) -> Dict[str, Any]:
        """Process natural language command with comprehensive error handling"""
        try:
            # Validate input
            if not self.validate_input(input_data):
                raise ValueError("Invalid input for LanguageUnderstandingModule")

            command = input_data.get("command", "")
            if not isinstance(command, str):
                raise ValueError(f"Command must be a string, got {type(command)}")

            if not command.strip():
                raise ValueError("Command cannot be empty")

            command_lower = command.lower()

            # Identify command type
            command_type = "unknown"
            for cmd_type, keywords in self.command_keywords.items():
                if any(keyword in command_lower for keyword in keywords):
                    command_type = cmd_type
                    break

            # Extract entities (objects, locations, etc.)
            entities = self._extract_entities(command)

            result = {
                "command_type": command_type,
                "entities": entities,
                "original_command": command,
                "parsed_command": self._parse_command(command, entities)
            }

            self.logger.info(f"Parsed command: {command_type} - {command}")
            return result

        except ValueError as ve:
            self.logger.error(f"Input validation error in LanguageUnderstandingModule: {str(ve)}")
            raise
        except Exception as e:
            self.logger.error(f"Unexpected error in LanguageUnderstandingModule: {str(e)}")
            raise

    def validate_input(self, input_data: Dict[str, Any]) -> bool:
        """Validate input for language understanding module"""
        if not isinstance(input_data, dict):
            return False
        if "command" not in input_data:
            return False
        return True

    def _extract_entities(self, command: str) -> Dict[str, Any]:
        """Extract entities from command"""
        # Simple entity extraction - in reality this would use NLP techniques
        entities = {
            "objects": [],
            "locations": [],
            "attributes": []
        }

        # Look for object references
        object_indicators = ["the", "a", "an"]
        words = command.lower().split()

        for i, word in enumerate(words):
            if word in object_indicators and i + 1 < len(words):
                next_word = words[i + 1]
                # Check if next word is likely an object
                if next_word not in ["to", "and", "or", "with", "for", "at", "in", "on"]:
                    entities["objects"].append(next_word)

        # Look for location references
        location_indicators = ["to", "at", "in", "on", "by", "near"]
        for i, word in enumerate(words):
            if word in location_indicators and i + 1 < len(words):
                next_word = words[i + 1]
                entities["locations"].append(next_word)

        return entities

    def _parse_command(self, command: str, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Parse command into structured format"""
        return {
            "action": self._determine_action(command),
            "target_objects": entities["objects"],
            "target_locations": entities["locations"],
            "context": entities["attributes"]
        }

    def _determine_action(self, command: str) -> str:
        """Determine the main action from the command"""
        command_lower = command.lower()

        if any(keyword in command_lower for keyword in self.command_keywords["navigation"]):
            return "navigate"
        elif any(keyword in command_lower for keyword in self.command_keywords["manipulation"]):
            return "manipulate"
        elif any(keyword in command_lower for keyword in self.command_keywords["perception"]):
            return "perceive"
        elif any(keyword in command_lower for keyword in self.command_keywords["communication"]):
            return "communicate"
        else:
            return "unknown"


class VisionProcessingModule(VLAModuleInterface):
    """Module for vision processing and scene understanding"""

    def __init__(self):
        super().__init__("VisionProcessing")
        self.is_available = False
        self.vision_processor = None

        # Import vision processor if available
        try:
            from .vision_processor import EnhancedVisionProcessor
            self.vision_processor = EnhancedVisionProcessor()
            self.is_available = True
            self.logger.info("Vision processor initialized successfully")
        except ImportError as e:
            self.logger.warning(f"Vision processor not available: {e}")
            self.vision_processor = None
            self.is_available = False
        except Exception as e:
            self.logger.error(f"Error initializing vision processor: {e}")
            self.vision_processor = None
            self.is_available = False

    def _process_internal(self, input_data: Dict[str, Any]) -> Dict[str, Any]:
        """Process visual information with comprehensive error handling"""
        try:
            if not self.validate_input(input_data):
                raise ValueError("Invalid input for VisionProcessingModule")

            self.logger.info("Processing visual information")

            if not self.is_available:
                # Return simulated results
                self.logger.warning("Vision processor not available, returning simulated results")
                return {
                    "objects_in_scene": [
                        {"name": "red cup", "location": [1.0, 2.0, 0.5], "confidence": 0.85},
                        {"name": "blue book", "location": [1.5, 2.0, 0.5], "confidence": 0.80}
                    ],
                    "spatial_relationships": [
                        {"object1": "red cup", "relationship": "left of", "object2": "blue book", "confidence": 0.75}
                    ],
                    "scene_description": "A red cup is on the table to the left of a blue book"
                }

            # Use actual vision processor
            image_data = input_data.get("image_data")
            search_query = input_data.get("search_query")

            if search_query:
                detection_result = self.vision_processor.detect_objects(image_data, search_query)
                scene_result = self.vision_processor.understand_scene(image_data)
            else:
                scene_result = self.vision_processor.understand_scene(image_data)
                detection_result = self.vision_processor.detect_objects(image_data)

            result = {
                "objects_in_scene": detection_result.objects if detection_result.success else [],
                "spatial_relationships": scene_result.spatial_relationships,
                "scene_description": scene_result.scene_description,
                "confidence": scene_result.confidence
            }

            self.logger.info(f"Vision processing completed, found {len(result['objects_in_scene'])} objects")
            return result

        except Exception as e:
            self.logger.error(f"Error in VisionProcessingModule: {str(e)}")
            # Return simulated results as fallback
            self.logger.warning("Returning simulated results as fallback")
            return {
                "objects_in_scene": [
                    {"name": "red cup", "location": [1.0, 2.0, 0.5], "confidence": 0.85},
                    {"name": "blue book", "location": [1.5, 2.0, 0.5], "confidence": 0.80}
                ],
                "spatial_relationships": [
                    {"object1": "red cup", "relationship": "left of", "object2": "blue book", "confidence": 0.75}
                ],
                "scene_description": "A red cup is on the table to the left of a blue book"
            }

    def validate_input(self, input_data: Dict[str, Any]) -> bool:
        """Validate input for vision processing module"""
        if not isinstance(input_data, dict):
            return False
        # image_data and search_query are optional, so no strict validation needed
        return True


class PlanningModule(VLAModuleInterface):
    """Module for task planning and decomposition"""

    def __init__(self):
        super().__init__("Planning")
        self.is_available = False
        self.llm_planner = None
        self.htn_planner = None

        # Import planning components if available
        try:
            from .llm_planner import EnhancedLLMPlanner
            from .hierarchical_task_network import EnhancedHTNPlanner
            self.llm_planner = EnhancedLLMPlanner()
            self.htn_planner = EnhancedHTNPlanner()
            self.is_available = True
            self.logger.info("Planning components initialized successfully")
        except ImportError as e:
            self.logger.warning(f"Planning components not available: {e}")
            self.llm_planner = None
            self.htn_planner = None
            self.is_available = False
        except Exception as e:
            self.logger.error(f"Error initializing planning components: {e}")
            self.llm_planner = None
            self.htn_planner = None
            self.is_available = False

    def _process_internal(self, input_data: Dict[str, Any]) -> Dict[str, Any]:
        """Generate plan from command and context with comprehensive error handling"""
        try:
            if not self.validate_input(input_data):
                raise ValueError("Invalid input for PlanningModule")

            command = input_data.get("command", "")
            context = input_data.get("context", {})

            self.logger.info(f"Planning for command: {command}")

            if not self.is_available:
                # Return simulated plan
                self.logger.warning("Planning components not available, returning simulated plan")
                return {
                    "action_sequence": [
                        {"action": "navigate_to_location", "parameters": {"location": [1.0, 2.0, 0.0]}},
                        {"action": "detect_object", "parameters": {"object_type": "target"}},
                        {"action": "grasp_object", "parameters": {"object_id": "target_1"}}
                    ],
                    "reasoning": "Simulated plan for demonstration",
                    "confidence": 0.7
                }

            # Use actual planners
            # First try LLM-based planning
            if self.llm_planner:
                try:
                    plan_result = self.llm_planner.plan_task(command, context)
                    if plan_result.success:
                        task_plan = self.llm_planner.create_task_plan(command, context)
                        if task_plan:
                            result = {
                                "action_sequence": task_plan.action_sequence,
                                "reasoning": task_plan.reasoning,
                                "confidence": task_plan.confidence
                            }
                            self.logger.info(f"LLM planning successful, generated {len(result['action_sequence'])} actions")
                            return result
                except Exception as e:
                    self.logger.warning(f"LLM planning failed, falling back to HTN: {e}")

            # Fallback to HTN planning
            if self.htn_planner:
                try:
                    from .hierarchical_task_network import Task
                    root_task = Task(
                        name="complex_task",
                        method="compound",
                        parameters={"command": command}
                    )
                    htn_plan = self.htn_planner.decompose_task(root_task, context)
                    result = {
                        "action_sequence": self._htn_to_action_sequence(htn_plan.root_task),
                        "reasoning": "HTN-based decomposition",
                        "confidence": 0.8
                    }
                    self.logger.info(f"HTN planning successful, generated {len(result['action_sequence'])} actions")
                    return result
                except Exception as e:
                    self.logger.warning(f"HTN planning failed: {e}")

            # Ultimate fallback
            self.logger.warning("All planning methods failed, returning empty plan")
            return {
                "action_sequence": [],
                "reasoning": "No planner available",
                "confidence": 0.0
            }

        except Exception as e:
            self.logger.error(f"Error in PlanningModule: {str(e)}")
            # Return fallback plan as safety measure
            return {
                "action_sequence": [],
                "reasoning": f"Fallback plan due to error: {str(e)}",
                "confidence": 0.0
            }

    def validate_input(self, input_data: Dict[str, Any]) -> bool:
        """Validate input for planning module"""
        if not isinstance(input_data, dict):
            return False
        if "command" not in input_data:
            return False
        return True

    def _htn_to_action_sequence(self, task) -> List[Dict[str, Any]]:
        """Convert HTN task to action sequence"""
        actions = []

        if task.method == 'primitive':
            actions.append({
                "action": task.name,
                "parameters": task.parameters,
                "reasoning": "Primitive action from HTN"
            })
        elif task.subtasks:
            for subtask in task.subtasks:
                actions.extend(self._htn_to_action_sequence(subtask))

        return actions


class ActionExecutionModule(VLAModuleInterface):
    """Module for action execution and ROS 2 interfacing"""

    def __init__(self):
        super().__init__("ActionExecution")
        self.is_action_mapping_available = False
        self.is_action_execution_available = False
        self.action_mapper = None
        self.action_executor = None

        # Import action components if available
        try:
            from .action_mapper import EnhancedActionMapper
            self.action_mapper = EnhancedActionMapper()
            self.is_action_mapping_available = True
            self.logger.info("Action mapper initialized successfully")
        except ImportError as e:
            self.logger.warning(f"Action mapper not available: {e}")
            self.action_mapper = None
            self.is_action_mapping_available = False
        except Exception as e:
            self.logger.error(f"Error initializing action mapper: {e}")
            self.action_mapper = None
            self.is_action_mapping_available = False

        try:
            from .ros2_action_clients import VLAActionExecutor
            self.action_executor = None  # Will be initialized with ROS 2 node
            self.is_action_execution_available = True
            self.logger.info("ROS 2 action executor interface available")
        except ImportError as e:
            self.logger.warning(f"ROS 2 action executor not available: {e}")
            self.action_executor = None
            self.is_action_execution_available = False
        except Exception as e:
            self.logger.error(f"Error initializing ROS 2 action executor: {e}")
            self.action_executor = None
            self.is_action_execution_available = False

    def _process_internal(self, input_data: Dict[str, Any]) -> Dict[str, Any]:
        """Execute action sequence with comprehensive error handling"""
        try:
            if not self.validate_input(input_data):
                raise ValueError("Invalid input for ActionExecutionModule")

            action_sequence = input_data.get("action_sequence", [])
            context = input_data.get("context", {})

            self.logger.info(f"Executing action sequence with {len(action_sequence)} actions")

            if not self.is_action_mapping_available:
                # Return simulated execution results
                self.logger.warning("Action mapper not available, returning simulated execution results")
                results = []
                for i, action in enumerate(action_sequence):
                    results.append({
                        "action_index": i,
                        "action": action,
                        "success": True,
                        "execution_time": 0.5,
                        "result": f"Simulated execution of {action.get('action', 'unknown')}"
                    })
                return {
                    "execution_results": results,
                    "all_successful": True,
                    "total_execution_time": len(action_sequence) * 0.5
                }

            # Map and execute actions
            mapped_actions = []
            execution_results = []

            for i, action in enumerate(action_sequence):
                if self.action_mapper:
                    try:
                        mapping_result = self.action_mapper.map_action(
                            action.get('action', ''),
                            action.get('parameters', {})
                        )
                        if mapping_result.success:
                            mapped_actions.extend(mapping_result.ros2_actions)
                        else:
                            execution_results.append({
                                "action_index": i,
                                "action": action,
                                "success": False,
                                "error": mapping_result.error_message
                            })
                            continue
                    except Exception as e:
                        self.logger.error(f"Action mapping failed for action {i}: {str(e)}")
                        execution_results.append({
                            "action_index": i,
                            "action": action,
                            "success": False,
                            "error": f"Mapping error: {str(e)}"
                        })
                        continue
                else:
                    # If no action mapper, just pass through the action
                    mapped_actions.append(action)

            # Execute mapped actions (simulated if ROS 2 not available)
            total_time = 0
            all_successful = True

            for i, mapped_action in enumerate(mapped_actions):
                try:
                    start_time = time.time()
                    # Simulate execution
                    execution_result = {
                        "action_index": i,
                        "action": mapped_action,
                        "success": True,
                        "execution_time": time.time() - start_time,
                        "result": f"Executed {mapped_action.get('action_name', 'unknown')}"
                    }
                    execution_results.append(execution_result)
                    total_time += time.time() - start_time
                except Exception as e:
                    self.logger.error(f"Action execution failed for action {i}: {str(e)}")
                    execution_results.append({
                        "action_index": i,
                        "action": mapped_action,
                        "success": False,
                        "execution_time": time.time() - start_time,
                        "error": f"Execution error: {str(e)}"
                    })
                    all_successful = False

            result = {
                "execution_results": execution_results,
                "all_successful": all(result.get('success', False) for result in execution_results),
                "total_execution_time": total_time
            }

            success_count = sum(1 for r in execution_results if r.get('success', False))
            self.logger.info(f"Action execution completed: {success_count}/{len(execution_results)} actions successful")
            return result

        except Exception as e:
            self.logger.error(f"Critical error in ActionExecutionModule: {str(e)}")
            # Return fallback results as safety measure
            return {
                "execution_results": [],
                "all_successful": False,
                "total_execution_time": 0.0,
                "error": f"Critical execution error: {str(e)}"
            }

    def validate_input(self, input_data: Dict[str, Any]) -> bool:
        """Validate input for action execution module"""
        if not isinstance(input_data, dict):
            return False
        if "action_sequence" not in input_data:
            return False
        if not isinstance(input_data["action_sequence"], list):
            return False
        return True


class VLASystemOrchestrator:
    """
    Main orchestrator for the complete VLA system
    Coordinates all modules and manages the end-to-end flow
    """

    def __init__(self):
        """Initialize the VLA orchestrator with comprehensive error handling and monitoring"""
        # Initialize monitoring system
        self.monitor = get_monitoring_system()

        # Set up logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger("VLA.Orchestrator")

        self.state = VLAState.IDLE
        self.modules = {}

        # Initialize modules with error handling
        module_classes = {
            "language": LanguageUnderstandingModule,
            "vision": VisionProcessingModule,
            "planning": PlanningModule,
            "execution": ActionExecutionModule
        }

        for name, module_class in module_classes.items():
            try:
                module = module_class()
                if module.initialize():
                    self.modules[name] = module
                    self.logger.info(f"Module {name} initialized successfully")
                    self.monitor.check_module_health(name, True)
                else:
                    self.logger.error(f"Module {name} failed to initialize properly")
                    self.monitor.check_module_health(name, False, {"error": "Initialization failed"})
                    # Create a fallback module or continue without it
                    self.modules[name] = module  # Still add it but it will fail on processing
            except Exception as e:
                self.logger.error(f"Could not create {name} module: {e}")
                self.monitor.check_module_health(name, False, {"error": str(e), "traceback": traceback.format_exc()})
                # Create a basic module as fallback
                self.modules[name] = module_class()
                # Attempt to initialize again, may fail but we'll know why
                try:
                    self.modules[name].initialize()
                except Exception as init_error:
                    self.logger.error(f"Module {name} cannot be initialized even with fallback: {init_error}")

        self.execution_history = []
        self.performance_metrics = {
            "total_executions": 0,
            "success_rate": 0.0,
            "avg_execution_time": 0.0,
            "module_times": {},
            "error_counts": {name: 0 for name in self.modules.keys()},
            "last_error": {name: None for name in self.modules.keys()}
        }

        self.logger.info("VLA Orchestrator initialized successfully")
        self.monitor.log_event(LogLevel.INFO, "Orchestrator", "VLA Orchestrator initialized successfully")

    def process_command(self, command: str, context: Optional[Dict[str, Any]] = None) -> VLAExecutionResult:
        """
        Process a complete VLA command from start to finish with comprehensive error handling and monitoring

        Args:
            command: Natural language command to process
            context: Optional context information

        Returns:
            VLAExecutionResult with complete execution information
        """
        start_time = time.time()
        steps_completed = []
        action_results = []
        error_info = None
        warnings = []

        try:
            self.logger.info(f"Processing command: {command}")
            self.monitor.log_event(LogLevel.INFO, "Orchestrator", f"Processing command: {command}")
            self.state = VLAState.LISTENING
            steps_completed.append("command_received")

            # Validate inputs
            if not command or not isinstance(command, str):
                raise ValueError("Command must be a non-empty string")

            # Step 1: Language Understanding
            self.state = VLAState.UNDERSTANDING
            lang_input = {"command": command}
            try:
                lang_result = self.modules["language"].process(lang_input)
                steps_completed.append("language_understanding")
                self.logger.info(f"Language understanding result: {lang_result['command_type']} command")
                self.monitor.log_event(LogLevel.INFO, "LanguageModule", f"Processed command type: {lang_result['command_type']}")
            except Exception as e:
                error_type = VLAErrorType.LANGUAGE_PROCESSING_ERROR
                error_msg = f"Language processing failed: {str(e)}"
                error_info = VLAErrorInfo(
                    error_type=error_type,
                    error_message=error_msg,
                    module_name="language",
                    timestamp=datetime.now(),
                    traceback_info=traceback.format_exc(),
                    severity="high"
                )
                self.monitor.log_event(LogLevel.ERROR, "LanguageModule", error_msg, {"traceback": str(e)})
                raise

            # Step 2: Vision Processing (if needed)
            self.state = VLAState.UNDERSTANDING
            vision_input = {"search_query": " ".join(lang_result.get("entities", {}).get("objects", []))}
            try:
                vision_result = self.modules["vision"].process(vision_input)
                steps_completed.append("vision_processing")
                self.logger.info(f"Vision processing found {len(vision_result['objects_in_scene'])} objects")
                self.monitor.log_event(LogLevel.INFO, "VisionModule", f"Found {len(vision_result['objects_in_scene'])} objects")
            except Exception as e:
                # Vision processing is not critical, add as warning
                warning_msg = f"Vision processing failed: {str(e)}"
                warnings.append(warning_msg)
                self.logger.warning(warning_msg)
                self.monitor.log_event(LogLevel.WARNING, "VisionModule", warning_msg, {"traceback": str(e)})
                # Provide default vision result
                vision_result = {
                    "objects_in_scene": [],
                    "spatial_relationships": [],
                    "scene_description": "No visual information available"
                }

            # Step 3: Planning
            self.state = VLAState.PLANNING
            planning_context = {**context} if context else {}
            planning_context.update({
                "objects_in_scene": vision_result["objects_in_scene"],
                "command_info": lang_result
            })
            planning_input = {
                "command": command,
                "context": planning_context
            }
            try:
                planning_result = self.modules["planning"].process(planning_input)
                steps_completed.append("planning")
                self.logger.info(f"Planning generated {len(planning_result['action_sequence'])} actions")
                self.monitor.log_event(LogLevel.INFO, "PlanningModule", f"Generated {len(planning_result['action_sequence'])} actions")
            except Exception as e:
                error_type = VLAErrorType.PLANNING_ERROR
                error_msg = f"Planning failed: {str(e)}"
                error_info = VLAErrorInfo(
                    error_type=error_type,
                    error_message=error_msg,
                    module_name="planning",
                    timestamp=datetime.now(),
                    traceback_info=traceback.format_exc(),
                    severity="high"
                )
                self.monitor.log_event(LogLevel.ERROR, "PlanningModule", error_msg, {"traceback": str(e)})
                raise

            # Step 4: Action Execution
            self.state = VLAState.EXECUTING
            execution_input = {
                "action_sequence": planning_result["action_sequence"],
                "context": planning_context
            }
            try:
                execution_result = self.modules["execution"].process(execution_input)
                action_results = execution_result.get("execution_results", [])
                steps_completed.append("execution")
                self.logger.info(f"Execution completed with {len(action_results)} actions")
                self.monitor.log_event(LogLevel.INFO, "ExecutionModule", f"Executed {len(action_results)} actions")
            except Exception as e:
                error_type = VLAErrorType.EXECUTION_ERROR
                error_msg = f"Action execution failed: {str(e)}"
                error_info = VLAErrorInfo(
                    error_type=error_type,
                    error_message=error_msg,
                    module_name="execution",
                    timestamp=datetime.now(),
                    traceback_info=traceback.format_exc(),
                    severity="high"
                )
                self.monitor.log_event(LogLevel.ERROR, "ExecutionModule", error_msg, {"traceback": str(e)})
                raise

            # Update state to completed
            self.state = VLAState.COMPLETED

            # Update performance metrics
            execution_time = time.time() - start_time
            self._update_performance_metrics(True, execution_time)
            self.monitor.record_command_execution(execution_time, True)

            # Record execution
            self.execution_history.append({
                "command": command,
                "start_time": start_time,
                "end_time": time.time(),
                "success": True,
                "execution_time": execution_time,
                "steps_completed": steps_completed.copy(),
                "error_info": None
            })

            self.logger.info(f"Command processed successfully in {execution_time:.2f}s")
            self.monitor.log_event(LogLevel.INFO, "Orchestrator", f"Command processed successfully in {execution_time:.2f}s")
            return VLAExecutionResult(
                success=True,
                final_state=self.state,
                execution_time=execution_time,
                steps_completed=steps_completed,
                action_results=action_results,
                warnings=warnings
            )

        except Exception as e:
            self.state = VLAState.ERROR
            execution_time = time.time() - start_time

            error_msg = f"VLA execution failed: {str(e)}"
            self.logger.error(error_msg)
            self.monitor.log_event(LogLevel.ERROR, "Orchestrator", error_msg, {"traceback": str(e)})
            self.monitor.record_command_execution(execution_time, False)

            if error_info is None:  # If error_info wasn't set in specific steps
                error_info = VLAErrorInfo(
                    error_type=VLAErrorType.UNKNOWN_ERROR,
                    error_message=error_msg,
                    module_name="orchestrator",
                    timestamp=datetime.now(),
                    traceback_info=traceback.format_exc(),
                    severity="critical"
                )

            # Update performance metrics
            self._update_performance_metrics(False, execution_time)

            # Record execution with error
            self.execution_history.append({
                "command": command,
                "start_time": start_time,
                "end_time": time.time(),
                "success": False,
                "execution_time": execution_time,
                "steps_completed": steps_completed.copy(),
                "error_info": error_info
            })

            return VLAExecutionResult(
                success=False,
                final_state=self.state,
                execution_time=execution_time,
                steps_completed=steps_completed,
                error_message=error_msg,
                action_results=action_results,
                error_info=error_info,
                warnings=warnings
            )

    def _update_performance_metrics(self, success: bool, execution_time: float):
        """Update performance metrics with error tracking"""
        self.performance_metrics["total_executions"] += 1

        # Update success rate
        total = self.performance_metrics["total_executions"]
        successful = sum(1 for record in self.execution_history if record.get("success", False))
        self.performance_metrics["success_rate"] = successful / total if total > 0 else 0.0

        # Update average execution time
        times = [record["execution_time"] for record in self.execution_history]
        self.performance_metrics["avg_execution_time"] = sum(times) / len(times) if times else 0.0

        # Log performance metrics periodically
        if total % 10 == 0:  # Log every 10 executions
            self.logger.info(
                f"Performance Metrics - Executions: {total}, "
                f"Success Rate: {self.performance_metrics['success_rate']:.2f}, "
                f"Avg Time: {self.performance_metrics['avg_execution_time']:.2f}s"
            )

    def get_system_status(self) -> Dict[str, Any]:
        """Get current system status with monitoring information"""
        # Get monitoring system metrics
        system_metrics = self.monitor.get_system_metrics()

        return {
            "state": self.state.value,
            "active_modules": [name for name, module in self.modules.items() if module.is_initialized],
            "performance_metrics": self.performance_metrics.copy(),
            "execution_count": len(self.execution_history),
            "recent_executions": self.execution_history[-5:],  # Last 5 executions
            "monitoring": {
                "system_health": system_metrics["health"],
                "performance_summary": system_metrics["performance"],
                "recent_logs": system_metrics["recent_logs"]
            }
        }

    def reset_system(self):
        """Reset the VLA system to initial state"""
        self.state = VLAState.IDLE
        self.execution_history = []

    def add_error_recovery_handler(self, error_type: str, handler: Callable):
        """Add an error recovery handler"""
        # This would be implemented in a more complex system
        pass

    def visualize_data_flow(self) -> str:
        """Generate a text-based visualization of the data flow"""
        flow_diagram = """
VLA System Data Flow:
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐    ┌──────────────────┐
│  Language       │───▶│  Vision          │───▶│  Planning       │───▶│  Action          │
│  Understanding  │    │  Processing      │    │  Module         │    │  Execution       │
│                 │    │                  │    │                 │    │                  │
│ • Command       │    │ • Object         │    │ • Task          │    │ • Action         │
│   parsing       │    │   detection      │    │   planning      │    │   mapping       │
│ • Entity        │    │ • Scene          │    │ • Reasoning     │    │ • Execution     │
│   extraction    │    │   understanding  │    │ • Decomposition │    │ • Feedback       │
└─────────────────┘    └──────────────────┘    └─────────────────┘    └──────────────────┘
         │                       │                        │                        │
         ▼                       ▼                        ▼                        ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                   VLA Orchestrator                                      │
│  Coordinates flow between modules, manages state, handles errors, tracks performance    │
└─────────────────────────────────────────────────────────────────────────────────────────┘
        """
        return flow_diagram

    def get_performance_summary(self) -> Dict[str, Any]:
        """Get performance summary"""
        return {
            "total_executions": self.performance_metrics["total_executions"],
            "success_rate": self.performance_metrics["success_rate"],
            "avg_execution_time": self.performance_metrics["avg_execution_time"],
            "recent_success_rate": self._calculate_recent_success_rate(10),
            "module_availability": {
                name: module.is_initialized for name, module in self.modules.items()
            }
        }

    def _calculate_recent_success_rate(self, n: int = 10) -> float:
        """Calculate success rate for the last n executions"""
        recent = self.execution_history[-n:]
        if not recent:
            return 0.0
        successful = sum(1 for record in recent if record.get("success", False))
        return successful / len(recent)


# Example usage and testing
def main():
    """Example usage of the VLA orchestrator"""
    print("Testing VLA Orchestrator...")

    # Initialize orchestrator
    vla_system = VLASystemOrchestrator()

    print(f"System status: {vla_system.get_system_status()['state']}")
    print(f"Active modules: {vla_system.get_system_status()['active_modules']}")

    # Test various commands
    test_commands = [
        "Go to the kitchen",
        "Find the red cup on the table",
        "Pick up the blue book",
        "Navigate to the living room"
    ]

    for i, command in enumerate(test_commands):
        print(f"\n{i+1}. Processing command: '{command}'")

        result = vla_system.process_command(command)

        print(f"   Success: {result.success}")
        print(f"   Execution time: {result.execution_time:.2f}s")
        print(f"   Steps completed: {len(result.steps_completed)}")
        print(f"   Final state: {result.final_state.value}")

        if result.action_results:
            print(f"   Actions executed: {len(result.action_results)}")

    # Show system status after executions
    print(f"\nSystem Status:")
    status = vla_system.get_system_status()
    print(f"  State: {status['state']}")
    print(f"  Total executions: {status['execution_count']}")
    print(f"  Success rate: {status['performance_metrics']['success_rate']:.2f}")
    print(f"  Avg execution time: {status['performance_metrics']['avg_execution_time']:.2f}s")

    # Show performance summary
    print(f"\nPerformance Summary:")
    perf_summary = vla_system.get_performance_summary()
    print(f"  Total executions: {perf_summary['total_executions']}")
    print(f"  Overall success rate: {perf_summary['success_rate']:.2f}")
    print(f"  Avg execution time: {perf_summary['avg_execution_time']:.2f}s")
    print(f"  Recent success rate (last 10): {perf_summary['recent_success_rate']:.2f}")
    print(f"  Module availability: {perf_summary['module_availability']}")

    # Show data flow visualization
    print(f"\nData Flow Visualization:")
    print(vla_system.visualize_data_flow())

    print("\nVLA orchestrator testing completed!")


if __name__ == "__main__":
    main()