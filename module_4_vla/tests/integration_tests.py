"""
Integration Tests for VLA System

This module contains comprehensive integration tests for the Vision-Language-Action system,
testing the end-to-end flow from language command to action execution.
"""

import unittest
import asyncio
from unittest.mock import Mock, patch
import sys
import os

# Add src directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from vla.vla_orchestrator import VLASystemOrchestrator, VLAState
from vla.language_understanding import LanguageUnderstandingModule
from vla.vision_processor import VisionProcessingModule
from vla.planning_module import PlanningModule
from vla.action_execution import ActionExecutionModule


class TestVLAIntegration(unittest.TestCase):
    """Integration tests for the complete VLA system"""

    def setUp(self):
        """Set up the VLA system for testing"""
        self.vla_system = VLASystemOrchestrator()

    def test_complete_vla_flow_navigation_command(self):
        """Test complete flow for a navigation command"""
        command = "Go to the kitchen"

        # Process the command
        result = self.vla_system.process_command(command)

        # Verify the result
        self.assertTrue(result.success)
        self.assertEqual(result.final_state, VLAState.COMPLETED)
        self.assertGreater(result.execution_time, 0)
        self.assertIn("command_received", result.steps_completed)
        self.assertIn("language_understanding", result.steps_completed)
        self.assertIn("planning", result.steps_completed)
        self.assertIn("execution", result.steps_completed)

    def test_complete_vla_flow_manipulation_command(self):
        """Test complete flow for a manipulation command"""
        command = "Pick up the red cup"

        # Process the command
        result = self.vla_system.process_command(command)

        # Verify the result
        self.assertTrue(result.success)
        self.assertEqual(result.final_state, VLAState.COMPLETED)
        self.assertGreater(result.execution_time, 0)
        self.assertIn("command_received", result.steps_completed)
        self.assertIn("language_understanding", result.steps_completed)
        self.assertIn("vision_processing", result.steps_completed)
        self.assertIn("planning", result.steps_completed)
        self.assertIn("execution", result.steps_completed)

    def test_complete_vla_flow_perception_command(self):
        """Test complete flow for a perception command"""
        command = "Find the blue book on the table"

        # Process the command
        result = self.vla_system.process_command(command)

        # Verify the result
        self.assertTrue(result.success)
        self.assertEqual(result.final_state, VLAState.COMPLETED)
        self.assertGreater(result.execution_time, 0)
        self.assertIn("command_received", result.steps_completed)
        self.assertIn("language_understanding", result.steps_completed)
        self.assertIn("vision_processing", result.steps_completed)
        self.assertIn("planning", result.steps_completed)
        self.assertIn("execution", result.steps_completed)

    def test_system_state_transitions(self):
        """Test that system transitions through correct states"""
        command = "Navigate to the living room"

        # Check initial state
        self.assertEqual(self.vla_system.state, VLAState.IDLE)

        # Process command and check state transitions
        result = self.vla_system.process_command(command)

        # After successful execution, state should be COMPLETED
        self.assertEqual(result.final_state, VLAState.COMPLETED)

    def test_error_handling_in_integration(self):
        """Test error handling in the integrated system"""
        # Mock a failing module to test error propagation
        original_process = self.vla_system.modules["planning"].process

        def failing_process(input_data):
            raise Exception("Planning module failed")

        self.vla_system.modules["planning"].process = failing_process

        try:
            result = self.vla_system.process_command("Go to the kitchen")

            # Should fail gracefully
            self.assertFalse(result.success)
            self.assertEqual(result.final_state, VLAState.ERROR)
            self.assertIsNotNone(result.error_message)
        finally:
            # Restore original function
            self.vla_system.modules["planning"].process = original_process

    def test_context_preservation_through_modules(self):
        """Test that context is properly passed between modules"""
        command = "Pick up the red cup"
        context = {"robot_position": [0, 0, 0], "environment_map": "test_map"}

        result = self.vla_system.process_command(command, context)

        # Verify execution completed successfully
        self.assertTrue(result.success)
        self.assertEqual(result.final_state, VLAState.COMPLETED)

    def test_multiple_consecutive_commands(self):
        """Test processing multiple commands in sequence"""
        commands = [
            "Go to the kitchen",
            "Find the red cup",
            "Pick up the blue book",
            "Navigate to the living room"
        ]

        results = []
        for command in commands:
            result = self.vla_system.process_command(command)
            results.append(result)

        # All should succeed
        for result in results:
            self.assertTrue(result.success)
            self.assertIn(result.final_state, [VLAState.COMPLETED, VLAState.ERROR])

    def test_performance_metrics_update(self):
        """Test that performance metrics are updated after execution"""
        initial_metrics = self.vla_system.performance_metrics.copy()

        # Execute a command
        result = self.vla_system.process_command("Go to the kitchen")

        # Check that metrics were updated
        final_metrics = self.vla_system.performance_metrics
        self.assertGreater(final_metrics["total_executions"], initial_metrics["total_executions"])

        if result.success:
            self.assertGreater(final_metrics["success_rate"], 0)

    def test_execution_history_tracking(self):
        """Test that execution history is properly maintained"""
        initial_history_count = len(self.vla_system.execution_history)

        # Execute a command
        self.vla_system.process_command("Go to the kitchen")

        # Check that history was updated
        final_history_count = len(self.vla_system.execution_history)
        self.assertEqual(final_history_count, initial_history_count + 1)

    def test_module_interdependency(self):
        """Test that modules properly depend on each other's outputs"""
        command = "Find the red cup and pick it up"

        result = self.vla_system.process_command(command)

        # Should have gone through all steps
        expected_steps = ["command_received", "language_understanding",
                         "vision_processing", "planning", "execution"]
        for step in expected_steps:
            self.assertIn(step, result.steps_completed)

    def tearDown(self):
        """Clean up after tests"""
        self.vla_system.reset_system()


class TestModuleIntegration(unittest.TestCase):
    """Tests for individual module integration"""

    def test_language_and_vision_integration(self):
        """Test integration between language understanding and vision modules"""
        vla_system = VLASystemOrchestrator()

        # Language module should extract entities that vision can use
        command = "Find the red cup on the table"

        # Process through language module first
        lang_input = {"command": command}
        lang_result = vla_system.modules["language"].process(lang_input)

        # Extract entities from language processing
        entities = lang_result.get("entities", {})
        objects = entities.get("objects", [])

        # Use those objects as search query for vision
        vision_input = {"search_query": " ".join(objects)}
        vision_result = vla_system.modules["vision"].process(vision_input)

        # Should have found objects related to the command
        self.assertIsInstance(vision_result["objects_in_scene"], list)

    def test_planning_and_execution_integration(self):
        """Test integration between planning and execution modules"""
        vla_system = VLASystemOrchestrator()

        # Generate a plan
        planning_input = {
            "command": "Go to the kitchen",
            "context": {"objects_in_scene": [], "command_info": {"command_type": "navigation"}}
        }
        planning_result = vla_system.modules["planning"].process(planning_input)

        # Execute the plan
        execution_input = {
            "action_sequence": planning_result["action_sequence"],
            "context": {}
        }
        execution_result = vla_system.modules["execution"].process(execution_input)

        # Execution should have results
        self.assertIn("execution_results", execution_result)
        self.assertIsInstance(execution_result["execution_results"], list)


def run_integration_tests():
    """Run all integration tests"""
    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(TestVLAIntegration)
    suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestModuleIntegration))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result.wasSuccessful()


if __name__ == "__main__":
    print("Running VLA Integration Tests...")
    success = run_integration_tests()

    if success:
        print("\nAll integration tests passed!")
    else:
        print("\nSome integration tests failed!")
        sys.exit(1)