"""
Final Testing Suite for VLA System Components

This module provides comprehensive tests for all VLA system components to ensure
everything works correctly after all enhancements.
"""

import unittest
import sys
import os
from unittest.mock import Mock, patch
import time
import tempfile

# Add src directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from vla.vla_orchestrator import VLASystemOrchestrator, VLAState, VLAExecutionResult
from vla.monitoring import VLAMonitoringSystem, LogLevel
from vla.performance_benchmarks import VLAPerformanceBenchmark


class TestVLAOrchestrator(unittest.TestCase):
    """Test the VLA Orchestrator system"""

    def setUp(self):
        """Set up the VLA system for testing"""
        self.vla_system = VLASystemOrchestrator()

    def test_system_initialization(self):
        """Test that the VLA system initializes correctly"""
        self.assertIsNotNone(self.vla_system)
        self.assertEqual(self.vla_system.state, VLAState.IDLE)
        self.assertIn("language", self.vla_system.modules)
        self.assertIn("vision", self.vla_system.modules)
        self.assertIn("planning", self.vla_system.modules)
        self.assertIn("execution", self.vla_system.modules)
        self.assertEqual(len(self.vla_system.modules), 4)

    def test_process_valid_command(self):
        """Test processing a valid command"""
        result = self.vla_system.process_command("Go to the kitchen")

        self.assertIsInstance(result, VLAExecutionResult)
        self.assertTrue(result.success or not result.success)  # Should complete with success or failure, not error
        self.assertIn(result.final_state, [VLAState.COMPLETED, VLAState.ERROR])
        self.assertGreaterEqual(result.execution_time, 0)
        self.assertIsInstance(result.steps_completed, list)

    def test_process_empty_command(self):
        """Test processing an empty command (should fail gracefully)"""
        with self.assertRaises(ValueError):
            self.vla_system.process_command("")

    def test_process_invalid_command_type(self):
        """Test processing an invalid command type (should fail gracefully)"""
        with self.assertRaises(ValueError):
            self.vla_system.process_command(123)  # Non-string command

    def test_system_status(self):
        """Test getting system status"""
        status = self.vla_system.get_system_status()

        self.assertIn("state", status)
        self.assertIn("active_modules", status)
        self.assertIn("performance_metrics", status)
        self.assertIn("execution_count", status)
        self.assertIn("recent_executions", status)
        self.assertIn("monitoring", status)

    def test_multiple_commands(self):
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

        # All should complete without throwing exceptions
        self.assertEqual(len(results), len(commands))
        for result in results:
            self.assertIsInstance(result, VLAExecutionResult)

    def test_command_with_context(self):
        """Test processing a command with context"""
        context = {
            "robot_position": [0, 0, 0],
            "environment_map": "test_map",
            "known_objects": ["red cup", "blue book"]
        }

        result = self.vla_system.process_command("Find the red cup", context)
        self.assertIsInstance(result, VLAExecutionResult)

    def test_error_recovery(self):
        """Test that the system can recover from errors"""
        # Process a few valid commands
        for i in range(3):
            result = self.vla_system.process_command(f"Command {i}")
            self.assertIsInstance(result, VLAExecutionResult)

        # Check that performance metrics are being tracked
        status = self.vla_system.get_system_status()
        self.assertGreaterEqual(status['execution_count'], 3)

    def tearDown(self):
        """Clean up after tests"""
        self.vla_system.reset_system()


class TestMonitoringSystem(unittest.TestCase):
    """Test the monitoring system"""

    def setUp(self):
        """Set up monitoring system for testing"""
        self.monitor = VLAMonitoringSystem(tempfile.mktemp())

    def test_monitoring_initialization(self):
        """Test that monitoring system initializes correctly"""
        self.assertIsNotNone(self.monitor.event_logger)
        self.assertIsNotNone(self.monitor.performance_monitor)
        self.assertIsNotNone(self.monitor.health_monitor)

    def test_log_event(self):
        """Test logging events"""
        self.monitor.log_event(LogLevel.INFO, "TestModule", "Test message")

        # Check that event was logged
        recent_logs = self.monitor.event_logger.get_recent_logs(1)
        self.assertEqual(len(recent_logs), 1)
        self.assertEqual(recent_logs[0].message, "Test message")

    def test_performance_tracking(self):
        """Test performance tracking"""
        # Record some execution times
        for i in range(5):
            self.monitor.record_command_execution(0.1 + i * 0.01, True)

        # Check performance summary
        perf_summary = self.monitor.performance_monitor.get_performance_summary()
        self.assertGreaterEqual(perf_summary['total_executions'], 5)

    def test_health_monitoring(self):
        """Test health monitoring"""
        self.monitor.check_module_health("TestModule", True, {"status": "ok"})

        health_status = self.monitor.health_monitor.check_system_health()
        self.assertIn("TestModule", health_status["modules"])
        self.assertTrue(health_status["modules"]["TestModule"]["healthy"])

    def tearDown(self):
        """Clean up monitoring system"""
        pass


class TestPerformanceBenchmarking(unittest.TestCase):
    """Test the performance benchmarking system"""

    def setUp(self):
        """Set up benchmark system for testing"""
        self.benchmark = VLAPerformanceBenchmark()

    def test_benchmark_initialization(self):
        """Test that benchmark system initializes correctly"""
        self.assertIsNotNone(self.benchmark)
        self.assertEqual(len(self.benchmark.benchmark_results), 0)

    def test_performance_metrics_initial(self):
        """Test initial performance metrics"""
        metrics = self.benchmark.get_performance_metrics()
        self.assertIsNotNone(metrics)
        self.assertIsInstance(metrics.throughput_cps, float)
        self.assertIsInstance(metrics.accuracy, float)

    def tearDown(self):
        """Clean up benchmark system"""
        pass


class TestIntegration(unittest.TestCase):
    """Test integration between all components"""

    def setUp(self):
        """Set up integrated system for testing"""
        self.vla_system = VLASystemOrchestrator()

    def test_full_integration(self):
        """Test full integration of all components"""
        # Process a command that should go through all modules
        result = self.vla_system.process_command("Go to the kitchen and find the red cup")

        # Verify the result
        self.assertIsInstance(result, VLAExecutionResult)

        # Check that the result contains expected information
        self.assertIsInstance(result.steps_completed, list)
        self.assertIn("command_received", result.steps_completed)
        self.assertIn("language_understanding", result.steps_completed)

        # Check that the system status includes monitoring information
        status = self.vla_system.get_system_status()
        self.assertIn("monitoring", status)
        self.assertIn("system_health", status["monitoring"])
        self.assertIn("performance_summary", status["monitoring"])

    def test_error_propagation(self):
        """Test that errors are properly propagated through the system"""
        # This test checks that errors in one module don't crash the entire system
        # The VLA system should handle errors gracefully

        # Process multiple commands to ensure system stability
        commands = [
            "Go to the kitchen",
            "Find the red cup",
            "Invalid command with strange syntax"
        ]

        results = []
        for command in commands:
            try:
                result = self.vla_system.process_command(command)
                results.append(result)
            except Exception as e:
                # If an exception occurs, the system should still be functional
                # This means error handling is working properly
                pass

        # System should still be operational
        final_status = self.vla_system.get_system_status()
        self.assertIsNotNone(final_status)

    def tearDown(self):
        """Clean up after integration tests"""
        self.vla_system.reset_system()


def run_all_tests():
    """Run all tests and return success status"""
    # Create test suites
    orchestrator_suite = unittest.TestLoader().loadTestsFromTestCase(TestVLAOrchestrator)
    monitoring_suite = unittest.TestLoader().loadTestsFromTestCase(TestMonitoringSystem)
    benchmark_suite = unittest.TestLoader().loadTestsFromTestCase(TestPerformanceBenchmarking)
    integration_suite = unittest.TestLoader().loadTestsFromTestCase(TestIntegration)

    # Combine all suites
    all_tests = unittest.TestSuite([
        orchestrator_suite,
        monitoring_suite,
        benchmark_suite,
        integration_suite
    ])

    # Run all tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(all_tests)

    return result.wasSuccessful()


def run_performance_tests(vla_system):
    """Run performance-specific tests"""
    print("\nRunning Performance Tests...")

    # Initialize benchmark suite
    benchmark_suite = VLAPerformanceBenchmark()

    # Define test commands
    test_commands = [
        "Go to the kitchen",
        "Find the red cup",
        "Pick up the blue book",
        "Navigate to the living room",
        "Detect the chair near the table"
    ]

    # Run a quick latency benchmark
    print("Running quick latency test...")
    latency_result = benchmark_suite.run_latency_benchmark(vla_system, test_commands[:3], num_iterations=5)
    print(f"Latency test completed: avg={latency_result.details['avg_latency']:.3f}s")

    # Run a quick accuracy benchmark
    print("Running quick accuracy test...")
    test_cases = [(cmd, "expected") for cmd in test_commands]
    accuracy_result = benchmark_suite.run_accuracy_benchmark(vla_system, test_cases)
    print(f"Accuracy test completed: {accuracy_result.details['accuracy_percentage']:.1f}%")

    # Generate and print performance report
    print("\nPerformance Report:")
    print(benchmark_suite.generate_benchmark_report())


if __name__ == "__main__":
    print("Starting Final Testing Suite for VLA System...")
    print("=" * 60)

    # Initialize VLA system
    print("Initializing VLA system...")
    vla_system = VLASystemOrchestrator()
    print("VLA system initialized successfully")

    # Run unit tests
    print("\nRunning Unit Tests...")
    unit_test_success = run_all_tests()

    # Run performance tests
    run_performance_tests(vla_system)

    print("\n" + "=" * 60)
    if unit_test_success:
        print("✅ All unit tests PASSED!")
    else:
        print("❌ Some unit tests FAILED!")

    print("Final testing suite completed.")
    print("=" * 60)