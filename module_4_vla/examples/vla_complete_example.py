"""
Complete VLA System Example

This example demonstrates the complete Vision-Language-Action pipeline
integrating all components to process a natural language command and execute it.
"""

import sys
import os
from typing import Dict, Any, Optional
import time
import json

# Add the src directory to the path to import VLA modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from vla.vla_orchestrator import VLASystemOrchestrator
from vla.llm_planner import EnhancedLLMPlanner
from vla.vision_processor import EnhancedVisionProcessor
from vla.action_mapper import EnhancedActionMapper
from vla.hierarchical_task_network import EnhancedHTNPlanner


def run_complete_vla_example():
    """Run a complete example of the VLA system"""
    print("=" * 60)
    print("VLA System Complete Example")
    print("=" * 60)

    # Initialize the complete VLA system
    print("\n1. Initializing VLA System...")
    vla_system = VLASystemOrchestrator()

    # Check system status
    status = vla_system.get_system_status()
    print(f"   System State: {status['state']}")
    print(f"   Active Modules: {status['active_modules']}")
    print(f"   Total Executions: {status['execution_count']}")

    # Display the data flow visualization
    print(f"\n2. VLA System Data Flow:")
    print(vla_system.visualize_data_flow())

    # Example 1: Simple navigation command
    print(f"\n3. Example 1: Simple Navigation Command")
    command1 = "Go to the kitchen"
    print(f"   Command: '{command1}'")

    result1 = vla_system.process_command(command1)
    print(f"   Success: {result1.success}")
    print(f"   Execution Time: {result1.execution_time:.2f}s")
    print(f"   Steps Completed: {result1.steps_completed}")
    print(f"   Final State: {result1.final_state.value}")

    # Example 2: Object detection and retrieval
    print(f"\n4. Example 2: Object Detection and Retrieval")
    command2 = "Find the red cup on the table and bring it to me"
    print(f"   Command: '{command2}'")

    result2 = vla_system.process_command(command2)
    print(f"   Success: {result2.success}")
    print(f"   Execution Time: {result2.execution_time:.2f}s")
    print(f"   Steps Completed: {len(result2.steps_completed)}")

    if result2.action_results:
        print(f"   Actions Executed: {len(result2.action_results)}")
        for i, action_result in enumerate(result2.action_results[:3]):  # Show first 3
            print(f"     {i+1}. {action_result.get('action', {}).get('action_name', 'unknown')}")

    # Example 3: Complex manipulation task
    print(f"\n5. Example 3: Complex Manipulation Task")
    command3 = "Move the blue book from the table to the shelf"
    print(f"   Command: '{command3}'")

    result3 = vla_system.process_command(command3)
    print(f"   Success: {result3.success}")
    print(f"   Execution Time: {result3.execution_time:.2f}s")

    # Show performance metrics
    print(f"\n6. Performance Summary:")
    perf_summary = vla_system.get_performance_summary()
    print(f"   Total Executions: {perf_summary['total_executions']}")
    print(f"   Success Rate: {perf_summary['success_rate']:.2f}")
    print(f"   Average Execution Time: {perf_summary['avg_execution_time']:.2f}s")
    print(f"   Recent Success Rate (last 10): {perf_summary['recent_success_rate']:.2f}")

    # Example 4: System introspection
    print(f"\n7. Example 4: System Introspection")
    print("   Demonstrating system analysis capabilities...")

    # Show detailed execution history
    history = vla_system.execution_history
    print(f"   Total executions processed: {len(history)}")

    if history:
        latest = history[-1]  # Most recent execution
        print(f"   Latest execution details:")
        print(f"     Command: '{latest['command']}'")
        print(f"     Success: {latest['success']}")
        print(f"     Time: {latest['execution_time']:.2f}s")
        print(f"     Steps: {latest['steps_completed']}")

    print(f"\n8. Example 5: Error Handling Demonstration")
    # This will demonstrate how the system handles potential errors
    try:
        # Test with a complex command that might challenge the system
        challenging_command = "Simultaneously navigate to the kitchen, detect all red objects, and prepare a meal"
        print(f"   Command: '{challenging_command}' (challenging multi-task command)")

        result4 = vla_system.process_command(challenging_command)
        print(f"   Success: {result4.success}")
        print(f"   Execution Time: {result4.execution_time:.2f}s")
        print(f"   State after execution: {result4.final_state.value}")
    except Exception as e:
        print(f"   Error handled gracefully: {e}")

    print(f"\n9. Example 6: Module Integration Test")
    print("   Testing individual module integration...")

    # Test language understanding module
    lang_module = vla_system.modules["language"]
    lang_result = lang_module.process({"command": "Pick up the green bottle"})
    print(f"   Language understanding: {lang_result['command_type']} command")

    # Test vision processing module
    vision_module = vla_system.modules["vision"]
    vision_result = vision_module.process({"search_query": "bottle"})
    print(f"   Vision processing: Found {len(vision_result['objects_in_scene'])} objects")

    # Test planning module
    planning_module = vla_system.modules["planning"]
    planning_result = planning_module.process({
        "command": "Pick up the green bottle",
        "context": {"objects_in_scene": vision_result['objects_in_scene']}
    })
    print(f"   Planning: Generated {len(planning_result['action_sequence'])} actions")

    # Test execution module
    execution_module = vla_system.modules["execution"]
    execution_result = execution_module.process({
        "action_sequence": planning_result['action_sequence'],
        "context": {}
    })
    print(f"   Execution: Processed {len(execution_result['execution_results'])} actions")

    print(f"\n10. Final System Status:")
    final_status = vla_system.get_system_status()
    print(f"    Current State: {final_status['state']}")
    print(f"    Active Modules: {final_status['active_modules']}")
    print(f"    Total Executions: {final_status['execution_count']}")

    # Show recent execution history
    recent = final_status['recent_executions'][-3:]  # Last 3 executions
    print(f"    Recent Executions:")
    for i, exec_record in enumerate(recent):
        print(f"      {i+1}. '{exec_record['command'][:30]}...' - {exec_record['success']} - {exec_record['execution_time']:.2f}s")

    print(f"\n" + "=" * 60)
    print("VLA System Complete Example - FINISHED")
    print("=" * 60)

    return vla_system


def demonstrate_component_integration():
    """Demonstrate how individual components work together"""
    print("\n" + "=" * 60)
    print("Component Integration Demonstration")
    print("=" * 60)

    print("\n1. Individual Component Initialization:")

    # Initialize each component
    print("   Initializing Language Planner...")
    llm_planner = EnhancedLLMPlanner()
    print(f"   LLM Planner available: {llm_planner.is_initialized}")

    print("   Initializing Vision Processor...")
    vision_processor = EnhancedVisionProcessor()
    print(f"   Vision Processor available: {vision_processor.is_initialized}")

    print("   Initializing Action Mapper...")
    action_mapper = EnhancedActionMapper()
    print(f"   Action Mapper available: {action_mapper.is_initialized}")

    print("   Initializing HTN Planner...")
    htn_planner = EnhancedHTNPlanner()
    print(f"   HTN Planner available: {htn_planner is not None}")

    print("\n2. Component Integration Example:")

    # Simulate the flow: Language -> Planning -> Vision -> Action Mapping
    command = "Navigate to the table and pick up the red cup"
    print(f"   Input Command: '{command}'")

    # Language processing would happen here (simplified)
    print("   Language Understanding: Command parsed for navigation and manipulation")

    # Planning phase
    print("   Planning Phase:")
    from vla.hierarchical_task_network import Task
    task = Task(
        name="complex_task",
        method="compound",
        parameters={"command": command}
    )
    htn_plan = htn_planner.decompose_task(task)
    print(f"   HTN Decomposition: {htn_plan.decomposition_depth} levels deep")

    # Vision processing (simulated)
    print("   Vision Processing: Scene analyzed for table and red cup")
    vision_result = vision_processor.understand_scene()
    print(f"   Found {len(vision_result.objects_in_scene)} objects in scene")

    # Action mapping
    print("   Action Mapping Phase:")
    # Create a sample plan to map
    sample_plan = [
        {"action": "navigate_to_location", "parameters": {"location": [1.0, 2.0, 0.0]}},
        {"action": "detect_object", "parameters": {"object_type": "red cup"}},
        {"action": "grasp_object", "parameters": {"object_id": "red_cup_1"}}
    ]

    mapping_result = action_mapper.map_plan(sample_plan)
    print(f"   Mapped to {len(mapping_result.ros2_actions)} ROS 2 actions")

    # Show first mapped action
    if mapping_result.ros2_actions:
        first_action = mapping_result.ros2_actions[0]
        print(f"   First ROS 2 Action: {first_action['action_name']}")
        print(f"   Parameters: {first_action['parameters']}")


def run_performance_benchmark():
    """Run a simple performance benchmark"""
    print("\n" + "=" * 60)
    print("Performance Benchmark")
    print("=" * 60)

    vla_system = VLASystemOrchestrator()

    # Test commands for benchmarking
    test_commands = [
        "Go to the kitchen",
        "Find the red cup",
        "Pick up the blue book",
        "Navigate to the table",
        "Describe the scene"
    ]

    print(f"Running benchmark with {len(test_commands)} commands...")

    start_time = time.time()
    results = []

    for i, command in enumerate(test_commands):
        print(f"  Processing command {i+1}/{len(test_commands)}: '{command}'")
        result = vla_system.process_command(command)
        results.append(result)
        print(f"    Success: {result.success}, Time: {result.execution_time:.2f}s")

    total_time = time.time() - start_time
    successful = sum(1 for r in results if r.success)
    avg_time = sum(r.execution_time for r in results) / len(results)

    print(f"\nBenchmark Results:")
    print(f"  Total Commands: {len(test_commands)}")
    print(f"  Successful: {successful}/{len(test_commands)} ({successful/len(test_commands)*100:.1f}%)")
    print(f"  Total Time: {total_time:.2f}s")
    print(f"  Average Time: {avg_time:.2f}s per command")
    print(f"  Throughput: {len(test_commands)/total_time:.2f} commands/second")


def main():
    """Main function to run all examples"""
    print("Starting VLA Complete System Examples...\n")

    # Run the complete VLA example
    vla_system = run_complete_vla_example()

    # Demonstrate component integration
    demonstrate_component_integration()

    # Run performance benchmark
    run_performance_benchmark()

    print(f"\nAll examples completed successfully!")
    print(f"System is ready for real-world deployment scenarios.")


if __name__ == "__main__":
    main()