# VLA Module - Troubleshooting Guide

## Table of Contents
1. [Installation Issues](#installation-issues)
2. [Runtime Errors](#runtime-errors)
3. [Module-Specific Issues](#module-specific-issues)
4. [Performance Problems](#performance-problems)
5. [ROS 2 Integration Issues](#ros-2-integration-issues)
6. [Simulation Environment Issues](#simulation-environment-issues)
7. [Common Error Messages](#common-error-messages)
8. [Debugging Strategies](#debugging-strategies)

## Installation Issues

### Python Package Installation Failures
**Problem:** `pip install` fails with compilation errors
**Solution:**
```bash
# Install build tools first
sudo apt update && sudo apt install build-essential python3-dev

# Then retry installation
pip install -r requirements.txt
```

### Missing Dependencies
**Problem:** ImportError when importing modules
**Solution:**
```bash
# Check if all dependencies are installed
pip list | grep -E "(torch|opencv|spacy|transformers)"

# Reinstall requirements
pip install -r requirements.txt --force-reinstall
```

### Virtual Environment Issues
**Problem:** Packages not found despite installation
**Solution:**
- Ensure virtual environment is activated: `source vla_env/bin/activate`
- Verify Python path: `which python`
- Check installed packages: `pip list`

## Runtime Errors

### Module Initialization Failures
**Problem:** VLA system fails to initialize
**Solution:**
1. Check if all required modules are available
2. Verify configuration files exist and are properly formatted
3. Ensure all dependencies are installed

```python
# Debug initialization
from src.vla.vla_orchestrator import VLASystemOrchestrator
import traceback

try:
    vla_system = VLASystemOrchestrator()
    print("System initialized successfully")
except Exception as e:
    print(f"Initialization failed: {e}")
    traceback.print_exc()
```

### Command Processing Failures
**Problem:** Commands fail to process correctly
**Solution:**
1. Enable debug mode in configuration
2. Check command format and syntax
3. Verify context information

## Module-Specific Issues

### Language Understanding Module Issues
**Problem:** Commands not parsed correctly
**Solution:**
- Verify command format follows expected patterns
- Check if spaCy model is downloaded: `python -m spacy validate`
- Enable debug logging to see parsing details

**Problem:** Entity extraction not working
**Solution:**
- Ensure language model is properly loaded
- Check command contains recognizable entities
- Update entity extraction rules if needed

### Vision Processing Module Issues
**Problem:** Object detection not working
**Solution:**
- Verify image data format and size
- Check if vision models are available
- Ensure OpenCV is properly installed

**Problem:** Scene understanding inaccurate
**Solution:**
- Use higher quality input images
- Verify camera calibration
- Update detection thresholds in config

### Planning Module Issues
**Problem:** Plans not generated correctly
**Solution:**
- Check if planning models are available
- Verify command is decomposable into actions
- Review planning configuration parameters

### Action Execution Module Issues
**Problem:** Actions fail to execute
**Solution:**
- Verify ROS 2 connectivity (if using ROS 2)
- Check action server availability
- Review action mapping configuration

## Performance Problems

### Slow Command Processing
**Problem:** Commands take too long to process
**Solution:**
1. Monitor system resources: `htop` or `top`
2. Check for memory leaks
3. Optimize configuration settings

```bash
# Monitor resource usage
htop
# Or check specific process
ps aux | grep python
```

### High Memory Usage
**Problem:** Memory consumption increases over time
**Solution:**
- Implement proper cleanup in modules
- Limit execution history size
- Use memory profiling tools

```python
# Monitor memory usage
import psutil
import os
process = psutil.Process(os.getpid())
print(f"Memory usage: {process.memory_info().rss / 1024 / 1024:.2f} MB")
```

### Inconsistent Performance
**Problem:** Performance varies between executions
**Solution:**
- Check for resource contention
- Monitor system load
- Implement performance caching where appropriate

## ROS 2 Integration Issues

### ROS 2 Not Found
**Problem:** ROS 2 modules not available
**Solution:**
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify installation
ros2 topic list
```

### ROS Domain ID Conflicts
**Problem:** Multiple ROS systems interfering
**Solution:**
```bash
# Set unique domain ID
export ROS_DOMAIN_ID=42
```

### Action Server Connection Issues
**Problem:** Cannot connect to action servers
**Solution:**
1. Verify action servers are running
2. Check network connectivity
3. Ensure correct action interfaces

## Simulation Environment Issues

### Gazebo Not Starting
**Problem:** Simulation environment fails to launch
**Solution:**
- Check GPU drivers and OpenGL support
- Verify Gazebo installation
- Check for conflicting processes

### Robot Model Not Loading
**Problem:** Robot model not visible in simulation
**Solution:**
- Verify URDF files exist and are valid
- Check model paths in configuration
- Validate ROS package dependencies

### Physics Simulation Issues
**Problem:** Robot behaves unexpectedly in simulation
**Solution:**
- Check physics engine configuration
- Verify robot controller settings
- Adjust simulation parameters

## Common Error Messages

### "Module not initialized"
**Cause:** Module failed to initialize properly
**Solution:** Check initialization sequence and dependencies

### "No planner available"
**Cause:** Planning module could not be loaded
**Solution:** Verify planning dependencies and configuration

### "Action mapping failed"
**Cause:** Could not map high-level action to ROS 2 action
**Solution:** Check action interface definitions and mapping configuration

### "Vision processing unavailable"
**Cause:** Vision module dependencies not met
**Solution:** Install required computer vision libraries

### "Command parsing failed"
**Cause:** Natural language command could not be understood
**Solution:** Check command format and language model availability

## Debugging Strategies

### Enable Verbose Logging
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Check System Status
```python
# Get detailed system status
status = vla_system.get_system_status()
print(f"System state: {status['state']}")
print(f"Active modules: {status['active_modules']}")
print(f"Performance metrics: {status['performance_metrics']}")
```

### Use Debug Configuration
```yaml
# In config/vla_config.yaml
general:
  debug_mode: true
  log_level: DEBUG
```

### Test Individual Modules
```python
# Test language module independently
from src.vla.language_understanding import LanguageUnderstandingModule
lang_module = LanguageUnderstandingModule()
lang_module.initialize()
result = lang_module.process({"command": "test command"})
print(result)
```

### Performance Monitoring
```python
# Monitor execution performance
import time
start = time.time()
result = vla_system.process_command("test command")
end = time.time()
print(f"Execution time: {end - start}s")
```

### Error Recovery Testing
```python
# Test error handling
try:
    result = vla_system.process_command("problematic command")
    if not result.success:
        print(f"Command failed: {result.error_message}")
except Exception as e:
    print(f"System error: {e}")
```

## System Health Checks

### Basic Health Check
```bash
# Check Python environment
python --version
pip list | grep -E "(torch|opencv|spacy)"

# Check ROS 2 (if applicable)
source /opt/ros/humble/setup.bash
ros2 doctor

# Check system resources
free -h
df -h
```

### Module Availability Check
```python
from src.vla.vla_orchestrator import VLASystemOrchestrator

system = VLASystemOrchestrator()
status = system.get_system_status()

print("Module Health Check:")
for module_name, is_initialized in status['performance_metrics']['module_availability'].items():
    print(f"  {module_name}: {'✓' if is_initialized else '✗'}")
```

## Prevention Strategies

### Regular Maintenance
- Update dependencies regularly
- Monitor system performance
- Review and clean up execution history
- Check for memory leaks

### Configuration Validation
- Validate configuration files
- Test with default settings first
- Gradually customize after verifying functionality

### Testing
- Run unit tests regularly
- Execute integration tests
- Monitor performance benchmarks
- Test edge cases and error conditions

## Getting Help

### Community Resources
- Check the GitHub issues page
- Review the documentation
- Search for similar problems

### Support Channels
- Create a GitHub issue with detailed error information
- Include system specifications and steps to reproduce
- Attach relevant log files

### Diagnostic Information to Include
When reporting issues, provide:
- Operating system and version
- Python version
- ROS 2 version (if applicable)
- Module version
- Full error message and stack trace
- Steps to reproduce the issue
- Expected vs. actual behavior

---

This troubleshooting guide provides solutions for common issues with the VLA module. If you encounter problems not covered here, please create an issue in the repository with detailed information about the problem.