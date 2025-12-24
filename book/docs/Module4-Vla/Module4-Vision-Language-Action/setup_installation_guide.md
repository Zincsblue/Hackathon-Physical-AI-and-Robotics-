# VLA Module - Setup and Installation Guide

## Prerequisites

### System Requirements
- Operating System: Ubuntu 20.04 LTS or Windows 10/11
- Python: 3.8 or higher
- RAM: 8GB minimum, 16GB recommended
- Storage: 10GB free space
- ROS 2: Humble Hawksbill or later (for full functionality)

### Software Dependencies
- Git
- pip (Python package manager)
- Virtual environment tool (venv or conda)

## Installation Steps

### 1. Clone the Repository
```bash
git clone https://github.com/your-organization/robotics-course.git
cd robotics-course
```

### 2. Set up Python Environment
```bash
# Create virtual environment
python -m venv vla_env

# Activate virtual environment
# On Linux/Mac:
source vla_env/bin/activate
# On Windows:
vla_env\Scripts\activate

# Upgrade pip
pip install --upgrade pip
```

### 3. Install ROS 2 (if not already installed)
Follow the official ROS 2 installation guide for your platform:
- [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)

For Ubuntu:
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

### 4. Install Module Dependencies
```bash
cd module_4_vla
pip install -r requirements.txt
```

If requirements.txt doesn't exist, install the core dependencies:
```bash
pip install numpy scipy opencv-python torch torchvision transformers openai-whisper
pip install rclpy  # ROS 2 Python client library
pip install spacy  # For NLP processing
python -m spacy download en_core_web_sm  # English language model
```

### 5. Set up Simulation Environment (Gazebo)
For full simulation capabilities:
```bash
# Install Gazebo Garden (or Humble's default)
sudo apt install ros-humble-gazebo-*

# Or install Ignition Gazebo
sudo apt install ignition-garden
```

### 6. Configure the VLA Module
```bash
# Navigate to the module directory
cd module_4_vla

# Create configuration directory if it doesn't exist
mkdir -p config

# Create default configuration
cat > config/vla_config.yaml << EOF
# VLA Module Configuration
general:
  debug_mode: false
  log_level: INFO
  max_execution_time: 30.0

language_understanding:
  model_type: "transformers"  # or "openai", "local"
  confidence_threshold: 0.7

vision_processing:
  detection_model: "yolo"  # or "rcnn", "transformers"
  confidence_threshold: 0.5

planning:
  planner_type: "htn"  # or "llm", "classical"
  max_plan_steps: 50

execution:
  ros2_interface: true
  simulation_mode: true
  action_timeout: 10.0

performance:
  metrics_enabled: true
  history_size: 100
EOF
```

### 7. Build and Test the Installation
```bash
# Test the installation by running the main orchestrator
python -c "from src.vla.vla_orchestrator import VLASystemOrchestrator; print('VLA Module imported successfully!')"

# Run the example
python src/vla/vla_orchestrator.py
```

## Configuration Options

### Environment Variables
Set these environment variables for customization:

```bash
export VLA_CONFIG_PATH="config/vla_config.yaml"
export VLA_DEBUG_MODE="true"  # Enable debug output
export ROS_DOMAIN_ID=42  # Set ROS 2 domain ID to avoid conflicts
```

### Configuration File Settings
The configuration file (`config/vla_config.yaml`) allows you to customize various aspects of the VLA system:

- `debug_mode`: Enable detailed logging and debugging information
- `log_level`: Set logging level (DEBUG, INFO, WARNING, ERROR)
- `max_execution_time`: Maximum time allowed for command execution
- `confidence_threshold`: Minimum confidence for accepting results
- `simulation_mode`: Run in simulation mode (true) or with real hardware (false)

## Running the VLA System

### Basic Usage
```bash
cd module_4_vla
python src/vla/vla_orchestrator.py
```

### Interactive Mode
```python
from src.vla.vla_orchestrator import VLASystemOrchestrator

# Initialize the system
vla_system = VLASystemOrchestrator()

# Process a command
result = vla_system.process_command("Go to the kitchen")
print(f"Command executed successfully: {result.success}")
```

### With Context
```python
context = {
    "robot_position": [0, 0, 0],
    "environment_map": "test_lab",
    "known_objects": ["red cup", "blue book"]
}

result = vla_system.process_command("Find the red cup", context)
```

## Docker Installation (Alternative)

For containerized deployment:

```bash
# Build the Docker image
docker build -t vla-module:latest -f Dockerfile .

# Run with GPU support (if needed)
docker run --gpus all -it vla-module:latest

# Or run without GPU
docker run -it vla-module:latest
```

## Verification Steps

After installation, verify everything is working:

1. **Check Python Environment:**
```bash
python --version  # Should be 3.8+
pip list | grep -E "(torch|opencv|spacy|transformers)"
```

2. **Test ROS 2 (if installed):**
```bash
source /opt/ros/humble/setup.bash
ros2 topic list  # Should show available topics
```

3. **Run Basic Test:**
```bash
cd module_4_vla
python -c "
from src.vla.vla_orchestrator import VLASystemOrchestrator
system = VLASystemOrchestrator()
print('System initialized successfully')
status = system.get_system_status()
print(f'System status: {status[\"state\"]}')
print(f'Active modules: {len(status[\"active_modules\"])}')
"
```

4. **Run Integration Tests:**
```bash
cd module_4_vla
python -m pytest tests/integration_tests.py -v
```

## Troubleshooting Common Issues

### Python Package Installation Issues
- If you encounter package conflicts, use a clean virtual environment
- For compilation issues, install build tools: `sudo apt install build-essential`

### ROS 2 Integration Issues
- Ensure ROS 2 environment is sourced before running VLA module
- Check ROS_DOMAIN_ID to avoid network conflicts
- Verify ROS 2 installation with `ros2 doctor`

### Performance Issues
- Ensure sufficient RAM and CPU resources
- Check GPU availability if using GPU-accelerated models
- Monitor system resources during execution

## Updating the Module

To update to the latest version:

```bash
git pull origin main
source vla_env/bin/activate  # Or your virtual environment
pip install -r requirements.txt --upgrade
```

## Uninstallation

To completely remove the VLA module:

```bash
# Deactivate virtual environment
deactivate

# Remove virtual environment
rm -rf vla_env

# Remove any configuration files
rm -rf config/
```

## Next Steps

After successful installation:

1. Review the [comprehensive documentation](comprehensive_documentation.md)
2. Run the example scenarios in `examples/`
3. Explore the API reference in `docs/api_reference.md`
4. Try the interactive tutorials in `tutorials/`

---

This installation guide provides all necessary steps to set up the Vision-Language-Action module. If you encounter issues not covered in this guide, please refer to the troubleshooting section or create an issue in the repository.