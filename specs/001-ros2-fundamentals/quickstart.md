# Quickstart Guide: ROS 2 Fundamentals Module

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Basic Python knowledge (variables, functions, classes)
- Terminal/command line familiarity

## Setup Your Environment

### 1. Verify ROS 2 Installation
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

### 2. Create a Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 3. Source the Environment
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Create Your First ROS 2 Package

### 1. Generate Package Structure
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_tutorial
cd my_robot_tutorial
```

### 2. Package Structure
```
my_robot_tutorial/
├── package.xml
├── setup.py
├── setup.cfg
├── my_robot_tutorial/
│   ├── __init__.py
│   └── my_node.py
└── test/
    ├── __init__.py
    └── test_copyright.py
    └── test_flake8.py
    └── test_pep257.py
```

## Run the Publisher and Subscriber Example

### 1. Terminal 1 - Run Publisher
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_tutorial joint_state_publisher
```

### 2. Terminal 2 - Run Subscriber
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_tutorial joint_state_subscriber
```

### 3. Verify Communication
```bash
# List active topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /joint_states sensor_msgs/msg/JointState

# Call a service
ros2 service call /set_joint_angle example_interfaces/srv/SetBool "{data: true}"
```

## Load URDF in RViz2

### 1. Start RViz2
```bash
rviz2
```

### 2. Add RobotModel Display
- Click "Add" in the Displays panel
- Select "RobotModel" under "Robot Models"
- Set "Robot Description" to your URDF parameter name

### 3. Publish URDF
```bash
# Publish URDF to the robot_description topic
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat urdf/my_robot.urdf)'
```

## Launch File Example

### 1. Create Launch Directory
```bash
mkdir ~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/launch
```

### 2. Create Launch File
```python
# my_robot_tutorial/launch/my_launch_file.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_tutorial',
            executable='joint_state_publisher',
            name='joint_publisher'
        ),
        Node(
            package='my_robot_tutorial',
            executable='joint_state_subscriber',
            name='joint_subscriber'
        )
    ])
```

### 3. Run Launch File
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_robot_tutorial my_launch_file.py
```

## Common Commands

### Package Management
```bash
# Create a new package
ros2 pkg create --build-type ament_python <package_name>

# Build all packages
colcon build --packages-select <package_name>

# Source the workspace
source install/setup.bash
```

### Node and Topic Management
```bash
# List nodes
ros2 node list

# List topics
ros2 topic list

# Echo a topic
ros2 topic echo <topic_name> <message_type>

# Publish to a topic once
ros2 topic pub <topic_name> <message_type> <values>
```

### Service Management
```bash
# List services
ros2 service list

# Call a service
ros2 service call <service_name> <service_type> <request_values>
```

## Troubleshooting

### Common Issues:
1. **"Command not found"** - Make sure ROS 2 environment is sourced
2. **"Package not found"** - Check package name and ensure workspace is built and sourced
3. **"Topic not found"** - Verify nodes are running and topic names match
4. **URDF errors** - Check XML syntax and required elements

### Useful Commands:
```bash
# Check if ROS 2 is properly sourced
printenv | grep ROS

# Find available message types
ros2 interface list | grep <keyword>

# Get detailed information about a node
ros2 node info <node_name>
```

## Next Steps

1. Complete the full tutorial modules in sequence
2. Experiment with different message types
3. Create more complex URDF models
4. Build your own robot applications