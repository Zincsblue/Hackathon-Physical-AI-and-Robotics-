# Data Model: ROS 2 Fundamentals Module

## Core Entities

### ROS 2 Node
- **Definition**: A process that performs computation, communicating with other nodes through messages, services, or actions
- **Attributes**:
  - Node name (string)
  - Node namespace (string, optional)
  - Parameters (key-value pairs)
  - Publishers (list of topic publishers)
  - Subscribers (list of topic subscribers)
  - Services (list of service servers/clients)
- **Relationships**: Communicates with other nodes via topics, services, actions

### ROS 2 Topic
- **Definition**: A named bus over which nodes exchange messages in a publish/subscribe pattern
- **Attributes**:
  - Topic name (string)
  - Message type (string, e.g., "std_msgs/msg/String")
  - Message queue size (integer)
  - Quality of Service (QoS) settings
- **Relationships**: Connected to publishers and subscribers

### ROS 2 Service
- **Definition**: A synchronous request/response communication pattern between nodes
- **Attributes**:
  - Service name (string)
  - Service type (string, e.g., "std_srvs/srv/SetBool")
  - Request message structure
  - Response message structure
- **Relationships**: Connected to service clients and servers

### URDF (Unified Robot Description Format)
- **Definition**: An XML format for representing robot models, including links, joints, and visual properties
- **Attributes**:
  - Robot name (string)
  - Links (list of link elements with visual properties)
  - Joints (list of joint elements connecting links)
  - Materials (list of material definitions)
- **Relationships**: Used by RViz2 for visualization

## Message Types (Standard)

### sensor_msgs/JointState
- **Purpose**: Represents the state of joints (position, velocity, effort)
- **Fields**:
  - name (array of strings): joint names
  - position (array of floats): joint positions in radians
  - velocity (array of floats): joint velocities in rad/s
  - effort (array of floats): joint efforts in Nm

### sensor_msgs/Imu
- **Purpose**: Represents inertial measurement unit data
- **Fields**:
  - orientation (geometry_msgs/Quaternion): orientation in quaternion form
  - angular_velocity (geometry_msgs/Vector3): angular velocity
  - linear_acceleration (geometry_msgs/Vector3): linear acceleration

### std_msgs/String
- **Purpose**: Simple string message for general communication
- **Fields**:
  - data (string): the actual string message

## Launch File Structure
- **Definition**: Python files that define how to launch multiple nodes together
- **Components**:
  - Node descriptions with parameters
  - Remapping rules
  - Conditional launches
- **Purpose**: Simplify multi-node execution

## Python Agent Interface
- **Definition**: Python scripts that interact with ROS 2 network via rclpy
- **Components**:
  - Node initialization
  - Publisher/subscriber setup
  - Service client/server implementation
  - Spin loop execution

## Validation Rules

### From Requirements:
- All URDF files must be valid XML
- All ROS 2 examples must use standard message types
- URDF examples must include only visual elements (no inertial/collision)
- Code examples must include basic error handling
- All examples must run on Ubuntu 22.04 with ROS 2 Humble

### State Transitions:
- ROS 2 Node: Uninitialized → Initialized → Active → Shutdown
- Launch Process: Configuration → Node Creation → Execution → Termination
- Communication: Setup → Active Messaging → Teardown

## Relationships and Constraints

### Node Communication Constraints:
- Publishers and subscribers must match topic names and message types
- Service clients must match service names and types with servers
- Nodes must be properly initialized before communication

### URDF Constraints:
- All links must have unique names
- Joints must connect existing links
- Visual elements must have valid geometry definitions
- Robot tag must contain a unique robot name

### Package Structure Constraints:
- Must follow ament_python conventions
- package.xml must contain proper dependencies
- setup.py must properly export nodes