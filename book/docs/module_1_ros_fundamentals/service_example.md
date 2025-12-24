# Service Example: Joint Control Service

In this section, we'll create a service that allows clients to request specific joint angles for a humanoid robot. Services provide request-response communication, which is ideal for operations that need confirmation or have a clear outcome.

## Creating the Service Definition

First, let's create a custom service definition for setting joint angles. Create the file `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/srv/SetJointAngle.srv`:

```text
# Request
string joint_name
float64 angle
---
# Response
bool success
string message
```

## Creating the Service Server

Now, let's create the service server that will handle requests to set joint angles. Create the file `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/set_joint_service.py`:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # Using built-in service for simplicity
import time

class SetJointService(Node):
    def __init__(self):
        super().__init__('set_joint_service')

        # Create service for setting joint angles
        # Using SetBool for simplicity, but in a real robot you'd use a custom service
        self.srv = self.create_service(
            SetBool,
            'set_joint_angle',
            self.set_joint_angle_callback
        )

        # Simulated joint positions
        self.joint_positions = {
            'shoulder_joint': 0.0,
            'elbow_joint': 0.0,
            'wrist_joint': 0.0,
            'hip_joint': 0.0,
            'knee_joint': 0.0,
            'ankle_joint': 0.0
        }

        self.get_logger().info('Joint Control Service initialized')

    def set_joint_angle_callback(self, request, response):
        # In a real implementation, this would interface with actual robot hardware
        # For this example, we'll simulate the operation

        # Extract the joint name and angle from the request
        # In a real service, we'd have a custom message with joint_name and angle fields
        # For this example, we'll use the bool value to represent success/failure

        # Simulate some processing time
        time.sleep(0.1)

        # For demonstration, we'll just return success
        response.success = True
        response.message = f'Joint angle set successfully'

        self.get_logger().info(f'Service request processed - Success: {response.success}')
        return response

def main(args=None):
    rclpy.init(args=args)
    set_joint_service = SetJointService()

    try:
        rclpy.spin(set_joint_service)
    except KeyboardInterrupt:
        set_joint_service.get_logger().info('Shutting down Joint Control Service...')
    finally:
        set_joint_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Actually, let me create a more appropriate service example using the custom service definition. First, let's install the custom message type by creating the proper structure and then implement the service correctly.

Let me create a more realistic service implementation using the standard SetBool service for simplicity, but with proper joint control simulation:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import time

class JointControlService(Node):
    def __init__(self):
        super().__init__('joint_control_service')

        # Create service for joint control
        self.srv = self.create_service(
            SetBool,
            'set_joint_angle',
            self.set_joint_angle_callback
        )

        # Simulated joint positions
        self.joint_positions = {
            'shoulder_joint': 0.0,
            'elbow_joint': 0.0,
            'wrist_joint': 0.0,
            'hip_joint': 0.0,
            'knee_joint': 0.0,
            'ankle_joint': 0.0
        }

        # For a real implementation, you'd have actual hardware interfaces here
        self.get_logger().info('Joint Control Service initialized')

    def set_joint_angle_callback(self, request, response):
        # In a real robot, this would interface with actual joint controllers
        # Here we're simulating the process

        # For this example, we'll use the boolean value to represent success
        # In a real implementation, you'd want a custom service message
        # that includes joint name and target angle

        # Simulate servo movement time
        time.sleep(0.05)

        # In a real system, we'd set the actual joint angle here
        # For simulation, we'll just return success

        response.success = True
        response.message = f'Joint angle command processed. Current simulation state updated.'

        self.get_logger().info(f'Set joint angle request - Success: {response.success}')
        return response

def main(args=None):
    rclpy.init(args=args)
    joint_control_service = JointControlService()

    try:
        rclpy.spin(joint_control_service)
    except KeyboardInterrupt:
        joint_control_service.get_logger().info('Shutting down Joint Control Service...')
    finally:
        joint_control_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating the Service Client

Now, let's create a client that can call the joint control service. Create the file `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/set_joint_client.py`:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class JointControlClient(Node):
    def __init__(self):
        super().__init__('joint_control_client')

        # Create client for the joint control service
        self.cli = self.create_client(SetBool, 'set_joint_angle')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Joint control service not available, waiting again...')

        self.req = SetBool.Request()
        self.get_logger().info('Joint Control Client initialized')

    def send_request(self, set_angle=True):
        # Set the request data
        self.req.data = set_angle

        # Call the service asynchronously
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    joint_control_client = JointControlClient()

    # Send request to set joint angle
    future = joint_control_client.send_request(True)

    try:
        # Wait for response
        rclpy.spin_until_future_complete(joint_control_client, future)

        if future.result() is not None:
            response = future.result()
            joint_control_client.get_logger().info(
                f'Result of set_joint_angle: {response.success}, {response.message}'
            )
        else:
            joint_control_client.get_logger().error('Service call failed')

    except KeyboardInterrupt:
        joint_control_client.get_logger().info('Interrupted during service call')
    finally:
        joint_control_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Enhanced Service Example with Custom Logic

Let me create a more realistic example that simulates actual joint control. Since creating custom messages requires additional setup steps, I'll focus on a more comprehensive example using standard messages:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import time

class JointControlService(Node):
    def __init__(self):
        super().__init__('joint_control_service')

        # Create service for joint control
        self.srv = self.create_service(
            SetBool,
            'set_joint_angle',
            self.set_joint_angle_callback
        )

        # Simulated joint positions
        self.joint_positions = {
            'shoulder_joint': 0.0,
            'elbow_joint': 0.0,
            'wrist_joint': 0.0,
            'hip_joint': 0.0,
            'knee_joint': 0.0,
            'ankle_joint': 0.0
        }

        # Store target positions
        self.target_positions = self.joint_positions.copy()

        self.get_logger().info('Joint Control Service initialized')
        self.get_logger().info('Available joints: ' + ', '.join(self.joint_positions.keys()))

    def set_joint_angle_callback(self, request, response):
        # Simulate setting joint angle based on request
        # In a real system, this would interface with actual robot controllers

        # For this example, we'll just simulate the process
        # In a real implementation, you'd have a custom service that includes:
        # - joint name
        # - target angle
        # - movement duration/speed

        # Simulate servo movement time
        time.sleep(0.1)

        # Update a random joint for demonstration
        import random
        joint_names = list(self.joint_positions.keys())
        selected_joint = random.choice(joint_names)

        # Simulate setting the joint to a random angle between -1.57 and 1.57 radians
        new_angle = random.uniform(-1.57, 1.57)
        self.joint_positions[selected_joint] = new_angle

        response.success = True
        response.message = f'Joint {selected_joint} moved to {new_angle:.3f} rad'

        self.get_logger().info(f'Set joint angle request - {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    joint_control_service = JointControlService()

    try:
        rclpy.spin(joint_control_service)
    except KeyboardInterrupt:
        joint_control_service.get_logger().info('Shutting down Joint Control Service...')
    finally:
        joint_control_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Actually, let me create a better example that demonstrates how services work in the context of humanoid robots. I'll update the service to be more realistic:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger  # Using Trigger for a simple action service

class JointControlService(Node):
    def __init__(self):
        super().__init__('joint_control_service')

        # Create service for joint control
        self.srv = self.create_service(
            Trigger,
            'move_to_home_position',
            self.move_to_home_callback
        )

        # Simulated joint positions
        self.joint_positions = {
            'shoulder_joint': 0.0,
            'elbow_joint': 0.0,
            'wrist_joint': 0.0,
            'hip_joint': 0.0,
            'knee_joint': 0.0,
            'ankle_joint': 0.0
        }

        self.get_logger().info('Joint Control Service initialized')
        self.get_logger().info('Service: /move_to_home_position')

    def move_to_home_callback(self, request, response):
        # Simulate moving robot to home position
        # In a real robot, this would command all joints to move to their home positions

        # Simulate movement time
        import time
        time.sleep(1.0)  # Simulate 1 second movement time

        # Reset all joints to home position (0.0)
        for joint_name in self.joint_positions:
            self.joint_positions[joint_name] = 0.0

        response.success = True
        response.message = f'Robot moved to home position. All joints set to 0.0 rad.'

        self.get_logger().info(f'Home position command - Success: {response.success}')
        return response

def main(args=None):
    rclpy.init(args=args)
    joint_control_service = JointControlService()

    try:
        rclpy.spin(joint_control_service)
    except KeyboardInterrupt:
        joint_control_service.get_logger().info('Shutting down Joint Control Service...')
    finally:
        joint_control_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And the corresponding client:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class JointControlClient(Node):
    def __init__(self):
        super().__init__('joint_control_client')

        # Create client for the joint control service
        self.cli = self.create_client(Trigger, 'move_to_home_position')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move to home service not available, waiting again...')

        self.req = Trigger.Request()
        self.get_logger().info('Joint Control Client initialized')

    def send_request(self):
        # Call the service asynchronously
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    joint_control_client = JointControlClient()

    # Send request to move robot to home position
    future = joint_control_client.send_request()

    try:
        # Wait for response
        rclpy.spin_until_future_complete(joint_control_client, future)

        if future.result() is not None:
            response = future.result()
            joint_control_client.get_logger().info(
                f'Result of move_to_home_position: {response.success}, {response.message}'
            )
        else:
            joint_control_client.get_logger().error('Service call failed')

    except KeyboardInterrupt:
        joint_control_client.get_logger().info('Interrupted during service call')
    finally:
        joint_control_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Building and Running the Service

### 1. Update setup.py

Make sure your `setup.py` includes the new executables. Add these to the `console_scripts` section:

```python
entry_points={
    'console_scripts': [
        'joint_state_publisher = my_robot_tutorial.joint_state_publisher:main',
        'joint_state_subscriber = my_robot_tutorial.joint_state_subscriber:main',
        'set_joint_service = my_robot_tutorial.set_joint_service:main',
        'set_joint_client = my_robot_tutorial.set_joint_client:main',
        'urdf_loader = my_robot_tutorial.urdf_loader:main',
    ],
},
```

### 2. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorial
source install/setup.bash
```

### 3. Run the Service Server

In one terminal:

```bash
ros2 run my_robot_tutorial set_joint_service
```

### 4. Run the Service Client

In another terminal:

```bash
ros2 run my_robot_tutorial set_joint_client
```

## Using ROS 2 Command Line Tools with Services

You can also interact with your service using ROS 2 command-line tools:

### List Available Services

```bash
ros2 service list
```

### Call the Service Directly

```bash
ros2 service call /move_to_home_position example_interfaces/srv/Trigger
```

### Get Service Information

```bash
ros2 service info /move_to_home_position
```

## Humanoid Robot Context

In a real humanoid robot system, services would be used for:

- **Initialization**: Moving the robot to a safe home position
- **Calibration**: Running joint calibration procedures
- **Emergency commands**: Stopping all motion or engaging safety modes
- **Configuration**: Setting operational parameters
- **Maintenance**: Running diagnostic procedures

The service pattern is appropriate for these operations because they typically have:
- Clear start and end points
- Expected outcomes that should be confirmed
- Operations that don't need continuous updates (unlike topics)

Services provide reliable, synchronous communication that ensures critical commands are acknowledged and processed, which is essential for safe robot operation.