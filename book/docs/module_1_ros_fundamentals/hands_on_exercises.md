# Hands-on Exercises

This section provides practical exercises to reinforce your understanding of ROS 2 concepts using humanoid robot examples. Each exercise builds upon the concepts covered in previous sections.

## Exercise 1: Basic Publisher/Subscriber

**Objective**: Create and run a simple publisher/subscriber pair that simulates sending sensor data from a humanoid robot's IMU.

### Tasks:
1. Create a new publisher node that publishes IMU data using `sensor_msgs/Imu`
2. Create a subscriber node that receives and processes the IMU data
3. Run both nodes and verify communication
4. Use ROS 2 command-line tools to inspect the communication

### Solution Steps:

**Step 1**: Create the IMU publisher node in `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/imu_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math
import random

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(Imu, 'imu_data', 10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('IMU Publisher started')

    def timer_callback(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate IMU data
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.i * 0.05) * 0.1
        msg.orientation.w = math.cos(self.i * 0.05) * 0.1

        # Angular velocity
        msg.angular_velocity.x = math.sin(self.i * 0.1) * 0.01
        msg.angular_velocity.y = math.cos(self.i * 0.1) * 0.01
        msg.angular_velocity.z = 0.0

        # Linear acceleration
        msg.linear_acceleration.x = math.sin(self.i * 0.07) * 9.81
        msg.linear_acceleration.y = math.cos(self.i * 0.07) * 9.81
        msg.linear_acceleration.z = 9.81 + math.sin(self.i * 0.09) * 0.5

        self.publisher.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Step 2**: Create the IMU subscriber node in `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/imu_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu_data',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('IMU Subscriber started')

    def imu_callback(self, msg):
        # Extract orientation
        roll = math.atan2(2 * (msg.orientation.w * msg.orientation.x + msg.orientation.y * msg.orientation.z),
                         1 - 2 * (msg.orientation.x * msg.orientation.x + msg.orientation.y * msg.orientation.y))
        pitch = math.asin(2 * (msg.orientation.w * msg.orientation.y - msg.orientation.z * msg.orientation.x))
        yaw = math.atan2(2 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y),
                        1 - 2 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z))

        self.get_logger().info(f'IMU Data - R: {math.degrees(roll):.2f}°, P: {math.degrees(pitch):.2f}°, Y: {math.degrees(yaw):.2f}°')

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()

    try:
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        imu_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import math
    main()
```

**Step 3**: Add these to your `setup.py` console scripts:

```python
'console_scripts': [
    'joint_state_publisher = my_robot_tutorial.joint_state_publisher:main',
    'joint_state_subscriber = my_robot_tutorial.joint_state_subscriber:main',
    'set_joint_service = my_robot_tutorial.set_joint_service:main',
    'set_joint_client = my_robot_tutorial.set_joint_client:main',
    'urdf_loader = my_robot_tutorial.urdf_loader:main',
    'imu_publisher = my_robot_tutorial.imu_publisher:main',
    'imu_subscriber = my_robot_tutorial.imu_subscriber:main',
],
```

**Step 4**: Build and run:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorial
source install/setup.bash

# Terminal 1
ros2 run my_robot_tutorial imu_publisher

# Terminal 2
ros2 run my_robot_tutorial imu_subscriber
```

## Exercise 2: Custom Service for Joint Control

**Objective**: Create a custom service that allows setting a specific joint to a target angle.

### Tasks:
1. Define a custom service interface
2. Implement the service server
3. Implement the service client
4. Test the service with command-line tools

### Solution Steps:

**Step 1**: Create a custom service definition in `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/srv/SetJointAngle.srv`:

```text
# Request
string joint_name
float64 target_angle
---
# Response
bool success
string message
```

**Step 2**: Update your package.xml to include the service definition:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_tutorial</name>
  <version>0.0.0</version>
  <description>ROS 2 tutorial package for humanoid robot examples</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>example_interfaces</depend>
  <depend>builtin_interfaces</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 3**: Create the service server in `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/joint_control_server.py`:

```python
import rclpy
from rclpy.node import Node
from my_robot_tutorial.srv import SetJointAngle  # Custom service
import time

class JointControlServer(Node):
    def __init__(self):
        super().__init__('joint_control_server')

        # In a real system, this would interface with actual hardware
        # For this example, we'll simulate joint positions
        self.joint_positions = {
            'shoulder_joint': 0.0,
            'elbow_joint': 0.0,
            'wrist_joint': 0.0,
            'hip_joint': 0.0,
            'knee_joint': 0.0,
            'ankle_joint': 0.0
        }

        # Create service
        self.srv = self.create_service(
            SetJointAngle,
            'set_joint_angle',
            self.set_joint_angle_callback
        )

        self.get_logger().info('Joint Control Server started')

    def set_joint_angle_callback(self, request, response):
        joint_name = request.joint_name
        target_angle = request.target_angle

        # Validate joint name
        if joint_name not in self.joint_positions:
            response.success = False
            response.message = f'Invalid joint name: {joint_name}'
            self.get_logger().error(response.message)
            return response

        # Validate angle range (example: -pi to pi)
        if target_angle < -3.14159 or target_angle > 3.14159:
            response.success = False
            response.message = f'Angle {target_angle} out of range [-3.14159, 3.14159]'
            self.get_logger().error(response.message)
            return response

        # Simulate setting the joint (in real system, this would send command to hardware)
        time.sleep(0.05)  # Simulate movement time
        self.joint_positions[joint_name] = target_angle

        response.success = True
        response.message = f'Successfully set {joint_name} to {target_angle:.3f} rad. Previous: {self.joint_positions[joint_name]:.3f} rad'

        self.get_logger().info(f'Set {joint_name} to {target_angle:.3f} rad')
        return response

def main(args=None):
    rclpy.init(args=args)
    joint_control_server = JointControlServer()

    try:
        rclpy.spin(joint_control_server)
    except KeyboardInterrupt:
        joint_control_server.get_logger().info('Shutting down Joint Control Server...')
    finally:
        joint_control_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Step 4**: Create the service client in `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/joint_control_client.py`:

```python
import rclpy
from rclpy.node import Node
from my_robot_tutorial.srv import SetJointAngle  # Custom service

class JointControlClient(Node):
    def __init__(self):
        super().__init__('joint_control_client')

        # Create client
        self.cli = self.create_client(SetJointAngle, 'set_joint_angle')

        # Wait for service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = SetJointAngle.Request()
        self.get_logger().info('Joint Control Client started')

    def send_request(self, joint_name, target_angle):
        self.req.joint_name = joint_name
        self.req.target_angle = target_angle

        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    joint_control_client = JointControlClient()

    # Example: Set shoulder joint to 45 degrees (0.785 rad)
    future = joint_control_client.send_request('shoulder_joint', 0.785)

    try:
        rclpy.spin_until_future_complete(joint_control_client, future)

        if future.result() is not None:
            response = future.result()
            joint_control_client.get_logger().info(
                f'Service response: {response.success}, {response.message}'
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

## Exercise 3: URDF Modification

**Objective**: Modify the provided URDF to add a simple hand to one of the arms.

### Tasks:
1. Add a hand link to the left arm
2. Add a joint connecting the hand to the left lower arm
3. Update the URDF file
4. Visualize the modified robot in RViz2

### Solution Steps:

**Step 1**: Add the hand to your URDF file (`~/ros2_ws/src/my_robot_tutorial/urdf/simple_humanoid.urdf`):

```xml
  <!-- Add after left_lower_arm link -->

  <joint name="left_wrist_to_hand" type="fixed">
    <parent link="left_lower_arm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  </joint>

  <link name="left_hand">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
  </link>
```

## Exercise 4: Launch File Integration

**Objective**: Create a comprehensive launch file that starts multiple nodes at once.

### Tasks:
1. Create a launch file that starts publisher, subscriber, and service
2. Add parameters to configure the nodes
3. Launch all nodes with a single command

### Solution Steps:

**Step 1**: Create `~/ros2_ws/src/my_robot_tutorial/my_robot_tutorial/launch/comprehensive_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Joint state publisher
        Node(
            package='my_robot_tutorial',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Joint state subscriber
        Node(
            package='my_robot_tutorial',
            executable='joint_state_subscriber',
            name='joint_state_subscriber',
            output='screen'
        ),

        # IMU publisher
        Node(
            package='my_robot_tutorial',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen'
        ),

        # IMU subscriber
        Node(
            package='my_robot_tutorial',
            executable='imu_subscriber',
            name='imu_subscriber',
            output='screen'
        ),

        # Robot state publisher for URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(os.path.join(
                    get_package_share_directory('my_robot_tutorial'),
                    'urdf',
                    'simple_humanoid.urdf'
                )).read()
            }],
            output='screen'
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
```

**Step 2**: Run the comprehensive launch:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorial
source install/setup.bash
ros2 launch my_robot_tutorial comprehensive_launch.py
```

## Exercise 5: Command-Line Tool Exploration

**Objective**: Use ROS 2 command-line tools to inspect your running system.

### Tasks:
1. List all active nodes
2. List all active topics
3. Echo messages from a topic
4. List all services
5. Call a service using command line

### Commands to Try:

```bash
# List nodes
ros2 node list

# List topics
ros2 topic list

# Echo joint states
ros2 topic echo /joint_states

# List services
ros2 service list

# Check a specific node
ros2 node info /joint_state_publisher

# Check topic info
ros2 topic info /joint_states

# Check service info
ros2 service info /set_joint_angle
```

## Summary Checklist

After completing these exercises, you should be able to:

- [ ] Create and run publisher/subscriber nodes
- [ ] Implement custom services
- [ ] Modify URDF files to add new links and joints
- [ ] Use launch files to start multiple nodes
- [ ] Use ROS 2 command-line tools for system inspection
- [ ] Visualize robots in RViz2
- [ ] Understand the structure of ROS 2 packages
- [ ] Debug basic ROS 2 communication issues

## Next Steps

After completing these exercises, you'll have a solid foundation in ROS 2 fundamentals for humanoid robotics. Consider exploring:

1. More complex robot models with multiple kinematic chains
2. ROS 2 actions for long-running tasks with feedback
3. Robot kinematics and inverse kinematics
4. Robot simulation with Gazebo (in future modules)
5. Robot control with ros2_controllers