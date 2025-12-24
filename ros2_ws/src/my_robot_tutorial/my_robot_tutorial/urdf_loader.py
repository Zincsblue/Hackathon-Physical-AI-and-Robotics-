import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class URDFLoader(Node):
    def __init__(self):
        super().__init__('urdf_loader')

        # This node serves as an example for loading URDF in the context
        # It publishes simple joint states to demonstrate the URDF functionality

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Joint names as defined in our URDF
        self.joint_names = [
            'torso_to_left_shoulder', 'left_shoulder_to_elbow',
            'torso_to_right_shoulder', 'right_shoulder_to_elbow',
            'torso_to_left_hip', 'left_hip_to_knee',
            'torso_to_right_hip', 'right_hip_to_knee'
        ]

        self.i = 0
        self.get_logger().info('URDF Loader node initialized')

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Generate oscillating joint positions for visualization
        positions = []
        for j, name in enumerate(self.joint_names):
            # Create different oscillation patterns for each joint
            pos = math.sin(self.i * 0.05 + j * 0.5) * 0.5
            positions.append(pos)

        msg.position = positions
        msg.velocity = [0.0] * len(positions)
        msg.effort = [0.0] * len(positions)

        self.publisher.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    urdf_loader = URDFLoader()

    try:
        rclpy.spin(urdf_loader)
    except KeyboardInterrupt:
        pass
    finally:
        urdf_loader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()