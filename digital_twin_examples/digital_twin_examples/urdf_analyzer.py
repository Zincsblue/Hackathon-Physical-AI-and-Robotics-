import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET
import os


class URDFAnalyzer(Node):
    """
    A tool for analyzing URDF files in the context of digital twin applications.
    This demonstrates how to parse and work with URDF files programmatically.
    """

    def __init__(self):
        super().__init__('urdf_analyzer')

        # Set the path to the digital twin robot URDF
        self.urdf_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'urdf',
            'digital_twin_robot.urdf'
        )

        # Analyze the URDF file
        self.analyze_urdf()

    def analyze_urdf(self):
        """
        Analyze the URDF file and print information about it.
        """
        try:
            # Parse the URDF file
            tree = ET.parse(self.urdf_path)
            root = tree.getroot()

            # Get robot name
            robot_name = root.get('name')
            self.get_logger().info(f'Robot Name: {robot_name}')

            # Count links
            links = root.findall('link')
            self.get_logger().info(f'Number of Links: {len(links)}')

            # Count joints
            joints = root.findall('joint')
            self.get_logger().info(f'Number of Joints: {len(joints)}')

            # Print link names
            link_names = [link.get('name') for link in links]
            self.get_logger().info(f'Links: {link_names}')

            # Print joint names and types
            for joint in joints:
                joint_name = joint.get('name')
                joint_type = joint.get('type')
                self.get_logger().info(f'Joint: {joint_name} (Type: {joint_type})')

            # Analyze joint limits for revolute joints
            revolute_joints = [j for j in joints if j.get('type') == 'revolute']
            self.get_logger().info(f'Number of Revolute Joints: {len(revolute_joints)}')

            for joint in revolute_joints:
                joint_name = joint.get('name')
                limit = joint.find('limit')
                if limit is not None:
                    lower = limit.get('lower')
                    upper = limit.get('upper')
                    self.get_logger().info(f'Joint {joint_name}: limits ({lower}, {upper})')

        except ET.ParseError as e:
            self.get_logger().error(f'Error parsing URDF file: {str(e)}')
        except FileNotFoundError:
            self.get_logger().error(f'URDF file not found: {self.urdf_path}')
        except Exception as e:
            self.get_logger().error(f'Error analyzing URDF: {str(e)}')

    def get_joint_names(self):
        """
        Get a list of all joint names from the URDF.
        """
        try:
            tree = ET.parse(self.urdf_path)
            root = tree.getroot()
            joints = root.findall('joint')
            return [joint.get('name') for joint in joints if joint.get('type') != 'fixed']
        except:
            return []

    def validate_urdf(self):
        """
        Validate the URDF file for basic structure and required elements.
        Returns a list of validation issues found.
        """
        issues = []
        try:
            tree = ET.parse(self.urdf_path)
            root = tree.getroot()

            # Check if robot element exists
            if root.tag != 'robot':
                issues.append('URDF must have a root "robot" element')
                return issues

            # Check if robot has a name
            if not root.get('name'):
                issues.append('Robot element must have a name attribute')

            # Get all links and joints
            links = root.findall('link')
            joints = root.findall('joint')

            # Check for duplicate link names
            link_names = [link.get('name') for link in links]
            if len(link_names) != len(set(link_names)):
                issues.append('Duplicate link names found')

            # Check for duplicate joint names
            joint_names = [joint.get('name') for joint in joints]
            if len(joint_names) != len(set(joint_names)):
                issues.append('Duplicate joint names found')

            # Check if all joint parent/child links exist
            all_link_names = set(link_names)
            for joint in joints:
                parent = joint.find('parent')
                child = joint.find('child')

                if parent is not None:
                    parent_link = parent.get('link')
                    if parent_link and parent_link not in all_link_names:
                        issues.append(f'Joint "{joint.get("name")}" references non-existent parent link "{parent_link}"')

                if child is not None:
                    child_link = child.get('link')
                    if child_link and child_link not in all_link_names:
                        issues.append(f'Joint "{joint.get("name")}" references non-existent child link "{child_link}"')

            # Check for at least one link
            if len(links) == 0:
                issues.append('URDF must contain at least one link')

            # Check for a fixed joint connecting to base (for connectedness)
            # This is a simplified check - in a real URDF, we'd want to ensure all links are connected
            fixed_joints = [j for j in joints if j.get('type') == 'fixed']
            if len(links) > 1 and len(joints) == 0:
                issues.append('Multiple links with no joints to connect them')

            return issues

        except ET.ParseError as e:
            issues.append(f'XML Parse Error: {str(e)}')
            return issues
        except FileNotFoundError:
            issues.append(f'URDF file not found: {self.urdf_path}')
            return issues
        except Exception as e:
            issues.append(f'Error validating URDF: {str(e)}')
            return issues


def main(args=None):
    """
    Main function to run the URDF analyzer.
    """
    rclpy.init(args=args)

    try:
        analyzer = URDFAnalyzer()

        # Print joint names as a simple demonstration
        joint_names = analyzer.get_joint_names()
        analyzer.get_logger().info(f'Movable joints: {joint_names}')

        # Validate the URDF file
        validation_issues = analyzer.validate_urdf()
        if validation_issues:
            analyzer.get_logger().warn('URDF Validation Issues Found:')
            for issue in validation_issues:
                analyzer.get_logger().warn(f'  - {issue}')
        else:
            analyzer.get_logger().info('URDF validation passed - no issues found')

    except KeyboardInterrupt:
        print('URDF analyzer stopped by user')
    except Exception as e:
        print(f'Error in URDF analyzer: {str(e)}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()