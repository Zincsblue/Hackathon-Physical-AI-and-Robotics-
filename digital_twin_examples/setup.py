from setuptools import setup
import os
from glob import glob

package_name = 'digital_twin_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Digital Twin Student',
    maintainer_email='student@physical-ai.org',
    description='Digital Twin examples for Python Agents & ROS 2 Controller Integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_agent = digital_twin_examples.python_agent:main',
            'ros2_controller = digital_twin_examples.ros2_controller:main',
            'digital_twin_bridge = digital_twin_examples.digital_twin_bridge:main',
            'urdf_analyzer = digital_twin_examples.urdf_analyzer:main',
        ],
    },
)