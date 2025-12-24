from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_tutorial'

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
    maintainer='ROS 2 Fundamentals Student',
    maintainer_email='student@physical-ai.org',
    description='ROS 2 tutorial package for humanoid robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = my_robot_tutorial.joint_state_publisher:main',
            'joint_state_subscriber = my_robot_tutorial.joint_state_subscriber:main',
            'set_joint_service = my_robot_tutorial.set_joint_service:main',
            'set_joint_client = my_robot_tutorial.set_joint_client:main',
        ],
    },
)