from setuptools import setup

package_name = 'isaac_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.launch'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'isaac_examples/launch/isaac_sim_launch.py',
            'isaac_examples/launch/perception_pipeline_launch.py',
            'isaac_examples/launch/navigation_launch.py',
            'isaac_examples/launch/ai_integration_launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'isaac_examples/config/perception_params.yaml',
            'isaac_examples/config/nav2_params.yaml',
            'isaac_examples/config/isaac_sim_config.json',
            'isaac_examples/config/ai_perception_config.yaml'
        ]),
        ('share/' + package_name + '/urdf', [
            'isaac_examples/urdf/humanoid_isaac_robot.urdf'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac Developer',
    maintainer_email='isaac@example.com',
    description='Examples for NVIDIA Isaac integration with ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_sim_integration = isaac_examples.isaac_sim_integration:main',
            'perception_pipeline = isaac_examples.perception_pipeline:main',
            'vslam_demo = isaac_examples.vslam_demo:main',
            'nav2_configurator = isaac_examples.nav2_configurator:main',
            'ai_perception_node = isaac_examples.ai_perception_node:main',
        ],
    },
)