from setuptools import setup
from glob import glob
import os

package_name = 'patrol_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adarsh',
    maintainer_email='adarsh@example.com',
    description='Patrol Control Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'control_node = patrol_control.control_node:main',
            'teleop_node = patrol_control.teleop_node:main',
            'emergency_node = patrol_control.emergency_node:main',
            'patrol_controller = patrol_control.patrol_controller:main',
        ],
    },
)
