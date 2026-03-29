from setuptools import setup

package_name = 'patrol_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
        ],
    },
)
