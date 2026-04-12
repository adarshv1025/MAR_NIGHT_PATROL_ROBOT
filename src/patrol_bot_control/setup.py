from setuptools import find_packages, setup

package_name = 'patrol_bot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='night_patrol_team',
    maintainer_email='night.patrol@example.com',
    description='Control-layer nodes: command mux, emergency manager, and demo dashboard.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control = patrol_bot_control.control:main',
            'emergency = patrol_bot_control.emergency:main',
            'dashboard = patrol_bot_control.dashboard:main',
            'wasd_teleop = patrol_bot_control.wasd_teleop:main',
        ],
    },
)
