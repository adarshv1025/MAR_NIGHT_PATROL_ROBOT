from setuptools import find_packages, setup

package_name = 'patrol_bot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='night_patrol_team',
    maintainer_email='night.patrol@example.com',
    description='Waypoint patrol and obstacle avoidance nodes for night patrol robot.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'patrol = patrol_bot_navigation.patrol:main',
            'avoid = patrol_bot_navigation.avoid:main',
        ],
    },
)
