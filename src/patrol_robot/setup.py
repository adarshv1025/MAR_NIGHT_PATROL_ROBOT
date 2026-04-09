from setuptools import setup

package_name = 'patrol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Patrol Robot Package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'fake_scan = patrol_robot.fake_scan:main',
            'intruder_detector = patrol_robot.intruder_detector:main',
        ],
    },
)
