from setuptools import setup

package_name = 'patrol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'intruder_detector = patrol_robot.intruder_detector:main',
            'fake_scan = patrol_robot.fake_scan:main',
        ],
    },
)
