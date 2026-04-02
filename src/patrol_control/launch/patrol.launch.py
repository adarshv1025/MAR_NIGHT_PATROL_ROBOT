"""
Night Patrol Robot Launch File
===============================
Launches the patrol controller node with parameter configuration.

Usage:
    ros2 launch patrol_control patrol.launch.py
    
    # With custom parameters:
    ros2 launch patrol_control patrol.launch.py linear_speed:=0.5
    
    # With custom config file:
    ros2 launch patrol_control patrol.launch.py config_file:=/path/to/config.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for patrol controller"""
    
    # Get package directory
    pkg_share = FindPackageShare('patrol_control')
    
    # Default config file path
    default_config_path = PathJoinSubstitution([
        pkg_share,
        'config',
        'patrol_params.yaml'
    ])
    
    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Full path to patrol parameter YAML file'
    )
    
    declare_linear_speed = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.3',
        description='Maximum linear velocity (m/s)'
    )
    
    declare_angular_speed = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.5',
        description='Maximum angular velocity (rad/s)'
    )
    
    declare_obstacle_threshold = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='0.8',
        description='Minimum distance to obstacle before avoidance (m)'
    )
    
    declare_noise_enabled = DeclareLaunchArgument(
        'noise_enabled',
        default_value='false',
        description='Enable random path deviation for realism'
    )
    
    # Patrol controller node
    patrol_controller_node = Node(
        package='patrol_control',
        executable='patrol_controller',
        name='patrol_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
                'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
                'noise_enabled': LaunchConfiguration('noise_enabled'),
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
            # ('cmd_vel', '/robot/cmd_vel'),
        ],
    )
    
    # Log info
    log_info = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            'Night Patrol Robot Controller Started\n',
            '=' * 60, '\n',
            'Configuration:\n',
            '  Config file: ', LaunchConfiguration('config_file'), '\n',
            '  Linear speed: ', LaunchConfiguration('linear_speed'), ' m/s\n',
            '  Angular speed: ', LaunchConfiguration('angular_speed'), ' rad/s\n',
            '  Obstacle threshold: ', LaunchConfiguration('obstacle_threshold'), ' m\n',
            '  Noise enabled: ', LaunchConfiguration('noise_enabled'), '\n',
            '=' * 60, '\n',
            'Topics:\n',
            '  Publishing: /cmd_vel (geometry_msgs/Twist)\n',
            '  Publishing: /patrol_status (std_msgs/String)\n',
            '  Subscribing: /scan (sensor_msgs/LaserScan)\n',
            '=' * 60, '\n'
        ]
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_linear_speed,
        declare_angular_speed,
        declare_obstacle_threshold,
        declare_noise_enabled,
        log_info,
        patrol_controller_node,
    ])
