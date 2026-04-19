from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_camera = LaunchConfiguration('enable_camera')
    use_dashboard = LaunchConfiguration('use_dashboard')
    start_teleop = LaunchConfiguration('start_teleop')
    use_control_gui = LaunchConfiguration('use_control_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    gui = LaunchConfiguration('gui')

    world_path = PathJoinSubstitution([
        FindPackageShare('patrol_bot_gazebo'),
        'worlds',
        'night.world',
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('patrol_bot_description'),
        'urdf',
        'patrol_bot.urdf.xacro',
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('patrol_bot_gazebo'),
        'rviz',
        'night_patrol.rviz',
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world_path, 'gui': gui}.items(),
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' enable_camera:=', enable_camera]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
        ],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'patrol_bot', '-x', '0.0', '-y', '0.0', '-z', '0.2'],
        output='screen',
    )

    emergency_node = Node(
        package='patrol_bot_control',
        executable='emergency',
        name='emergency_manager',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
    )

    cmd_mux_node = Node(
        package='patrol_bot_control',
        executable='control',
        name='cmd_mux',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
    )

    patrol_node = Node(
        package='patrol_bot_navigation',
        executable='patrol',
        name='patrol_node',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    avoid_node = Node(
        package='patrol_bot_navigation',
        executable='avoid',
        name='avoid_node',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    detect_node = Node(
        package='patrol_bot_detection',
        executable='detect',
        name='intruder_detection',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    teleop_node = Node(
        package='patrol_bot_control',
        executable='wasd_teleop',
        name='wasd_teleop',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_teleop),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    dashboard_node = Node(
        package='patrol_bot_control',
        executable='dashboard',
        name='demo_dashboard',
        output='screen',
        condition=IfCondition(use_dashboard),
    )

    control_gui_node = Node(
        package='patrol_bot_control',
        executable='control_gui',
        name='control_gui',
        output='screen',
        condition=IfCondition(use_control_gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('use_dashboard', default_value='false'),
        DeclareLaunchArgument('start_teleop', default_value='false'),
        DeclareLaunchArgument('use_control_gui', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        emergency_node,
        cmd_mux_node,
        patrol_node,
        avoid_node,
        detect_node,
        teleop_node,
        dashboard_node,
        control_gui_node,
        rviz_node,
    ])
