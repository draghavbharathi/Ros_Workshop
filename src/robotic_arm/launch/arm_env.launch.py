#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix


def generate_launch_description():

    package_share = FindPackageShare('robotic_arm')

    default_world = PathJoinSubstitution(
        [package_share, 'worlds', 'robotic_arm_world.sdf']
    )

    urdf_file = PathJoinSubstitution(
        [package_share, 'urdf', 'robotic_arm.urdf']
    )

    controllers_file = PathJoinSubstitution(
        [package_share, 'config', 'ros2_controllers.yaml']
    )

    gz_launch = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    rack_urdf_file = PathJoinSubstitution(
        [package_share, 'urdf', 'rack.urdf']
    )

    table_urdf_file = PathJoinSubstitution(
        [package_share, 'urdf', 'table.urdf']
    )

    box_urdf_file = PathJoinSubstitution(
        [package_share, 'urdf', 'package_box.urdf']
    )

    camera_urdf_file = PathJoinSubstitution(
        [package_share, 'urdf', 'camera.urdf']
    )

    warehouse_urdf_file = PathJoinSubstitution(
        [package_share, 'urdf', 'warehouse.urdf']
    )

    # ------------------ ROBOT DESCRIPTION ------------------

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    camera_description = ParameterValue(
        Command(['xacro ', camera_urdf_file]),
        value_type=str
    )

    camera_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='camera_state_publisher',
        parameters=[{'robot_description': camera_description, 'use_sim_time': True}],
        remappings=[
            ('/robot_description', '/camera_description')
        ],
        output='screen',
    )

    # ------------------ SPAWN ROBOT ------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world_name'),
            '-name', LaunchConfiguration('entity_name'),
            '-allow_renaming', 'true',
            '-string', Command(['xacro ', urdf_file]),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
    )

    spawn_rack = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world_name'),
            '-name', 'rack',
            '-allow_renaming', 'true',
            '-file', rack_urdf_file,
            '-x', '0.4',
            '-y', '0.3',
            '-z', '0.1',
        ],
    )

    spawn_table = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world_name'),
            '-name', 'table',
            '-allow_renaming', 'true',
            '-file', table_urdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.02',
        ],
    )

    spawn_box = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world_name'),
            '-name', 'package_box',
            '-allow_renaming', 'true',
            '-file', box_urdf_file,
            '-x', '0.0',
            '-y', '0.3',
            '-z', '0.3',
            '-P', '-1.570796327'
        ],
    )


    spawn_warehouse = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world_name'),
            '-name', 'warehouse',
            '-allow_renaming', 'true',
            '-file', warehouse_urdf_file,
            '-x', '0.0',
            '-y', '18.0',
            '-z', '0.01',
        ],
    )


    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world_name'),
            '-name', 'camera',
            '-allow_renaming', 'true',
            '-file', camera_urdf_file,
            '-x', '-1.0',
            '-y', '0.4',
            '-z', '1.1',
            '-P', '0.5',

        ],
    )


    # Static TF for Camera
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-1.0', '0.4', '1.1', '0.0', '0.5', '0.0', 'world', 'camera_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ------------------ CONTROLLERS ------------------

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/aligned_depth_to_color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/infra1/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/infra1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/infra2/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/infra2/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )
    # ------------------ LAUNCH DESCRIPTION ------------------

    return LaunchDescription([

        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the SDF world file to load',
        ),

        DeclareLaunchArgument(
            'world_name',
            default_value='robotic_arm_world',
        ),

        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),

        DeclareLaunchArgument(
            'entity_name',
            default_value='robotic_arm',
        ),

        UnsetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE'),

        SetEnvironmentVariable(
            'QT_QPA_PLATFORM',
            'xcb'
        ),

        # Let Gz Sim find the link-attacher plugin
        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            os.path.join(
                get_package_prefix('gz_link_attacher'), 'lib')
            + os.pathsep
            + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        ),

        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            [
                PathJoinSubstitution([package_share, '..']),
                os.pathsep,
                PathJoinSubstitution([package_share]),
                os.pathsep,
                EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={
                'gz_args': ['-r ', LaunchConfiguration('world')],
            }.items(),
        ),

        clock_bridge,

        # 🔥 IMPORTANT — publish robot_description FIRST
        robot_state_publisher_node,
        camera_state_publisher_node,

        # spawn warehouse first so environment is ready before robot appears
        TimerAction(
            period=1.0,
            actions=[spawn_warehouse],
        ),

        # spawn table first so environment is ready before robot appears
        TimerAction(
            period=2.0,
            actions=[spawn_table],
        ),

        # spawn robot
        TimerAction(
            period=3.0,
            actions=[spawn_robot],
        ),

        # wait for gz_ros2_control to initialize
        TimerAction(
            period=6.0,
            actions=[spawn_joint_state_broadcaster],
        ),

        # wait for controller manager to be ready
        TimerAction(
            period=8.0,
            actions=[spawn_arm_controller],
        ),

        # spawn environment objects
        TimerAction(
            period=10.0,
            actions=[spawn_rack, spawn_box, spawn_camera, static_tf_camera],
        ),
    ])
