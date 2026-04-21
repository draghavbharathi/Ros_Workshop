#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gz_link_attacher',
            executable='link_attacher_service_node.py',
            name='link_attacher_service_node',
            output='screen',
        ),
    ])
