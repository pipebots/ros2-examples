#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='publisher_server',
            namespace='publisher_server_ns',
            executable='publisher_server_exec',
            name='p_s'
        ),
        Node(
            package='publisher_client',
            namespace='publisher_client_ns',
            executable='publisher_client_exec',
            name='p_c'
        )
    ])
