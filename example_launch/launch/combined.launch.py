#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='combined_server',
            namespace='combined_server_ns',
            executable='combined_server_exec',
            name='c_s'
        ),
        Node(
            package='combined_client',
            namespace='combined_client_ns',
            executable='combined_client_exec',
            name='c_c'
        )
    ])
