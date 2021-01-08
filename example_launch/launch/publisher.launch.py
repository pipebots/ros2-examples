#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Note: the namespace must match if the two nodes are going to talk to each other!
    return LaunchDescription([
        Node(
            package='publisher_server',
            namespace='publisher_test_ns',
            executable='publisher_server_exec',
            name='p_s'
        ),
        Node(
            package='publisher_client',
            namespace='publisher_test_ns',
            executable='publisher_client_exec',
            name='p_c'
        )
    ])
