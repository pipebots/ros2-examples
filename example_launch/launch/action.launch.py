#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Note: the namespace must match if the two nodes are going to talk to
    # each other!
    return LaunchDescription([
        Node(
            package='action_server',
            namespace='action_test_ns',
            executable='action_server_exec',
            name='s_s'
        ),
        Node(
            package='action_client',
            namespace='action_test_ns',
            executable='action_client_exec',
            name='s_c'
        )
    ])
