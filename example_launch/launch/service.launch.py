#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Note: the namespace must match if the two nodes are going to talk to
    # each other!
    return LaunchDescription([
        Node(
            package='service_server',
            namespace='service_test_ns',
            executable='service_server_exec'
        ),
        Node(
            package='service_client',
            namespace='service_test_ns',
            executable='service_client_exec'
        )
    ])
