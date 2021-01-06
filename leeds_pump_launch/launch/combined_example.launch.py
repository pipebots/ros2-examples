#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leeds_pump_combined_client',
            namespace='combined_client_ns',
            executable='combined_client',
            name='c_c'
        ),
        Node(
            package='leeds_pump_combined_server',
            namespace='combined_server_ns',
            executable='combined_server',
            name='c_s'
        )
    ])
