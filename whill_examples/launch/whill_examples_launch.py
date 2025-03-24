# Copyright (c) 2024 WHILL, Inc.
# Released under the MIT license
# https://opensource.org/licenses/mit-license.php

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='whill_driver',
            namespace='',
            executable='whill',
            name='whill',
            parameters=[PathJoinSubstitution([FindPackageShare('whill_examples'), 'params.yaml'])],
        ),
        Node(
            package='whill_examples',
            namespace='',
            executable='turn_demo_client',
            name='client'
        ),
    ])