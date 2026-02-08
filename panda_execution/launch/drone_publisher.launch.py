#!/usr/bin/env -S ros2 launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path al tuo file URDF
    urdf_path = '/home/alexsoft55/ws/x500.urdf' 
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. Robot State Publisher con Remapping
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            # Aggiungiamo il remapping qui
            remappings=[
                ('robot_description', 'drone_description')
            ]
        ),
        # 2. Il tuo nodo Bridge PX4 -> TF
        Node(
            package='panda_execution',
            executable='px4_to_tf.py',
            name='px4_to_tf'
        ),
    ])