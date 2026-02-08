#!/usr/bin/env -S ros2 launch
from os import path
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition # Importante

def generate_launch_description() -> LaunchDescription:
    declared_arguments = generate_declared_arguments()

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    log_level = LaunchConfiguration("log_level")
    launch_gz = LaunchConfiguration("launch_gz") # Flag

    launch_descriptions = [
        # Launch Gazebo Harmonic (SOLO SE launch_gz è true)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
            ),
            launch_arguments=[("gz_args", [world, " -r -v ", gz_verbosity])],
            condition=IfCondition(launch_gz) # Condizione Magica
        ),
    ]
    # Questo nodo è il "ponte" che sposta tutto il robot su RViz
    # Deve corrispondere ESATTAMENTE a dove spawni il robot in Gazebo
    
    nodes = [
        # I bridge servono sempre, anche se Gazebo è lanciato esternamente
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="log",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "--ros-args", "--log-level", log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]
    
    return LaunchDescription(declared_arguments + launch_descriptions + nodes)

def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument("world", default_value=path.join(get_package_share_directory("panda_execution"), "worlds", "my_world.sdf")),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gz_verbosity", default_value="2"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("launch_gz", default_value="false"), # Default true per compatibilità
    ]