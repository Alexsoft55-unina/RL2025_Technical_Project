#!/usr/bin/env -S ros2 launch
from typing import List
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() -> LaunchDescription:
    declared_arguments = generate_declared_arguments()

    model = LaunchConfiguration("model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")


    sdf_file_path = PathJoinSubstitution([
        FindPackageShare("panda_description"),
        "panda", 
        "model.sdf"
    ])
    panda_description_share = get_package_share_directory("panda_description")
    gz_resource_path_update = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(os.path.dirname(panda_description_share))
    )

    initial_joint_positions = [
    '-J', 'panda_joint1', '0.0',
    '-J', 'panda_joint2', '-0.785',
    '-J', 'panda_joint3', '0.0',
    '-J', 'panda_joint4', '-2.356',
    '-J', 'panda_joint5', '0.0',
    '-J', 'panda_joint6', '1.571',
    '-J', 'panda_joint7', '0.785',
]

    # 2. Modifica il comando di spawn per usare "-file" invece di "-topic"
    spawn_robot_gemini = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-topic", "robot_description",     # <--- QUI: Usiamo il file SDF diretto
            "-name", "panda",           # Importante: deve coincidere col nome nel SDF
            *initial_joint_positions,
            "-x", x, "-y", y, "-z", z,  # Coordinate
            "--ros-args", "--log-level", log_level
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    spawn_robot_originale = Node(
            package="ros_gz_sim",
            executable="create",
            output="log",
            arguments=[
                "-file", model, 
                #"-name", "panda", # Importante dare un nome fisso o parametrico
                "-x", x, "-y", y, "-z", z, # Coordinate
                "--ros-args", "--log-level", log_level
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            prefix=['gnome-terminal --'],
    )


    nodes = [
        spawn_robot_gemini,
        #spawn_robot_originale,
    ]
    # Ensure robot_state_publisher is publishing correctly first
    # (This node is already in your list, just ensure it runs)

    

    return LaunchDescription(declared_arguments + [gz_resource_path_update] + nodes)

def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument("model", default_value="panda"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),
    ]