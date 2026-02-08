#!/usr/bin/env -S ros2 launch
from os import path
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description() -> LaunchDescription:
    declared_arguments = generate_declared_arguments()

    world_type = LaunchConfiguration("world_type")
    robot_type = LaunchConfiguration("robot_type")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    log_level = LaunchConfiguration("log_level")
    
    # Argomenti passanti
    launch_gz = LaunchConfiguration("launch_gz")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    declared_arguments.append(DeclareLaunchArgument("__world_launch_basename", default_value=["world_", world_type, ".launch.py"]))
    declared_arguments.append(DeclareLaunchArgument("__robot_launch_basename", default_value=["robot_", robot_type, ".launch.py"]))

    launch_descriptions = [
        # Launch World (Condizionale all'interno del file world)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("panda_execution"), "launch", "worlds", LaunchConfiguration("__world_launch_basename")])
            ),
            launch_arguments=[
                ("use_sim_time", use_sim_time),
                ("gz_verbosity", gz_verbosity), 
                ("log_level", log_level),
                ("launch_gz", launch_gz), # Passiamo il flag
            ],
        ),
        # Spawn robot (Con coordinate)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("panda_execution"), "launch", "robots", LaunchConfiguration("__robot_launch_basename")])
            ),
            launch_arguments=[
                ("use_sim_time", use_sim_time),
                ("gz_verbosity", gz_verbosity),
                ("log_level", log_level),
                ("x", x), ("y", y), ("z", z), # Passiamo coordinate
            ],
        ),
        # Launch move_group (Invariato)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare([robot_type, "_moveit_config"]), "launch", "move_group.launch.py"])
            ),
            launch_arguments=[
                ("ros2_control_plugin", "gz"), 
                ("ros2_control_command_interface", "position"),
                #TODO: Re-enable colligion geometry for manipulator arm once spawning with specific joint configuration is enabled
                ("collision_arm", "false"),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
                ("safety_limits", "false"),
                 # AGGIUNGI QUESTA RIGA:
                ("capabilities", "move_group/ExecuteTaskSolutionCapability"),
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions)

def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument("world_type", default_value="default"),
        DeclareLaunchArgument("robot_type", default_value="panda"),
        DeclareLaunchArgument("rviz_config", default_value=path.join(get_package_share_directory("panda_execution"), "rviz", "ign_moveit2_examples.rviz")),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gz_verbosity", default_value="2"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        # Nuovi argomenti passanti
        DeclareLaunchArgument("launch_gz", default_value="false", description="Launch Gazebo Simulator"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),
    ]