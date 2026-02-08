#!/usr/bin/env -S ros2 launch
"""Launch Python example for MTC Pick and Place Strategy B (Spawn in existing World)"""

from os import path
from typing import List
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                            RegisterEventHandler, ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description() -> LaunchDescription:
    declared_arguments = generate_declared_arguments()

    # Argomenti base
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    moveit_config_package = "panda_moveit_config"
    robot_type = LaunchConfiguration("robot_type")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_verbosity = LaunchConfiguration("gz_verbosity") 
    log_level = LaunchConfiguration("log_level")
    
    # Argomenti Custom
    launch_gz = LaunchConfiguration("launch_gz")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    # --- 1. Load Robot Description ---
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), description_filepath]),
            " ",
            "name:=", robot_type,
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    _robot_description_semantic_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", "panda.srdf.xacro"]),
            " ",
            "name:=", robot_type,
        ]
    )
    robot_description_semantic = {"robot_description_semantic": _robot_description_semantic_xml}

    # Caricamento configurazioni MoveIt per il nodo MTC locale
    kinematics_yaml = load_yaml(moveit_config_package, "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    joint_limits_yaml = load_yaml(moveit_config_package, "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}
    trajectory_execution_yaml = load_yaml(moveit_config_package, "config/trajectory_execution.yaml")
    moveit_controllers_yaml = load_yaml(moveit_config_package, "config/moveit_controller_manager.yaml")
    
    ompl_planning_yaml = load_yaml(moveit_config_package, "config/ompl_planning.yaml")
    adapters = [
        "default_planner_request_adapters/FixWorkspaceBounds",
        "default_planner_request_adapters/FixStartStateBounds",
        "default_planner_request_adapters/FixStartStateCollision",
        "default_planner_request_adapters/FixStartStatePathConstraints",
        "default_planner_request_adapters/AddTimeOptimalParameterization"
    ]
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join(adapters),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # --- 2. Include Chain (Default -> MoveGroup -> Controllers) ---
    # Questa catena lancerà 'move_group.launch.py', il quale leggerà il YAML
    # modificato al punto 1 e lancerà gli spawner corretti.
    launch_descriptions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("panda_execution"), "launch", "default.launch.py"])
            ),
            launch_arguments=[
                ("world_type", "mine"),
                ("robot_type", robot_type),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("gz_verbosity", gz_verbosity),
                ("log_level", log_level),
                ("launch_gz", launch_gz), 
                ("x", x), ("y", y), ("z", z),
            ],
        ),
    ]

    # --- 3. Nodi Locali MTC e Bridge ---
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/panda/attach@std_msgs/msg/Empty@gz.msgs.Empty',
            '/panda/detach@std_msgs/msg/Empty@gz.msgs.Empty',
            '/panda/state@std_msgs/msg/String[gz.msgs.StringMsg',
            '/model/scatola/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
        ],
        remappings=[('/model/scatola/pose', '/target_pose')],
        output='log'
    )
    # Questo nodo ascolta /panda/state e lo ri-pubblica periodicamente su /panda/current_grip_status
    grip_state_publisher = Node(
        package='panda_execution',
        executable='grip_monitor_node.py', 
        output='screen'
    )
    trajectory_commander = Node(
        package="panda_execution",
        executable="trajectory_selector_commander",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution_yaml,   
            moveit_controllers_yaml,      
            {"use_sim_time": use_sim_time},
            {"object_name": "scatola"},
        ],
    )

    trajectory_action = Node(
        package="panda_execution",
        executable="trajectory_action",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution_yaml,   
            moveit_controllers_yaml,      
            {"use_sim_time": use_sim_time},
            {"object_name": "scatola"},
        ],
    )
    Node_manager= Node(
        package="panda_execution",
        executable="node_manager",
        output="screen",
    )
    Panda_scan_server= Node(
        package="panda_execution",
        executable="panda_scan_server",
        output="screen",
    )
    Visual_servoing_server = Node(
        package="panda_execution",
        executable="visual_servoing",
        output="screen",
    )
    Pick_n_place_server = Node(
        package="panda_execution",
        executable="pick_n_place_manager",
        output="screen",
    )

    Pick_task_srv = Node(
        package="panda_execution",
        executable="panda_pick_task_srv",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution_yaml,   
            moveit_controllers_yaml,      
            {"use_sim_time": use_sim_time},
            {"object_name": "scatola"},
        ],
        #prefix='gnome-terminal --',
    )
    Move_task_srv = Node(
        package="panda_execution",
        executable="panda_move_task",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution_yaml,   
            moveit_controllers_yaml,      
            {"use_sim_time": use_sim_time},
            {"object_name": "scatola"},
        ],
        #prefix='gnome-terminal --',
    )

    detach_command = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/panda/detach', 'std_msgs/msg/Empty', '{}'],
        output='log'
    )

    detach_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=bridge_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[detach_command]
                )
            ]
        )
    )

    target_marker_publisher = Node(
            package="panda_execution",
            executable="target_marker_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                {"use_sim_time": use_sim_time},
            ],
        )
    
    tf_world_to_robot = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '1.0', 
            '--y', '0.0', 
            '--z', '0.0', 
            '--yaw', '0.0', 
            '--pitch', '0.0', 
            '--roll', '0.0', 
            '--frame-id', 'world', 
            '--child-frame-id', 'panda_link0'
        ],
        output='log'
    )
# -------------SPAWN SCATOLA-----------
    
    # 1. Trova il percorso del file SDF
    scatola_sdf_path = PathJoinSubstitution([
        FindPackageShare("panda_execution"),
        "worlds",
        "scatola.sdf"
    ])

    # 2. Definisci il nodo di spawn (lo stesso comando che ha funzionato da terminale)
    spawn_scatola_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'scatola',
            '-file', scatola_sdf_path,
            '-x', '1.6',
            '-y', '0.0',
            '-z', '0.4',
            '-R', '1.57'
        ],
        output='log'
    )

    # 3. Crea un Timer per ritardare lo spawn (es. 5 secondi)
    # Questo dà tempo a Gazebo di caricare il mondo prima di inserire l'oggetto
    delayed_spawn_scatola = TimerAction(
        period=3.0, 
        actions=[spawn_scatola_node]
    )

    bridge_camera = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
        '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        '--ros-args', 
        '--remap', '/camera:=/ros_camera',
        '--remap', '/camera_info:=/ros_camera_info',
    ],
    output='log'
)
        
    # --- NODO ARUCO MARKER PUBLISHER (Rileva TUTTI i tag) ---
    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.06,         # Dimensione 6cm
            'reference_frame': 'panda_camera_link', 
            'camera_frame': 'panda_camera_link',
            'use_sim_time': True ,
            'dictionary': 10,  # AGGIUNGI: 0=DICT_4X4_50, 10=DICT_ARUCO_ORIGINAL, 16=DICT_6X6_250
        }],
        remappings=[
            ('/image', '/ros_camera'),
            ('/camera_info', '/ros_camera_info')
        ],
        output='log'
    )
    bridge_set_pose = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/aruco_world/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean',
        ],
        remappings=[
            ('/world/aruco_world/set_pose', '/set_aruco_pose'),
        ],
        output='log'
        )
    
        # ----------------INITIAL POSITIONS------------------------
    send_ready_pose = ExecuteProcess(
    cmd=[[
        'ros2 topic pub --once /joint_trajectory_controller/joint_trajectory ',
        'trajectory_msgs/msg/JointTrajectory ',
        '"{joint_names: [\'panda_joint1\', \'panda_joint2\', \'panda_joint3\', \'panda_joint4\', \'panda_joint5\', \'panda_joint6\', \'panda_joint7\'], ',
        'points: [{positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], time_from_start: {sec: 2, nanosec: 0}}]}"'
    ]],
    
    shell=True
    )
    delayed_send_ready_pose = TimerAction(
        period = 7.0, # Nome dell'azione che lancia il controller
        actions=[send_ready_pose],
    )
    
    #----------------------------
    nodes = [
        bridge_node,
        #target_marker_publisher,
        detach_handler,
        delayed_spawn_scatola,
        tf_world_to_robot,
        #trajectory_action,
       #trajectory_commander,
       grip_state_publisher,

       bridge_camera,
       bridge_set_pose,
       aruco_marker_publisher,
       delayed_send_ready_pose,
       Node_manager,
       Panda_scan_server,
       Visual_servoing_server,
       Pick_task_srv,
       Pick_n_place_server,
       Move_task_srv,
    ]


    return LaunchDescription(declared_arguments + launch_descriptions + nodes)

# ... (Funzioni helper load_yaml, parse_yaml rimangono uguali) ...
def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)

def parse_yaml(absolute_file_path: str):
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument("robot_type", default_value="panda"),
        DeclareLaunchArgument("rviz_config", default_value=path.join(get_package_share_directory("panda_execution"), "rviz", "MTC_config.rviz")),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gz_verbosity", default_value="2"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("model", default_value="panda"),
        DeclareLaunchArgument("description_package", default_value="panda_description"),
        DeclareLaunchArgument("description_filepath", default_value=path.join("urdf", "panda.urdf.xacro")),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),
        DeclareLaunchArgument("launch_gz", default_value="false"),
    ]