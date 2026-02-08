cmake_minimum_required(VERSION 3.5)
project(panda_execution)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing and linting
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# --- FIND DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED) 
find_package(moveit_task_constructor_msgs REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# Dipendenze per Service e Action
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Dipendenze per Servoing
find_package(aruco_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Drone
find_package(offboard_rl REQUIRED)  # <--- FONDAMENTALE


# --- GENERAZIONE MESSAGGI CUSTOM (SERVIZIO) ---
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
# Install C++ examples directory
set(EXAMPLES_CPP_DIR examples/cpp)



# Example 2 - target_marker_publisher
set(EXECUTABLE_2 target_marker_publisher)
add_executable(${EXECUTABLE_2} ${EXAMPLES_CPP_DIR}/${EXECUTABLE_2}.cpp)
ament_target_dependencies(${EXECUTABLE_2}
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
    visualization_msgs
)
install(TARGETS ${EXECUTABLE_2} DESTINATION lib/${PROJECT_NAME})

# Example 3 - panda_move_task (MTC)
set(EXECUTABLE_3 panda_move_task)
add_executable(${EXECUTABLE_3} ${EXAMPLES_CPP_DIR}/${EXECUTABLE_3}.cpp)
ament_target_dependencies(${EXECUTABLE_3}
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
    moveit_task_constructor_core
    moveit_task_constructor_msgs
    visualization_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    rclcpp_action
    control_msgs
    trajectory_msgs
)

target_link_libraries(${EXECUTABLE_3} "${cpp_typesupport_target}")
install(TARGETS ${EXECUTABLE_3} DESTINATION lib/${PROJECT_NAME})

# Example 4 - roll_pitch_yaw_ee_servo 
set(EXECUTABLE_4 roll_pitch_yaw_ee_servo)
add_executable(${EXECUTABLE_4} ${EXAMPLES_CPP_DIR}/${EXECUTABLE_4}.cpp)
ament_target_dependencies(${EXECUTABLE_4}
    rclcpp
    geometry_msgs
    std_srvs
    aruco_msgs
    tf2_ros
    tf2_geometry_msgs
    control_msgs
)
install(TARGETS ${EXECUTABLE_4} DESTINATION lib/${PROJECT_NAME})

# --- EXAMPLE 5 (IL TUO NODO NUOVO) ---
# Assicurati che il file .cpp nella cartella examples/cpp si chiami "panda_pick_task_srv.cpp"
set(EXECUTABLE_5 panda_pick_task_srv)
add_executable(${EXECUTABLE_5} ${EXAMPLES_CPP_DIR}/${EXECUTABLE_5}.cpp)
ament_target_dependencies(${EXECUTABLE_5}
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
    moveit_task_constructor_core
    moveit_task_constructor_msgs
    visualization_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    rclcpp_action
    control_msgs
    trajectory_msgs
)

# --- QUESTA È LA PARTE CHE AVEVI SBAGLIATO ---
# Serve a linkare il codice generato del servizio all'eseguibile
target_link_libraries(${EXECUTABLE_5} "${cpp_typesupport_target}")

install(TARGETS ${EXECUTABLE_5} DESTINATION lib/${PROJECT_NAME})
# ---------------------------------------------


# Example 6 - panda_pick_task_commander
set(EXECUTABLE_6 trajectory_selector_commander)
add_executable(${EXECUTABLE_6} ${EXAMPLES_CPP_DIR}/${EXECUTABLE_6}.cpp)
ament_target_dependencies(${EXECUTABLE_6}
    rclcpp
    geometry_msgs
    rclcpp_action
    control_msgs
    std_msgs
)
install(TARGETS ${EXECUTABLE_6} DESTINATION lib/${PROJECT_NAME})

# Example 7 - trajectory action
set(EXECUTABLE_7 trajectory_action)
add_executable(${EXECUTABLE_7} ${EXAMPLES_CPP_DIR}/${EXECUTABLE_7}.cpp)
ament_target_dependencies(${EXECUTABLE_7}
    rclcpp
    geometry_msgs
    rclcpp_action
    control_msgs
    std_msgs
)
install(TARGETS ${EXECUTABLE_7} DESTINATION lib/${PROJECT_NAME})

# Example 8 - servo_test_node
set(EXECUTABLE_8 servo_test)
add_executable(${EXECUTABLE_8} ${EXAMPLES_CPP_DIR}/${EXECUTABLE_8}.cpp)
ament_target_dependencies(${EXECUTABLE_8}
    rclcpp
    geometry_msgs
    std_srvs
)
install(TARGETS ${EXECUTABLE_8} DESTINATION lib/${PROJECT_NAME})

# Example 9 - visual_servoing
set(EXECUTABLE_9 visual_servoing)
add_executable(${EXECUTABLE_9} ${EXAMPLES_CPP_DIR}/${EXECUTABLE_9}.cpp)
ament_target_dependencies(${EXECUTABLE_9}
    rclcpp
    geometry_msgs
    std_srvs
    aruco_msgs
    tf2_ros
    tf2_geometry_msgs
    control_msgs
)
install(TARGETS ${EXECUTABLE_9} DESTINATION lib/${PROJECT_NAME})

# Install Python examples
set(EXAMPLES_PY_DIR examples/py)
install(PROGRAMS
    ${EXAMPLES_PY_DIR}/ex_follow_target.py
    ${EXAMPLES_PY_DIR}/ex_throw_object.py
    ${EXAMPLES_PY_DIR}/px4_to_tf.py
    ${EXAMPLES_PY_DIR}/grip_monitor_node.py
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(node_manager examples/cpp/node_manager.cpp)
ament_target_dependencies(node_manager
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  offboard_rl
)

# Linkare le interfacce generate nello stesso pacchetto
rosidl_target_interfaces(node_manager
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  node_manager
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(panda_scan_server examples/cpp/panda_scan_server.cpp)
ament_target_dependencies(panda_scan_server 
  rclcpp 
  std_msgs 
  std_srvs 
  sensor_msgs      
  trajectory_msgs
)

install(TARGETS
  panda_scan_server
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(pick_n_place_manager examples/cpp/pick_n_place_manager.cpp)
ament_target_dependencies(pick_n_place_manager 
  rclcpp 
  std_srvs 
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  offboard_rl
  controller_manager_msgs
)

target_link_libraries(pick_n_place_manager "${cpp_typesupport_target}")


install(TARGETS
  pick_n_place_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch rviz worlds DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()