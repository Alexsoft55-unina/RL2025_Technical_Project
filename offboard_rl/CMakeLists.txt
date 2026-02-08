cmake_minimum_required(VERSION 3.8)
project(offboard_rl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

# Necessario per generare i messaggi
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# Generazione delle interfacce custom
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DronePose.srv"
  "srv/CheckAtBase.srv"
  DEPENDENCIES geometry_msgs
)

# --- ESEGUIBILI ESISTENTI ---
add_executable(go_to_point_server src/go_to_point_server.cpp)
ament_target_dependencies(go_to_point_server rclcpp px4_msgs Boost geometry_msgs Eigen3 std_msgs)
install(TARGETS 
  go_to_point_server
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(trajectory_drone_manager src/trajectory_drone_manager.cpp)
ament_target_dependencies(trajectory_drone_manager rclcpp px4_msgs Boost geometry_msgs Eigen3 std_msgs std_srvs)
install(TARGETS 
  trajectory_drone_manager
  DESTINATION lib/${PROJECT_NAME})

add_executable(px4_monitor_node src/px4_monitor_node.cpp)

# 1. Dipendenze esterne
ament_target_dependencies(px4_monitor_node 
  rclcpp 
  px4_msgs 
  std_srvs 
  geometry_msgs
)

# 2. Dipendenze interne (CRUCIALE per DronePose.srv)
# Questo dice a CMake di aspettare che i messaggi siano generati prima di compilare il nodo
# e di linkare le librerie generate.
rosidl_target_interfaces(px4_monitor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  px4_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)
  
# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()