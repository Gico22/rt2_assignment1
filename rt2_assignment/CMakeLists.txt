cmake_minimum_required(VERSION 3.8)
project(rt2_assignment)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rt2_interfaces REQUIRED)

# server
add_library(server_comp SHARED src/set_position_server.cpp)
ament_target_dependencies(server_comp
  rclcpp rclcpp_action rclcpp_components
  nav_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  rt2_interfaces)

rclcpp_components_register_node(server_comp
  PLUGIN "rt2_assignment::SetPositionServer"
  EXECUTABLE set_position_server)

# client
add_library(client_comp SHARED src/set_position_client.cpp)
ament_target_dependencies(client_comp
  rclcpp rclcpp_action rclcpp_components
  geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  rt2_interfaces)

rclcpp_components_register_node(client_comp
  PLUGIN "rt2_assignment::SetPositionClient"
  EXECUTABLE set_position_client)

# intall
install(TARGETS server_comp client_comp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS set_position_server set_position_client
  DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()