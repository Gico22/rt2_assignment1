cmake_minimum_required(VERSION 3.8)
project(rt2_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)      
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rt2_interfaces REQUIRED)

add_library(action_server SHARED src/action_server.cpp) 
ament_target_dependencies(action_server
  rclcpp rclcpp_action rclcpp_components nav_msgs geometry_msgs rt2_interfaces)

add_library(action_client SHARED src/action_client.cpp) 
ament_target_dependencies(action_client
  rclcpp rclcpp_action rclcpp_components rt2_interfaces)

rclcpp_components_register_node(action_server   
  PLUGIN "rt2_assignment::SetPositionServer" EXECUTABLE server_exe)
rclcpp_components_register_node(action_client   
  PLUGIN "rt2_assignment::SetPositionClient" EXECUTABLE client_exe)

install(TARGETS action_server action_client
  ARCHIVE DESTINATION lib           
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()