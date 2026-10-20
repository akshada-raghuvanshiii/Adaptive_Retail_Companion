cmake_minimum_required(VERSION 3.8)
project(teb_local_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(teb_controller SHARED
  src/teb_controller.cpp
)

ament_target_dependencies(teb_controller
  rclcpp
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  geometry_msgs
  nav_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_core teb_controller.xml)

install(
  TARGETS teb_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES teb_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
