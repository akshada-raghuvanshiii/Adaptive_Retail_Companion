cmake_minimum_required(VERSION 3.8)
project(arc_person_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(person_path_follower
  src/person_path_follower.cpp
)

ament_target_dependencies(person_path_follower
  rclcpp
  geometry_msgs
  nav_msgs
)

install(
  TARGETS person_path_follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
