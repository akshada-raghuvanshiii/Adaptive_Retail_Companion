cmake_minimum_required(VERSION 3.8)

project(arc_behavior_tree)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(nav_msgs REQUIRED)

find_package(behaviortree_cpp REQUIRED)

find_package(ament_index_cpp REQUIRED)

find_package(ros_gz_interfaces REQUIRED)

find_package(tf2 REQUIRED)

find_package(tf2_ros REQUIRED)


include_directories(include)


# ============================================================
# PERSON STATE INPUT
# ============================================================

add_executable(
  person_state_input
  src/person_state_input.cpp
)

ament_target_dependencies(
  person_state_input
  rclcpp
  std_msgs
)


# ============================================================
# BEHAVIOUR TREE
# ============================================================

add_executable(
  person_assistant_bt
  src/person_assistant_bt.cpp
)

ament_target_dependencies(
  person_assistant_bt
  rclcpp
  std_msgs
  behaviortree_cpp
  ament_index_cpp
)


# ============================================================
# FAKE PERSON CONTROLLER
# ============================================================

add_executable(
  fake_person_tracker
  src/fake_person_tracker.cpp
)

ament_target_dependencies(
  fake_person_tracker
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
)


# ============================================================
# GAZEBO PERSON POSE
# ============================================================

add_executable(
  gazebo_person_pose
  src/gazebo_person_pose.cpp
)

ament_target_dependencies(
  gazebo_person_pose
  rclcpp
  geometry_msgs
  ros_gz_interfaces
)


# ============================================================
# MOTION GATE (TEB /cmd_vel_nav -> BT policy -> /cmd_vel)
# ============================================================

add_executable(
  arc_motion_gate
  src/arc_motion_gate.cpp
)

ament_target_dependencies(
  arc_motion_gate
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
)


# NOTE: src/follow_goal_generator.cpp is intentionally NOT built.
# /person_follow_goal must stay inactive (single goal source:
# shopping_manager).


# ============================================================
# INSTALL EXECUTABLES
# ============================================================

install(
  TARGETS
    person_state_input
    person_assistant_bt
    fake_person_tracker
    gazebo_person_pose
    arc_motion_gate

  DESTINATION
    lib/${PROJECT_NAME}
)


# ============================================================
# INSTALL BT XML
# ============================================================

install(
  DIRECTORY
    behavior_trees

  DESTINATION
    share/${PROJECT_NAME}
)


# ============================================================
# INSTALL CONFIG + LAUNCH
# ============================================================

install(
  DIRECTORY
    config
    launch

  DESTINATION
    share/${PROJECT_NAME}
)


# ============================================================
# INSTALL GAZEBO MODEL
# ============================================================

install(
  DIRECTORY
    models

  DESTINATION
    share/${PROJECT_NAME}
)


ament_package()
