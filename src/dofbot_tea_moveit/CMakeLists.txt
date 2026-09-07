cmake_minimum_required(VERSION 3.8)
project(dofbot_tea_moveit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(tea_task src/tea_task.cpp)

ament_target_dependencies(
  tea_task
  rclcpp
  std_srvs
  geometry_msgs
  shape_msgs
  visualization_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

install(TARGETS tea_task DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
