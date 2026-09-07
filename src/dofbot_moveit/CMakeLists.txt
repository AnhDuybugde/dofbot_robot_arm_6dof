cmake_minimum_required(VERSION 3.22)
project(dofbot_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_visual_tools REQUIRED) 
find_package(rviz_visual_tools REQUIRED)   
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)

# 声明可执行文件
add_executable(cartesian_path src/cartesian_path.cpp)
add_executable(multi_track_motion src/multi_track_motion.cpp)
add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
add_executable(random_move src/random_move.cpp)
add_executable(set_scene src/set_scene.cpp)
add_executable(set_target_joints src/set_target_joints.cpp)
add_executable(set_target_position src/set_target_position.cpp)

# 1. cartesian_path
ament_target_dependencies(cartesian_path
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_visual_tools  
  rviz_visual_tools    
  graph_msgs
  pluginlib
  tf2_ros
  tf2_eigen
)
target_link_libraries(cartesian_path
  ${moveit_visual_tools_LIBRARIES}
  ${rviz_visual_tools_LIBRARIES}
)

# 2. multi_track_motion
ament_target_dependencies(multi_track_motion
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_visual_tools  
  rviz_visual_tools   
  graph_msgs
  pluginlib
  tf2_ros
  tf2_eigen
)
target_link_libraries(multi_track_motion
  ${moveit_visual_tools_LIBRARIES}
  ${rviz_visual_tools_LIBRARIES}
)

# 3. set_target_joints
ament_target_dependencies(set_target_joints
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_visual_tools  
  rviz_visual_tools    
  graph_msgs
  pluginlib
  tf2_ros
  tf2_eigen
)
target_link_libraries(set_target_joints
  ${moveit_visual_tools_LIBRARIES}
  ${rviz_visual_tools_LIBRARIES}
)

ament_target_dependencies(obstacle_avoidance
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  graph_msgs
  pluginlib
  tf2_ros
  tf2_eigen
)

ament_target_dependencies(random_move
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  graph_msgs
  pluginlib
  tf2_ros
  tf2_eigen
)

ament_target_dependencies(set_scene
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  graph_msgs
  pluginlib
  tf2_ros
  tf2_eigen
)

ament_target_dependencies(set_target_position
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  graph_msgs
  pluginlib
  tf2_ros
  tf2_eigen
)

# 安装可执行文件
install(TARGETS
  cartesian_path
  multi_track_motion
  obstacle_avoidance
  random_move
  set_scene
  set_target_joints
  set_target_position
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch/config文件
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE
  )
endif()
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
