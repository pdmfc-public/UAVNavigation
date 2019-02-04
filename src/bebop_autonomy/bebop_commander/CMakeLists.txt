cmake_minimum_required(VERSION 2.8.3)
project(bebop_commander)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_msgs 
  moveit_ros_planning 
  moveit_ros_planning_interface
  ros_utils
  roscpp
  std_msgs
  sensor_msgs

)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs sensor_msgs moveit_core moveit_ros_planning_interface
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(bebop_commander_node src/bebop_commander_node.cpp src/Bebop_commander.cpp)

add_dependencies(bebop_commander_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bebop_commander_node ${catkin_LIBRARIES})


