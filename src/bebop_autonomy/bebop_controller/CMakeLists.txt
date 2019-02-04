cmake_minimum_required(VERSION 2.8.3)
project(bebop_controller)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

include_directories(SYSTEM "msg_gen/cpp/include")

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  trajectory_msgs
  actionlib_msgs
  std_msgs
  message_generation
  genmsg
  tf
  nav_msgs
  ros_utils
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
        cfg/controller.cfg
)

include_directories(include ${catkin_INCLUDE_DIRS} ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
link_directories(${catkin_LIBRARY_DIRS})

add_action_files(DIRECTORY action FILES MultiDofFollowJointTrajectory.action)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  actionlib_msgs
  bebop_controller
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
  CATKIN_DEPENDS message_runtime
  moveit_core
  dynamic_reconfigure
)

# add_library(action_controller src/actionController.cpp)
add_executable(bebop_controller_node src/bebop_controller_node.cpp src/Bebop_controller.cpp include/bebop_controller/Bebop_controller.hpp)
target_link_libraries(bebop_controller_node ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(bebop_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#install(TARGETS action_controller LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


