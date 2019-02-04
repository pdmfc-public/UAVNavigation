cmake_minimum_required(VERSION 2.8.3)
project(bebop_waypoint)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        roslib
        tf
        sensor_msgs
        geometry_msgs
	nav_msgs
	ros_utils
        #message_generation

)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bebop_waypoint_controller
  #CATKIN_DEPENDS cmake_modules geometry_msgs glog_catkin  roscpp sensor_msgs planning_msgs tf
  CATKIN_DEPENDS roscpp rospy std_msgs roslib tf sensor_msgs geometry_msgs nav_msgs# message_runtime
#  DEPENDS system_lib
)


include_directories(include  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(bebop_waypoint_node src/bebop_waypoint_node.cpp src/Bebop_waypoint.cpp include/bebop_waypoint/Bebop_waypoint.hpp)

target_link_libraries(bebop_waypoint_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


