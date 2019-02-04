cmake_minimum_required(VERSION 2.8.3)
project(bebop_planning)

add_definitions(-std=c++11)

set(CURL_LIBRARY "-lcurl")
find_package(CURL REQUIRED)

set(CMAKE_MODULE_PATH "/usr/share/SFML/cmake/Modules" ${CMAKE_MODULE_PATH})
find_package(SFML REQUIRED audio graphics network system window)

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
        message_generation
		gps_common
		bebop_msgs
)

add_message_files(
		FILES
		connectFTP.msg
)

generate_messages(
		DEPENDENCIES
		std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bebop_planning_lib
  #CATKIN_DEPENDS cmake_modules geometry_msgs glog_catkin  roscpp sensor_msgs planning_msgs tf
  CATKIN_DEPENDS roscpp rospy std_msgs roslib tf sensor_msgs geometry_msgs nav_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(include  ${catkin_INCLUDE_DIRS} ${CURL_INCLUDE_DIR} ${SFML_INCLUDE_DIR})

add_executable(bebop_planning_node src/bebop_planning_node.cpp src/Bebop_planning.cpp include/bebop_planning/Bebop_planning.hpp)

target_link_libraries(bebop_planning_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CURL_LIBRARIES} ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})



