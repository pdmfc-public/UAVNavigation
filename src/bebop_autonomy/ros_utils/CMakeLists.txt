cmake_minimum_required(VERSION 2.8.3)
project(ros_utils)

find_package( catkin REQUIRED COMPONENTS roscpp )

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)


include_directories( include ${catkin_INCLUDE_DIRS} )

add_library( ${PROJECT_NAME} src/Console.cpp )

target_link_libraries( ${PROJECT_NAME} )

