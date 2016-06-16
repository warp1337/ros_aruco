cmake_minimum_required(VERSION 2.8.3)
PROJECT(ros_aruco)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp tf std_msgs sensor_msgs geometry_msgs message_generation genmsg cv_bridge image_transport)

SET(CMAKE_MODULE_PATH ${ARUCO_PATH}/lib/cmake)
SET(ARUCO_INCLUDE_DIRS ${ARUCO_PATH}/include/aruco)

## System dependencies are found with CMake's conventions
FIND_PACKAGE(aruco REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

INCLUDE(FindPkgConfig)

SET(ROS_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -march=core-avx-i -O2")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	${ARUCO_INCLUDE_DIRS}
)

add_executable(ros_aruco
	src/ros_aruco.cpp 
)

add_dependencies(ros_aruco ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_aruco 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${aruco_LIBS}
)

#############
## Install ##
#############

# TODO


#############
## Testing ##
#############

# TODO
