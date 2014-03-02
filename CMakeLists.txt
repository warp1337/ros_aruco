cmake_minimum_required(VERSION 2.8.3)
project(ros_aruco)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs sensor_msgs geometry_msgs message_generation genmsg cv_bridge image_transport)

## TODO: Make this configurable via variable
SET(CMAKE_MODULE_PATH ${ARUCO_PATH}/lib/cmake )
SET(ARUCO_INCLUDE_DIRS ${ARUCO_PATH}/include/aruco )

## System dependencies are found with CMake's conventions
find_package(aruco REQUIRED )
find_package(OpenCV REQUIRED )

INCLUDE(FindPkgConfig)

set(ROS_BUILD_TYPE Debug)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## generate actions:
# add_action_files(
# 	DIRECTORY action 
# 	FILES register_access.action
# )


## Generate messages in the 'msg' folder
# add_message_files(
#	FILES
#	joint_state.msg
#	device_info.msg
#	device_state.msg
#	bus_state.msg
#	health_state.msg
#	smoothmotion_joint_target.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   register_access.srv
#   server_state.srv
# )



## Generate added messages and services with any dependencies listed here
# generate_messages(
#	DEPENDENCIES
#	std_msgs
#	xsc3
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
  # INCLUDE_DIRS include
  # CATKIN_DEPENDS message_runtime actionlib_msgs
  # DEPENDS system_lib
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	${ARUCO_INCLUDE_DIRS}
)

add_executable(ros_aruco
	src/ros_aruco.cpp 
)
			
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_xsc3.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
