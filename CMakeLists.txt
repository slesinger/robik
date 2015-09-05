cmake_minimum_required(VERSION 2.8.3)
project(robik)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	OpenCV
	roscpp
	tf
	std_msgs
	message_generation
	genmsg
	actionlib_msgs
	hardware_interface
	controller_manager
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#Profiling flags
#SET(GCC_COVERAGE_COMPILE_FLAGS "-pg")
#SET(GCC_COVERAGE_LINK_FLAGS    "-pg")
#SET(GCC_COVERAGE_COMPILE_FLAGS "-ggdb")
#SET(GCC_COVERAGE_LINK_FLAGS    "-ggdb")


#######################################
## Declare ROS messages and services ##
#######################################

## Actionlib
add_action_files(
	DIRECTORY
		action
	FILES
		move.action
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
	setParkingPhase.srv
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
	ArmControl.msg
	GenericControl.msg
	GenericStatus.msg
	VelocityControl.msg
)

## Generate added messages and services with any dependencies listed here
#Honza: This is needed here otherwise messages do not get generated properly
generate_messages(
    DEPENDENCIES
	std_msgs
	actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES robik
#  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS rospy roscpp sound_play
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(robik
#   src/${PROJECT_NAME}/robik.cpp
# )

## Declare a cpp executable
add_executable(robik_driver src/robik_driver.cpp src/robik_util.cpp src/robik_move.cpp)
add_executable(robik_hw_ifce src/robik_hw_ifce.cpp src/robik_util.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(robik_driver robik_hw_ifce robik_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robik_driver
    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
)
target_link_libraries(robik_hw_ifce
    ${catkin_LIBRARIES}
)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/robik_ai.py
   scripts/robik_action_move.py
   scripts/arduino_dummy.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark executables and/or libraries for installation
# install(TARGET
#   robik_driver
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robik.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
