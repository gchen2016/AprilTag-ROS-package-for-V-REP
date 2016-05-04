cmake_minimum_required(VERSION 2.8.3)
project(state_est_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  geometry_msgs
  message_generation
  cmake_modules
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  StateEstControl.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES state_est_controller
 CATKIN_DEPENDS std_msgs roscpp geometry_msgs message_runtime
 DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(state_est_controller
  src/state_est_controller.cpp
)
target_link_libraries(state_est_controller
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
add_dependencies(state_est_controller 
state_est_control_generate_messages_cpp)

## Declare a C++ executable
add_executable(state_est_controller_node src/state_est_controller_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(state_est_controller_node
  ${catkin_LIBRARIES}
  state_est_controller
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS state_est_controller state_est_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/state_est_control
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}

)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
