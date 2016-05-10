# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "state_est_control: 1 messages, 0 services")

set(MSG_I_FLAGS "-Istate_est_control:/home/yisha/FYP/ROS/indigo/catkin_ws/src/state_est_control/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(state_est_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yisha/FYP/ROS/indigo/catkin_ws/src/state_est_control/msg/StateEstControl.msg" NAME_WE)
add_custom_target(_state_est_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_est_control" "/home/yisha/FYP/ROS/indigo/catkin_ws/src/state_est_control/msg/StateEstControl.msg" "geometry_msgs/Point:std_msgs/Float32:geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/PoseStamped:std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(state_est_control
  "/home/yisha/FYP/ROS/indigo/catkin_ws/src/state_est_control/msg/StateEstControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_est_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(state_est_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_est_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(state_est_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(state_est_control_generate_messages state_est_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yisha/FYP/ROS/indigo/catkin_ws/src/state_est_control/msg/StateEstControl.msg" NAME_WE)
add_dependencies(state_est_control_generate_messages_cpp _state_est_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_est_control_gencpp)
add_dependencies(state_est_control_gencpp state_est_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_est_control_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(state_est_control
  "/home/yisha/FYP/ROS/indigo/catkin_ws/src/state_est_control/msg/StateEstControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_est_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(state_est_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_est_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(state_est_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(state_est_control_generate_messages state_est_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yisha/FYP/ROS/indigo/catkin_ws/src/state_est_control/msg/StateEstControl.msg" NAME_WE)
add_dependencies(state_est_control_generate_messages_lisp _state_est_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_est_control_genlisp)
add_dependencies(state_est_control_genlisp state_est_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_est_control_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(state_est_control
  "/home/yisha/FYP/ROS/indigo/catkin_ws/src/state_est_control/msg/StateEstControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_est_control
)

### Generating Services

### Generating Module File
_generate_module_py(state_est_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_est_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(state_est_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(state_est_control_generate_messages state_est_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yisha/FYP/ROS/indigo/catkin_ws/src/state_est_control/msg/StateEstControl.msg" NAME_WE)
add_dependencies(state_est_control_generate_messages_py _state_est_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_est_control_genpy)
add_dependencies(state_est_control_genpy state_est_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_est_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_est_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_est_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(state_est_control_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(state_est_control_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_est_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_est_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(state_est_control_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(state_est_control_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_est_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_est_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_est_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(state_est_control_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(state_est_control_generate_messages_py geometry_msgs_generate_messages_py)
