# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "im_msgs: 5 messages, 1 services")

set(MSG_I_FLAGS "-Iim_msgs:/home/zhang/catkin_ws/src/im_msgs/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/indigo/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/indigo/share/trajectory_msgs/cmake/../msg;-Ivisualization_msgs:/opt/ros/indigo/share/visualization_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(im_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Bumper.msg" NAME_WE)
add_custom_target(_im_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "im_msgs" "/home/zhang/catkin_ws/src/im_msgs/msg/Bumper.msg" "std_msgs/Header:im_msgs/BumperState"
)

get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/srv/SetRGB.srv" NAME_WE)
add_custom_target(_im_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "im_msgs" "/home/zhang/catkin_ws/src/im_msgs/srv/SetRGB.srv" ""
)

get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Battery.msg" NAME_WE)
add_custom_target(_im_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "im_msgs" "/home/zhang/catkin_ws/src/im_msgs/msg/Battery.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Voltage.msg" NAME_WE)
add_custom_target(_im_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "im_msgs" "/home/zhang/catkin_ws/src/im_msgs/msg/Voltage.msg" ""
)

get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg" NAME_WE)
add_custom_target(_im_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "im_msgs" "/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg" ""
)

get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/WheelVel.msg" NAME_WE)
add_custom_target(_im_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "im_msgs" "/home/zhang/catkin_ws/src/im_msgs/msg/WheelVel.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/im_msgs
)
_generate_msg_cpp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/WheelVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/im_msgs
)
_generate_msg_cpp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/Voltage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/im_msgs
)
_generate_msg_cpp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/Battery.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/im_msgs
)
_generate_msg_cpp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/Bumper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/im_msgs
)

### Generating Services
_generate_srv_cpp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/srv/SetRGB.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/im_msgs
)

### Generating Module File
_generate_module_cpp(im_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/im_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(im_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(im_msgs_generate_messages im_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Bumper.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_cpp _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/srv/SetRGB.srv" NAME_WE)
add_dependencies(im_msgs_generate_messages_cpp _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Battery.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_cpp _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Voltage.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_cpp _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_cpp _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/WheelVel.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_cpp _im_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(im_msgs_gencpp)
add_dependencies(im_msgs_gencpp im_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS im_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/im_msgs
)
_generate_msg_lisp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/WheelVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/im_msgs
)
_generate_msg_lisp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/Voltage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/im_msgs
)
_generate_msg_lisp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/Battery.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/im_msgs
)
_generate_msg_lisp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/Bumper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/im_msgs
)

### Generating Services
_generate_srv_lisp(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/srv/SetRGB.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/im_msgs
)

### Generating Module File
_generate_module_lisp(im_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/im_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(im_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(im_msgs_generate_messages im_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Bumper.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_lisp _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/srv/SetRGB.srv" NAME_WE)
add_dependencies(im_msgs_generate_messages_lisp _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Battery.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_lisp _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Voltage.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_lisp _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_lisp _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/WheelVel.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_lisp _im_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(im_msgs_genlisp)
add_dependencies(im_msgs_genlisp im_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS im_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/im_msgs
)
_generate_msg_py(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/WheelVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/im_msgs
)
_generate_msg_py(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/Voltage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/im_msgs
)
_generate_msg_py(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/Battery.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/im_msgs
)
_generate_msg_py(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/msg/Bumper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/im_msgs
)

### Generating Services
_generate_srv_py(im_msgs
  "/home/zhang/catkin_ws/src/im_msgs/srv/SetRGB.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/im_msgs
)

### Generating Module File
_generate_module_py(im_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/im_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(im_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(im_msgs_generate_messages im_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Bumper.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_py _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/srv/SetRGB.srv" NAME_WE)
add_dependencies(im_msgs_generate_messages_py _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Battery.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_py _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/Voltage.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_py _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/BumperState.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_py _im_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_ws/src/im_msgs/msg/WheelVel.msg" NAME_WE)
add_dependencies(im_msgs_generate_messages_py _im_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(im_msgs_genpy)
add_dependencies(im_msgs_genpy im_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS im_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/im_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/im_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(im_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(im_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(im_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(im_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(im_msgs_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()
if(TARGET visualization_msgs_generate_messages_cpp)
  add_dependencies(im_msgs_generate_messages_cpp visualization_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/im_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/im_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(im_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(im_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(im_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(im_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(im_msgs_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()
if(TARGET visualization_msgs_generate_messages_lisp)
  add_dependencies(im_msgs_generate_messages_lisp visualization_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/im_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/im_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/im_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(im_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(im_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(im_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(im_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(im_msgs_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
if(TARGET visualization_msgs_generate_messages_py)
  add_dependencies(im_msgs_generate_messages_py visualization_msgs_generate_messages_py)
endif()
