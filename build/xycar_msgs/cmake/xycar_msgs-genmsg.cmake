# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xycar_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ixycar_msgs:/home/seunggeon/catkin_ws/src/xycar_msgs/msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xycar_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg" NAME_WE)
add_custom_target(_xycar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xycar_msgs" "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg" "sensor_msgs/Range:std_msgs/Header"
)

get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg" NAME_WE)
add_custom_target(_xycar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xycar_msgs" "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(xycar_msgs
  "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Range.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xycar_msgs
)
_generate_msg_cpp(xycar_msgs
  "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xycar_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(xycar_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xycar_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xycar_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xycar_msgs_generate_messages xycar_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg" NAME_WE)
add_dependencies(xycar_msgs_generate_messages_cpp _xycar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg" NAME_WE)
add_dependencies(xycar_msgs_generate_messages_cpp _xycar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xycar_msgs_gencpp)
add_dependencies(xycar_msgs_gencpp xycar_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xycar_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(xycar_msgs
  "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Range.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xycar_msgs
)
_generate_msg_eus(xycar_msgs
  "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xycar_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(xycar_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xycar_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xycar_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xycar_msgs_generate_messages xycar_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg" NAME_WE)
add_dependencies(xycar_msgs_generate_messages_eus _xycar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg" NAME_WE)
add_dependencies(xycar_msgs_generate_messages_eus _xycar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xycar_msgs_geneus)
add_dependencies(xycar_msgs_geneus xycar_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xycar_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(xycar_msgs
  "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Range.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xycar_msgs
)
_generate_msg_lisp(xycar_msgs
  "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xycar_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(xycar_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xycar_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xycar_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xycar_msgs_generate_messages xycar_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg" NAME_WE)
add_dependencies(xycar_msgs_generate_messages_lisp _xycar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg" NAME_WE)
add_dependencies(xycar_msgs_generate_messages_lisp _xycar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xycar_msgs_genlisp)
add_dependencies(xycar_msgs_genlisp xycar_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xycar_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(xycar_msgs
  "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Range.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xycar_msgs
)
_generate_msg_nodejs(xycar_msgs
  "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xycar_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(xycar_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xycar_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xycar_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xycar_msgs_generate_messages xycar_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg" NAME_WE)
add_dependencies(xycar_msgs_generate_messages_nodejs _xycar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg" NAME_WE)
add_dependencies(xycar_msgs_generate_messages_nodejs _xycar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xycar_msgs_gennodejs)
add_dependencies(xycar_msgs_gennodejs xycar_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xycar_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(xycar_msgs
  "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Range.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xycar_msgs
)
_generate_msg_py(xycar_msgs
  "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xycar_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(xycar_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xycar_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xycar_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xycar_msgs_generate_messages xycar_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_ultrasounds.msg" NAME_WE)
add_dependencies(xycar_msgs_generate_messages_py _xycar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/seunggeon/catkin_ws/src/xycar_msgs/msg/xycar_motor.msg" NAME_WE)
add_dependencies(xycar_msgs_generate_messages_py _xycar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xycar_msgs_genpy)
add_dependencies(xycar_msgs_genpy xycar_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xycar_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xycar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xycar_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(xycar_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xycar_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xycar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xycar_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(xycar_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xycar_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xycar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xycar_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(xycar_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xycar_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xycar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xycar_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(xycar_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xycar_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xycar_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xycar_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xycar_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(xycar_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xycar_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
