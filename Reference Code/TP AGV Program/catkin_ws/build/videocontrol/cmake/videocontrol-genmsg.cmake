# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "videocontrol: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ivideocontrol:/home/rac/catkin_ws/src/videocontrol/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(videocontrol_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg" NAME_WE)
add_custom_target(_videocontrol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "videocontrol" "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg" NAME_WE)
add_custom_target(_videocontrol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "videocontrol" "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(videocontrol
  "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/videocontrol
)
_generate_msg_cpp(videocontrol
  "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/videocontrol
)

### Generating Services

### Generating Module File
_generate_module_cpp(videocontrol
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/videocontrol
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(videocontrol_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(videocontrol_generate_messages videocontrol_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg" NAME_WE)
add_dependencies(videocontrol_generate_messages_cpp _videocontrol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg" NAME_WE)
add_dependencies(videocontrol_generate_messages_cpp _videocontrol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(videocontrol_gencpp)
add_dependencies(videocontrol_gencpp videocontrol_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS videocontrol_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(videocontrol
  "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/videocontrol
)
_generate_msg_eus(videocontrol
  "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/videocontrol
)

### Generating Services

### Generating Module File
_generate_module_eus(videocontrol
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/videocontrol
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(videocontrol_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(videocontrol_generate_messages videocontrol_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg" NAME_WE)
add_dependencies(videocontrol_generate_messages_eus _videocontrol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg" NAME_WE)
add_dependencies(videocontrol_generate_messages_eus _videocontrol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(videocontrol_geneus)
add_dependencies(videocontrol_geneus videocontrol_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS videocontrol_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(videocontrol
  "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/videocontrol
)
_generate_msg_lisp(videocontrol
  "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/videocontrol
)

### Generating Services

### Generating Module File
_generate_module_lisp(videocontrol
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/videocontrol
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(videocontrol_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(videocontrol_generate_messages videocontrol_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg" NAME_WE)
add_dependencies(videocontrol_generate_messages_lisp _videocontrol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg" NAME_WE)
add_dependencies(videocontrol_generate_messages_lisp _videocontrol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(videocontrol_genlisp)
add_dependencies(videocontrol_genlisp videocontrol_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS videocontrol_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(videocontrol
  "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/videocontrol
)
_generate_msg_nodejs(videocontrol
  "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/videocontrol
)

### Generating Services

### Generating Module File
_generate_module_nodejs(videocontrol
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/videocontrol
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(videocontrol_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(videocontrol_generate_messages videocontrol_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg" NAME_WE)
add_dependencies(videocontrol_generate_messages_nodejs _videocontrol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg" NAME_WE)
add_dependencies(videocontrol_generate_messages_nodejs _videocontrol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(videocontrol_gennodejs)
add_dependencies(videocontrol_gennodejs videocontrol_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS videocontrol_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(videocontrol
  "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/videocontrol
)
_generate_msg_py(videocontrol
  "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/videocontrol
)

### Generating Services

### Generating Module File
_generate_module_py(videocontrol
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/videocontrol
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(videocontrol_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(videocontrol_generate_messages videocontrol_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/debug.msg" NAME_WE)
add_dependencies(videocontrol_generate_messages_py _videocontrol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/videocontrol/msg/dyna.msg" NAME_WE)
add_dependencies(videocontrol_generate_messages_py _videocontrol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(videocontrol_genpy)
add_dependencies(videocontrol_genpy videocontrol_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS videocontrol_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/videocontrol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/videocontrol
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(videocontrol_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(videocontrol_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/videocontrol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/videocontrol
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(videocontrol_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(videocontrol_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/videocontrol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/videocontrol
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(videocontrol_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(videocontrol_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/videocontrol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/videocontrol
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(videocontrol_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(videocontrol_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/videocontrol)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/videocontrol\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/videocontrol
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(videocontrol_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(videocontrol_generate_messages_py std_msgs_generate_messages_py)
endif()
