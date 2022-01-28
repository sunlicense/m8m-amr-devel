# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ntuc: 1 messages, 0 services")

set(MSG_I_FLAGS "-Intuc:/home/rac/catkin_ws/src/ntuc/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ntuc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rac/catkin_ws/src/ntuc/msg/lift.msg" NAME_WE)
add_custom_target(_ntuc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ntuc" "/home/rac/catkin_ws/src/ntuc/msg/lift.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ntuc
  "/home/rac/catkin_ws/src/ntuc/msg/lift.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ntuc
)

### Generating Services

### Generating Module File
_generate_module_cpp(ntuc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ntuc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ntuc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ntuc_generate_messages ntuc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/ntuc/msg/lift.msg" NAME_WE)
add_dependencies(ntuc_generate_messages_cpp _ntuc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ntuc_gencpp)
add_dependencies(ntuc_gencpp ntuc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ntuc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ntuc
  "/home/rac/catkin_ws/src/ntuc/msg/lift.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ntuc
)

### Generating Services

### Generating Module File
_generate_module_eus(ntuc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ntuc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ntuc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ntuc_generate_messages ntuc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/ntuc/msg/lift.msg" NAME_WE)
add_dependencies(ntuc_generate_messages_eus _ntuc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ntuc_geneus)
add_dependencies(ntuc_geneus ntuc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ntuc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ntuc
  "/home/rac/catkin_ws/src/ntuc/msg/lift.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ntuc
)

### Generating Services

### Generating Module File
_generate_module_lisp(ntuc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ntuc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ntuc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ntuc_generate_messages ntuc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/ntuc/msg/lift.msg" NAME_WE)
add_dependencies(ntuc_generate_messages_lisp _ntuc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ntuc_genlisp)
add_dependencies(ntuc_genlisp ntuc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ntuc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ntuc
  "/home/rac/catkin_ws/src/ntuc/msg/lift.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ntuc
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ntuc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ntuc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ntuc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ntuc_generate_messages ntuc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/ntuc/msg/lift.msg" NAME_WE)
add_dependencies(ntuc_generate_messages_nodejs _ntuc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ntuc_gennodejs)
add_dependencies(ntuc_gennodejs ntuc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ntuc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ntuc
  "/home/rac/catkin_ws/src/ntuc/msg/lift.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ntuc
)

### Generating Services

### Generating Module File
_generate_module_py(ntuc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ntuc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ntuc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ntuc_generate_messages ntuc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/ntuc/msg/lift.msg" NAME_WE)
add_dependencies(ntuc_generate_messages_py _ntuc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ntuc_genpy)
add_dependencies(ntuc_genpy ntuc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ntuc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ntuc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ntuc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ntuc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ntuc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ntuc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ntuc_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ntuc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ntuc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ntuc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ntuc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ntuc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ntuc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ntuc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ntuc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ntuc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ntuc_generate_messages_py std_msgs_generate_messages_py)
endif()
