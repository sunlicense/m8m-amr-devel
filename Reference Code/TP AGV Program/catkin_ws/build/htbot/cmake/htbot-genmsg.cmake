# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "htbot: 27 messages, 5 services")

set(MSG_I_FLAGS "-Ihtbot:/home/rac/catkin_ws/src/htbot/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(htbot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/robot.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/robot.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/Command.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/Command.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/queue.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/queue.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/go.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/go.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/path.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/path.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/move.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/Empty.srv" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/srv/Empty.srv" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/debug.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/debug.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/sound.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/sound.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/status.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/status.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/clear.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/clear.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/odom.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/odom.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/stat.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/task.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/task.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move_status.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/move_status.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/goal.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/goal.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/dyna.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/dyna.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/velstat.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/velstat.msg" ""
)

get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lift.msg" NAME_WE)
add_custom_target(_htbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "htbot" "/home/rac/catkin_ws/src/htbot/msg/lift.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/robot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/queue.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/go.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/path.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/move.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/debug.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/dyna.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/clear.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/odom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/stat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/task.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/move_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/goal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/sound.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/velstat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_msg_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/lift.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)

### Generating Services
_generate_srv_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_srv_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_srv_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_srv_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/srv/Empty.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)
_generate_srv_cpp(htbot
  "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
)

### Generating Module File
_generate_module_cpp(htbot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(htbot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(htbot_generate_messages htbot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/robot.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/Command.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/queue.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/go.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/path.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/Empty.srv" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/debug.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/sound.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/clear.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/odom.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/task.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move_status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/goal.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/dyna.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/velstat.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lift.msg" NAME_WE)
add_dependencies(htbot_generate_messages_cpp _htbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(htbot_gencpp)
add_dependencies(htbot_gencpp htbot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS htbot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/robot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/queue.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/go.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/path.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/move.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/debug.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/dyna.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/clear.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/odom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/stat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/task.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/move_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/goal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/sound.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/velstat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_msg_eus(htbot
  "/home/rac/catkin_ws/src/htbot/msg/lift.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)

### Generating Services
_generate_srv_eus(htbot
  "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_srv_eus(htbot
  "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_srv_eus(htbot
  "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_srv_eus(htbot
  "/home/rac/catkin_ws/src/htbot/srv/Empty.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)
_generate_srv_eus(htbot
  "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
)

### Generating Module File
_generate_module_eus(htbot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(htbot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(htbot_generate_messages htbot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/robot.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/Command.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/queue.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/go.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/path.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/Empty.srv" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/debug.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/sound.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/clear.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/odom.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/task.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move_status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/goal.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/dyna.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/velstat.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lift.msg" NAME_WE)
add_dependencies(htbot_generate_messages_eus _htbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(htbot_geneus)
add_dependencies(htbot_geneus htbot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS htbot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/robot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/queue.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/go.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/path.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/move.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/debug.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/dyna.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/clear.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/odom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/stat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/task.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/move_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/goal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/sound.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/velstat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_msg_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/msg/lift.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)

### Generating Services
_generate_srv_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_srv_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_srv_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_srv_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/srv/Empty.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)
_generate_srv_lisp(htbot
  "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
)

### Generating Module File
_generate_module_lisp(htbot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(htbot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(htbot_generate_messages htbot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/robot.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/Command.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/queue.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/go.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/path.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/Empty.srv" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/debug.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/sound.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/clear.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/odom.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/task.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move_status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/goal.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/dyna.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/velstat.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lift.msg" NAME_WE)
add_dependencies(htbot_generate_messages_lisp _htbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(htbot_genlisp)
add_dependencies(htbot_genlisp htbot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS htbot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/robot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/queue.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/go.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/path.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/move.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/debug.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/dyna.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/clear.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/odom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/stat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/task.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/move_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/goal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/sound.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/velstat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_msg_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/msg/lift.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)

### Generating Services
_generate_srv_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_srv_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_srv_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_srv_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/srv/Empty.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)
_generate_srv_nodejs(htbot
  "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
)

### Generating Module File
_generate_module_nodejs(htbot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(htbot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(htbot_generate_messages htbot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/robot.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/Command.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/queue.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/go.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/path.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/Empty.srv" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/debug.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/sound.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/clear.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/odom.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/task.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move_status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/goal.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/dyna.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/velstat.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lift.msg" NAME_WE)
add_dependencies(htbot_generate_messages_nodejs _htbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(htbot_gennodejs)
add_dependencies(htbot_gennodejs htbot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS htbot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/robot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/queue.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/go.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/path.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/move.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/debug.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/dyna.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/clear.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/odom.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/stat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/task.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/move_status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/goal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/sound.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/velstat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_msg_py(htbot
  "/home/rac/catkin_ws/src/htbot/msg/lift.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)

### Generating Services
_generate_srv_py(htbot
  "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_srv_py(htbot
  "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_srv_py(htbot
  "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_srv_py(htbot
  "/home/rac/catkin_ws/src/htbot/srv/Empty.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)
_generate_srv_py(htbot
  "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
)

### Generating Module File
_generate_module_py(htbot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(htbot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(htbot_generate_messages htbot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/robot.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/Command.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/queue.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/go.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/path.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/Empty.srv" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/debug.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/sound.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/clear.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/odom.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/task.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/move_status.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/goal.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/dyna.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/velstat.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rac/catkin_ws/src/htbot/msg/lift.msg" NAME_WE)
add_dependencies(htbot_generate_messages_py _htbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(htbot_genpy)
add_dependencies(htbot_genpy htbot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS htbot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/htbot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(htbot_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(htbot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/htbot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(htbot_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(htbot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/htbot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(htbot_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(htbot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/htbot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(htbot_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(htbot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/htbot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(htbot_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(htbot_generate_messages_py std_msgs_generate_messages_py)
endif()
