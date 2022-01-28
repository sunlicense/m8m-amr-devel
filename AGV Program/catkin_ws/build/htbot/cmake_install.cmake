# Install script for directory: /home/rac/catkin_ws/src/htbot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rac/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/htbot/msg" TYPE FILE FILES
    "/home/rac/catkin_ws/src/htbot/msg/Command.msg"
    "/home/rac/catkin_ws/src/htbot/msg/status.msg"
    "/home/rac/catkin_ws/src/htbot/msg/move.msg"
    "/home/rac/catkin_ws/src/htbot/msg/move_status.msg"
    "/home/rac/catkin_ws/src/htbot/msg/clear.msg"
    "/home/rac/catkin_ws/src/htbot/msg/sound.msg"
    "/home/rac/catkin_ws/src/htbot/msg/queue.msg"
    "/home/rac/catkin_ws/src/htbot/msg/go.msg"
    "/home/rac/catkin_ws/src/htbot/msg/debug.msg"
    "/home/rac/catkin_ws/src/htbot/msg/scanCmd.msg"
    "/home/rac/catkin_ws/src/htbot/msg/odom.msg"
    "/home/rac/catkin_ws/src/htbot/msg/stat.msg"
    "/home/rac/catkin_ws/src/htbot/msg/velstat.msg"
    "/home/rac/catkin_ws/src/htbot/msg/lumstatus.msg"
    "/home/rac/catkin_ws/src/htbot/msg/stat_speed.msg"
    "/home/rac/catkin_ws/src/htbot/msg/navstatus.msg"
    "/home/rac/catkin_ws/src/htbot/msg/goal.msg"
    "/home/rac/catkin_ws/src/htbot/msg/motorcmd.msg"
    "/home/rac/catkin_ws/src/htbot/msg/path.msg"
    "/home/rac/catkin_ws/src/htbot/msg/robot.msg"
    "/home/rac/catkin_ws/src/htbot/msg/weblaser.msg"
    "/home/rac/catkin_ws/src/htbot/msg/cleanlist.msg"
    "/home/rac/catkin_ws/src/htbot/msg/dyna.msg"
    "/home/rac/catkin_ws/src/htbot/msg/ultraSS.msg"
    "/home/rac/catkin_ws/src/htbot/msg/task.msg"
    "/home/rac/catkin_ws/src/htbot/msg/agv_status.msg"
    "/home/rac/catkin_ws/src/htbot/msg/lift.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/htbot/srv" TYPE FILE FILES
    "/home/rac/catkin_ws/src/htbot/srv/srvcmd.srv"
    "/home/rac/catkin_ws/src/htbot/srv/sendgoal.srv"
    "/home/rac/catkin_ws/src/htbot/srv/Empty.srv"
    "/home/rac/catkin_ws/src/htbot/srv/scanMcmd.srv"
    "/home/rac/catkin_ws/src/htbot/srv/mqueue.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/htbot/cmake" TYPE FILE FILES "/home/rac/catkin_ws/build/htbot/catkin_generated/installspace/htbot-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/rac/catkin_ws/devel/include/htbot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/rac/catkin_ws/devel/share/roseus/ros/htbot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/rac/catkin_ws/devel/share/common-lisp/ros/htbot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/rac/catkin_ws/devel/share/gennodejs/ros/htbot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/rac/catkin_ws/devel/lib/python2.7/dist-packages/htbot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/rac/catkin_ws/devel/lib/python2.7/dist-packages/htbot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rac/catkin_ws/build/htbot/catkin_generated/installspace/htbot.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/htbot/cmake" TYPE FILE FILES "/home/rac/catkin_ws/build/htbot/catkin_generated/installspace/htbot-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/htbot/cmake" TYPE FILE FILES
    "/home/rac/catkin_ws/build/htbot/catkin_generated/installspace/htbotConfig.cmake"
    "/home/rac/catkin_ws/build/htbot/catkin_generated/installspace/htbotConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/htbot" TYPE FILE FILES "/home/rac/catkin_ws/src/htbot/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/htbot" TYPE DIRECTORY FILES "/home/rac/catkin_ws/src/htbot/include/htbot/")
endif()

