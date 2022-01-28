cmake_minimum_required(VERSION 2.8.3)
project(htbot)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cv_bridge
  image_transport
  move_base_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
	actionlib_msgs
  message_generation
  dynamic_reconfigure
)
find_package(PkgConfig)
pkg_check_modules(gsl REQUIRED gsl)
pkg_check_modules(csm REQUIRED csm)
find_package( OpenCV REQUIRED )
FIND_LIBRARY(libEposCmd_LIBRARIES libEposCmd.so.6.6.1.0 /opt/EposCmdLib_6.6.1.0/lib/x86_64)
FIND_LIBRARY(sqlite3_LIBRARIES sqlite3 /opt/sqlite3)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Command.msg
	status.msg
	move.msg
	move_status.msg
	clear.msg
	sound.msg
	queue.msg
	go.msg
	debug.msg
	scanCmd.msg
	odom.msg
	stat.msg
	velstat.msg
	lumstatus.msg
	stat_speed.msg
	navstatus.msg
	goal.msg
	motorcmd.msg
	path.msg
	robot.msg
	weblaser.msg
	cleanlist.msg
	dyna.msg
	ultraSS.msg
	task.msg
	agv_status.msg
	lift.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   srvcmd.srv
	 sendgoal.srv
	 Empty.srv
	 scanMcmd.srv
	 mqueue.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES htbot
  CATKIN_DEPENDS 
		cv_bridge 
		image_transport 
		roscpp 
		sensor_msgs 
		std_msgs 
		move_base_msgs 
		actionlib 
		tf 
		std_srvs 
		actionlib_msgs 
		message_runtime
		dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
	${csm_INCLUDE_DIRS}
  /home/rac/catkin_ws/src/htbot/include
)
link_directories(${csm_LIBRARY_DIRS})

## Declare a cpp library
# add_library(htbot
#   src/${PROJECT_NAME}/htbot.cpp
# )

## Declare a cpp executable
# add_executable(htbot_node src/htbot_node.cpp)

###################################################

# 	add_executable(htbot src/bot2wheel_driver.cpp src/EPOS2.cpp src/bot_node.cpp)
# 	target_link_libraries ( htbot ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(htbot htbot_generate_messages_cpp)

 	add_executable(UsbEpos src/UsbEpos.cpp)
 	target_link_libraries ( UsbEpos ${libEposCmd_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(UsbEpos htbot_generate_messages_cpp)

#	add_executable(bar src/barcode_node.cpp)
#  target_link_libraries ( bar ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(bar htbot_generate_messages_cpp)

	add_executable(sgoal src/sendgoal_node.cpp)
 	target_link_libraries ( sgoal ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(sgoal htbot_generate_messages_cpp)

#	add_executable(ngoal src/sendgoal_new.cpp)
# 	target_link_libraries ( ngoal ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(ngoal htbot_generate_messages_cpp)

 	add_executable(jnode src/joy_node.cpp)
 	target_link_libraries ( jnode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(jnode htbot_generate_messages_cpp)

 	add_executable(webNode src/web_node.cpp)
 	target_link_libraries ( webNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(webNode htbot_generate_messages_cpp)

	add_executable(movNode src/move_node.cpp)
 	target_link_libraries ( movNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(movNode htbot_generate_messages_cpp)
	
#	add_executable(nmovNode src/newmove_node.cpp)
# 	target_link_libraries ( nmovNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(nmovNode htbot_generate_messages_cpp)

 	add_executable(clearNode src/clearMap_node.cpp)
	target_link_libraries ( clearNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(clearNode htbot_generate_messages_cpp)

	add_executable(talkNode src/talk_node.cpp)
 	target_link_libraries ( talkNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(talkNode htbot_generate_messages_cpp)

	add_executable(debugNode src/debug_node.cpp)
 	target_link_libraries ( debugNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )  
	add_dependencies(debugNode htbot_generate_messages_cpp)

	add_executable(eventNode src/event_node.cpp)
 	target_link_libraries ( eventNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ) 
	add_dependencies(eventNode htbot_generate_messages_cpp)

	add_executable(STNode src/start_node.cpp)
 	target_link_libraries ( STNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(STNode htbot_generate_messages_cpp)

	add_executable(arNode src/arduino_node.cpp)
 	target_link_libraries ( arNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(arNode htbot_generate_messages_cpp)

	add_executable(laserNode src/laser_node.cpp)
 	target_link_libraries ( laserNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(laserNode htbot_generate_messages_cpp)

#	add_executable(llaser src/left_laser.cpp)
# 	target_link_libraries ( llaser ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(llaser htbot_generate_messages_cpp)

#	add_executable(rlaser src/right_laser.cpp)
# 	target_link_libraries ( rlaser ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(rlaser htbot_generate_messages_cpp)

	add_executable(match src/match_node.cpp)
	target_link_libraries ( match ${catkin_LIBRARIES} ${gsl_LIBRARIES} ${csm_LIBRARIES} )
	add_dependencies(match htbot_generate_messages_cpp)

	add_executable(podom src/odom_node.cpp)
	target_link_libraries ( podom ${catkin_LIBRARIES} ${gsl_LIBRARIES} ${csm_LIBRARIES} )
	add_dependencies(podom htbot_generate_messages_cpp)

  add_executable(qnode src/queue_node.cpp)
	target_link_libraries ( qnode ${catkin_LIBRARIES} ${gsl_LIBRARIES} ${csm_LIBRARIES} )
	add_dependencies(qnode htbot_generate_messages_cpp)

  add_executable(dash src/dash_node.cpp)
	target_link_libraries ( dash ${catkin_LIBRARIES} ${gsl_LIBRARIES} ${csm_LIBRARIES} )
	add_dependencies(dash htbot_generate_messages_cpp)

  add_executable(rfid2 src/rfid2.cpp)
	target_link_libraries ( rfid2 ${catkin_LIBRARIES} ${gsl_LIBRARIES} ${csm_LIBRARIES} )
	add_dependencies(rfid2 htbot_generate_messages_cpp)

	add_executable(rfid src/rfid.cpp)
	target_link_libraries ( rfid ${catkin_LIBRARIES} ${gsl_LIBRARIES} ${csm_LIBRARIES} )
	add_dependencies(rfid htbot_generate_messages_cpp)

	add_executable(rfid_server src/rfid_server.cpp)
	target_link_libraries ( rfid_server ${catkin_LIBRARIES} ${gsl_LIBRARIES} ${csm_LIBRARIES} )
	add_dependencies(rfid_server htbot_generate_messages_cpp)

#	add_executable(tlaser src/top_laser.cpp)
# 	target_link_libraries ( tlaser ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(tlaser htbot_generate_messages_cpp)

#	add_executable(fmclient src/tcpclient_node.cpp)
# 	target_link_libraries ( fmclient ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(fmclient htbot_generate_messages_cpp)
	
	add_executable(mapzone src/mapzone_node.cpp)
 	target_link_libraries ( mapzone ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(mapzone htbot_generate_messages_cpp)

#	add_executable(lumNode src/lum_node.cpp)
# 	target_link_libraries ( lumNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(lumNode htbot_generate_messages_cpp)

#	add_executable(nysNode src/bot2wheel_driver.cpp src/EPOS2.cpp src/nysgoal_node.cpp)
# 	target_link_libraries ( nysNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(nysNode htbot_generate_messages_cpp)

#	add_executable(CMapNode src/costmap_node.cpp)
# 	target_link_libraries ( CMapNode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(CMapNode htbot_generate_messages_cpp)

	add_executable(check src/checkPlan_node.cpp)
 	target_link_libraries ( check ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(check htbot_generate_messages_cpp)

	add_executable(tt src/test.cpp)
 	target_link_libraries ( tt ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(tt htbot_generate_messages_cpp)

#	add_executable(motor src/bot2wheel_driver.cpp src/EPOS2.cpp src/motor_node.cpp)
# 	target_link_libraries ( motor ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(motor htbot_generate_messages_cpp)

#	add_executable(motorc src/bot2wheel_driver.cpp src/EPOS2.cpp src/motor_control.cpp)
# 	target_link_libraries ( motorc ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(motorc htbot_generate_messages_cpp)

	add_executable(refscan src/RefScan_node.cpp)
 	target_link_libraries ( refscan ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(refscan htbot_generate_messages_cpp)

	add_executable(amclpose src/amclpose_node.cpp)
 	target_link_libraries ( amclpose ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(amclpose htbot_generate_messages_cpp)

#	add_executable(depnode src/depth_node.cpp)
# 	target_link_libraries ( depnode ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(depnode htbot_generate_messages_cpp)

#	add_executable(bmp src/bmp.cpp)
# 	target_link_libraries ( bmp ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(bmp htbot_generate_messages_cpp)

#	add_executable(mcontroller src/bot2wheel_driver.cpp src/EPOS2.cpp src/mcontroller.cpp)
# 	target_link_libraries ( mcontroller ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(mcontroller htbot_generate_messages_cpp)

	add_executable(weblaser src/weblaser_node.cpp)
 	target_link_libraries ( weblaser ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(weblaser htbot_generate_messages_cpp)

	add_executable(multilaser src/multilaser_node.cpp)
 	target_link_libraries ( multilaser ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(multilaser htbot_generate_messages_cpp)

	add_executable(dyna src/reconfigure_node.cpp)
 	target_link_libraries ( dyna ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(dyna htbot_generate_messages_cpp)

	add_executable(sql src/sql_node.cpp)
 	target_link_libraries ( sql ${sqlite3_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
	add_dependencies(sql htbot_generate_messages_cpp)

	add_executable(multisick src/multisick_node.cpp)
 	target_link_libraries ( multisick ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(multisick htbot_generate_messages_cpp)

	add_executable(multisickpA src/multisickpA_node.cpp)
 	target_link_libraries ( multisickpA ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
	add_dependencies(multisickpA htbot_generate_messages_cpp)

#	add_executable(monitor src/monitor_node.cpp)
# 	target_link_libraries ( monitor ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(monitor htbot_generate_messages_cpp)

#	add_executable(clock src/clock_node.cpp)
# 	target_link_libraries ( clock ${catkin_LIBRARIES})
#	add_dependencies(clock htbot_generate_messages_cpp)

#	add_executable(botLS src/bottomRS_node.cpp)
# 	target_link_libraries ( botLS ${catkin_LIBRARIES})
#	add_dependencies(botLS htbot_generate_messages_cpp)

	add_executable(uSS src/ultraSS_node.cpp)
 	target_link_libraries ( uSS ${catkin_LIBRARIES})
	add_dependencies(uSS htbot_generate_messages_cpp)

	add_executable(trolley src/trolley_node.cpp)
 	target_link_libraries ( trolley ${catkin_LIBRARIES})
	add_dependencies(trolley htbot_generate_messages_cpp)

#	add_executable(rsdist src/rsdist.cpp)
# 	target_link_libraries ( rsdist ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
#	add_dependencies(rsdist htbot_generate_messages_cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(htbot_node htbot_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(htbot_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS htbot htbot_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_htbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
