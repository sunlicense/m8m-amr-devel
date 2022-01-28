/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Modified : 3.12.2014
*		Changes :
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include "htbot/move_status.h"
#include "htbot/status.h"
#include "boost/algorithm/string.hpp"
#include <math.h>
#include "htbot/clear.h"
#include "htbot/sound.h"
#include "std_srvs/Empty.h"
#include "htbot/debug.h"

using namespace std;


ros::Subscriber clearmap;
ros::Publisher debug_pub;
ros::Publisher play_pub;
ros::ServiceClient clearcostmap;

// prototype
void publish_debug(string s);
void publish_move_status(int stat);
void publish_sound(int id);

void publish_debug(string s)
{
	htbot::debug status;
	status.msg = s;
	debug_pub.publish(status);
	return;
}

void publish_sound(int id)
{
	htbot::sound cmd;
	cmd.id = id;
	play_pub.publish(cmd);
	return;
}

bool clear(const htbot::clear::ConstPtr& msg)
{
	std_srvs::Empty srv;
	//publish_sound(9);
	//publish_debug("ClearMap : Dummy Clear");
	
	if (clearcostmap.call(srv)) {
		ROS_INFO("--------------- ClearMap : Cleared Costmaps OK ----------------");
	} else {
		ROS_INFO("------------------ ClearMap : Cleared Costmaps Failed -------------");
	}
	
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clear Costmap Server");
  ros::NodeHandle n;

	clearcostmap = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	clearmap = n.subscribe<htbot::clear>("clearMap", 1000, clear);  // move_test to test. move is actual
  debug_pub = n.advertise<htbot::debug>("event",100);
	play_pub = n.advertise<htbot::sound>("sound", 100);

  //ROS_INFO("Ready to clear CostMap.");
  ros::spin();

  return 0;
}



