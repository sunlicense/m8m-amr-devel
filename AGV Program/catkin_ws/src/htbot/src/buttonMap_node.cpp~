/*
 * This node is to do mapping using buttons  
 */
/* 	History
*		Date Modified : 24.11.2015
*		Changes :
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/simple_action_client.h>
#include "htbot/Command.h"
#include "htbot/status.h"
#include "htbot/srvcmd.h"
#include "htbot/sendgoal.h"
#include "htbot/move.h"
#include "htbot/move_status.h"
#include "htbot/dbcmd.h"
#include "htbot/Empty.h"
#include "boost/algorithm/string.hpp"
#include "std_srvs/Empty.h"
#include "htbot/clear.h"
#include "htbot/debug.h"
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"

using namespace std;
using namespace boost::algorithm;

#define MARKSTATUSCMD 80

class ButtonNode
{
public:
	ButtonNode();	
	void buttonCallback(const std_msgs::UInt16::ConstPtr& msg);
	void publish_debug(string s);
	void publish_toggleButton(unsigned short cmd);	
	bool dbCommand(int cmd, int GP, int LP);
	bool moveCommand(int cmd,int GP, int LP);
	void mainLoop();

	htbot::dbcmd db_srv;
	htbot::srvcmd mv_srv;
	ros::ServiceClient db_client;
	ros::ServiceClient move_client;
	ros::Publisher debug_pub;

	// Uno 
	ros::Subscriber button_sub;
	ros::Publisher toggleButton_pub;



private:
	ros::NodeHandle nh;
};

ButtonNode::ButtonNode()
{
	ros::NodeHandle nh;
	debug_pub = nh.advertise<htbot::debug>("debug",100);
	db_client = nh.serviceClient<htbot::dbcmd>("dbase_cmd");
	move_client = nh.serviceClient<htbot::srvcmd>("move_service");
  // Uno
	button_sub = nh.subscribe<std_msgs::UInt16>("button", 100, &ButtonNode::buttonCallback,this);
	toggleButton_pub = nh.advertise<std_msgs::UInt16>("fbutton",100);
}

void ButtonNode::publish_toggleButton(unsigned short cmd)
{
	char buf[100];
	string s1;

	std_msgs::UInt16 msg;
	msg.data = cmd;
	toggleButton_pub.publish(msg);
	sprintf(buf,"Button_Node : Toggle_Button sent = %d",cmd);
	s1.assign(buf,strlen(buf));
	//publish_debug(s1);
	return;
}

void ButtonNode::buttonCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	ros::NodeHandle rn; 
	int nGP, nLP, butNo;
	bool ok;
	char buf[100];
	string s1;

	butNo = msg->data; 
	rn.getParam("current_group",nGP);
	sprintf(buf,"Button_Node : Button Code Received = %d",butNo);
	s1.assign(buf,strlen(buf));
	//publish_debug(s1);
	nLP = butNo;
	switch (butNo) {
		case 0:  // NavMode.
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:						
			ok = moveCommand(11,nGP,nLP);
			break;
		case 10: // low power. Move to charging Station. voltage < 24.3V 
			ok = moveCommand(4,0,0);
			publish_debug("Low Power. Noving to Charging Station");
			break;
		case 11: // robot charged. Can move again. voltage > 28.3V
			ok = moveCommand(41,0,0);
			publish_debug("Charging Done. Ready to Move");
			break;
		case 13: // robot charging now. Stop robot
			ok = moveCommand(45,0,0);
			publish_debug("Docking Achieved. Stop Robot");
			break;
		case 60:  // MapMode. Mark Station
		case 61:
		case 62:
		case 63:
		case 64:
		case 65:
		case 66:
		case 67:
			ok = dbCommand(30,nGP,nLP-60);
			if (ok) {
				publish_toggleButton(nLP - 10);
			} else {
				publish_toggleButton(99); // error
			}
			break;	
		case 90: // Set to MapMode
			ok = dbCommand(10,0,0);  // Set to MapMode and  shutdown
			publish_debug("Set to Map Mode");
			break;
		case 91: // Save Map and Switch to NavMode
			ok = dbCommand(31,0,0);  // Save Map. 
			ok = dbCommand(1,0,0);  // Set to NavMode and  shutdown
			publish_debug("Save Map and Shutdown");
			break;
		case 92: // low power. Move to charging Station
			ok = moveCommand(4,0,0);
			publish_debug("Low Power. Noving to Charging Station");
			break;
		case 93: // robot charged. Can move again
			ok = moveCommand(41,0,0);
			publish_debug("Charging Done. Ready to Move");
			break;
	}
	return;
}

bool ButtonNode::dbCommand(int cmd, int GP, int LP) {
	bool ret;

	db_srv.request.GN = GP;  
	db_srv.request.LP = LP;  		
	db_srv.request.cmd = cmd;
			
	if (db_client.call(db_srv)) {		
		if (db_srv.response.status == cmd) {	 
			ret = true;
		} else {
	    ROS_ERROR("Failed to call dbase service");	
			ret = false;
	 	}
	}
	return ret;
}

bool ButtonNode::moveCommand(int cmd,int GP, int LP) {
	bool ret;

	mv_srv.request.cGP = GP;  
	mv_srv.request.cLP = LP;  		
	mv_srv.request.cmd = cmd;
			
	if (move_client.call(mv_srv)) {		
		if (mv_srv.response.status == cmd) {	 
			ret = true;
		} else {
	    ROS_ERROR("Failed to call move service");	
			ret = false;
	 	}
	}
	return ret;
}


void ButtonNode::publish_debug(string s)
{
	htbot::debug deb;
	deb.msg = s;
	debug_pub.publish(deb);
	return;
}
 
void ButtonNode::mainLoop()
{
	ros::NodeHandle n;  
	ros::Rate r(1.0);  // every sec
	int mapflag,cgpflag;
	int markedstatus; 
	 
	n.getParam("mapflag",mapflag);
	while(ros::ok())
	{
		//ROS_INFO("Button loop..");
		n.getParam("cgp_flag",cgpflag);
		if (cgpflag == 7) {
			// send marked status to Uno
			n.getParam("Marked_Status",markedstatus);
			publish_toggleButton(MARKSTATUSCMD);
			publish_toggleButton(markedstatus);
			n.setParam("cgp_flag",0);
		}		
		ros::spinOnce();
		r.sleep();
	}
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Button node");  
  ros::NodeHandle rn;  		
	ButtonNode bNode;
	int ready;
	int cgpflag;
	int markedstatus;
	ros::Rate r(1.0);
	rn.getParam("RobotReady",ready);
	
  ROS_INFO("Button Node Running...");
	//ros::MultiThreadedSpinner spinner(4);

  //spinner.spin();
	//boost::thread main_thread(boost::bind(&ButtonNode::mainLoop, &bNode));
	//main_thread.interrupt() ;
	//main_thread.join() ;
	while(ros::ok())
	{
		//ROS_INFO("Button loop..");
		rn.getParam("cgp_flag",cgpflag);
		if (cgpflag == 7) {
			// send marked status to Uno
			rn.getParam("Marked_Status",markedstatus);
			bNode.publish_toggleButton(MARKSTATUSCMD);
			bNode.publish_toggleButton(markedstatus);
			rn.setParam("cgp_flag",0);
		}		
		ros::spinOnce();
		r.sleep();
	}
  return 0;
}


