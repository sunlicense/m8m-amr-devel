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
	//ros::NodeHandle rn; 
	int nGP, nLP, butNo;
	//bool ok;
	geometry_msgs::Twist pos;
	bool ok;
	char buf [100];
	string s;

	butNo = msg->data; 
	//rn.getParam("current_group",nGP);
	//ROS_INFO("Button_Node : Button Code Received = %d",butNo);
	sprintf(buf,"Button_Node : Button Code Received = %d",butNo);
	s.assign(buf,strlen(buf));
	publish_debug(s);

	switch (butNo) {
		case 1:
			targetGP = 0;
			targetLP = 1;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;
		case 2:
			targetGP = 0;
			targetLP = 2;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;
		case 3:
			targetGP = 0;
			targetLP = 3;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;
		case 4:
			targetGP = 0;
			targetLP = 4;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;
		case 5:
			targetGP = 0;
			targetLP = 5;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;
		case 6:
			targetGP = 0;
			targetLP = 6;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;		
		case 13: // robot charging now. Stop robot
			// stop robot.
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.0;  // 7.0
			pos_pub.publish(pos);
			publish_status("Docking Activated");
			publish_debug("Move Node : Docking Achieved.Stop Robot");
			break;
		case 15:
			// stop robot.
			if (!estop) {
				pos.angular.z = 0.0;
				pos.linear.x = 0.0;
				pos.linear.z = 7.8;  // 7.0
				pos_pub.publish(pos);
				publish_status("Emergency Stop Activated");
				publish_debug("Move Node : Emergency Stop Activated");
				estop = true;
				publish_sound(15);		
				sleep(2);
			}
			break;
		case 16:
			if (estop) {
				pos.angular.z = 0.0;
				pos.linear.x = 0.0;
				pos.linear.z = 7.9;  // 7.0
				pos_pub.publish(pos);
				publish_status("Emergency Stop Released");
				publish_debug("Move Node : Emergency Released");
				estop = false;
			}
			break;
	}

/*
	if (butNo == 13) {
		// stop robot.
		pos.angular.z = 0.0;
		pos.linear.x = 0.0;
		pos.linear.z = 7.0;  // 7.0
		pos_pub.publish(pos);
		publish_status("Docking Activated");
		publish_debug("Move Node : Docking Achieved.Stop Robot");
		//ROS_INFO("Move Node : Stop Robot");
	}
	if (butNo == 15) {  // emergency stop
		// stop robot.
		pos.angular.z = 0.0;
		pos.linear.x = 0.0;
		pos.linear.z = 7.5;  // 7.0
		pos_pub.publish(pos);
		publish_debug("Move Node : Emergency Stop Activated");
		estop = true;
		publish_sound(12);		
		sleep(2);
		//publish_sound(13);
		//ROS_INFO("Move Node : EStop Activated");
		//system("sudo shutdown -h +1");
	}
*/
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


