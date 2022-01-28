/*
 * This node is to monitor all the sensors 
 */
/* 	History
*   Date created : 30.11.20
*		Date Modified : 30.11.20
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
#include "boost/algorithm/string.hpp"
#include <math.h>
#include "htbot/goal.h"
#include <nav_msgs/GetPlan.h>
#include "htbot/move.h"
#include "htbot/path.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "htbot/debug.h"
#include "htbot/angle.h"

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "htbot/PLAYSOUND.h"
#include "htbot/sound.h"

#define PI 3.141593

using namespace std;


ros::Subscriber RSBottom_sub;
ros::Subscriber RSMiddle_sub;
ros::Subscriber RSLeft_sub;
ros::Subscriber RSRight_sub;
ros::Subscriber FrontSick_sub;
ros::Subscriber RearSick_sub;
ros::Subscriber RoboSense_sub;
ros::Subscriber Motor_sub;

ros::Publisher play_pub;

bool bottomRS,middleRS,leftRS,rightRS,frontSk,rearSk,Robo,Motor;
bool bottomRSF,middleRSF,leftRSF,rightRSF,frontSkF,rearSkF,RoboF,MotorF,allF;


// prototype
void RSBottomCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void RSMiddleCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void RSLeftCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void RSRightCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void FrontSickCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void RearSickCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void RoboSenseCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void MotorCallBack(const nav_msgs::Odometry::ConstPtr& lscan);
void publish_sound(int id,int startdelay,int restartdelay);

/*
void publish_event(string s)
{
	htbot::debug status;
	status.msg = s;
	event_pub.publish(status);
	return;
}
*/

void RSBottomCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {
	//
	bottomRS = true;
}

void RSMiddleCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {
	//
	middleRS = true;
}

void RSLeftCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {
	//
	leftRS = true;
}

void RSRightCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {
	//
	rightRS = true;
}

void FrontSickCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {
	//
	frontSk = true;
}

void RearSickCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {
	//
	rearSk = true;
}

void RoboSenseCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {
	//
	Robo = true;
}

void MotorCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
	//
	Motor = true;
}

void publish_sound(int id,int startdelay,int restartdelay)
{
	htbot::sound cmd;
	cmd.id = id;
	cmd.startdelay = startdelay;
	cmd.restartdelay = restartdelay;
	play_pub.publish(cmd);
	return;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Monitor_Node");
  ros::NodeHandle n;
	ros::Rate loop_rate(1.0);
	int cnt;
	int cbrs,cmrs,clrs,crrs,cfsk,crsk,crb,cmt;

	bottomRS = false;
	middleRS = false;
  leftRS = false;
  rightRS = false;
  frontSk = false;
  rearSk = false;
  Robo = false;
  Motor = false;
	bottomRSF = false;
	middleRSF = false;
  leftRSF = false;
  rightRSF = false;
  frontSkF = false;
  rearSkF = false;
  RoboF = false;
  MotorF = false;
	allF = false;

	n.setParam("ALLSENSORSOK",false); 

	RSBottom_sub= n.subscribe<sensor_msgs::LaserScan>("/bottomScan",100, RSBottomCallBack);	 
	RSMiddle_sub= n.subscribe<sensor_msgs::LaserScan>("/middleScan",100, RSMiddleCallBack);
	RSLeft_sub= n.subscribe<sensor_msgs::LaserScan>("/leftScan",100, RSLeftCallBack);		
	RSRight_sub= n.subscribe<sensor_msgs::LaserScan>("/rightScan",100, RSRightCallBack);	
	FrontSick_sub= n.subscribe<sensor_msgs::LaserScan>("/scanF",100, FrontSickCallBack);
	RearSick_sub= n.subscribe<sensor_msgs::LaserScan>("/scanR",100, RearSickCallBack);
	RoboSense_sub= n.subscribe<sensor_msgs::LaserScan>("/RoboScan",100, RoboSenseCallBack);
	Motor_sub= n.subscribe<nav_msgs::Odometry>("/odom",100, MotorCallBack);
	play_pub = n.advertise<htbot::sound>("sound", 1);

	cnt = 0;
	cbrs = 0;
  cmrs = 0;
  clrs = 0;
  crrs = 0;
  cfsk = 0;
  crsk = 0;
  crb = 0;
  cmt = 0;

	while (true) { 
		//ROS_INFO("-------- Monitor ---------------");
		ros::spinOnce();	
		
		if (bottomRS) {
			if (!bottomRSF) {
				publish_sound(BOTTOMRSON,0,5); 
				ROS_INFO("-------- Monitor : BottomRS---------------");
				bottomRSF = true;
			}
			bottomRS = false;
		} else {
			if (bottomRSF) {
				cbrs++;
				if (cbrs >= 10) {
					publish_sound(BOTTOMRSOFF,0,5);				
					bottomRSF = false;
					cbrs = 0;
				}
			}
		}
		if (middleRS) {
			if (!middleRSF) {
				publish_sound(MIDDLERSON,0,5); 
				ROS_INFO("-------- Monitor : MiddleRS---------------");
				middleRSF = true;
			}
			middleRS = false;
		} else {
			if (middleRSF) {
				cmrs++;
				if (cmrs >= 10) {
					publish_sound(MIDDLERSOFF,0,5);				
					middleRSF = false;
					cmrs = 0;
				}
			}
		}
		if (leftRS) {
			if (!leftRSF) {
				publish_sound(LEFTRS,0,5); 
				ROS_INFO("-------- Monitor : LeftRS---------------");
				leftRSF = true;
			}
			leftRS = false;
		} else {
			if (leftRSF) {
				clrs++;
				if (clrs >= 10) {
					publish_sound(LEFTRSOFF,0,5);				
					leftRSF = false;
					clrs = 0;
				}
			}
		}
		if (rightRS) {
			if (!rightRSF) {
				publish_sound(RIGHTRS,0,5); 
				ROS_INFO("-------- Monitor : RightRS---------------");
				rightRSF = true;
			}
			rightRS = false;
		} else {
			if (rightRSF) {
				crrs++;
				if (crrs >= 10) {
					publish_sound(RIGHTRSOFF,0,5);				
					rightRSF = false;
					crrs = 0;
				}
			}
		}
		if (frontSk) {
			if (!frontSkF) {
				publish_sound(FRONTSICKON,0,5); 
				ROS_INFO("-------- Monitor : Front Sick---------------");
				frontSkF = true;
			}
			frontSk = false;
		} else {
			if (frontSkF) {
				cfsk++;
				if (cfsk >= 10) {
					publish_sound(FRONTSICKOFF,0,5);				
					frontSkF = false;
					cfsk = 0;
				}
			}
		}
		if (rearSk) {
			if (!rearSkF) {
				publish_sound(REARSICKON,0,5); 
				ROS_INFO("-------- Monitor : Rear Sick---------------");
				rearSkF = true;
			}
			rearSk = false;
		} else {
			if (rearSkF) {
				crsk++;
				if (crsk >= 10) {
					publish_sound(REARSICKOFF,0,5);				
					rearSkF = false;
					crsk = 0;
				}
			}
		}
		if (Robo) {
			if (!RoboF) {
				publish_sound(LS3D,0,5); 
				ROS_INFO("-------- Monitor : 3D Laser---------------");
				RoboF = true;
			}
			Robo = false;
		} else {
			if (RoboF) {
				crb++;
				if (crb >= 10) {
					publish_sound(LS3DOFF,0,5);				
					RoboF = false;
					crb = 0;
				}
			}
		}
		if (Motor) {
			if (!MotorF) {
				publish_sound(MOTOROK,0,5); 
				MotorF = true;
			}
			Motor = false;
		} else {
			if (MotorF) {
				cmt++;
				if (cmt >= 10) {
					publish_sound(MOTOROFF,0,5);				
					MotorF = false;
					cmt = 0;
				}
			}
		}
		
		if (bottomRSF && middleRSF && leftRSF && rightRSF && frontSkF && rearSkF && RoboF && MotorF && !allF) {
			n.setParam("ALLSENSORSOK",true);			
			allF = true;
		}
		
  	loop_rate.sleep();
	}
	
  return 0;
}



