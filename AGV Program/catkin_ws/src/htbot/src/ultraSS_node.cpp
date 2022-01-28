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
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "htbot/ultraSS.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "boost/algorithm/string.hpp"

#define MAX_RANGE 1.5
#define MIN_RANGE 0.05  //0.03
#define DEGPIDX 0.33
#define RADPIDX 0.00576
#define PI2 6.2832
#define PI 3.14159
#define PIHALF 1.5708
#define MAXSIZE 60  // 20 degree divided by 0.33
#define MIN_ANGLE -0.1745  // 10 degrees
#define MAX_ANGLE 0.1745
#define SCANFREQ 15.0


using namespace std;

ros::Publisher uRB_pub;
ros::Publisher uRF_pub;
ros::Publisher uLB_pub;
ros::Publisher uLF_pub;
ros::Publisher uFR_pub;
ros::Publisher uFL_pub;

ros::Subscriber uFR_sub,uFL_sub;
ros::Subscriber uLB_sub,uLF_sub;
ros::Subscriber uRB_sub,uRF_sub;

double time_increment;
sensor_msgs::LaserScan scan_msg;

boost::mutex mut;

double uFRR,uFLR,uRFR,uRBR,uLFR,uLBR;

// prototype
void uFRCallback(const std_msgs::Float64::ConstPtr& msg);
void uFLCallback(const std_msgs::Float64::ConstPtr& msg);
void uRFCallback(const std_msgs::Float64::ConstPtr& msg);
void uRBCallback(const std_msgs::Float64::ConstPtr& msg);
void uLFCallback(const std_msgs::Float64::ConstPtr& msg);
void uLBCallback(const std_msgs::Float64::ConstPtr& msg);

void publish_uRB(void);
void publish_uRF(void);
void publish_uLF(void);
void publish_uLB(void);
void publish_uFL(void);
void publish_uFR(void);

void publish_uRB(void) {

	ros::NodeHandle nx;	
	double dist;

	if (uRBR < MIN_RANGE) {
		uRBR = MIN_RANGE;
	} else {
		if (uRBR > MAX_RANGE) {
			uRBR = MAX_RANGE;
		}
	}

	scan_msg.ranges.resize(MAXSIZE);
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "/uRBFrame";
	scan_msg.angle_min = MIN_ANGLE;
	scan_msg.angle_max = MAX_ANGLE;
	scan_msg.angle_increment = RADPIDX;
	scan_msg.time_increment = time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;

	for (int i=0;i<MAXSIZE;i++) {
		scan_msg.ranges[i] = uRBR;
	}
	uRB_pub.publish(scan_msg);
}

void publish_uRF(void) {

	if (uRFR < MIN_RANGE) {
		uRFR = MIN_RANGE;
	} else {
		if (uRFR > MAX_RANGE) {
			uRFR = MAX_RANGE;
		}
	}

	scan_msg.ranges.resize(MAXSIZE);
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "/uRFFrame";
	scan_msg.angle_min = MIN_ANGLE;
	scan_msg.angle_max = MAX_ANGLE;
	scan_msg.angle_increment = RADPIDX;
	scan_msg.time_increment = time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;

	for (int i=0;i<MAXSIZE;i++) {
		scan_msg.ranges[i] = uRFR;
	}
	uRF_pub.publish(scan_msg);
}

void publish_uLF(void) {

	if (uLFR < MIN_RANGE) {
		uLFR = MIN_RANGE;
	} else {
		if (uLFR > MAX_RANGE) {
			uLFR = MAX_RANGE;
		}
	}

	scan_msg.ranges.resize(MAXSIZE);
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "/uLFFrame";
	scan_msg.angle_min = MIN_ANGLE;
	scan_msg.angle_max = MAX_ANGLE;
	scan_msg.angle_increment = RADPIDX;
	scan_msg.time_increment = time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;

	for (int i=0;i<MAXSIZE;i++) {
		scan_msg.ranges[i] = uLFR;
	}
	uLF_pub.publish(scan_msg);
}

void publish_uLB(void) {

	if (uLBR < MIN_RANGE) {
		uLBR = MIN_RANGE;
	} else {
		if (uLBR > MAX_RANGE) {
			uLBR = MAX_RANGE;
		}
	}

	scan_msg.ranges.resize(MAXSIZE);
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "/uLBFrame";
	scan_msg.angle_min = MIN_ANGLE;
	scan_msg.angle_max = MAX_ANGLE;
	scan_msg.angle_increment = RADPIDX;
	scan_msg.time_increment = time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;

	for (int i=0;i<MAXSIZE;i++) {
		scan_msg.ranges[i] = uLBR;
	}
	uLB_pub.publish(scan_msg);
}

void publish_uFR(void) {

	if (uFRR < MIN_RANGE) {
		uFRR = MIN_RANGE;
	} else {
		if (uFRR > MAX_RANGE) {
			uFRR = MAX_RANGE;
		}
	}

	scan_msg.ranges.resize(MAXSIZE);
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "/uFRFrame";
	scan_msg.angle_min = MIN_ANGLE;
	scan_msg.angle_max = MAX_ANGLE;
	scan_msg.angle_increment = RADPIDX;
	scan_msg.time_increment = time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;

	for (int i=0;i<MAXSIZE;i++) {
		scan_msg.ranges[i] = uFRR;
	}
	uFR_pub.publish(scan_msg);
}

void publish_uFL(void) {

	if (uFLR < MIN_RANGE) {
		uFLR = MIN_RANGE;
	} else {
		if (uFLR > MAX_RANGE) {
			uFLR = MAX_RANGE;
		}
	}

	scan_msg.ranges.resize(MAXSIZE);
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "/uFLFrame";
	scan_msg.angle_min = MIN_ANGLE;
	scan_msg.angle_max = MAX_ANGLE;
	scan_msg.angle_increment = RADPIDX;
	scan_msg.time_increment = time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;

	for (int i=0;i<MAXSIZE;i++) {
		scan_msg.ranges[i] = uFLR;
	}
	uFL_pub.publish(scan_msg);
}

void uFRCallback(const std_msgs::Float32::ConstPtr& msg)
{
	{
		boost::mutex::scoped_lock lock(mut);
		uFRR = msg->data / 100.0;
	}
	//ROS_INFO("------------------- ultraSS : uFRR = %.3f -----------------------",uFRR);
}

void uFLCallback(const std_msgs::Float32::ConstPtr& msg)
{
	{
		boost::mutex::scoped_lock lock(mut);
		uFLR = msg->data / 100.0;
	}
	//ROS_INFO("------------------- ultraSS : uFLR = %.3f -----------------------",uFLR);
}

void uRBCallback(const std_msgs::Float32::ConstPtr& msg)
{
	{
		boost::mutex::scoped_lock lock(mut);
		uRBR = msg->data / 100.0;
	}
	//ROS_INFO("*********************** ultraSS : uRBR = %.3f *******************\n",uRBR);
}

void uRFCallback(const std_msgs::Float32::ConstPtr& msg)
{
	{
		boost::mutex::scoped_lock lock(mut);
		uRFR = msg->data / 100.0;
	}
	//ROS_INFO("*********************** ultraSS : uRFR = %.3f ************************",uRFR);
}

void uLBCallback(const std_msgs::Float32::ConstPtr& msg)
{
	{
		boost::mutex::scoped_lock lock(mut);
		uLBR = msg->data  / 100.0;
	}
	//ROS_INFO("------------------- ultraSS : uLBR = %.3f -----------------------",uLBR);
}

void uLFCallback(const std_msgs::Float32::ConstPtr& msg)
{
	{
		boost::mutex::scoped_lock lock(mut);
		uLFR = msg->data / 100.0;
	}
	//ROS_INFO("------------------- ultraSS : uLFR = %.3f -----------------------",uLFR);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "bottomRS_Node");
  ros::NodeHandle n;
	double rate;
	
	int cnt;
	ros::Rate loop_rate(SCANFREQ);
	
	uRB_pub = n.advertise<sensor_msgs::LaserScan>("/ultraRB", 10);
	uRF_pub = n.advertise<sensor_msgs::LaserScan>("/ultraRF", 10);
	uLB_pub = n.advertise<sensor_msgs::LaserScan>("/ultraLB", 10);
	uLF_pub = n.advertise<sensor_msgs::LaserScan>("/ultraLF", 10);
	uFR_pub = n.advertise<sensor_msgs::LaserScan>("/ultraFR", 10);
	uFL_pub = n.advertise<sensor_msgs::LaserScan>("/ultraFL", 10);
	
	//uFR_sub = n.subscribe<std_msgs::Float32>("/uFR", 100, uFRCallback);
	uFL_sub = n.subscribe<std_msgs::Float32>("/uFL", 100, uFLCallback);
	//uRB_sub = n.subscribe<std_msgs::Float32>("/uRB", 100, uRBCallback);
	uRF_sub = n.subscribe<std_msgs::Float32>("/uRF", 100, uRFCallback);
	//uLB_sub = n.subscribe<std_msgs::Float32>("/uLB", 100, uLBCallback);
	uLF_sub = n.subscribe<std_msgs::Float32>("/uLF", 100, uLFCallback);

	time_increment = (1.0 / SCANFREQ) / MAXSIZE;
  cnt = 0;
	while(true) {
		//publish_uRB();
		publish_uRF();
		//publish_uLB();
		publish_uLF();
		//publish_uFR();
		publish_uFL();
		cnt++;
		if (cnt > 22) {
			cnt = 0;
			//ROS_INFO("----- ultra : uFR=%.2f. uRB=%.2f. uLB=%.2f. ------",uFRR,uRBR,uLBR);
			//ROS_INFO("----- ultra : uFL=%.2f.uRF=%.2f. uLF=%.2f. ------",uFLR,uRFR,uLFR);
			//ROS_INFO("----- ultra : uFL=%.2f. uFR=%.2f. uRF=%.2f. uRB=%.2f. uLF=%.2f. uLB=%.2f. ------",uFLR,uFRR,uRFR,uRBR,uLFR,uLBR);
		}

		ros::spinOnce();	
		loop_rate.sleep();
	}

  return 0;
}



