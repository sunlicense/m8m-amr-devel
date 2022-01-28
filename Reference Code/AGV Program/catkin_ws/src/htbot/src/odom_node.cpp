/*
 * This node is to play sound 
 */
/* 	History
*		Date Modified : 15.7.2015
*		Changes :
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
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
#include "htbot/odom.h"
#include "std_srvs/Empty.h"
#include <actionlib_msgs/GoalID.h>

using namespace std;

ros::Subscriber odom_sub;
double cur_x,cur_y,cur_theta;
ros::Time current_time, last_time;
double cov_x, cov_y, cov_th;
double cov_vx, cov_vy, cov_vth;
ros::Publisher odom_pub;
nav_msgs::Odometry odom;

bool publish_odom(const htbot::odom::ConstPtr& msg)
{
	double distance, angle;

	distance = msg->dist;
	angle = msg->angle;	

	cur_theta += angle;
	cur_x += distance * cos(cur_theta);
	cur_y += distance * sin(cur_theta); 

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();

	//ROS_INFO("Odom. Dist : %.3f. Angle : %.3f. dt : %.3f",distance,angle,dt);
	//ROS_INFO("cov_x=%.4f. cov_y=%.4f. cov_th=%.4f",cov_x,cov_y,cov_th);
	// convert rotation about z into quaternion
	geometry_msgs::Quaternion odom_quat;
	odom_quat.z = sin(cur_theta/2.0);
	odom_quat.w = cos(cur_theta/2.0);

	//next, we'll publish the odometry message over ROS
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	//set the position (with respect to header.frame)
	odom.pose.pose.position.x = cur_x;
	odom.pose.pose.position.y = cur_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	odom.pose.covariance[0] = cov_x;
	odom.pose.covariance[7] = cov_y;
	odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = 1e9;
	odom.pose.covariance[35] = cov_th;

	//set the velocity (with respect to child_frame)
	odom.twist.twist.linear.x = distance/dt;
	odom.twist.twist.angular.z = angle/dt;
	odom.twist.covariance[0] = cov_vx;
	odom.twist.covariance[7] = cov_vy;
	odom.twist.covariance[14] = odom.twist.covariance[21] = odom.twist.covariance[28] = 1e9;
	odom.twist.covariance[35] = cov_vth;

	//publish the message
	odom_pub.publish(odom);
	last_time = current_time;
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Odom Server");
  ros::NodeHandle n;

	current_time = last_time = ros::Time::now();
	cur_x = 0.0;
	cur_y = 0.0;
	cur_theta = 0.0;
	cov_x = 1e-3;
	cov_y = 1e-3;
	cov_th = 1e-3;
	cov_vx = 1e-3;
	cov_vy = 1e-3;
	cov_vth = 1e-3;
	odom_sub = n.subscribe<htbot::odom>("podom", 10, publish_odom);  // 
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  //ROS_INFO("Ready to Play Sound.");
  ros::spin();

  return 0;
}



