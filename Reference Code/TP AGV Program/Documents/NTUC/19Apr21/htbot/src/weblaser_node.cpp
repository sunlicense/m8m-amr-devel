/*
 * This node is to generate web laser topic for web browser
 */
/* 	History
*		Date Modified : 2.12.2019
*		Changes :
*/

#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h> 
#include <fcntl.h>
#include <signal.h>
#include <string>
#include <sys/time.h>
#include "htbot/move.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "boost/algorithm/string.hpp"
#include <tf/transform_datatypes.h>
#include <stdint.h>
#include <sensor_msgs/LaserScan.h>
#include "htbot/weblaser.h"

#define MAX_RANGE 25.0
#define MIN_RANGE 0.05  //0.03

#define PI_HALF 1.570796
#define PI 3.14159
#define PI2 6.28319


using namespace std;

ros::Subscriber laser_sub;
ros::Subscriber pose_sub;
ros::Publisher web_pub;

int laser_size;
int cidx;
boost::mutex mut;
double px,py,pz,prx,pry,prz,prw,yawp;
htbot::weblaser wlaser_msg;
int wlaser_size;
int WEBLASERGAP;
int cnt;
bool flag;

// prototype
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		px = msg->position.x;
		py = msg->position.y;
		pz = msg->position.z;
		prx = msg->orientation.x;
		pry = msg->orientation.y;
		prz = msg->orientation.z;
		prw = msg->orientation.w;		
	}
	//ROS_INFO("------------ Swendgoal : Robot Pose px=%.3f. py=%.3f-----------",px,py);
}

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double an,rn,dx,dy,incr;
	int j;

	if (!flag) {
		return;
	}
	flag = false;
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		tf::Quaternion qp(0.0,0.0,prz,prw);
		yawp = tf::getYaw(qp);
		laser_size = lscan->ranges.size();
		wlaser_size = laser_size / (1.0 * WEBLASERGAP);
		wlaser_msg.px.resize(wlaser_size+5);
		wlaser_msg.py.resize(wlaser_size+5);
		wlaser_msg.size = wlaser_size;
		cidx = (int)(laser_size / 2.0);
		incr = lscan->angle_increment;
		j = 0;
		for (int i=0;i<laser_size;i=i+WEBLASERGAP) {
			if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
				if (lscan->ranges[i] > MAX_RANGE) {
					wlaser_msg.px[j] = -999.0;  // not to be display at webpage
					wlaser_msg.py[j] = -999.0;
					j++;
					continue;
				}
				if (lscan->ranges[i] < MIN_RANGE) {
					wlaser_msg.px[j] = -999.0;  // not to be display at webpage
					wlaser_msg.py[j] = -999.0;
					j++;
					continue;
				}
				rn = lscan->ranges[i];
				if (i < cidx) {
					an = yawp - (incr * (cidx - i));
				} else {
					an = yawp + (incr * (i - cidx));
				}
				if (an > PI2) {
					an = an - PI2;
					if (an > PI) {
						an = PI2 - an;
					}
				}
				if (an < -PI2) {
					an = an + PI2;
					if (an < -PI) {
						an = PI2 - an;
					}
				}
			
				if (an >= 0.0) {
					if (an < PI_HALF) {
						// 1st Q 0 to 90
						dx = rn * cos(an);
						dy = rn * sin(an);
						wlaser_msg.px[j] = px + dx;  
						wlaser_msg.py[j] = py + dy;
						j++;
					} else {
						// 2nd Q. 90 to 180
						an = PI - an;
						dx = rn * cos(an);
						dy = rn * sin(an);
						wlaser_msg.px[j] = px - dx;  
						wlaser_msg.py[j] = py + dy;
						j++;
					}
				} else {
					if (an > -PI_HALF) {
						// 4th Q. 0 to -90
						an = - an;
						dx = rn * cos(an);
						dy = rn * sin(an);
						wlaser_msg.px[j] = px + dx;  
						wlaser_msg.py[j] = py - dy;
						j++;
					} else {
						// 3rd Q. -90 to -180
						an = PI + an;
						dx = rn * cos(an);
						dy = rn * sin(an);
						wlaser_msg.px[j] = px - dx;  
						wlaser_msg.py[j] = py - dy;
						j++;
					}
				}
			}
		}
		wlaser_msg.size = j;
		web_pub.publish(wlaser_msg);	
	}
	//ROS_INFO("------------- WebLaser Node : Laser callback. Count = %d--------",cnt++);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Web Laser Node");
  ros::NodeHandle n;
	double rate;	
	
	n.getParam("WEBLASERGAP",WEBLASERGAP);
	n.getParam("WEBLASER_Rate",rate);
	ros::Rate loop_rate(rate);
	cnt = 0;
	flag = false;

	ROS_INFO("------------- WebLaser Node : WEBLASERGAP = %.d. Rate = %.2f ----------------",WEBLASERGAP,rate);
	laser_sub= n.subscribe<sensor_msgs::LaserScan>("/scan",100, laserCallBack);	 // /scan_base - org
	web_pub = n.advertise<htbot::weblaser>("/weblaser", 10);
	pose_sub = n.subscribe<geometry_msgs::Pose>("/robot_pose", 1,poseCallback);

	while (true) {  	
		flag = true;
		ros::spinOnce();	
		//ROS_INFO("------------- WebLaser Node : Main Loop--------");
  	loop_rate.sleep();
	}

  return 0;
}



