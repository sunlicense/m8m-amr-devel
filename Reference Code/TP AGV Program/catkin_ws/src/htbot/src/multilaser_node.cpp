/*
 * This node listen to 2 laser scans and generate a combined laserscan 
 */
/* 	History
*		Date Modified : 5.12.2019
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
#include "std_msgs/String.h"
#include "boost/thread/mutex.hpp"



#define MAX_RANGE1 25.0
#define MIN_RANGE1 0.05  
#define MAX_RANGE2 30.0
#define MIN_RANGE2 0.04  
#define MAX_RANGE 30.0
#define MIN_RANGE 0.05  
#define MAXSIZE 1100
#define NUMREADING 1090
#define CCIDX 545
#define CCIDX2 1090
#define SCANFREQ 15
#define DIST_TO_BASELINK1 0.3  
#define DIST_TO_BASELINK2 0.3

#define DEGPIDX 0.33
#define RADPIDX 0.00576
#define PI 3.14159

using namespace std;

ros::Subscriber laser1_sub; // front
ros::Subscriber laser2_sub; // rear
ros::Publisher multiscan_pub;
boost::mutex mut;

int laser1_size,laser2_size;

//sensor_msgs::LaserScan scan_msg;
double laser_data[MAXSIZE];
bool flag1,flag2;
double scan_time,time_increment;

// prototype
void laser1CallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void laser2CallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void initialise_laserdata();
void publish_multiscan();

void laser1CallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	double ld,min,an,angle_increment,time_increment,rn;
	double x,y;
	int ix,cidx;
	
	if (!flag1) {
		return;
	}
	flag1 = false;
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		laser1_size = lscan->ranges.size();
		cidx = (int)(laser1_size / 2.0);
		angle_increment = lscan->angle_increment;
		time_increment = lscan->time_increment;
		ROS_INFO("------- Laser1 : size=%d. cidx=%d. a_incr=%.5f. t_incr=%.6f. ------------",laser1_size,cidx,angle_increment,time_increment);
		for (int i=0;i<laser1_size;i++) {
			if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
				if (lscan->ranges[i] > MAX_RANGE1) {
					continue;
				} else {
					if (lscan->ranges[i] < MIN_RANGE1) {  		
						continue;
					} else {
						ld = lscan->ranges[i];
						if (i<cidx) {
							an = (cidx-i)*angle_increment;
							x = ld * cos(an);
							y = ld * sin(an);
							x = x + DIST_TO_BASELINK1;
							an = atan(y/x);
							ix = CCIDX - ((int)(an/RADPIDX));
							rn = x / cos(an);
							if (rn < MAX_RANGE) {
								laser_data[ix] = rn;
							}
						} else {
							an = (i - cidx)*angle_increment;
							x = ld * cos(an);
							y = ld * sin(an);
							x = x + DIST_TO_BASELINK1;
							an = atan(y/x);
							ix = CCIDX + ((int)(an/RADPIDX));
							rn = x / cos(an);
							if (rn < MAX_RANGE) {
								laser_data[ix] = rn;
							}
						}
					}				
				}
			} else {
				continue;
			}
		}
	}
}

void laser2CallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	double ld,min,an,angle_increment,time_increment,rn;
	double x,y;
	int ix,cidx;
	
	if (!flag2) {
		return;
	}
	flag2 = false;
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		laser2_size = lscan->ranges.size();
		cidx = (int)(laser2_size / 2.0);
		angle_increment = lscan->angle_increment;
		time_increment = lscan->time_increment;
		ROS_INFO("------- Laser2 : size=%d. cidx=%d. a_incr=%.3f. t_incr=%.5f. ------------",laser2_size,cidx,angle_increment,time_increment);
		for (int i=0;i<laser2_size;i++) {
			if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
				if (lscan->ranges[i] >= MAX_RANGE2) {
					continue;
				} else {
					if (lscan->ranges[i] <= MIN_RANGE2) {  		
						continue;
					} else {
						ld = lscan->ranges[i];
						if (i<cidx) {
							an = (cidx-i)*angle_increment;
							x = ld * cos(an);
							y = ld * sin(an);
							x = x + DIST_TO_BASELINK2;
							an = atan(y/x);							
							ix = CCIDX + ((int)((PI-an)/RADPIDX));
							rn = x / cos(an);
							//laser_data[ix] = rn;
							if (rn < MAX_RANGE) {
								laser_data[ix] = rn;
							}
						} else {
							an = (i - cidx)*angle_increment;
							x = ld * cos(an);
							y = ld * sin(an);
							x = x + DIST_TO_BASELINK2;
							an = atan(y/x);
							ix = CCIDX - ((int)((PI-an)/RADPIDX));
							rn = x / cos(an);
							//laser_data[ix] = rn;
							if (rn < MAX_RANGE) {
								laser_data[ix] = rn;
							}
						}
					}				
				}
			} else {
				continue;
			}
		}
	}
}

void initialise_laserdata() {
	for (int i=0;i<MAXSIZE;i++) {
		laser_data[i] = MAX_RANGE;
	}
}

void publish_multiscan() {
	sensor_msgs::LaserScan scan_msg;
	scan_msg.ranges.resize(NUMREADING);
	//scan_msg.intensities.resize(NUMREADING);
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "base_link";;
	scan_msg.angle_min = -3.14;
	scan_msg.angle_max = 3.14;
	scan_msg.angle_increment = RADPIDX;
	//scan_msg.scan_time = scan_time;
	scan_msg.time_increment = time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;
	for (int i=0;i<NUMREADING;i++) {
		scan_msg.ranges[i] = laser_data[i];
	}
	multiscan_pub.publish(scan_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_filter");
  ros::NodeHandle n;	
	double rate;	
	
	n.getParam("MultiLaser_Rate",rate);
	ros::Rate loop_rate(rate);
	flag1 = flag2 = false;

	laser1_sub= n.subscribe<sensor_msgs::LaserScan>("/scan",100, laser1CallBack);	 
	laser2_sub= n.subscribe<sensor_msgs::LaserScan>("/scanh",100, laser2CallBack);	 
	multiscan_pub = n.advertise<sensor_msgs::LaserScan>("/multiscan", 100);

	//initialise_laserdata();
	scan_time = 1.0/SCANFREQ;
	time_increment = (1.0 / SCANFREQ) / NUMREADING;
  while (true) {  
		initialise_laserdata();
		flag1 = flag2 = true;	
		//flag2 = true;	
		ros::spinOnce();	
		//ROS_INFO("------------- multilaser Node : Main Loop--------");
		if (!flag1 && !flag2) {
			publish_multiscan();
		}
  	loop_rate.sleep();
	}

  return 0;
}



