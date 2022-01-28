
/* 	History
*		Date Modified : 21.10.2019
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

#define STEP 0.1  // m
#define STEP1 0.2


using namespace std;

ros::Subscriber laser_sub;
ros::Publisher scan_pub;

int laser_size;

sensor_msgs::LaserScan scan_msg;

double maxRange,minRange;
int laser_cnt;
double laser_data[1000];
int laserscan_cnt[1000];

// prototype
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);



void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld;
	int ninf,nerr;
	double t0,t1,t2,t3,t4,t5,t6,t7;

	laser_size = lscan->ranges.size();

	ROS_INFO("****** DepthNode Laser Size : %d.   *******",laser_size);	
	
	scan_msg.ranges.resize(laser_size);
	scan_msg.header.stamp = lscan->header.stamp;
	scan_msg.header.frame_id = lscan->header.frame_id;
	scan_msg.angle_min = lscan->angle_min;
	scan_msg.angle_max = lscan->angle_max;
	scan_msg.angle_increment = lscan->angle_increment;
	scan_msg.scan_time = lscan->scan_time;
	scan_msg.time_increment = lscan->time_increment;
	scan_msg.range_min = lscan->range_min;
	scan_msg.range_max = lscan->range_max;
	minRange = lscan->range_min;
	maxRange = lscan->range_max;
	
	//ROS_INFO("ainc : %.6f. stime : %.6f. t_inc : %.6f",lscan->angle_increment,lscan->scan_time,lscan->time_increment);
	//ROS_INFO("**** pt A od : %.2f. ******",od);
	ROS_INFO("--------------DepthNode : Point 0 ------------");
	// filter start
	for (int i=0;i<laser_size;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			if (lscan->ranges[i] > maxRange) {
				laser_data[i] = maxRange  ;
			} else {
				if (lscan->ranges[i] < minRange) {  //MIN_RANGE					
					laser_data[i] = maxRange ;
				} else {
					laser_data[i] = lscan->ranges[i];
				}
			}
		}
	}
	ROS_INFO("--------------DepthNode : Point 1 ------------");
	if (laser_size > 6) {
		for (int i=0;i<laser_size;i++) {
			if (i<3) {
				t3 = laser_data[i];
				t4 = laser_data[i+1];
				t5 = laser_data[i+2];
				t6 = laser_data[i+3]; 
				if ( (fabs(t4-t3) > STEP) || (fabs(t5-t3) > STEP) || (fabs(t6-t3) > STEP) ) {
					laser_data[i] = maxRange;
				}
			} else {
				if (i < laser_size - 3) {
					t0 = laser_data[i-3];
					t1 = laser_data[i-2];
					t2 = laser_data[i-1];
					t3 = laser_data[i];
					t4 = laser_data[i+1];
					t5 = laser_data[i+2];
					t6 = laser_data[i+3];
					if ( (fabs(t4-t3) > STEP) || (fabs(t5-t3) > STEP) || (fabs(t6-t3) > STEP) ) {
						if ( (fabs(t3-t2) > STEP) || (fabs(t3-t1) > STEP) || (fabs(t3-t0) > STEP) ) {
							laser_data[i] = maxRange;
						} 						
					}
				} else {
					t0 = laser_data[i-3];
					t1 = laser_data[i-2];
					t2 = laser_data[i-1];
					t3 = laser_data[i];
					if ( (fabs(t3-t2) > STEP) || (fabs(t3-t1) > STEP) || (fabs(t3-t0) > STEP) ) {
						laser_data[i] = maxRange;
					}
				}
			}
		}
	}
	ROS_INFO("--------------DepthNode : Point 2 ------------");
	for (int i=0;i<laser_size;i++) {
		scan_msg.ranges[i] = laser_data[i];
	}
	ROS_INFO("--------------DepthNode : Point 4 ------------");
	scan_pub.publish(scan_msg);
	ROS_INFO("--------------DepthNode : Point 5 ------------");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "depth_filter");
  ros::NodeHandle n;
	
	for (int i=0;i<1000;i++) {
		laser_data[i] = 0.0;
		laserscan_cnt[i] = 0;
	}
	laser_cnt = 0;
	laser_sub= n.subscribe<sensor_msgs::LaserScan>("depth2",100, laserCallBack);	 // /scan_base - org
	scan_pub = n.advertise<sensor_msgs::LaserScan>("depthlaserscan", 10);
  ros::spin();

  return 0;
}



