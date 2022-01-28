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

#define MAX_RANGE 30.0
#define MIN_RANGE 0.02  //0.03
using namespace std;

ros::Subscriber laser_sub;
ros::Publisher scan_pub;

int laser_size;
sensor_msgs::LaserScan scan_msg;

// prototype
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	
	laser_size = lscan->ranges.size();
	//ROS_INFO("Laser Size : %d.",laser_size);	
	scan_msg.ranges.resize(laser_size);
	//scan_msg.intensities.resize(laser_size);
	scan_msg.header.stamp = lscan->header.stamp;
	scan_msg.header.frame_id = lscan->header.frame_id;
	scan_msg.angle_min = lscan->angle_min;
	scan_msg.angle_max = lscan->angle_max;
	scan_msg.angle_increment = lscan->angle_increment;
	scan_msg.scan_time = lscan->scan_time;
	scan_msg.time_increment = lscan->time_increment;
	//scan_msg.range_min = lscan->range_min;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;
	//ROS_INFO("ainc : %.6f. stime : %.6f. t_inc : %.6f",lscan->angle_increment,lscan->scan_time,lscan->time_increment);
	for (int i=0;i<laser_size;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			//ROS_INFO("Here P2");
			if (lscan->ranges[i] > MAX_RANGE) {
				scan_msg.ranges[i] = MAX_RANGE - 0.1;
			} else {
				if (lscan->ranges[i] < MIN_RANGE) {
					scan_msg.ranges[i] = MAX_RANGE - 0.1;
				} else {
					scan_msg.ranges[i] = lscan->ranges[i];
				}
			}
		} else {
			scan_msg.ranges[i] = MAX_RANGE - 0.1;
		}
		//ROS_INFO("Here P3");
		//scan_msg.intensities[i] = lscan->intensities[i];
		//ROS_INFO("Here P4");
	}
	scan_pub.publish(scan_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_filter");
  ros::NodeHandle n;
	
	laser_sub= n.subscribe<sensor_msgs::LaserScan>("/scannys",1, laserCallBack);	 // /scan_base - org
	scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 10);

  ros::spin();

  return 0;
}



