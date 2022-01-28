/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Modified : 7.5.21 : 10.34am
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

#define MAX_RANGE 25.0
#define MIN_RANGE 0.03  //0.03
#define DWIDTH 0.3  //0.35// 0.4m
#define DBOX 2.0
#define DFRONT 0.02  //0.1
#define DEGPIDX 0.33 //0.25 - hku
#define RADPIDX 0.00576 //0.00436-hku
#define PI 3.14159
#define DM 8.44 //28.07 // dm = atan(DWIDTH / (DBOX + DFRONT)) * (180.0 / PI);  // degree
#define DMIDX 34 //112  // (int)(dm / DEGPIDX)
#define DG  86 //70 // dg = atan(DWIDTH / DFRONT) * (180.0 / PI);  // degree
#define DGIDX 345 //280 // (int)(dg / DEGPIDX)
#define LASEROFFSET 0.03
#define BASEOFFSET 0.31
#define DISTGT350MM  0.6
#define RADPERDEG 0.0174533

#define FILTER_SCAN_SIZE 7
#define FILTER_SPIKE_SIZE 3
#define STEP 0.08  // m
#define FILTER_LEVEL_SIZE 0.04 // m

using namespace std;

ros::Subscriber laser_sub;
ros::Publisher scan_pub;

int laser_size;
int cidx,lidx,ridx,ttidx;
double min_angle,max_angle;
sensor_msgs::LaserScan scan_msg;	
ros::Time last_laser_time;
int right_idx,left_idx;
double laser_data[1000];

// prototype
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld;
	int j;
	
	laser_size = lscan->ranges.size();
	cidx = (int)(laser_size / 2.0);

	right_idx = (int) (-min_angle / RADPIDX);
	left_idx = (int) (max_angle / RADPIDX);
	
	ridx = cidx - right_idx;
	lidx = cidx + left_idx;
	ttidx = right_idx+left_idx;
	//ROS_INFO("****** RefScan : min=%.3f. max=%.3f  ******",min_angle,max_angle);
	//ROS_INFO("****** RefScan : right_idx=%d. left_idx=%d  ******",right_idx,left_idx);
	//ROS_INFO("******  Laser Size : %d. cidx=%d. ridx=%d. lidx=%d  *******",laser_size,cidx,ridx,lidx);	
	scan_msg.ranges.resize(ttidx+5);
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "/refscan";
	scan_msg.angle_min = min_angle;
	scan_msg.angle_max = max_angle;
	//scan_msg.angle_increment = (max_angle-min_angle) / ttidx;
	scan_msg.angle_increment = lscan->angle_increment;
	scan_msg.time_increment = lscan->time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;

	j=0;
	//ROS_INFO("\n\n--------------------------------------- RefScan --------------------------------");
	for (int i=0;i<laser_size;i++) {
		if (i >=ridx && i<=lidx) {
			if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
				if (lscan->ranges[i] > MAX_RANGE) {
					scan_msg.ranges[j++] = MAX_RANGE;
				} else {
					if (lscan->ranges[i] < MIN_RANGE) {
						scan_msg.ranges[j++] = MIN_RANGE;						
					} else {
						ld = lscan->ranges[i];						
						scan_msg.ranges[j++] = lscan->ranges[i];
					}
				}
			} else {
				scan_msg.ranges[j++] = MAX_RANGE;
			}
		} else {
			continue;
		}
	}
	scan_pub.publish(scan_msg);	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Reference Scan Node");
  ros::NodeHandle n;
	ros::NodeHandle np("~");
	
	last_laser_time = ros::Time::now();
	np.param("min_angle",min_angle,-0.7);		
	np.param("max_angle",max_angle,0.7);	
	
	laser_sub= n.subscribe<sensor_msgs::LaserScan>("/scanFF",100, laserCallBack);	 // /scan_base - org
	scan_pub = n.advertise<sensor_msgs::LaserScan>("/refscan", 100);
  ros::spin();

  return 0;
}



