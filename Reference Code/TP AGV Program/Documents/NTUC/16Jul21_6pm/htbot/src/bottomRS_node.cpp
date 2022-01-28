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
#include "std_msgs/String.h"

#define MAX_RANGE 3.0
#define MIN_RANGE 0.05  //0.03
#define DWIDTH 0.3  //0.35// 0.4m
#define DBOX 2.0
#define DFRONT 0.02  //0.1
#define DEGPIDX 0.25
#define RADPIDX 0.00436
#define PI 3.14159
#define DM 8.44 //28.07 // dm = atan(DWIDTH / (DBOX + DFRONT)) * (180.0 / PI);  // degree
#define DMIDX 34 //112  // (int)(dm / DEGPIDX)
#define DG  86 //70 // dg = atan(DWIDTH / DFRONT) * (180.0 / PI);  // degree
#define DGIDX 345 //280 // (int)(dg / DEGPIDX)
#define LASEROFFSET 0.03
#define BASEOFFSET 0.31
#define DISTGT350MM  0.6
#define DETECTRANGE 1.5

#define FILTER_SCAN_SIZE 7
#define FILTER_SPIKE_SIZE 3
#define STEP 0.08  // m
#define FILTER_LEVEL_SIZE 0.04 // m

using namespace std;

ros::Subscriber laser_sub;
ros::Publisher scan_pub;
//ros::Publisher obs_pub;

int laser_size;
int clearObs,lookforObs,cidx,lidx,ridx,lmidx,rmidx;
double refr,lrange,dm,dg,dt,od, tod;
sensor_msgs::LaserScan scan_msg;
std_msgs::Float32 obs_msg;	
ros::Time last_laser_time;
int rightcount, leftcount;
double laser_data[1000];

// prototype
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld;
	int ninf,nerr;
	double t1,t2,t3,t4,t5,t6,t7;
	
	//bool clear ;
	
	laser_size = lscan->ranges.size();
	cidx = (int)(laser_size / 2.0);
	rmidx = cidx - DMIDX;
	lmidx = cidx + DMIDX;
	ridx = cidx - DGIDX;
	lidx = cidx + DGIDX;
	od = 0.0; // obs dist in front of robot
	rightcount = 0;
	leftcount = 0;
	ninf = 0;
	nerr = 0;
	//ROS_INFO("****** BottomRS  Laser Size : %d.   *******",laser_size);
	//ROS_INFO("****** BottomRS Range : %3f.   *******",lscan->ranges[cidx]);
	//ROS_INFO("****** BottomRS  frame : %s.   *******",lscan->header.frame_id.c_str());
	//ROS_INFO("**** BottomRS mina=%.3f. maxa=%.3f. an=%.5f   *****",lscan->angle_min,lscan->angle_max,lscan->angle_increment);		
	/*
	scan_msg.ranges.resize(laser_size);
	scan_msg.header.stamp = lscan->header.stamp;
	scan_msg.header.frame_id = lscan->header.frame_id;
	scan_msg.angle_min = lscan->angle_min;
	scan_msg.angle_max = lscan->angle_max;
	scan_msg.angle_increment = lscan->angle_increment;
	scan_msg.scan_time = lscan->scan_time;
	scan_msg.time_increment = lscan->time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;
	*/
	//ROS_INFO("ainc : %.6f. stime : %.6f. t_inc : %.6f",lscan->angle_increment,lscan->scan_time,lscan->time_increment);
	//ROS_INFO("**** pt A od : %.2f. ******",od);

	// filter start
	
	for (int i=0;i<laser_size;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			if ( (lscan->ranges[i] < MAX_RANGE) && (lscan->ranges[i] > MIN_RANGE) ){
				if (lscan->ranges[i] < DETECTRANGE) {
					ROS_INFO("****** BottomRS Range : id:%d. %3f.   *******",i,lscan->ranges[i]);
				}
			} 
		}
	}
	
	/*
	for (int i=0;i<laser_size-4;i++) {
		t1 = laser_data[i];
		t2 = laser_data[i+1];
		t3 = laser_data[i+2];
		t4 = laser_data[i+3];
		t5 = laser_data[i+4];
		//t6 = laser_data[i+5];
		//t7 = laser_data[i+6];
		
		if ((fabs(t2 - t1) > STEP)) {
			if (fabs(t3-t2) > STEP) {
				laser_data[i+1] = t1;
			} else {
				if (fabs(t4-t3) > STEP) {
					laser_data[i+1] = t1;
				} else {
					if (fabs(t5-t4) > STEP) {
						laser_data[i+1] = t1;
					}
				}
			}			 
		}
	}
	*/
	// filter end


}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "bottomRS_Node");
  ros::NodeHandle n;
	
	last_laser_time = ros::Time::now();
	//n.setParam("MainLaserStatus",0);	
	laser_sub= n.subscribe<sensor_msgs::LaserScan>("/bottomScan",100, laserCallBack);	 // /scan_base - org
	scan_pub = n.advertise<sensor_msgs::LaserScan>("/bottomLS", 10);
  ros::spin();

  return 0;
}



