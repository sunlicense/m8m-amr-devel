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

#define MAX_RANGE 25.0
#define MIN_RANGE 0.05  //0.03
#define DWIDTH 0.4  //0.35// 0.4m
#define DBOX 2.0
#define DFRONT 0.02  //0.1
#define DEGPIDX 0.33
#define RADPIDX 0.0057596
#define PI 3.14159
#define DM 13.9 //28.07 // dm = atan(DWIDTH / (DBOX + DFRONT)) * (180.0 / PI);  // degree
#define DMIDX 42 //112  // (int)(dm / DEGPIDX)
#define DG  87 //70 // dg = atan(DWIDTH / DFRONT) * (180.0 / PI);  // degree
#define DGIDX 264 //280 // (int)(dg / DEGPIDX)
#define LASEROFFSET 0.03
#define BASEOFFSET 0.31
#define DISTGT350MM  0.6

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
	

	//clear = false;
	//nx.getParam("clearObs",clearObs);
	//nx.getParam("lookforObs",lookforObs);
	//if (lookforObs == 1) {
	//	ROS_INFO("Look for Obs");
	//}
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
	//ROS_INFO("******  Laser Size : %d.   *******",laser_size);	
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
		//scan_msg.ranges[i] = MAX_RANGE ;
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			if (lscan->ranges[i] > MAX_RANGE) {
				laser_data[i] = MAX_RANGE  ;
			} else {
				if (lscan->ranges[i] < MIN_RANGE) {  //MIN_RANGE					
					laser_data[i] = MAX_RANGE ;
				} else {
					laser_data[i] = lscan->ranges[i];
				}				
			}
		} else {
			laser_data[i] = MAX_RANGE;
		}
	}
	
	// filter end

	for (int i=0;i<laser_size;i++) {
		ld = laser_data[i];
		//scan_msg.ranges[i] = ld;
		refr = 0.0;		
		tod = 0.0;
		
		// check for obstacles	
		if ((i >= ridx) && (i <= rmidx)) {
			dt = (cidx-i) * RADPIDX;  // rad
			refr = DWIDTH / sin(dt);
			if ((refr > 0.0) && (ld < refr)) {  // obs in right side of box
				tod = ld * cos(dt);
				if ((tod < od) || (od == 0.0)) {
					od = tod;
				}
			}
			continue;
		}		
		if ((i > rmidx) && (i <= cidx)) {
			dt = (cidx-i) * RADPIDX;
			refr = (DBOX + DFRONT) / cos(dt);
			if ((refr > 0.0) && (ld < refr)) {  // obs in box
				tod = ld * cos(dt);
				if ((tod < od) || (od == 0.0) ) {
					od = tod;
				}
			}
			continue;
		}	
		if ((i > cidx) && (i <= lmidx)) {
			dt = (i - cidx) * RADPIDX;
			refr = (DBOX + DFRONT) / cos(dt);
			if ((refr > 0.0) && (ld < refr)) {  // obs in box
				tod = ld * cos(dt);
				if ((tod < od) || (od == 0.0)) {
					od = tod;
				}
			}
			continue;
		}

		if ((i >= lmidx) && (i <= lidx)) {
			dt = (i - cidx) * RADPIDX;
			refr = DWIDTH / sin(dt);
			if ((refr > 0.0) && (ld < refr)) {  // obs in right side of box
				tod = ld * cos(dt);
				if ((tod < od) || (od == 0.0)) {
					od = tod;
				}
			}
			continue;
		}
					
	}
	
	
	//ROS_INFO("**** pt F od : %.2f. ******",od);
	if (od > 0.0) {
		od = od - LASEROFFSET; // + BASEOFFSET;
		nx.setParam("ProfileMoveObsDist",od);					
	} else {
		nx.setParam("ProfileMoveObsDist",5.0);
	}
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_filter");
  ros::NodeHandle n;
	
	last_laser_time = ros::Time::now();
	laser_sub= n.subscribe<sensor_msgs::LaserScan>("/scanF",100, laserCallBack);	 // /scan_base - org
  ros::spin();

  return 0;
}



