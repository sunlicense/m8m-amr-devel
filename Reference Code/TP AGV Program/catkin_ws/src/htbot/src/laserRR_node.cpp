/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Modified : 3.12.2014
                    7.5.21 : 10.30am
										10.5.21 : 5.35pm
*		Changes :
*   19.5.21 : 1pm 
		add rear laser removing trolley legs
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

#define MAX_RANGE 4.0
#define MIN_RANGE 0.02  //0.03
#define DWIDTH 0.45 //0.15 //0.4  //0.35// 0.4m
#define DBOX 2.0
#define RLFRONT 0.26  //0.1
#define DEGPIDX 0.36
#define RADPIDX 0.0062832
#define PI 3.14159
#define DM 11.26 //13.9 //28.07 // dm = atan(DWIDTH / (DBOX + RLFRONT)) * (180.0 / PI);  // degree
#define DMIDX 31 //112  // (int)(dm / DEGPIDX)
#define DG  60 //70 // dg = atan(DWIDTH / RLFRONT) * (180.0 / PI);  // degree
#define DGIDX 167 //264 //280 // (int)(dg / DEGPIDX)
#define PIHALF 1.5708
#define LASEROFFSET 0.03

using namespace std;

ros::Subscriber laserR_sub;
ros::Publisher scanR_pub;

int laserR_size;
int clearObs,lookforObs,cidx,lidx,ridx,lmidx,rmidx;
double refr,lrange,dm,dg,dt,od, tod;
sensor_msgs::LaserScan scan_msg;
std_msgs::Float32 obs_msg;	

double laser_data[1000];
//int frtidx,rrtidx,fltidx,rltidx;
bool rflag;
int rlridx,rlrmidx,rllmidx,rllidx;

// prototype
void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void calidxRL(void);

void calidxRL(void) {
	double an,an1;

	an = atan(DWIDTH / RLFRONT);
	rlridx = cidx - (int)(an / RADPIDX);
	an1 = atan(DWIDTH / (DBOX+RLFRONT));
	rlrmidx = cidx - (int)(an1 / RADPIDX);	
	rllmidx = cidx + (int)(an1 / RADPIDX);
	rllidx = cidx + (int)(an / RADPIDX);
	ROS_INFO("---- Rear Laser Idx : Size=%d. ridx=%d. rmidx=%d. cidx=%d. lmidx=%d. lidx=%d ------",laserR_size,rlridx,rlrmidx,cidx,rllmidx,rllidx);

}

void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld,rd,an;	
	double t1,t2,t3,t4,t5,t6,t7;

	laserR_size = lscan->ranges.size();
	cidx = (int)(laserR_size / 2.0);
	rmidx = cidx - DMIDX;
	lmidx = cidx + DMIDX;
	ridx = cidx - DGIDX;
	lidx = cidx + DGIDX;
	od = 0.0; // obs dist in front of robot
	
	//ROS_INFO("******  Laser Size : %d.   *******",laser_size);	
	if (!rflag) {
		calidxRL();
		rflag = true;
	}
	/*
	scan_msg.ranges.resize(laserR_size);
	scan_msg.header.stamp = lscan->header.stamp;
	scan_msg.header.frame_id = lscan->header.frame_id;
	scan_msg.angle_min = lscan->angle_min;
	scan_msg.angle_max = lscan->angle_max;
	scan_msg.angle_increment = lscan->angle_increment;
	scan_msg.scan_time = lscan->scan_time;
	scan_msg.time_increment = lscan->time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;
	
	//ROS_INFO("ainc : %.6f. stime : %.6f. t_inc : %.6f",lscan->angle_increment,lscan->scan_time,lscan->time_increment);
	//ROS_INFO("**** pt A od : %.2f. ******",od);
	*/
	// filter start
	for (int i=0;i<laserR_size;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			ld = lscan->ranges[i];
			if (ld > MAX_RANGE) {
				laser_data[i] = MAX_RANGE  ;
			} else {
				if (ld < MIN_RANGE) {  //MIN_RANGE					
					laser_data[i] = MAX_RANGE ;
				} else {
					laser_data[i] = ld;					
				}
			}				
		} else {
			laser_data[i] = MAX_RANGE;
			scan_msg.ranges[i] = MAX_RANGE;
		}
	}
	
	// filter end

	for (int i=0;i<laserR_size;i++) {
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
			refr = (DBOX + RLFRONT) / cos(dt);
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
			refr = (DBOX + RLFRONT) / cos(dt);
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
		nx.setParam("RLObsDist",od);					
	} else {
		nx.setParam("RLObsDist",5.0);
	}
	scanR_pub.publish(scan_msg);
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "Rear_laser_Node");
  ros::NodeHandle n;
	
	rflag = false;
	
	laserR_sub= n.subscribe<sensor_msgs::LaserScan>("/scanrear",100, laserRCallBack);	 // /scan_base - org
	//scanR_pub = n.advertise<sensor_msgs::LaserScan>("/scanRB", 100);
  ros::spin();

  return 0;
}



