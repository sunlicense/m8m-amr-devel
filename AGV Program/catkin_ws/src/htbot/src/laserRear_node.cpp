/*
 * This node is subscribe to rear laser and calculate the obstacle distance 
 */
/* 	History
*		Date Created : 30.8.2021
                    
*		Changes :
*   30.8.21 : 
			1. 1pm : create new rear laser obstacle distance calculation
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

#define TROLLEYLEN 0.65 // 1.3 / 2
#define TROLLEYWD	0.4
#define RLDISTTOCENTER 0.39 // distance of rear laser from robot center
#define RLFRONT 0.26 //(TROLLEYLEN - RLDISTTOCENTER)

#define MAX_RANGE 4.0
#define MIN_RANGE 0.02  //0.03
#define DBOX 2.0
#define DEGPIDX 0.36
#define RADPIDX 0.0062832
#define PI 3.14159
#define DM 10 //28.07 // dm = atan(TROLLEYWD / (DBOX + RLFRONT)) * (180.0 / PI);  // degree
#define DMIDX 28 //112  // (int)(dm / DEGPIDX)
#define DG  57 //70 // dg = atan(TROLLEYWD / RLFRONT) * (180.0 / PI);  // degree
#define DGIDX 158 //280 // (int)(dg / DEGPIDX)
#define PIHALF 1.5708

using namespace std;

ros::Subscriber laserR_sub;

int laserR_size;
int cidx,lidx,ridx,lmidx,rmidx;
double refr,lrange,dt,od,tod;

double laser_data[1000];
bool flag;

// prototype
void caltrolleyidxR(void) ;
void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);

void caltrolleyidxR(void) {
	double an;

	rmidx = cidx - DMIDX;
	lmidx = cidx + DMIDX;
	ridx = cidx - DGIDX;
	lidx = cidx + DGIDX;
	//ROS_INFO("----- Rear Laser Trolley Leg Idx : laserSize=%d. cidx=%d. -------",laserR_size,cidx);

}

void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld,rd,an;	
	double t1,t2,t3,t4,t5,t6,t7;
	//bool clear ;
	
	laserR_size = lscan->ranges.size();
	cidx = (int)(laserR_size / 2.0);
	od = 0.0; // obs dist in front of robot
	
	//ROS_INFO("******  Laser Size : %d.   *******",laser_size);	
	if (!flag) {
		caltrolleyidxR();
		flag = true;
	}
	
	//ROS_INFO("ainc : %.6f. stime : %.6f. t_inc : %.6f",lscan->angle_increment,lscan->scan_time,lscan->time_increment);
	//ROS_INFO("**** pt A od : %.2f. ******",od);

	for (int i=0;i<laserR_size;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			ld = lscan->ranges[i];
			if ((ld >= MAX_RANGE) || (ld <= MIN_RANGE)) {
				continue;
			}
			refr = 0.0;		
			tod = 0.0;
			if ((i >= ridx) && (i <= rmidx)) {
				dt = (cidx-i) * RADPIDX;  // rad
				refr = TROLLEYWD / sin(dt);
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
				refr = TROLLEYWD / sin(dt);
				if ((refr > 0.0) && (ld < refr)) {  // obs in right side of box
					tod = ld * cos(dt);
					if ((tod < od) || (od == 0.0)) {
						od = tod;
					}
				}
				continue;
			}
		} 					
	}
		
	//ROS_INFO("**** pt F od : %.2f. ******",od);
	if (od > 0.0) {
		nx.setParam("RearObsDist",od);					
	} else {
		nx.setParam("RearObsDist",10.0);
	}
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserRear_filter");
  ros::NodeHandle n;
	
	flag = false;
	laserR_sub= n.subscribe<sensor_msgs::LaserScan>("/scanRR",100, laserRCallBack);	 // /scan_base - org

  ros::spin();

  return 0;
}



