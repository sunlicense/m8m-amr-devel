/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Created : 30.8.2021
                    
*		Changes :
*   30.8.21 : 
		1. 1pm " upgraded front laser filtered trolley legs and calculating frontobsdist.
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
#define FLDISTTOCENTER 0.384 // distance of front laser from robot center
#define FLFRONT (TROLLEYLEN - FLDISTTOCENTER)

#define MAX_RANGE 25.0
#define MIN_RANGE 0.05  //0.03
#define DWIDTH 0.15 //0.4  //0.35// 0.4m
#define DBOX 2.0
//#define DFRONT 0.02  //0.1
#define DEGPIDX 0.33
#define RADPIDX 0.0057596
#define PI 3.14159
#define DM 10 //28.07 // dm = atan(TROLLEYWD / (DBOX + FLFRONT)) * (180.0 / PI);  // degree
#define DMIDX 30 //112  // (int)(dm / DEGPIDX)
#define DG  56 //70 // dg = atan(TROLLEYWD / FLFRONT) * (180.0 / PI);  // degree
#define DGIDX 170 //280 // (int)(dg / DEGPIDX)
#define PIHALF 1.5708

using namespace std;

ros::Subscriber laserF_sub;
ros::Publisher scanF_pub;

int laserF_size;
int cidx,lidx,ridx,lmidx,rmidx;
double refr,lrange,dt,od,tod;
sensor_msgs::LaserScan scan_msg;

double laser_data[1000];
int frtidx,rrtidx,fltidx,rltidx;
bool fflag;

// prototype
void caltrolleyidxF(void) ;
void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);

void caltrolleyidxF(void) {
	double an;

	an = atan(TROLLEYWD / FLFRONT);
	frtidx = cidx - (int)(an / RADPIDX);
	fltidx = cidx + (int)(an / RADPIDX);
	rmidx = cidx - DMIDX;
	lmidx = cidx + DMIDX;
	ridx = cidx - DGIDX;
	lidx = cidx + DGIDX;
	ROS_INFO("----- Front Laser Trolley Leg Idx : laserSize=%d. rtidx=%d. cidx=%d. ltidx=%d. -------",laserF_size,frtidx,cidx,fltidx);

}

void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld,rd,an;	
	double t1,t2,t3,t4,t5,t6,t7;
	//bool clear ;
	

	laserF_size = lscan->ranges.size();
	cidx = (int)(laserF_size / 2.0);
	//rmidx = cidx - DMIDX;
	//lmidx = cidx + DMIDX;
	//ridx = cidx - DGIDX;
	//lidx = cidx + DGIDX;
	od = 0.0; // obs dist in front of robot
	
	//ROS_INFO("******  Laser Size : %d.   *******",laser_size);	
	if (!fflag) {
		caltrolleyidxF();
		fflag = true;
	}
	
	scan_msg.ranges.resize(laserF_size);
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

	// filter start
	for (int i=0;i<laserF_size;i++) {
		//scan_msg.ranges[i] = MAX_RANGE ;
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			ld = lscan->ranges[i];
			if (ld > MAX_RANGE) {
				laser_data[i] = MAX_RANGE  ;
				scan_msg.ranges[i] = MAX_RANGE;
			} else {
				if (ld < MIN_RANGE) {  //MIN_RANGE					
					laser_data[i] = MAX_RANGE ;
					scan_msg.ranges[i] = MAX_RANGE;
				} else {
					laser_data[i] = ld;
					scan_msg.ranges[i] = ld;
					if ((i<frtidx)) {
						//an = i * RADPIDX;
						an = ((cidx-i) * RADPIDX) ;
						rd = TROLLEYWD / sin(an);
						if (ld < rd) {
							scan_msg.ranges[i] = MAX_RANGE;
						}
					} else {
						if ((i<cidx)) {
							//an = PIHALF - (i * RADPIDX);
							an = ((cidx-i) * RADPIDX);
							rd = FLFRONT / cos(an);
							if (ld < rd) {
								scan_msg.ranges[i] = MAX_RANGE;
							}
						} else {
							if ((i<fltidx)) {
								//an = (i * RADPIDX) - PIHALF;
								an = ((i-cidx) * RADPIDX);
								rd = FLFRONT / cos(an);
								if (ld < rd) {
									scan_msg.ranges[i] = MAX_RANGE;
								}
							} else {
								//an = PI - (i * RADPIDX);
								an = ((i-cidx) * RADPIDX);
								rd = TROLLEYWD / sin(an);
								if (ld < rd) {
									scan_msg.ranges[i] = MAX_RANGE;
								}
							}
						}
					}
				}				
			}
		} else {
			laser_data[i] = MAX_RANGE;
			scan_msg.ranges[i] = MAX_RANGE;
		}
	}
	
	// filter end

	for (int i=0;i<laserF_size;i++) {
		ld = laser_data[i];
		refr = 0.0;		
		tod = 0.0;
		
		// check for obstacles	
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
			refr = (DBOX + FLFRONT) / cos(dt);
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
			refr = (DBOX + FLFRONT) / cos(dt);
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
		
	//ROS_INFO("**** pt F od : %.2f. ******",od);
	if (od > 0.0) {
		//od = od - LASEROFFSET; // + BASEOFFSET;
		nx.setParam("FrontObsDist",od);					
	} else {
		nx.setParam("FrontObsDist",10.0);
	}
	scanF_pub.publish(scan_msg);
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserFront_filter");
  ros::NodeHandle n;
	
	fflag = false;
	laserF_sub= n.subscribe<sensor_msgs::LaserScan>("/scanFF",100, laserFCallBack);	 // /scan_base - org
	scanF_pub = n.advertise<sensor_msgs::LaserScan>("/scanFB", 100);

  ros::spin();

  return 0;
}



