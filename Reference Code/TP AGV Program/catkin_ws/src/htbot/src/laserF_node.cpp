/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Modified : 3.12.2014
                    7.5.21 : 10.30am
										10.5.21 : 5.35pm
*		Changes :
*   19.5.21 : 
			1. 1pm : add rear laser removing trolley legs
		10.8.21
			1. 11am : updated front laser. removed other lasers
	
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
#define TROLLEYLEN 1.1
#define TROLLEYWD 0.7
#define TROLLEYDIST 0.8
#define DWIDTH 0.45 //0.15 //0.4  //0.35// 0.4m
#define DBOX 2.0
#define DFRONT 0.02  //0.1
#define DEGPIDX 0.33
#define RADPIDX 0.0057596
#define PI 3.14159
#define DM 11.2 //13.9 //28.07 // dm = atan(DWIDTH / (DBOX + FLFRONT)) * (180.0 / PI);  // degree
#define DMIDX 34 //112  // (int)(dm / DEGPIDX)
#define DG  59 //70 // dg = atan(DWIDTH / FLFRONT) * (180.0 / PI);  // degree
#define DGIDX 179 //264 //280 // (int)(dg / DEGPIDX)
#define LASEROFFSET 0.03
#define BASEOFFSET 0.31
#define DISTGT350MM  0.6
#define PIHALF 1.5708

#define FILTER_SCAN_SIZE 7
#define FILTER_SPIKE_SIZE 3
#define STEP 0.08  // m
#define FILTER_LEVEL_SIZE 0.04 // m

#define FTROBOXW 0.45
#define FTROBOXL 0.45
#define FLFRONT 0.27  // 1.3/2 - 0.384

#define RDEGPIDX 0.36   // rear laser deg per idx
#define RRADPIDX 0.0062832
#define MAX_RANGER 4.0
#define MIN_RANGER 0.02  
#define RTROBOXW 0.5
#define RTROBOXL 0.3 // 

using namespace std;

ros::Subscriber laserF_sub;
ros::Publisher scanF_pub;
ros::Subscriber laserR_sub;
ros::Publisher scanR_pub;
//ros::Publisher obs_pub;

int laserF_size,laserR_size;
int clearObs,lookforObs,cidx,lidx,ridx,lmidx,rmidx;
double refr,lrange,dm,dg,dt,od, tod;
sensor_msgs::LaserScan scan_msg;
std_msgs::Float32 obs_msg;	

double laser_data[1000];
//int frtidx,rrtidx,fltidx,rltidx;
bool fflag,lflag;
int flridx,flrmidx,fllmidx,fllidx;

// prototype
//void caltrolleyidxF(void) ;
//void caltrolleyidxR(void) ;
void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void calidxFL(void);


void calidxFL(void) {
	double an,an1;

	an = atan(FTROBOXW / FLFRONT);
	flridx = cidx - (int)(an / RADPIDX);
	an1 = atan(FTROBOXW / (DBOX+FLFRONT));
	flrmidx = cidx - (int)(an1 / RADPIDX);	
	fllmidx = cidx + (int)(an1 / RADPIDX);
	fllidx = cidx + (int)(an / RADPIDX);
	ROS_INFO("---- Front Laser Idx : Size=%d. ridx=%d. rmidx=%d. cidx=%d. lmidx=%d. lidx=%d ------",laserF_size,flridx,flrmidx,cidx,fllmidx,fllidx);

}

void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld,rd,an;	
	double t1,t2,t3,t4,t5,t6,t7;
	
	laserF_size = lscan->ranges.size();
	cidx = (int)(laserF_size / 2.0);
	rmidx = cidx - DMIDX;
	lmidx = cidx + DMIDX;
	ridx = cidx - DGIDX;
	lidx = cidx + DGIDX;
	od = 0.0; // obs dist in front of robot
	
	//ROS_INFO("******  Laser Size : %d.   *******",laser_size);	
	if (!fflag) {
		calidxFL();
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
					if ((i<flridx)) {
						//an = i * RADPIDX;
						an = PIHALF - ((cidx-i) * RADPIDX) ;
						rd = FTROBOXW / sin(an);
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
							if ((i<fllidx)) {
								//an = (i * RADPIDX) - PIHALF;
								an = ((i-cidx) * RADPIDX);
								rd = FLFRONT / cos(an);
								if (ld < rd) {
									scan_msg.ranges[i] = MAX_RANGE;
								}
							} else {
								//an = PI - (i * RADPIDX);
								an = PIHALF - ((i-cidx) * RADPIDX);
								rd = FTROBOXW / cos(an);
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
		nx.setParam("FLObsDist",od);					
	} else {
		nx.setParam("FLObsDist",26.0);
	}
	scanF_pub.publish(scan_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "front_laser_node");
  ros::NodeHandle n;
	
	fflag = false;
	lflag = false;
	laserF_sub= n.subscribe<sensor_msgs::LaserScan>("/scanF",100, laserFCallBack);	 // /scan_base - org
	scanF_pub = n.advertise<sensor_msgs::LaserScan>("/scanFB", 100);

  ros::spin();

  return 0;
}



