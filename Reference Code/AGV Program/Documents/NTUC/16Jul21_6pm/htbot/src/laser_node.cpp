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

#define MAX_RANGE 25.0
#define MIN_RANGE 0.05  //0.03
#define TROLLEYLEN 1.1
#define TROLLEYWD 0.7
#define TROLLEYDIST 0.8
#define DWIDTH 0.15 //0.4  //0.35// 0.4m
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
#define PIHALF 1.5708

#define FILTER_SCAN_SIZE 7
#define FILTER_SPIKE_SIZE 3
#define STEP 0.08  // m
#define FILTER_LEVEL_SIZE 0.04 // m

#define FTROBOXW 0.6
#define FTROBOXL 0.35

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
int frtidx,rrtidx,fltidx,rltidx;
bool fflag,lflag;

// prototype
void caltrolleyidxF(void) ;
void caltrolleyidxR(void) ;
void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);

void caltrolleyidxF(void) {
	double an,an1;
	int ixx;

	an = atan(FTROBOXL / FTROBOXW);
	frtidx = (int)(an / RADPIDX);
	an = atan(FTROBOXW / FTROBOXL);
	ixx = (int)(an / RADPIDX);	
	//fltidx = laserF_size - frtidx;
	fltidx = cidx + ixx;
	ROS_INFO("----- Front Laser Trolley Leg Idx : laserSize=%d. rtidx=%d. cidx=%d. ltidx=%d. -------",laserF_size,frtidx,cidx,fltidx);

}

void caltrolleyidxR(void) {
	double an,an1;
	int ixx;

	an = atan(RTROBOXW / RTROBOXL);
	ixx = (int)(an / RRADPIDX);
	rrtidx = cidx - ixx;
	an = atan(RTROBOXW / RTROBOXL);
	ixx = (int)(an / RRADPIDX);	
	//rltidx = laserR_size - rrtidx;
	rltidx = cidx + ixx;
	ROS_INFO("----- Rear Laser Trolley Leg Idx : laserSize=%d. rtidx=%d. cidx=%d. ltidx=%d. -------",laserR_size,rrtidx,cidx,rltidx);

}

void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld,rd,an;	
	double t1,t2,t3,t4,t5,t6,t7;
	//bool clear ;
	

	//clear = false;
	//nx.getParam("clearObs",clearObs);
	//nx.getParam("lookforObs",lookforObs);
	//if (lookforObs == 1) {
	//	ROS_INFO("Look for Obs");
	//}
	laserF_size = lscan->ranges.size();
	cidx = (int)(laserF_size / 2.0);
	rmidx = cidx - DMIDX;
	lmidx = cidx + DMIDX;
	ridx = cidx - DGIDX;
	lidx = cidx + DGIDX;
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
						an = i * RADPIDX;
						an = PIHALF - ((cidx-i) * RADPIDX) ;
						rd = FTROBOXW / cos(an);
						if (ld < rd) {
							scan_msg.ranges[i] = MAX_RANGE;
						}
					} else {
						if ((i<cidx)) {
							//an = PIHALF - (i * RADPIDX);
							an = ((cidx-i) * RADPIDX);
							rd = FTROBOXL / cos(an);
							if (ld < rd) {
								scan_msg.ranges[i] = MAX_RANGE;
							}
						} else {
							if ((i<fltidx)) {
								//an = (i * RADPIDX) - PIHALF;
								an = ((i-cidx) * RADPIDX);
								rd = FTROBOXL / cos(an);
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
	scanF_pub.publish(scan_msg);
}

void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld,rd,an;
	double t1,t2,t3,t4,t5,t6,t7;

	laserR_size = lscan->ranges.size();
	cidx = (int)(laserR_size / 2.0);
	
	//ROS_INFO("******  Laser Size : %d.   *******",laserR_size);	
	if (!lflag) {
		caltrolleyidxR();
		lflag = true;
	}
	
	scan_msg.ranges.resize(laserR_size);
	scan_msg.header.stamp = lscan->header.stamp;
	scan_msg.header.frame_id = lscan->header.frame_id;
	scan_msg.angle_min = lscan->angle_min;
	scan_msg.angle_max = lscan->angle_max;
	scan_msg.angle_increment = lscan->angle_increment;
	scan_msg.scan_time = lscan->scan_time;
	scan_msg.time_increment = lscan->time_increment;
	scan_msg.range_min = MIN_RANGER;
	scan_msg.range_max = MAX_RANGER;
	
	//ROS_INFO("ainc : %.6f. stime : %.6f. t_inc : %.6f",lscan->angle_increment,lscan->scan_time,lscan->time_increment);
	//ROS_INFO("**** pt A od : %.2f. ******",od);

	// filter start
	for (int i=0;i<laserR_size;i++) {
		//scan_msg.ranges[i] = MAX_RANGE ;
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			ld = lscan->ranges[i];
			if (ld > MAX_RANGER) {
				laser_data[i] = MAX_RANGER  ;
				scan_msg.ranges[i] = MAX_RANGER;
			} else {
				if (ld < MIN_RANGER) {  //MIN_RANGE					
					laser_data[i] = MAX_RANGER ;
					scan_msg.ranges[i] = MAX_RANGER;
				} else {
					laser_data[i] = ld;
					scan_msg.ranges[i] = ld;
					if ((i<rrtidx)) {
						an = i * RRADPIDX;
						an = PIHALF - ((cidx-i) * RRADPIDX) ;
						rd = RTROBOXW / cos(an);
						if (ld < rd) {
							scan_msg.ranges[i] = MAX_RANGER;
						}
					} else {
						if ((i<cidx)) {
							an = ((cidx-i) * RRADPIDX);
							rd = RTROBOXL / cos(an);
							if (ld < rd) {
								scan_msg.ranges[i] = MAX_RANGER;
							}
						} else {
							if ((i<rltidx)) {
								an = ((i-cidx) * RRADPIDX);
								rd = RTROBOXL / cos(an);
								if (ld < rd) {
									scan_msg.ranges[i] = MAX_RANGER;
								}
							} else {
								an = PIHALF - ((i-cidx) * RRADPIDX);
								rd = RTROBOXW / cos(an);
								if (ld < rd) {
									scan_msg.ranges[i] = MAX_RANGER;
								}
							}
						}
					}
				}				
			}
		} else {
			laser_data[i] = MAX_RANGER;
			scan_msg.ranges[i] = MAX_RANGER;
		}
	}
	
	// filter end
	scanR_pub.publish(scan_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_filter");
  ros::NodeHandle n;
	
	fflag = false;
	lflag = false;
	laserF_sub= n.subscribe<sensor_msgs::LaserScan>("/scanFF",100, laserFCallBack);	 // /scan_base - org
	scanF_pub = n.advertise<sensor_msgs::LaserScan>("/scanFB", 100);
	laserR_sub= n.subscribe<sensor_msgs::LaserScan>("/scanRR",100, laserRCallBack);	 // /scan_base - org
	scanR_pub = n.advertise<sensor_msgs::LaserScan>("/scanRB", 100);
  ros::spin();

  return 0;
}



