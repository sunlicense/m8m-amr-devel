/*
 * This node listen to 2 laser scans and generate a combined laserscan 
 */
/* 	History
*		Date Modified : 5.11.2020
*		Changes : Front Laser on the left side and Rear laser on the right side
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
#include "boost/thread/mutex.hpp"


#define DWIDTH 0.3 //0.3  //0.35// 0.4m
#define BWIDTH 0.25 //0.25  //0.35// 0.4m
#define BHEIGHT 0.25 //0.25  //0.35// 0.4m
#define DBOX 2.0
#define DFRONT 0.05  //0.1
#define MAX_RANGEF 25.0
#define MIN_RANGEF 0.05  
#define MAX_RANGER 25.0
#define MIN_RANGER 0.05  
#define MAX_RANGE 25.0
#define MIN_RANGE 0.35 //0.05  
#define MAXSIZE 1100
#define NUMREADING 1090
#define CCIDX 545
#define DM 7.43  // dm = atan(DWIDTH / (DBOX+DFRONT_BHEIGHT)) * (180.0 / PI);  // degree
#define DMIDX 23 // (int)(dm / DEGPIDX)
#define DG  45 // dg = atan(DWIDTH / (DFRONT+BHEIGHT) * (180.0 / PI);  // degree
#define DGIDX 136 // (int)(dg / DEGPIDX)
#define SCANFREQ 15
#define DIST_TO_BASELINK1 0.3  
#define DIST_TO_BASELINK2 0.3

#define DEGPIDX 0.33
#define RADPIDX 0.00576
#define PI2 6.2832
#define PI 3.14159
#define PIHALF 1.5708
#define LASERTOCENTER_W 0.1
#define LASERTOCENTER_H 0.145

using namespace std;

ros::Subscriber laserF_sub; // front
ros::Subscriber laserR_sub; // rear
ros::Publisher multiscan_pub;
boost::mutex mut;

int laserF_size,laserR_size;
int cidx,lidx,ridx,lmidx,rmidx;
double refr,dt,od,tod;

//sensor_msgs::LaserScan scan_msg;
double laser_data[MAXSIZE];
bool flagF,flagR;
double scan_time,time_increment;
bool bothLaserInit;
string sickCFrame;

// prototype
void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void initialise_laserdata();
void publish_multiscan();

void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	double ld,min,an,angle_increment,rn;
	double x,y;
	double rx,ry,rr,ran;
	int cidx,rix;
	
	if (!flagF) {
		return;
	}
	flagF = false;
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		laserF_size = lscan->ranges.size();
		cidx = (int)(laserF_size / 2.0);
		angle_increment = lscan->angle_increment;
		//time_increment = lscan->time_increment;
		//ROS_INFO("------- LaserF : size=%d. cidx=%d. a_incr=%.5f. ------------",laserF_size,cidx,angle_increment);
		for (int i=0;i<laserF_size;i++) {
			if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
				if (lscan->ranges[i] > MAX_RANGEF) {
					continue;
				} else {
					if (lscan->ranges[i] < MIN_RANGEF) {  		
						continue;
					} else {
						ld = lscan->ranges[i];
						an = i * angle_increment;
						if (an < PIHALF) {
							x = ld * sin(an);
							y = ld * cos(an);
							if (y > LASERTOCENTER_W) {
								rx = x + LASERTOCENTER_H;
								ry = y - LASERTOCENTER_W;
								rr = sqrt((rx * rx) + (ry * ry));
								ran = (atan(rx/ry) + PIHALF);
								rix = (int)(ran / angle_increment);
							} else {
								if (y == LASERTOCENTER_W) {
									rx = x + LASERTOCENTER_H;
									ry = 0.0;
									rr = rx;
									ran = PI;
									rix = (int)(ran / angle_increment);
								} else {
									rx = x + LASERTOCENTER_H;
									ry = LASERTOCENTER_W - y;
									rr = sqrt((rx * rx) + (ry * ry));
									ran = atan(ry/rx) + PI;
									rix = (int)(ran / angle_increment);
								}
							}
							laser_data[rix] = rr;
						} else {
							if (an == PIHALF) {
								rx = ld + LASERTOCENTER_H;
								ry = LASERTOCENTER_W;
								rr = sqrt((rx * rx) + (ry * ry));
								ran = atan(ry/rx) + PI;
								rix = (int)(ran / angle_increment);
							} else {
								if ((an > PIHALF) && (an < PI)) {
									x = ld * cos(an - PIHALF);
									y = ld * sin(an - PIHALF); 									
									rx = x + LASERTOCENTER_H;
									ry = y + LASERTOCENTER_W;
									rr = sqrt((rx * rx) + (ry * ry));
									ran = atan(ry/rx) + PI;
									rix = (int)(ran / angle_increment);
								} else {
									if (an == PI) {
										rx = LASERTOCENTER_H;
										ry = ld + LASERTOCENTER_W;
										rr = sqrt((rx * rx) + (ry * ry));
										ran = atan(ry/rx) + PI;
										rix = (int)(ran / angle_increment);
									} else {
										x = ld * sin(an - PI);
										y = ld * cos(an - PI); 
										if (x < LASERTOCENTER_H) {
											rx = LASERTOCENTER_H - x;
											ry = y + LASERTOCENTER_W;
											rr = sqrt((rx * rx) + (ry * ry));
											ran = (PI + PIHALF) - atan(rx/ry);
											rix = (int)(ran / angle_increment);
										} else {
											if (x == LASERTOCENTER_H) {
												rx = 0.0;
												ry = y + LASERTOCENTER_W;
												rr = ry;
												ran = PI + PIHALF;
												rix = (int)(ran / angle_increment);
											} else {
												rx = x - LASERTOCENTER_H;
												ry = y + LASERTOCENTER_W;
												rr = sqrt((rx * rx) + (ry * ry));
												ran = atan(rx/ry) + PI + PIHALF;
												rix = (int)(ran / angle_increment);
											}
										}
									}
								}
							}
							laser_data[rix] = rr;
						}
					}				
				}
			} else {
				continue;
			}
		}
	}
}

void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	double ld,min,an,angle_increment,rn;
	double x,y;
	double rx,ry,rr,ran;
	int cidx,rix;
	
	if (!flagR) {
		return;
	}
	flagR = false;
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		laserR_size = lscan->ranges.size();
		cidx = (int)(laserR_size / 2.0);
		angle_increment = lscan->angle_increment;
		//time_increment = lscan->time_increment;
		//ROS_INFO("------- LaserR : size=%d. cidx=%d. a_incr=%.5f.  ------------",laserR_size,cidx,angle_increment);
		for (int i=0;i<laserR_size;i++) {
			if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
				if (lscan->ranges[i] > MAX_RANGER) {
					continue;
				} else {
					if (lscan->ranges[i] < MIN_RANGER) {  		
						continue;
					} else {
						ld = lscan->ranges[i];
						an = i * angle_increment;
						if (an < PIHALF) {
							x = ld * sin(an);
							y = ld * cos(an);
							if (y < LASERTOCENTER_W) {
								rx = x + LASERTOCENTER_H;
								ry = LASERTOCENTER_W - y;
								rr = sqrt((rx * rx) + (ry * ry));
								ran = atan(ry/rx);
								rix = (int)(ran / angle_increment);
							} else {
								if (y == LASERTOCENTER_W) {
									rx = x + LASERTOCENTER_H;
									ry = 0.0;
									rr = rx;
									ran = angle_increment;
									rix = (int)(ran / angle_increment);
								} else {
									rx = x + LASERTOCENTER_H;
									ry = y - LASERTOCENTER_W;
									rr = sqrt((rx * rx) + (ry * ry));
									ran = PI2 - atan(ry/rx);
									rix = (int)(ran / angle_increment);
								}
							}
							laser_data[rix] = rr;
						} else {
							if (an == PIHALF) {
								rx = ld + LASERTOCENTER_H;
								ry = LASERTOCENTER_W;
								rr = sqrt((rx * rx) + (ry * ry));
								ran = atan(ry/rx) ;
								rix = (int)(ran / angle_increment);
							} else {
								if ((an > PIHALF) && (an < PI)) {
									x = ld * cos(an - PIHALF);
									y = ld * sin(an - PIHALF); 									
									rx = x + LASERTOCENTER_H;
									ry = y + LASERTOCENTER_W;
									rr = sqrt((rx * rx) + (ry * ry));
									ran = atan(ry/rx);
									rix = (int)(ran / angle_increment);
								} else {
									if (an == PI) {
										rx = LASERTOCENTER_H;
										ry = ld + LASERTOCENTER_W;
										rr = sqrt((rx * rx) + (ry * ry));
										ran = atan(ry/rx);
										rix = (int)(ran / angle_increment);
									} else {
										x = ld * sin(an - PI);
										y = ld * cos(an - PI); 
										if (x < LASERTOCENTER_H) {
											rx = LASERTOCENTER_H - x;
											ry = y + LASERTOCENTER_W;
											rr = sqrt((rx * rx) + (ry * ry));
											ran = PIHALF - atan(rx/ry);
											rix = (int)(ran / angle_increment);
										} else {
											if (x == LASERTOCENTER_H) {
												rx = 0.0;
												ry = y + LASERTOCENTER_W;
												rr = ry;
												ran = PIHALF;
												rix = (int)(ran / angle_increment);
											} else {
												rx = x - LASERTOCENTER_H;
												ry = y + LASERTOCENTER_W;
												rr = sqrt((rx * rx) + (ry * ry));
												ran = atan(rx/ry) + PIHALF;
												rix = (int)(ran / angle_increment);
											}
										}
									}
								}
							}
							laser_data[rix] = rr;
						}
					}				
				}
			} else {
				continue;
			}
		}
	}
}

void initialise_laserdata() {
	for (int i=0;i<MAXSIZE;i++) {
		laser_data[i] = MAX_RANGE-0.1;
	}
}

void publish_multiscan() {
	ros::NodeHandle nx;	
	double ld;
	sensor_msgs::LaserScan scan_msg;
	scan_msg.ranges.resize(NUMREADING);
	//scan_msg.intensities.resize(NUMREADING);
	scan_msg.header.stamp = ros::Time::now();
	//scan_msg.header.frame_id = "/sickC";
	scan_msg.header.frame_id = sickCFrame;
	scan_msg.angle_min = -3.14159;
	scan_msg.angle_max = 3.14159;
	scan_msg.angle_increment = RADPIDX;
	//scan_msg.scan_time = scan_time;
	scan_msg.time_increment = time_increment;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;	 	
	od = 0.0;
	for (int i=0;i<NUMREADING;i++) {
		if (laser_data[i] < MAX_RANGE) {
			scan_msg.ranges[i] = laser_data[i];
		} else {
			scan_msg.ranges[i] = MAX_RANGE - 0.1;
		}
		
		// check for obstacle in front of robot		
		ld = laser_data[i];
		if (ld > (BHEIGHT+DFRONT)) {
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
				refr = (DBOX + DFRONT + BHEIGHT) / cos(dt);
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
				refr = (DBOX + DFRONT + BHEIGHT) / cos(dt);
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
				
	}
	if (od > 0.0) {		
		nx.setParam("ProfileMoveObsDist",od);					
	} else {
		nx.setParam("ProfileMoveObsDist",5.0);
	}
	multiscan_pub.publish(scan_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_filter");
  ros::NodeHandle n;	
	double rate;	
	
	n.getParam("MultiLaser_Rate",rate);
	n.getParam("sickCFrame",sickCFrame);
	ros::Rate loop_rate(rate);
	flagF = flagR = false;
	bothLaserInit = false;
	ROS_INFO("------------- multilaser Node : Rate = %.3f--------",rate);
	laserF_sub= n.subscribe<sensor_msgs::LaserScan>("/scanF",100, laserFCallBack);	 
	laserR_sub= n.subscribe<sensor_msgs::LaserScan>("/scanR",100, laserRCallBack);	 
	multiscan_pub = n.advertise<sensor_msgs::LaserScan>("/scanC", 100);

	cidx = CCIDX;
	rmidx = cidx - DMIDX;
	lmidx = cidx + DMIDX;
	ridx = cidx - DGIDX;
	lidx = cidx + DGIDX;

	//initialise_laserdata();
	time_increment = (1.0 / SCANFREQ) / NUMREADING;
  while (true) {  
		//initialise_laserdata();
		
		//flagF = false;	// testing
		ros::spinOnce();	
		//ROS_INFO("------------- multilaser Node : Main Loop--------");
		if (!flagF && !flagR) {
			publish_multiscan();
			initialise_laserdata();
			flagF = flagR = true;	
		}
  	loop_rate.sleep();
	}

  return 0;
}



