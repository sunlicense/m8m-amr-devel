/*
 * This node listen to 2 laser scans and generate a combined laserscan 
 */
/* 	History
* 	date created : 5.11.2020
*		Date Modified : 5.11.2020
*		Changes : For Globotix prototype A. Front Laser is on the right side and Rear laser is on the left side
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
#define BWIDTH 0.45 //0.25 //0.25  //0.35// 0.4m
#define BHEIGHT 0.65 //0.25  //0.35// 0.4m
#define DBOX 2.0
#define DFRONT 0.05  //0.1
#define MAX_RANGEF 25.0
#define MIN_RANGEF 0.05  
#define MAX_RANGER 25.0
#define MIN_RANGER 0.05  
#define MAX_RANGE 25.0
#define MAX_RANGEL 25.0 //24.9
#define MIN_RANGE 0.35 //0.05  
#define MAXSIZE 1100
#define NUMREADING 1090
//#define NUMREADING 2250
#define REFSCANNUMREADING 545
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
//#define LASERTOCENTER_W 0.37124
//#define LASERTOCENTER_H 0.43673
#define LASERF_W 0.32314
#define LASERF_H 0.447
#define LASERR_W 0.32314
#define LASERR_H 0.392

using namespace std;

ros::Subscriber laserF_sub; // front
ros::Subscriber laserR_sub; // rear
ros::Publisher multiscan_pub;
//ros::Publisher refscan_pub;
boost::mutex mut;

int laserF_size,laserR_size;
int cidx,lidx,ridx,lmidx,rmidx;
double refr,dt,od,tod;

//sensor_msgs::LaserScan scan_msg;
double Flaser_data[MAXSIZE];
double Rlaser_data[MAXSIZE];
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
							x = ld * cos(an);
							y = ld * sin(an);
							if (x > LASERF_H) {  // 1
								rx = x - LASERF_H;
								ry = y + LASERF_W;
								rr = sqrt((rx * rx) + (ry * ry));
								ran = (atan(ry/rx));
								rix = (int)(ran / angle_increment);
							} else {
								if (x == LASERF_H) { // 2
									rx = 0.0;
									ry = y + LASERF_W;
									rr = ry;
									ran = PIHALF;
									rix = (int)(ran / angle_increment);
								} else { // 3
									rx = LASERF_H - x;
									ry = LASERF_W + y;
									rr = sqrt((rx * rx) + (ry * ry));
									ran = atan(rx/ry) + PIHALF;
									rix = (int)(ran / angle_increment);
								}
							}
							if (rr < Flaser_data[rix]) {
								if ((rix < MAXSIZE) && (rr <MAX_RANGE) && (rr > MIN_RANGE)) {
								//if ((rix < MAXSIZE) && (rr < MAX_RANGE) && (rr > MIN_RANGE) && !( (rx <= BHEIGHT) && (rx <= BWIDTH) ) ) {
									Flaser_data[rix] = rr;
								}
							}
						} else {
							if (an == PIHALF) {
								rx = LASERF_H;
								ry = y + LASERF_W;
								rr = sqrt((rx * rx) + (ry * ry));
								ran = atan(rx/ry) + PIHALF;
								rix = (int)(ran / angle_increment);
							} else {
								if ((an > PIHALF) && (an < PI)) {
									x = ld * sin(an - PIHALF);
									y = ld * cos(an - PIHALF); 									
									rx = x + LASERF_H;
									ry = y + LASERF_W;
									rr = sqrt((rx * rx) + (ry * ry));
									ran = atan(rx/ry) + PIHALF;
									rix = (int)(ran / angle_increment);
								} else {
									if (an == PI) {
										rx = ld + LASERF_H;
										ry = LASERF_W;
										rr = sqrt((rx * rx) + (ry * ry));
										ran = atan(rx/ry) + PIHALF;
										rix = (int)(ran / angle_increment);
									} else {
										x = ld * cos(an - PI);
										y = ld * sin(an - PI); 
										if (y < LASERF_W) {  // 1
											rx = LASERF_H + x;
											ry = LASERF_W - y;
											rr = sqrt((rx * rx) + (ry * ry));
											ran = PIHALF + atan(rx/ry);
											rix = (int)(ran / angle_increment);
										} else {
											if (y == LASERF_W) { // 2
												rx = LASERF_H + ld;
												ry = 0.0;
												rr = rx;
												ran = PI;
												rix = (int)(ran / angle_increment);
											} else { // 3
												rx = x + LASERF_H;
												ry = y - LASERF_W;
												rr = sqrt((rx * rx) + (ry * ry));
												ran = atan(ry/rx) + PI;
												rix = (int)(ran / angle_increment);
											}
										}
									}
								}
							}
							if (rr < Flaser_data[rix]) {
								if ((rix < MAXSIZE) && (rr <MAX_RANGE) && (rr > MIN_RANGE)) {
								//if ((rix < MAXSIZE) && (rr < MAX_RANGE) && (rr > MIN_RANGE) && (rx >= BHEIGHT) && (rx >= BWIDTH)) {
								//if ((rix < MAXSIZE) && (rr < MAX_RANGE) && (rr > MIN_RANGE) && !( (rx <= BHEIGHT) && (rx <= BWIDTH) ) ) {
									Flaser_data[rix] = rr;
								}
							}
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
							x = ld * cos(an);
							y = ld * sin(an);
							if (x < LASERR_H) { // 1
								rx = LASERR_H - x;
								ry = LASERR_W + y;
								rr = sqrt((rx * rx) + (ry * ry));
								ran = atan(rx/ry) + PI + PIHALF;
								rix = (int)(ran / angle_increment);
							} else {
								if (x == LASERR_H) { // 2
									rx = 0.0;
									ry = y + LASERR_W;
									rr = ry;
									ran = PI + PIHALF;
									rix = (int)(ran / angle_increment);
								} else { // 3
									rx = x - LASERR_H;
									ry = y + LASERR_W;
									rr = sqrt((rx * rx) + (ry * ry));
									ran = (PI + PIHALF) - atan(rx/ry);
									rix = (int)(ran / angle_increment);
								}
							}
							if (rr < Rlaser_data[rix]) {
								if ((rix < MAXSIZE) && (rr < MAX_RANGE) && (rr > MIN_RANGE)) {
								//if ((rix < MAXSIZE) && (rr < MAX_RANGE) && (rr > MIN_RANGE) && (rx >= BHEIGHT) && (rx >= BWIDTH)) {
								//if ((rix < MAXSIZE) && (rr < MAX_RANGE) && (rr > MIN_RANGE) && !( (rx <= BHEIGHT) && (rx <= BWIDTH) ) ) {
									Rlaser_data[rix] = rr;
									//ROS_INFO("-------- scanR : id=%d. ld = %.2f ------------",rix,rr);
								}
							}
						} else {
							if (an == PIHALF) {
								rx = LASERR_H;
								ry = ld + LASERR_W;
								rr = sqrt((rx * rx) + (ry * ry));
								ran = atan(rx/ry) + PI + PIHALF;
								rix = (int)(ran / angle_increment);
							} else {
								if ((an > PIHALF) && (an < PI)) {
									x = ld * cos(PI - an);
									y = ld * sin(PI - an); 									
									rx = x + LASERR_H;
									ry = y + LASERR_W;
									rr = sqrt((rx * rx) + (ry * ry));
									ran = (PI2) - atan(ry/rx);
									rix = (int)(ran / angle_increment);
								} else {
									if (an == PI) {
										rx = ld + LASERR_H;
										ry = LASERR_W;
										rr = sqrt((rx * rx) + (ry * ry));
										ran = atan(ry/rx) + PI2;
										rix = (int)(ran / angle_increment);
									} else {
										x = ld * cos(an - PI);
										y = ld * sin(an - PI); 
										if (y < LASERR_W) { // 1
											rx = LASERR_H + x;
											ry = LASERR_W - y;
											rr = sqrt((rx * rx) + (ry * ry));
											ran = PI2 - atan(ry/rx);
											rix = (int)(ran / angle_increment);
										} else {
											if (y == LASERR_W) { //2
												rx = x + LASERR_H;
												ry = 0.0;
												rr = rx;
												ran = PI2;
												rix = (int)(ran / angle_increment);
											} else { // 3
												rx = x + LASERR_H;
												ry = y - LASERR_W;
												rr = sqrt((rx * rx) + (ry * ry));
												ran = atan(ry/rx);
												rix = (int)(ran / angle_increment);
											}
										}
									}
								}
							}
							if (rr < Rlaser_data[rix]) {
								if ((rix < MAXSIZE) && (rr < MAX_RANGE) && (rr > MIN_RANGE)) {
								//if ((rix < MAXSIZE) && (rr <MAX_RANGE) && (rr > MIN_RANGE) && (rx >= BHEIGHT) && (rx >= BWIDTH)) {
								//if ((rix < MAXSIZE) && (rr < MAX_RANGE) && (rr > MIN_RANGE) && !( (rx <= BHEIGHT) && (rx <= BWIDTH) ) ) {
									Rlaser_data[rix] = rr;
									//ROS_INFO("-------- scanR : id=%d. ld = %.2f ------------",rix,rr);
								}
							}
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
		Flaser_data[i] = MAX_RANGEL;
		Rlaser_data[i] = MAX_RANGEL;
	}
}

void publish_multiscan() {
	ros::NodeHandle nx;	
	double ld,lf,lr;
	sensor_msgs::LaserScan scan_msg;
	//sensor_msgs::LaserScan refscan_msg;
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

	/*
	refscan_msg.ranges.resize(REFSCANNUMREADING);
	refscan_msg.header.stamp = ros::Time::now();
	refscan_msg.header.frame_id = "/refscan";
	refscan_msg.angle_min = -1.5;
	refscan_msg.angle_max = 1.5;
	refscan_msg.angle_increment = RADPIDX;
	refscan_msg.time_increment = time_increment;
	refscan_msg.range_min = MIN_RANGE;
	refscan_msg.range_max = MAX_RANGE;	 

	for (int i=0;i<REFSCANNUMREADING;i++) {
		refscan_msg.ranges[i] = Flaser_data[i];;
	}	
	*/

	od = 0.0;
	for (int i=0;i<NUMREADING;i++) {
		//if (laser_data[i] < MAX_RANGE) {
		//	scan_msg.ranges[i] = laser_data[i];			
		//} else {
		//	scan_msg.ranges[i] = MAX_RANGE - 0.1;
		//}
		lf = Flaser_data[i];
		lr = Rlaser_data[i];
		if (lf < MAX_RANGEL) {
			scan_msg.ranges[i] = lf;
		} else {
			scan_msg.ranges[i] = lr;
		}
		//scan_msg.ranges[i] = laser_data[i];
		//ROS_INFO("------- scanC : ld=%.2f.  ------------",laser_data[i]);
		// check for obstacle in front of robot		
		ld = scan_msg.ranges[i];
		if (ld > (BHEIGHT+DFRONT)) {
			if ((i >= ridx) && (i <= rmidx)) {
				dt = (cidx-i) * RADPIDX;  // rad
				refr = BWIDTH / sin(dt);
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
				refr = BWIDTH / sin(dt);
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
	//refscan_pub.publish(refscan_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_filter");
  ros::NodeHandle n;	
	double rate;	
	
	n.getParam("MultiSick_Rate",rate);
	n.getParam("sickCFrame",sickCFrame);
	ros::Rate loop_rate(rate);
	initialise_laserdata();
	flagF = flagR = true;
	bothLaserInit = false;
	ROS_INFO("------------- multiSick Node : Rate = %.3f--------",rate);
	laserF_sub= n.subscribe<sensor_msgs::LaserScan>("/scanF",1000, laserFCallBack);	 
	laserR_sub= n.subscribe<sensor_msgs::LaserScan>("/scanR",1000, laserRCallBack);	 
	multiscan_pub = n.advertise<sensor_msgs::LaserScan>("/scanC", 1000);
	//refscan_pub = n.advertise<sensor_msgs::LaserScan>("/refscan", 1000);

	cidx = CCIDX;
	rmidx = cidx - DMIDX;
	lmidx = cidx + DMIDX;
	ridx = cidx - DGIDX;
	lidx = cidx + DGIDX;
	
	//sleep(1);
	//flagF = flagR = true;
	time_increment = (1.0 / SCANFREQ) / NUMREADING;
	ROS_INFO("------------- multiSick Node : Point A ------------");
  while (true) {  
		//initialise_laserdata();
		
		//flagF = false;	// testing
		ros::spinOnce();	
		//ROS_INFO("------------- multilaser Node : Main Loop--------");
		if (!flagF && !flagR) {
			//initialise_laserdata();
			publish_multiscan();
			initialise_laserdata();
			flagF = flagR = true;
			//flagF = true;
		}
  	loop_rate.sleep();
	}

  return 0;
}



