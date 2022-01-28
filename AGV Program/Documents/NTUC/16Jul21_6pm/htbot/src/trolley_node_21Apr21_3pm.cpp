/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Modified : 21.4.2021
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

#define MAX_RANGEF 25.0
#define MIN_RANGEF 0.05  //0.03
#define SCANRANGE 1.0
#define RADPIDXF 0.0057596
#define MAX_RANGE 4.0
#define MIN_RANGE 0.04  //0.03
#define DWIDTH 0.3  //0.35// 0.4m
#define DBOX 2.0
#define DFRONT 0.02  //0.1
#define DEGPIDX 0.36
#define RADPIDX 0.006283
#define PI 3.14159
#define DM 77.9052 //28.07 // dm = atan(BOXL / BOXW) * (180.0 / PI);  // degree
#define DMIDX 216 //112  // (int)(dm / DEGPIDX)
#define DG  86 //70 // dg = atan(DWIDTH / DFRONT) * (180.0 / PI);  // degree
#define DGIDX 345 //280 // (int)(dg / DEGPIDX)
#define LASEROFFSET 0.03
#define BASEOFFSET 0.31
#define DISTGT350MM  0.6

#define FILTER_SCAN_SIZE 7
#define FILTER_SPIKE_SIZE 3
#define STEP 0.08  // m
#define FILTER_LEVEL_SIZE 0.04 // m

//#define BOXW	0.15
//#define BOXL  0.7

using namespace std;

ros::Subscriber laserF_sub;
ros::Subscriber laserR_sub;
ros::Subscriber laserL_sub;
//ros::Publisher scan_pub;
//ros::Publisher obs_pub;

int laserF_size;
int laserR_size;
int laserL_size;
int cidx,lidx,ridx;

bool rflag,lflag;
bool leginRzone,leginLzone,STARTDETECTLEG,STARTDETECTLEGFRONT;
bool RightLeg,LeftLeg,trolley,TrolleyAlign;

double OUTLIMIT,INLIMIT,trolleyleg,BOXW,BOXL;

// prototype
void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void laserLCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);

void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld;
	int nleg,rlegix,llegix;
	double rlegdist,llegdist,rlan,llan;
	bool rlegf,llegf;
	
	nx.getParam("STARTDETECTLEGFRONT",STARTDETECTLEGFRONT);
	if (!STARTDETECTLEGFRONT) {
		//nx.setParam("TrolleyAlign",false);
		return;
	}
	ROS_INFO("--------- Trolley : Front Laser-------------------");
	laserF_size = lscan->ranges.size();
	cidx = (int)(laserF_size / 2.0);
	ridx = cidx - 160;  // around 45 deg
	lidx = cidx + 160;
	nleg = 0;
	// detect right leg
	//ROS_INFO("--------- Trolley : lsize=%d. cidx=%d. ridx=%d. lidx=%d -------------------",\
	//						laserF_size,cidx,ridx,lidx);
	rlegf = false;
	for (int i=cidx;i>ridx;i--) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			ld = lscan->ranges[i];
			//ROS_INFO("------- RL : range=%.3f. i=%d-------------",ld,i);
			if ((ld > MIN_RANGEF) && (ld < SCANRANGE)) {
				// detected leg
				nleg++;
				if (nleg >= 2) {
					rlegix = i;
					rlegdist = ld;
					rlan = RADPIDXF * (cidx - i);
					rlegf = true;
					//ROS_INFO("---------- Detected Right Leg.------------");
					break;
				}
			} 
		}
	}
	// detect left leg
	nleg = 0;
	llegf = false;
	for (int i=cidx;i<lidx;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			ld = lscan->ranges[i];
			if ((ld > MIN_RANGEF) && (ld < SCANRANGE)) {
				// detected leg
				nleg++;
				if (nleg >= 2) {
					llegix = i;
					llegdist = ld;
					llan = RADPIDXF * (i-cidx);
					llegf = true;
					//ROS_INFO("---------- Detected Left Leg.------------");
					break;
				}
			} 
		}
	}
	if (rlegf && llegf) {
		//ROS_INFO("----------- Trolley Front.-----------");
		nx.setParam("TrolleyAlign",true);
		nx.setParam("RightLegIdx",rlegix);
		nx.setParam("RightLegdist",rlegdist);
		nx.setParam("RightLegAngle",rlan);
		nx.setParam("LeftLegIdx",llegix);
		nx.setParam("LeftLegdist",llegdist);
		nx.setParam("LeftLegAngle",llan);
	} else {
		//nx.setParam("TrolleyAlign",false);
	}
}

void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld,ld1,ld2,degperindx,rr,an,radperindx,dm;
	int nleg;
	
	nx.getParam("STARTDETECTLEG",STARTDETECTLEG);
	if (!STARTDETECTLEG) {
		//nx.setParam("RightZone",false);
		//nx.setParam("RightLeg",false);
		return;
	}

	nx.getParam("BOXW",BOXW);
	nx.getParam("BOXL",BOXL);
	laserR_size = lscan->ranges.size();
	degperindx = 180.0 / laserR_size;
	radperindx = degperindx * (PI / 180.0);	
	cidx = (int)(laserR_size / 2.0);
	dm = atan(BOXL / BOXW) * (180.0 / PI);
	ridx = (int)(dm / degperindx) + cidx ; 
	if (!rflag) {
		rflag = true;
		ROS_INFO("--- RLaser Trolley : size=%d. cidx=%d. degidx=%.3f. radidx=%.3f. ridx=%d. dm=%.3f -------",laserR_size,cidx,degperindx,radperindx,ridx,dm);
	}
			
	// look for trolley legs
	//ROS_INFO("--- Right Laser : BOXW=%.3f. BOXL=%.3f. trolleyleg=%.2f. ------",BOXW,BOXL,trolleyleg);
	leginRzone = false;
	nleg = 0;
	for (int i=cidx;i<ridx;i++) {
		// right laser
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			ld = lscan->ranges[i];
			an = (i-cidx) * radperindx;
			rr = (double)(BOXW / cos(an));			
			if (ld < rr) {
				nleg++;
				if (nleg >= 2) {
					leginRzone = true;
					break;
				}
			}
		}
	}
	if (!RightLeg) {
		ld = MAX_RANGE;
		if(!std::isnan(lscan->ranges[cidx-1]) && !std::isinf(lscan->ranges[cidx-1])){
			ld = lscan->ranges[cidx-1];
		}
		ld1 = MAX_RANGE;
		if(!std::isnan(lscan->ranges[cidx]) && !std::isinf(lscan->ranges[cidx])){
			ld1 = lscan->ranges[cidx];
		}
		ld2 = MAX_RANGE;
		if(!std::isnan(lscan->ranges[cidx+1]) && !std::isinf(lscan->ranges[cidx+1])){
			ld2 = lscan->ranges[cidx+1];
		}
			
		if ((ld < trolleyleg) || (ld1 < trolleyleg) || (ld2 < trolleyleg)){
			// hit leg of trolley
			RightLeg = true;
			nx.setParam("RightLeg",RightLeg);	
			//ROS_INFO("----------- Trolley : RightLeg. ----------");			
		} else {
			//nx.setParam("RightLeg",false);	
		}
	}
	if (leginRzone) {
		nx.setParam("RightZone",true);	
		//ROS_INFO("----------- Trolley : RightZone. ----------");
		return;
	}
	nleg = 0;
	for (int i=ridx;i<laserR_size;i++) {
		// right laser
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			ld = lscan->ranges[i];
			an = (i-cidx) * radperindx;
			rr = (double)(BOXL / sin(an));			
			if (ld < rr) {
				nleg++;
				if (nleg >= 2) {
					leginRzone = true;
					break;
				}
			}
		}
	}
	if (leginRzone) {
		nx.setParam("RightZone",true);	
		//ROS_INFO("----------- Trolley : RightZone. ----------");
	} else {
		nx.setParam("RightZone",false);
	}
	
}

void laserLCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld,ld1,ld2,degperindx,rr,an,radperindx,dm;
	int nleg;
	
	nx.getParam("STARTDETECTLEG",STARTDETECTLEG);
	if (!STARTDETECTLEG) {
		//nx.setParam("LeftLeg",false);	
		//nx.setParam("LeftZone",false);
		return;
	}
	nx.getParam("BOXW",BOXW);
	nx.getParam("BOXL",BOXL);		

	laserL_size = lscan->ranges.size();
	degperindx = 180.0 / laserL_size;
	radperindx = degperindx * (PI / 180.0);
	cidx = (int)(laserL_size / 2.0);
	dm = atan(BOXW / BOXL) * (180.0 / PI);
	lidx = (int)(dm / degperindx) ; 
	
	if (!lflag) {
		lflag = true;
		ROS_INFO("--- LLaser Trolley : size=%d. cidx=%d. degidx=%.3f. radidx=%.3f. lidx=%d. dm=%.3f --------",laserL_size,cidx,degperindx,radperindx,lidx,dm);
	}
	
	// look for trolley legs
	//ROS_INFO("--- Left Laser : BOXW=%.3f. BOXL=%.3f. trolleyleg=%.2f. ------",BOXW,BOXL,trolleyleg);
	leginLzone = false;
	nleg = 0;
	for (int i=0;i<lidx;i++) {
		// left laser
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			ld = lscan->ranges[i];
			an = (i) * radperindx;
			rr = (double)(BOXL / cos(an));			
			if (ld < rr) {
				nleg++;
				if (nleg >= 2) {
					leginLzone = true;
					break;
				}
			}
		}
	}
	if (!LeftLeg) {
		ld = MAX_RANGE;
		if(!std::isnan(lscan->ranges[cidx-1]) && !std::isinf(lscan->ranges[cidx-1])){
			ld = lscan->ranges[cidx-1];
		}
		ld1 = MAX_RANGE;
		if(!std::isnan(lscan->ranges[cidx]) && !std::isinf(lscan->ranges[cidx])){
			ld1 = lscan->ranges[cidx];
		}
		ld2 = MAX_RANGE;
		if(!std::isnan(lscan->ranges[cidx+1]) && !std::isinf(lscan->ranges[cidx+1])){
			ld2 = lscan->ranges[cidx+1];
		}
		if ((ld < trolleyleg) || (ld1 < trolleyleg) || (ld2 < trolleyleg)){
			// hit leg of trolley
			LeftLeg = true;
			nx.setParam("LeftLeg",LeftLeg);	
			//ROS_INFO("----------- Trolley : LeftLeg. ----------");			
		} else {
			//nx.setParam("LeftLeg",false);	
		}
	}
	if (leginLzone) {
		nx.setParam("LeftZone",true);	
		//ROS_INFO("----------- Trolley : LeftZone. ----------");
		return;
	} 
	nleg = 0;
	for (int i=lidx;i<cidx;i++) {
		// left laser
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			ld = lscan->ranges[i];
			an = (i) * radperindx;
			rr = (double)(BOXW / sin(an));			
			if (ld < rr) {
				nleg++;
				if (nleg >= 2) {
					leginLzone = true;
					break;
				}
			}
		}
	}
	if (leginLzone) {
		nx.setParam("LeftZone",true);	
		//ROS_INFO("----------- Trolley : LeftZone. ----------");
	} else {
		nx.setParam("LeftZone",false);
	}
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_trolley");
  ros::NodeHandle n;
	
	//n.setParam("MainLaserStatus",0);	
	OUTLIMIT = 1.5;
	INLIMIT = 0.5;
	trolleyleg = 0.5;
	n.getParam("OUTLIMIT",OUTLIMIT);
	n.getParam("INLIMIT",INLIMIT);
	n.getParam("trolleyleg",trolleyleg);
	
	ROS_INFO("---------- Trolley : trolleyleg = %.2f. -----------",trolleyleg);
	rflag = false;
	lflag = false;
	leginRzone = false;
	leginLzone = false;
	RightLeg = false;
	LeftLeg = false;
	STARTDETECTLEG = false;
	STARTDETECTLEGFRONT = false;
	TrolleyAlign = false;
	n.setParam("TrolleyAlign",TrolleyAlign);

	laserF_sub= n.subscribe<sensor_msgs::LaserScan>("/scanF",100, laserFCallBack);	 // /scan_base - org
	laserR_sub= n.subscribe<sensor_msgs::LaserScan>("/scanright",100, laserRCallBack);
	laserL_sub= n.subscribe<sensor_msgs::LaserScan>("/scanleft",100, laserLCallBack);
	//scan_pub = n.advertise<sensor_msgs::LaserScan>("scanx", 10);
  ros::spin();

  return 0;
}



