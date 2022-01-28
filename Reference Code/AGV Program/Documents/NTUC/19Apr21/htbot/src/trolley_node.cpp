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

ros::Subscriber laserR_sub;
ros::Subscriber laserL_sub;
ros::Publisher scan_pub;
//ros::Publisher obs_pub;

int laserR_size;
int laserL_size;
int cidx,lidx,ridx;

bool rflag,lflag;
bool leginRzone,leginLzone,STARTDETECTLEG;

double OUTLIMIT,INLIMIT,trolleyleg,BOXW,BOXL;

// prototype
//void laserFCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void laserLCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);

void laserRCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld,degperindx,rr,an,radperindx;
	int nleg;
	
	nx.getParam("STARTDETECTLEG",STARTDETECTLEG);
	if (!STARTDETECTLEG) {
		return;
	}

	laserR_size = lscan->ranges.size();
	degperindx = 180.0 / laserR_size;
	radperindx = degperindx * (PI / 180.0);
	ridx = (int)(77.905 / degperindx) ; 
	cidx = (int)(laserR_size / 2.0);
	if (!rflag) {
		rflag = true;
		ROS_INFO("--- Trolley Right Laser : size=%d. cidx=%d. degidx=%.3f. radidx=%.3f. ridx=%d. -------",laserR_size,cidx,degperindx,radperindx,ridx);
	}
	nx.getParam("BOXW",BOXW);
	nx.getParam("BOXL",BOXL);		
	// look for trolley legs
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
	if(!std::isnan(lscan->ranges[cidx]) && !std::isinf(lscan->ranges[cidx])){
		ld = lscan->ranges[cidx];
		if (ld < trolleyleg) {
			// hit leg of trolley
			nx.setParam("RightLeg",true);	
		} else {
			nx.setParam("RightLeg",false);	
		}
	}
	if (leginRzone) {
		nx.setParam("RightZone",true);	
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
	} else {
		nx.setParam("RightZone",false);
	}
	
}

void laserLCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;	
	double ld,degperindx,rr,an,radperindx;
	int nleg;
	
	nx.getParam("STARTDETECTLEG",STARTDETECTLEG);
	if (!STARTDETECTLEG) {
		return;
	}

	laserL_size = lscan->ranges.size();
	degperindx = 180.0 / laserL_size;
	radperindx = degperindx * (PI / 180.0);
	lidx = (int)(12.095 / degperindx) ; 
	cidx = (int)(laserL_size / 2.0);
	if (!lflag) {
		lflag = true;
		ROS_INFO("--- Trolley Left Laser : size=%d. cidx=%d. degidx=%.3f. radidx=%.3f. lidx=%d. --------",laserL_size,cidx,degperindx,radperindx,lidx);
	}
	nx.getParam("BOXW",BOXW);
	nx.getParam("BOXL",BOXL);		
	// look for trolley legs
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
	if(!std::isnan(lscan->ranges[cidx]) && !std::isinf(lscan->ranges[cidx])){
		ld = lscan->ranges[cidx];
		if (ld < trolleyleg) {
			// hit leg of trolley
			nx.setParam("LeftLeg",true);	
		} else {
			nx.setParam("LeftLeg",false);	
		}
	}
	if (leginLzone) {
		nx.setParam("LeftZone",true);	
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
	} else {
		nx.setParam("LeftZone",false);
	}
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_filter");
  ros::NodeHandle n;
	
	//n.setParam("MainLaserStatus",0);	
	OUTLIMIT = 1.5;
	INLIMIT = 0.5;
	trolleyleg = 0.5;
	n.getParam("OUTLIMIT",OUTLIMIT);
	n.getParam("INLIMIT",INLIMIT);
	n.getParam("trolleyleg",trolleyleg);
	rflag = false;
	lflag = false;
	leginRzone = false;
	leginLzone = false;
	STARTDETECTLEG = false;

	//laserF_sub= n.subscribe<sensor_msgs::LaserScan>("/scanF",100, laserFCallBack);	 // /scan_base - org
	laserR_sub= n.subscribe<sensor_msgs::LaserScan>("/scanright",100, laserRCallBack);
	laserL_sub= n.subscribe<sensor_msgs::LaserScan>("/scanleft",100, laserLCallBack);
	//scan_pub = n.advertise<sensor_msgs::LaserScan>("scanx", 10);
  ros::spin();

  return 0;
}



