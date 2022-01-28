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

#define MAX_RANGE 4.0
#define MIN_RANGE 0.05
#define ANGLE_RES 0.36
#define ANGLE_RES_RAD 0.006283
#define RADRANGE 1.57  
#define	MAX_IDX	420
#define OBS_DIST 0.08
#define RLASER_START_IDX 37
#define ANGLE_INC_RAD 0.00384  // RADRANGE / MAX_IDX

#define DEG2RAD(x) ((x)*ANGLE_RES*3.14159/180.0)
#define RAD2DEG(x) ((x)*180.0/3.14159)

#define DR 2.0 // detection range in metre
#define HDR 1.4 // height of detection range in meter
#define HL 0.49 // height of laser from ground in meter
#define BS 0.18 // Bottom start of detection from laser
#define TS 0.18 // Top start of detection from laser



using namespace std;

ros::Subscriber laser_sub;
ros::Publisher scan_pub;

int laser_size,idx;
sensor_msgs::LaserScan scan_msg;
double Dist2Center,dist_min;
double detection_range, detection_height;
double rd,dist,rn,an,obs_dist;
int maxid;
std::string Laserfile;
FILE *fp;
bool inverted = false;  // false > right side laser. true > left side laser
double lrange[700];
double start_rad;  // starting radian from +z axis for left and -z axis for right
bool slaserStarted;
int saveSideLaserData;

// prototype
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void readSideLaserData();
double comp (double f1, double f2);
void generateRefLaserData();

double comp (double f1, double f2) {
	if (fabs(f1-f2) > 0.15) {
		printf("\ngreater");
	} else {
		printf("\nequal or less");
	}
}

void readSideLaserData() {
	double rr,d;

	fp = fopen(Laserfile.c_str(), "r");
  if (fp == NULL) {
  	ROS_INFO("I couldn't open sideLaserfile for reading.\n");    
  	return;
  }	

	fscanf(fp,"%d\n",&maxid);

	for (int i=0;i<maxid;i++) {
		fscanf(fp,"%lf\n",&rr);
		lrange[i] = rr;
	}
	/*
	printf("\nmaxid : %d\n %.3f,",maxid,lrange[0]);
	for (int i=1;i<maxid;i++) {
		if ((i%10) == 0) {
			printf("%.3f\n",lrange[i]);
		} else {
			printf("%.3f,",lrange[i]);
		}
	}
	*/
}

void generateRefLaserData() {
	double ans,an,rd,ane,anf,an1;
	int ix,iy;
	ans = atan(BS/HL);  // start angle of laser profile
	// bottom zone
	ix = 0;
	ane = atan(DR / HL);
	while(true) {
		an = ans + ix * ANGLE_RES_RAD;
		if (an > 1.570) {
			break;
		}
		if (an < ane) {
			rd = HL / cos(an);
		} else {
			rd = DR / sin(an);
		}		
		lrange[ix] = rd;
		ix++;
	}
	ROS_INFO("1st: ane=%.3f. ix=%d.",ane,ix);
	iy = 0;
	ane = atan((HDR - HL)/ DR);
	anf = 1.5709 - atan(TS/(HDR-HL));
	while (true) {
		an = iy * ANGLE_RES_RAD;
		if (an > anf) {
			break;
		}
		if (an < ane) {
			rd = DR / cos(an);			
		} else {
			rd = (HDR-HL) / sin(an);
		}		
		lrange[ix] = rd;
		ix++;
		iy++;
	}	
	ROS_INFO("2nd: ane=%.3f. anf=%.3f. ix=%d. iy=%d",ane,anf,ix,iy);
	ROS_INFO(" ============== Laser Profile ======================= ");
	for (int x=0;x<ix;x++) {
		rd = lrange[x];
		ROS_INFO("Laser : rd=%.3f. id=%d",rd,x);
	}
}

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	double temp [500];
	ros::NodeHandle nx;
	bool save ;
	int extra;	
	int ts;
	double min_angle;
	double t1,t2,t3,t4,t5,t6,t7,t8,t9;
	int leftLaser,rightLaser;

	// laser ready
	/*
	if (!slaserStarted) {
		leftLaser = 0;
		nx.getParam("LeftLaser",leftLaser);	
		if (leftLaser == 0) {
			nx.setParam("LeftLaser",1);
			slaserStarted = true;
		} else {
			nx.setParam("RightLaser",1);
			slaserStarted = true;
		}
	}
	*/
	/*
	if (!slaserStarted) {
		if (inverted) {
			// left
			leftLaser = 0;
			nx.getParam("LeftLaser",leftLaser);	
			if (leftLaser == 0) {
				nx.setParam("LeftLaser",1);
				slaserStarted = true;
			}
		} else {
			rightLaser = 0;
			nx.getParam("RightLaser",rightLaser);	
			if (rightLaser == 0) {
				nx.setParam("RightLaser",1);
				slaserStarted = true;
			}
		}
	}
	*/
	save = false;
	if (!slaserStarted) {
		//nx.getParam("saveSideLaserData",saveSideLaserData);		
		//nx.getParam("saveSideLaserData",saveSideLaserData);
		if (saveSideLaserData == 77) {
			save = true;
			//nx.setParam("saveSideLaserData",0);
		}
		slaserStarted = true;
	}
	scan_msg.ranges.resize(MAX_IDX);
	if (save) {
		ROS_INFO(" ====================================");
		ROS_INFO("Saving laser data into file : %s",Laserfile.c_str());
		fp = fopen(Laserfile.c_str(), "w");
		if (fp == NULL) {
  		ROS_INFO("I couldn't open Laserfile for writing.\n");    
  		return;
  	}
		laser_size = lscan->ranges.size();
		ROS_INFO("Laser Size : %d",laser_size);
		fprintf(fp,"%d\n",MAX_IDX);
		for (int i=0;i<MAX_IDX;i++) {
			if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
				if (lscan->ranges[i] > detection_range) {
					scan_msg.ranges[i] = detection_range;
				} else {
					if (lscan->ranges[i] < MIN_RANGE) {
						scan_msg.ranges[i] = detection_range;
					} else {
						scan_msg.ranges[i] = lscan->ranges[i];
					}
				}
			} else {
				scan_msg.ranges[i] = detection_range;
			}
			fprintf(fp,"%.3f\n",scan_msg.ranges[i]);
		}
		fclose(fp);		
		return;
	}

	//laser_size = lscan->ranges.size();
	//ts = laser_size;
	//printf("Laser_size : %d\n",ts);
	//if (laser_size > maxid) {
	//	laser_size = maxid;
	//}
	//scan_msg.ranges.resize(MAX_IDX);
	scan_msg.header.stamp = lscan->header.stamp;
	scan_msg.header.frame_id = lscan->header.frame_id;	
	if (inverted) { 
		// left laser
		scan_msg.angle_min = 0.0;  
		scan_msg.angle_max = RADRANGE;
	} else {
		// right laser
		scan_msg.angle_min = -RADRANGE;  
		scan_msg.angle_max = 0.0;
	}
	min_angle = lscan->angle_min;
	scan_msg.angle_increment = lscan->angle_increment;
	//scan_msg.angle_increment = ANGLE_INC_RAD;
	scan_msg.scan_time = lscan->scan_time;
	scan_msg.time_increment = lscan->time_increment;
	scan_msg.range_min = MIN_RANGE; 
	scan_msg.range_max = MAX_RANGE;

	for (int i=0;i<MAX_IDX;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			if (lscan->ranges[i] > detection_range) {
				temp[i] = MAX_RANGE; // - 0.1;
			} else {
				if (lscan->ranges[i] < MIN_RANGE) {
					temp[i] = MAX_RANGE; // - 0.1;
				} else {
					t2 = lscan->ranges[i];					
					temp[i] = t2;
				}
			}
		} else {
			temp[i] = MAX_RANGE; //- 0.1;
		}
		scan_msg.ranges[i] = MAX_RANGE; // - 0.1;
	}
	/*
	t1 = 0;
	for (int i=0;i<MAX_IDX-2;i++) {
		t1 = temp[i]+t1;
		t1 = t1 / (i+1.0);
		t2 = temp[i+1];
		t3 = temp[i+2];
		if (fabs(t2 - t1) > 0.8) {
			//t2 = (t3+t1)/2.0;
			temp[i+1] = t1;
		}
	}
	*/
	/*
	for (int i=1;i<MAX_IDX-7;i++) {
		t1 = temp[i];
		t2 = temp[i-1];
		t3 = temp[i+1];
		t4 = temp[i+2];
		t5 = temp[i+3];
		t6 = temp[i+4];
		t7 = temp[i+5];
		t8 = temp[i+6];
		t9 = temp[i+7];
		//if ((fabs(t2 - t1) > 0.1) && (fabs(t3-t2)>0.07)) {
		if ((fabs(t2 - t1) > 0.07)) {
			if ((fabs(t3-t2) < 0.04) || (fabs(t4-t2) < 0.04) || (fabs(t5-t2) < 0.05) || (fabs(t6-t2) < 0.05) || (fabs(t7-t2) < 0.06) || (fabs(t8-t2) < 0.06) || (fabs(t9-t2) < 0.07)) {
				// noise
				ROS_INFO("Noise ==============");
				temp[i] = t2;
			}
		}
	}
	*/
	/*
	for (int i=0;i<MAX_IDX-2;i++) {
		t1 = temp[i];
		t2 = temp[i+1];
		t3 = temp[i+2];
		if (fabs(t2 - t1) > 1.5) {
			//t2 = (t3+t1)/2.0;
			temp[i+1] = t2;
		}
	}
	*/
	int dcnt;
	dcnt = 0;
	for (int i=0;i<MAX_IDX;i++) {
		rd = temp[i];
		if (rd > detection_range) {
			continue;
		}
		if ((lrange[i] - rd) > obs_dist) {
			// obs detected
			if (inverted) {
				// left side laser. scan goes from top to bottom. right to left
				dist = rd * sin(DEG2RAD(i)+(1.5709 + min_angle));
				an = atan(Dist2Center / dist);
				rn = Dist2Center / sin(an);
				idx = (int) (RAD2DEG(an) / ANGLE_RES) ;			
				//idx = (int) (an / ANGLE_INC_RAD) ;			
			} else {
				// right side laser. scan goes from bottom to top. right to left
				dist = rd * sin(DEG2RAD(i)+(1.5709 + min_angle));				
				an = atan(Dist2Center / dist);
				rn = Dist2Center / sin(an);
				idx = ((int) (RAD2DEG(1.57 - an) / ANGLE_RES)) ;				
				//idx = MAX_IDX - ((int) (an / ANGLE_INC_RAD)) ;			
				//printf("\nRight Detected. rd : %.3f. dist : %.3f. an : %.3f. rn : %.3f. idx : %d.\n",rd,dist,an,rn,idx);
			}
			scan_msg.ranges[idx] = rn;
		}
	}
	
	scan_pub.publish(scan_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "side laser");
  ros::NodeHandle n;
	ros::NodeHandle np("~");
		
	np.param<double>("Detection_Range",detection_range,2.0);
	np.param<double>("Dist2Center",Dist2Center,0.24);
	np.getParam("Laserfile",Laserfile);
	np.param<double>("Obs_Dist", obs_dist,0.08);
	np.param<bool>("inverted", inverted, "false");
	np.param<double>("StartRad", start_rad,0.7916);
	np.param<int>("saveSideLaserData",saveSideLaserData,77);		
	ROS_INFO("Obs_Dist : %.3f. Detection Range : %.3f. save=%d. Lfile : %s \n",obs_dist,detection_range,saveSideLaserData,Laserfile.c_str());	
	//readSideLaserData();
	generateRefLaserData();
	laser_sub= n.subscribe<sensor_msgs::LaserScan>("/scanin",1, laserCallBack);	 // /scan_base - org
	scan_pub = n.advertise<sensor_msgs::LaserScan>("/scanout", 2000);
	slaserStarted = false;
	/*
	comp(1.5,1.3);
	comp(1.5,-1.3);
	comp(-1.5,1.3);
	comp(1.3,1.5);
	comp(1.3,-1.5);
	comp(-1.3,1.5);
	comp(-1.3,-1.5);
	comp(-1.3,-1.4);
	comp(-1.4,-1.3);
	comp(1.3,1.4);
	*/
  ros::spin();

  return 0;
}



