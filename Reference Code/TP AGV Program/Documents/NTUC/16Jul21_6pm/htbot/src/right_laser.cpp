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
#define MAX_250 1.5
#define MIN_RANGE 0.05
#define ANGLE_RES 0.352
// 88 degrees
#define RADRANGE 1.57 
#define	MAX_IDX	250
#define OBS_DIST 0.08
#define RLASER_START_IDX 37

#define DEG2RAD(x) ((x)*ANGLE_RES*3.14159/180.0)
#define RAD2DEG(x) ((x)*180.0/3.14159)

#define TTH	1.6   // Total scanning height to detect obs
#define LSH 0.295   // Laser Scanner height from ground
#define LTT TTH-LSH  // laser to top scanning height
#define TPW 0.275  // transporter width 0.25 0.275
#define HSD 2.0   // horizontal scanning distance
#define THET_R atan((LTT)/TPW)   // 
#define THET_R1 atan((LTT)/HSD)
#define THET_L atan(LSH/TPW)
#define THET_L1 atan(LSH/HSD)
#define PI 3.141593
#define PI_2 1.5708

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
double lrange[550];
double temp [550];
double profile [550];
double start_rad;  // starting radian from +z axis for left and -z axis for right
bool slaserStarted;
int saveSideLaserData;
double leftlaserobsdist,rightlaserobsdist;
ros::Time last_laser_time;

// prototype
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void readSideLaserData();
double comp (double f1, double f2);

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
  	ROS_INFO("I couldn't open rightLaserfile for reading.\n");    
  	return;
  }	

	fscanf(fp,"%d\n",&maxid);

	for (int i=0;i<maxid;i++) {
		fscanf(fp,"%lf\n",&rr);
		lrange[i] = rr;
	}
	ROS_INFO("Read Right Laser Data");
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


void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	
	ros::NodeHandle nx;
	bool save ;
	int extra;	
	int ts;
	double min_angle,an,range,CentralLaserObsDist;
	double t1,t2,t3,t4,t5,t6,t7;
	
	if (!slaserStarted) {
		fp = fopen(Laserfile.c_str(), "w");
		if (fp == NULL) {
  		ROS_INFO("I couldn't open Right Laserfile for writing.\n");    
  		return;
  	}
		slaserStarted = true;
		laser_size = lscan->ranges.size();
		maxid = laser_size;
		ROS_INFO("****************  Right Laser Size : %d  ***********************",laser_size);
		//fprintf(fp,"%d\n",MAX_IDX);
		fprintf(fp,"Right Laser Size = %d\n",laser_size);
		for (int i=0;i<laser_size;i++) {
			// calcuate range based on geometry
			an = ((PI_2-THET_L)+DEG2RAD(i));
			if (an < (PI_2-THET_L1)) {
				range = LSH/cos(an);
			} else {
				if (an < PI_2) {
					range = HSD/sin(an);
				} else {
					if (an < ((PI_2)+THET_R1)) {
						range = HSD/(cos(an - PI_2));
					} else {
						range = LTT/sin(an-PI_2);
					}
				}
			}		
			// save in file and array
			fprintf(fp,"idx=%d. Range = %.3f\n",i,range);	
			profile[i] = range;
		}
		ROS_INFO(" ************* Right Laser Template Profile Generated *****************");
		fclose(fp);
		return;
	}
	if (maxid == 0) {
		return;
	}
	scan_msg.ranges.resize(maxid);
	scan_msg.header.stamp = lscan->header.stamp;
	scan_msg.header.frame_id = lscan->header.frame_id;	
	scan_msg.angle_min = -RADRANGE;  
	scan_msg.angle_max = 0.0;
	min_angle = lscan->angle_min;
	scan_msg.angle_increment = lscan->angle_increment;
	scan_msg.scan_time = lscan->scan_time;
	scan_msg.time_increment = lscan->time_increment;
	scan_msg.range_min = MIN_RANGE; 
	scan_msg.range_max = MAX_RANGE;

	for (int i=0;i<maxid;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			if (lscan->ranges[i] > MAX_RANGE) {
				temp[i] = MAX_RANGE; // - 0.1;
			} else {
				if (lscan->ranges[i] < MIN_RANGE) {
					temp[i] = MAX_RANGE; // - 0.1;
				} else {
					t2 = lscan->ranges[i];		
					if (t2 < TPW) {
						temp[i] = MAX_RANGE;
					} else { 			
						temp[i] = t2;
					}
				}
			}
		} else {
			temp[i] = MAX_RANGE; //- 0.1;
		}
		scan_msg.ranges[i] = MAX_RANGE; // - 0.1;
	}
	/*
	for (int i=0;i<MAX_IDX-7;i++) {
		t1 = temp[i];
		t2 = temp[i+1];
		t3 = temp[i+2];
		t4 = temp[i+3];
		t5 = temp[i+5];
		t6 = temp[i+6];
		t7 = temp[i+7];
		//if (((t1 - t2) > 0.1) && ((t3-t2)>0.07)) {
		if ((fabs(t2 - t1) > 0.025)) {
			if ( (fabs(t3-t2) > 0.025) || (fabs(t4-t3) > 0.025) || (fabs(t5-t4) > 0.025) || (fabs(t6-t5) > 0.025) || (fabs(t7-t6) > 0.025) ) {
				temp[i+1] = MAX_RANGE;				
			} 
		}
	}
	*/
	
	for (int i=0;i<maxid-1;i++) {
		t1 = temp[i];
		t2 = temp[i+1];
		if ((t1 - t2) > 0.15) {
			temp[i+1] = t1;
		}
	}
	
	rightlaserobsdist = 3.0;
	//rightlaserobsdist = 3.0;

	for (int i=0;i<maxid;i++) {
		rd = temp[i];
		if ((profile[i] - rd) > obs_dist) {
			// obs detected			
			dist = rd * sin(DEG2RAD(i)+(PI_2 + min_angle));
			if (dist < TPW ) {
				continue;
			}
			an = atan(dist / Dist2Center);
			rn = dist / sin(an);
			idx = (int) (RAD2DEG(an) / ANGLE_RES) ;			
			//idx = (int) (an / ANGLE_INC_RAD) ;	
			if (dist < rightlaserobsdist) {
				rightlaserobsdist = dist;
			}	
			scan_msg.ranges[idx] = rn;
		}
	}

	scan_pub.publish(scan_msg);
	nx.setParam("RightLaserStatus",0);	
	nx.setParam("RightLaserObsDist",rightlaserobsdist);	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "right laser");
  ros::NodeHandle n;
	ros::NodeHandle np("~");
	
	laser_sub= n.subscribe<sensor_msgs::LaserScan>("/scanin",1, laserCallBack);	 // /scan_base - org
	scan_pub = n.advertise<sensor_msgs::LaserScan>("/scanout", 2000);
	np.param<double>("Detection_Range",detection_range,2.0);
	np.param<double>("Dist2Center",Dist2Center,0.24);
	np.getParam("Laserfile",Laserfile);
	np.param<double>("Obs_Dist", obs_dist,0.08);
	np.param<bool>("inverted", inverted, "false");
	np.param<double>("StartRad", start_rad,0.7916);
	np.param<int>("saveSideLaserData",saveSideLaserData,77);	
	ROS_INFO("Obs_Dist : %.3f. Detection Range : %.3f. Lfile : %s \n",obs_dist,detection_range,Laserfile.c_str());	
	slaserStarted = false;
	maxid = 0;

	//if (saveSideLaserData != 77) {
	//	readSideLaserData();
	//}
	ROS_INFO("==== Right Laser Started =====");

  ros::spin();

  return 0;
}



