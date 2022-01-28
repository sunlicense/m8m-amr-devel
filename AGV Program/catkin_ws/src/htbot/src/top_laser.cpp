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
#define ANGLE_RES 0.352
#define ANGLE_RES_RAD 0.006144
// 88 degrees
#define RADRANGE 1.5359  
#define	MAX_IDX	180
#define OBS_DIST 0.08
#define RLASER_START_IDX 37
#define MINMAX_ANGLE 0.4918 //atan(1.5/2.8) assume 24 deg inclination of laser

// Lumileds Details. Assume 45 deg inclination of laser
#define HEIGHT_OF_LASER 0.965  // metre measured from floor to laser center
#define WIDTH_OF_DETECTION 1.5  // 1.0 meter from center of robot
#define DETECTION_RANGE 2.0
#define DETECTION_HEIGHT 1.3
#define TRANGE 3.14  //

#define DEG2RAD(x) ((x)*ANGLE_RES*3.14159/180.0)
#define RAD2DEG(x) ((x)*180.0/3.14159)


using namespace std;

ros::Subscriber laser_sub;
ros::Publisher scan_pub;

int laser_size,cidx,ridx,lidx;
sensor_msgs::LaserScan scan_msg;
double Dist2Center,dist_min;
double detection_range, detection_height;
double rd,dist,rn,rv,an,obs_dist,distToObs;
int maxid;
std::string Laserfile;
FILE *fp;
bool inverted = false;  // false > right side laser. true > left side laser
double lrange[520];
double start_rad;  // starting radian from +z axis for left and -z axis for right
bool toplaserStarted;
bool saveLaserScan;
int saveTopLaserData;

// prototype
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void readTopLaserData();
double comp (double f1, double f2);
bool setupSpec();

double comp (double f1, double f2) {
	if (fabs(f1-f2) > 0.15) {
		printf("\ngreater");
	} else {
		printf("\nequal or less");
	}
}

bool setupSpec() {
	int id;
	bool ret;
	ret = false;
	// calculate start (right side) index of laser scan to center and left side
	if (toplaserStarted) {
		// calculate right and left index of laser scan data
		if (laser_size > 0) {
			cidx = (int)(laser_size / 2.0);
			id = (int)(RAD2DEG(atan(WIDTH_OF_DETECTION / TRANGE)) / ANGLE_RES) ;  // index from center to right side
			ridx = cidx - id;
			lidx = cidx + id;
		}		
		ret = true;
	}	
	return ret;
}


void readTopLaserData() {
	double rr,d;

	fp = fopen(Laserfile.c_str(), "r");
  if (fp == NULL) {
  	ROS_INFO("I couldn't open topLaserfile for reading.\n");    
  	return;
  }	

	fscanf(fp,"%d\n",&maxid);

	for (int i=0;i<maxid;i++) {
		fscanf(fp,"%lf\n",&rr);
		lrange[i] = rr;
		//ROS_INFO("Laser Data: idx=%d. rd=%.3f",i,rr);
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


void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	double temp [520];
	ros::NodeHandle nx;
	bool save ;
	int extra;	
	int ts;
	double min_angle,an;
	double t1,t2,t3,t4,t5,t6,t7;
	int id,idx,tidx;


	if (!saveLaserScan) {
		//nx.getParam("saveSideLaserData",saveSideLaserData);
		//save = false;
		//nx.getParam("saveTopLaserData",saveTopLaserData);
		//ROS_INFO("saveTopLaserData=%d. ",saveTopLaserData);
		if (saveTopLaserData == 77) {
			saveLaserScan = true;
			//nx.setParam("saveTopeLaserData",0);
		} else {
			saveLaserScan = false;
		}
		
		if (!saveLaserScan) {
			ROS_INFO("Save Top Laser Data");
			fp = fopen(Laserfile.c_str(), "w");
			if (fp == NULL) {
  			ROS_INFO("I couldn't open TopLaserfile for writing.\n");    
  			return;
  		}
			laser_size = lscan->ranges.size();
			//scan_msg.ranges.resize(laser_size);
			cidx = (int)(laser_size / 2.0);
			//id = (int)(RAD2DEG(atan(WIDTH_OF_DETECTION / TRANGE)) / ANGLE_RES) ;  // index from center to right side
			//ridx = cidx - id;
			//lidx = cidx + id;			
			an = atan((DETECTION_HEIGHT - HEIGHT_OF_LASER) / DETECTION_RANGE);
			tidx = (int)(an / ANGLE_RES_RAD) + cidx;
			ROS_INFO("Top Laser Size : %d. cidx:%d. tidx:%d",laser_size,cidx,tidx);
			fprintf(fp,"%d\n",laser_size);
			for (int i=0;i<laser_size;i++) {
				if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
					if (lscan->ranges[i] > MAX_RANGE) {
						rd = MIN_RANGE; // - 0.1;
					} else {
						if (lscan->ranges[i] < MIN_RANGE) {
							rd = MIN_RANGE; // - 0.1;
						} else {
							rd = lscan->ranges[i];											
						}
					}
				} else {
					rd = MIN_RANGE;
				}
				//rd = lscan->ranges[i];
				if (i > tidx) {
					an = (i - cidx) * ANGLE_RES_RAD;
					rd = (DETECTION_HEIGHT - HEIGHT_OF_LASER) / sin(an) ;
					//ROS_INFO("Top : id : %d. range : %.3f",i,rd);
				}
				//ROS_INFO("Save Data. id : %d. range : %.3f",i,rd);
				fprintf(fp,"%.3f\n",rd);
			}
			fclose(fp);		
			saveLaserScan = true;		
			return;
		}
	}
	laser_size = lscan->ranges.size();
	cidx = (int)(laser_size / 2.0);
	/*
	scan_msg.ranges.resize(laser_size);
	scan_msg.header.stamp = lscan->header.stamp;
	scan_msg.header.frame_id = lscan->header.frame_id;	
	scan_msg.angle_min = -1.5708;
	scan_msg.angle_max = 1.5708;
	scan_msg.angle_increment = (3.14159) / MAX_IDX ;
	scan_msg.scan_time = lscan->scan_time;
	scan_msg.time_increment = lscan->time_increment;
	scan_msg.range_min = MIN_RANGE; 
	scan_msg.range_max = MAX_RANGE;
	*/
	
	
	for (int i=0;i<laser_size;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			if (lscan->ranges[i] > MAX_RANGE) {
				temp[i] = MAX_RANGE; // - 0.1;
			} else {
				if (lscan->ranges[i] < MIN_RANGE) {
					temp[i] = MAX_RANGE; // - 0.1;
				} else {
					temp[i] = lscan->ranges[i];										
				}
			}
		} else {
			temp[i] = MAX_RANGE; //- 0.1;
		}
		//scan_msg.ranges[i] = MAX_RANGE; // - 0.1;
	}

	/*
	for (int i=0;i<laser_size-7;i++) {
		t1 = temp[i];
		t2 = temp[i+1];
		t3 = temp[i+2];
		t4 = temp[i+3];
		t5 = temp[i+5];
		t6 = temp[i+6];
		t7 = temp[i+7];
		//if ((fabs(t2 - t1) > 0.1) && (fabs(t3-t2)>0.07)) {
		if ((fabs(t2 - t1) > 0.025)) {
			if ( (fabs(t3-t2) > 0.025) || (fabs(t4-t3) > 0.025) || (fabs(t5-t4) > 0.025) || (fabs(t6-t5) > 0.025) || (fabs(t7-t6) > 0.025) ) {
				temp[i+1] = MAX_RANGE;				
			} 
		}
	}
	*/
	distToObs = 2.5;
	for (int i=0;i<laser_size;i++) {
		rd = temp[i];
		if ((lrange[i] - rd) > obs_dist) {
			// obs detected			
			//ROS_INFO("Top Laser Obs Detected. lrange=%.3f. rd=%.3f. idx=%d",lrange[i],rd,i);
			if (i < cidx) {
				dist = rd * cos(DEG2RAD(cidx-i));				
			} else {
				dist = rd * cos(DEG2RAD(i-cidx));
			}
			//ROS_INFO("Obs Detected : rd = %.3f. rn = %.3f",rd,rn);
			//scan_msg.ranges[i] = rn-0.2;
			if (dist < distToObs) {
				distToObs = dist;
			}
		}
		//scan_msg.ranges[i] = rd;
	}	
	//scan_pub.publish(scan_msg);
	nx.setParam("TopLaserObsDist",distToObs);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "top laser");
  ros::NodeHandle n;
	ros::NodeHandle np("~");
	
	saveLaserScan = false;
	laser_size = 0;
	laser_sub= n.subscribe<sensor_msgs::LaserScan>("/scanin",1, laserCallBack);	 // /scan_base - org
	//scan_pub = n.advertise<sensor_msgs::LaserScan>("/scanout", 2000);
	np.getParam("Laserfile",Laserfile);
	np.param<double>("Obs_Dist", obs_dist,0.08);
	np.param<int>("saveTopLaserData",saveTopLaserData,77);	
	ROS_INFO("Obs_Dist : %.3f. Lfile : %s \n",obs_dist,Laserfile.c_str());	

	toplaserStarted = false;

	readTopLaserData();

  ros::spin();

  return 0;
}



