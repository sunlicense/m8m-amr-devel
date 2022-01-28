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
#define MIN_RANGE 0.02  //0.03
#define OBS_DIST 0.08
#define pi 3.14159
using namespace std;

ros::Subscriber llaser_sub;
ros::Subscriber rlaser_sub;
ros::Publisher scanL_pub;
ros::Publisher scanR_pub;

int laser_size;
sensor_msgs::LaserScan scan_msg;
FILE *rfp;  // right laser file
FILE *lfp;  // left laser file
int saveLeftSideLaserData,saveRightSideLaserData;
std::string rightLaserfile;
std::string leftLaserfile;
std::string lframeid;
double llrange[500];
double rlrange[500];
int llaser_size,rlaser_size;
double Dist2Center;
double llaser_min_angle,llaser_max_angle;
double rlaser_min_angle,rlaser_max_angle;
double left_laser_min_angle,left_laser_max_angle;
double right_laser_min_angle,right_laser_max_angle;

// prototype
void leftlaserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void rightlaserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void readSideLaserData();

void readSideLaserData() {
	double rr,d;

	lfp = fopen(leftLaserfile.c_str(), "r");
  if (lfp == NULL) {
  	ROS_INFO("I couldn't open leftLaserfile for reading.\n");    
  	return;
  }
	rfp = fopen(rightLaserfile.c_str(), "r");
	if (rfp == NULL) {
  	ROS_INFO("I couldn't open rightLaserfile for reading.\n");    
  	return;
  }
	//left_laser_min_angle = -2.35;
	//left_laser_max_angle = -0.55;
	//right_laser_min_angle = 0.55;
	//right_laser_max_angle = 2.35;
	fscanf(lfp,"%d\n",&llaser_size);
	fscanf(rfp,"%d\n",&rlaser_size);
	for (int i=0;i<llaser_size;i++) {
		fscanf(lfp,"%lf\n",&rr);
		llrange[i] = rr;
	}

	d = llrange[0] * sin(pi + left_laser_min_angle);
	llaser_min_angle = atan(Dist2Center / d);
	d = llrange[llaser_size-1] * sin(-left_laser_max_angle);
	llaser_max_angle = atan(Dist2Center / d);

	for (int i=0;i<rlaser_size;i++) {
		fscanf(rfp,"%lf\n",&rr);
		rlrange[i] = rr;
	}
	d = rlrange[0] * sin(right_laser_min_angle);
	rlaser_min_angle = -atan(Dist2Center / d);
	d = rlrange[llaser_size-1] * sin(pi - right_laser_max_angle);
	rlaser_max_angle = -atan(Dist2Center / d);
	/*
	printf("Left Laser Size : %d, min_angle : %.3f, max_angle : %.3f \n",llaser_size,llaser_min_angle,llaser_max_angle);
	printf("%.3f,",llrange[0]);
	for (int i=1;i<llaser_size;i++) {
		if ((i%10) == 0) {
			printf("%.3f\n",llrange[i]);
		} else {
			printf("%.3f,",llrange[i]);
		}
	}
	printf("====================================================\n");
	printf("Right Laser Size : %d\n",rlaser_size);
	printf("Right Laser Size : %d, min_angle : %.3f, max_angle : %.3f \n",rlaser_size,rlaser_min_angle,rlaser_max_angle);
	printf("%.3f,",rlrange[0]);
	for (int i=1;i<rlaser_size;i++) {
		if ((i%10) == 0) {
			printf("%.3f\n",rlrange[i]);
		} else {
			printf("%.3f,",rlrange[i]);
		}
	}
	*/
}

void leftlaserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;
	bool save ;
	int extra;

	//ROS_INFO("Left Laser");
	save = false;
	nx.getParam("saveLeftSideLaserData",saveLeftSideLaserData);
	if (saveLeftSideLaserData == 77) {
		save = true;
		nx.setParam("saveLeftSideLaserData",0);
	}
	if (!save) {
		laser_size = lscan->ranges.size();		
		extra = 0;
		if (laser_size < llaser_size) {
			extra = llaser_size - laser_size;
		}
		scan_msg.ranges.resize(llaser_size);		
		scan_msg.header.stamp = lscan->header.stamp;		
		scan_msg.header.frame_id = lscan->header.frame_id;
		//scan_msg.header.frame_id = lframeid;
		//printf("left id : %s\n",scan_msg.header.frame_id.c_str());
		scan_msg.angle_min = llaser_min_angle;
		scan_msg.angle_max = llaser_max_angle;
		scan_msg.angle_increment = (llaser_max_angle - llaser_min_angle) / ((double) llaser_size );
		scan_msg.scan_time = lscan->scan_time;
		scan_msg.time_increment = lscan->scan_time / ((double)llaser_size) ;		
		scan_msg.range_min = MIN_RANGE;
		scan_msg.range_max = MAX_RANGE;

		for (int i=0;i<laser_size;i++) {
			if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
				if (lscan->ranges[i] > MAX_RANGE) {
					scan_msg.ranges[i] = MAX_RANGE - 0.1;
				} else {
					if (lscan->ranges[i] < MIN_RANGE) {
						scan_msg.ranges[i] = MAX_RANGE - 0.1;
					} else {
						// check if obs
						if ((llrange[i] - lscan->ranges[i]) > OBS_DIST) {
							scan_msg.ranges[i] = lscan->ranges[i];
						} else {
							scan_msg.ranges[i] = MAX_RANGE - 0.1;
						}						
					}
				}
			} else {
				scan_msg.ranges[i] = MAX_RANGE - 0.1;
			}
		}
		if (extra > 0) {
			for (int i=laser_size;i<laser_size+extra;i++) {
				scan_msg.ranges[i] = MAX_RANGE - 0.1;
			}
		}
		scanL_pub.publish(scan_msg);
		return;
	}
	lfp = fopen(leftLaserfile.c_str(), "w");
	if (lfp == NULL) {
  	ROS_INFO("I couldn't open leftLaserfile for writing.\n");    
  	return;
  }
	laser_size = lscan->ranges.size();
	left_laser_min_angle = lscan->angle_min;
	left_laser_max_angle = lscan->angle_max;
	for (int i=0;i<laser_size;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			//ROS_INFO("Here P2");
			if (lscan->ranges[i] > MAX_RANGE) {
				scan_msg.ranges[i] = MAX_RANGE - 0.1;
			} else {
				if (lscan->ranges[i] < MIN_RANGE) {
					scan_msg.ranges[i] = MAX_RANGE - 0.1;
				} else {
					scan_msg.ranges[i] = lscan->ranges[i];
				}
			}
		} else {
			scan_msg.ranges[i] = MAX_RANGE - 0.1;
		}
		fprintf(lfp,"%.3f\n",scan_msg.ranges[i]);
	}
	save = false;
	fclose(lfp);
}

void rightlaserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {	
	ros::NodeHandle nx;
	bool save ;

	//ROS_INFO("Right Laser");
	save = false;
	nx.getParam("saveRightSideLaserData",saveRightSideLaserData);
	if (saveRightSideLaserData == 77) {
		save = true;
		nx.setParam("saveRightSideLaserData",0);
	}
	if (!save) {
		return;
	}
	rfp = fopen(rightLaserfile.c_str(), "w");
	if (rfp == NULL) {
  	ROS_INFO("I couldn't open rightLaserfile for writing.\n");    
  	return;
  }
	laser_size = lscan->ranges.size();
	ROS_INFO("Right Laser Size : %d.",laser_size);	
	scan_msg.ranges.resize(laser_size);
	//scan_msg.intensities.resize(laser_size);
	scan_msg.header.stamp = lscan->header.stamp;
	scan_msg.header.frame_id = lscan->header.frame_id;
	right_laser_min_angle = lscan->angle_min;
	right_laser_max_angle = lscan->angle_max;
	scan_msg.angle_min = lscan->angle_min;
	scan_msg.angle_max = lscan->angle_max;
	scan_msg.angle_increment = lscan->angle_increment;
	scan_msg.scan_time = lscan->scan_time;
	scan_msg.time_increment = lscan->time_increment;
	//scan_msg.range_min = lscan->range_min;
	scan_msg.range_min = MIN_RANGE;
	scan_msg.range_max = MAX_RANGE;

	fprintf(rfp,"%d\n",laser_size);
	//ROS_INFO("ainc : %.6f. stime : %.6f. t_inc : %.6f",lscan->angle_increment,lscan->scan_time,lscan->time_increment);
	for (int i=0;i<laser_size;i++) {
		if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
			//ROS_INFO("Here P2");
			if (lscan->ranges[i] > MAX_RANGE) {
				scan_msg.ranges[i] = MAX_RANGE - 0.1;
			} else {
				if (lscan->ranges[i] < MIN_RANGE) {
					scan_msg.ranges[i] = MAX_RANGE - 0.1;
				} else {
					scan_msg.ranges[i] = lscan->ranges[i];
				}
			}
		} else {
			scan_msg.ranges[i] = MAX_RANGE - 0.1;
		}
		//ROS_INFO("Here P3");
		//scan_msg.intensities[i] = lscan->intensities[i];
		//ROS_INFO("Here P4");
		fprintf(rfp,"%.3f\n",scan_msg.ranges[i]);
	}
	save = false;
	fclose(rfp);
	//scan_pub.publish(scan_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_filter");
  ros::NodeHandle n;
	Dist2Center = 0.2425;
	
	lframeid = "/base_link";
	n.getParam("rightLaserfile",rightLaserfile);
	n.getParam("leftLaserfile",leftLaserfile);
	n.getParam("Dist2Center",Dist2Center);
	n.getParam("left_laser_min_angle",left_laser_min_angle);
	n.getParam("left_laser_max_angle",left_laser_max_angle);
	n.getParam("right_laser_min_angle",right_laser_min_angle);
	n.getParam("right_laser_max_angle",right_laser_max_angle);
	readSideLaserData();
	llaser_sub= n.subscribe<sensor_msgs::LaserScan>("/scanL",1, leftlaserCallBack);	 // /scan_base - org
	rlaser_sub= n.subscribe<sensor_msgs::LaserScan>("/scanR",1, rightlaserCallBack);
	scanL_pub = n.advertise<sensor_msgs::LaserScan>("scanLL", 10);
	scanR_pub = n.advertise<sensor_msgs::LaserScan>("scanRR", 10);

  ros::spin();

  return 0;
}



