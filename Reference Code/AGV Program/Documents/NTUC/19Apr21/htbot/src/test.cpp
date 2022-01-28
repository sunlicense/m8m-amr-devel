#include "boost/algorithm/string.hpp"
#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <ros/ros.h>
#include "htbot/angle.h"
#include "htbot/sound.h"
#include "htbot/PLAYSOUND.h"
#include <dirent.h>
#include <stdlib.h>
#include <string.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.14159

using namespace std;
using namespace boost::algorithm;

double sysruntime,sysidletime,sysdowntime,sysrecordtime,systriptime,totaltriptime,totalruntime;
int tm_hour,tm_min,tm_year,tm_mon,tm_mday;
struct tm dashStartTime;
FILE *tfp;  
ros::Time initialstartTime;
ros::Duration totalTime;
int startTime;
ros::Publisher play_pub;
string maparray[10];
double **cleanInfo ;

void calcQuat(double sx, double sy,double ex, double ey, double *qx,double *qy, double *qz, double *qw)
{
	double an,diff;
	tf2::Quaternion quat;
	geometry_msgs::Quaternion qq;
	double x,y,z,w;
	if ( (ex > sx) && (ey > sy)) {
		// Q1
		an = atan((ey-sy)/(ex-sx));
	} else {
		if ( (sx > ex) && (ey > sy) ) {
			// Q2
			an = atan((ey-sy)/(sx-ex));
			an = PI - an;
		} else {
			if ( (sx > ex) && (sy > ey) ) {
				// Q3
				an = atan((sy-ey)/(sx-ex));
				an = -(PI - an);
			} else {
				if ( (ex > sx) && (sy > ey) ) {
					// Q4
					an = -(atan((sy-ey)/(ex-sx)));
				} else {
					// left y axis
					if ((ex == sx) && (ey > sy)) {
						an = PI / 2.0;
					} else {
						// lower x axis
						if ((ey == sy) && (sx > ex)) {
							an = PI;
						} else {
							// right y axis
							if ((ex == sx) && (sy > ey)) {
								an = -(PI / 2.0);
							} else {
								// upper x axis
								if ((ey == sy) && (ex > sx)) {
									an = 0.0;
								} else {
									an = 0.0;
								}
							}
						}
					}
				}
			}
		}
	}
	quat.setRPY( 0.0, 0.0,an);
	quat.normalize();
	qq = tf2::toMsg(quat);
	*qx = qq.x;
	*qy = qq.y;
	*qz = qq.z;
	*qw = qq.w;
	return;
}

void loadCleanPlanPath() {

	FILE *fp;	
	int ret,d;	
	std::string cpp,s1,s2;
	double x,y,x1,y1,x2,y2,dx,dy,ddist,status;
	double cx,cy,cz,crx,cry,crz,crw;
	double sx,sy,sz,srx,sry,srz,srw;
	double ex,ey,ez,erx,ery,erz,erw;
	double dsx,dsy,dsz,dsrx,dsry,dsrz,dsrw;
	int numcleanpoints;

	fp = fopen("/home/rac/globotix.txt", "r");
  if (fp == NULL) {
		ROS_INFO("----- Test : I couldn't load for reading Cleaning Plan. -----");  	
  	return;
  }
	ROS_INFO("-------------- Here A ------------------------");
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &sx,&sy,&sz,&srx,&sry,&srz,&srw); // start position
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &ex,&ey,&ez,&erx,&ery,&erz,&erw); // end position
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &dsx,&dsy,&dsz,&dsrx,&dsry,&dsrz,&dsrw); // docking station position

	// start of cleaning plan path
	ROS_INFO("-------------- Here B ------------------------");
	d = 0;
	numcleanpoints = 0;
	while(true) {
		if (fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf\n",&cx,&cy,&cz,&crx,&cry,&crz,&crw,&status) == EOF) {
			break;
		}
		d++;
	}
	numcleanpoints = d;
	ROS_INFO("-------------- Here C : %d------------------------",numcleanpoints);
	cleanInfo = (double **)malloc(numcleanpoints * sizeof(double *));
	ROS_INFO("-------------- Here D ------------------------");
	for (int i=0;i<numcleanpoints;i++) {
		cleanInfo[i] = (double *)malloc(10 * sizeof(double));
		ROS_INFO("-------------- Here E : %d------------------------",i);
	}
	fclose(fp);

	fp = fopen("/home/rac/globotix.txt", "r");
  if (fp == NULL) {
		ROS_INFO("----- Test : I couldn't load for reading Cleaning Plan. -----");  	
  	return;
  }
	
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &sx,&sy,&sz,&srx,&sry,&srz,&srw); // start position
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &ex,&ey,&ez,&erx,&ery,&erz,&erw); // end position
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &dsx,&dsy,&dsz,&dsrx,&dsry,&dsrz,&dsrw); // docking station position
	ROS_INFO("----- Start : x=%.2f. y=%.2f. z=%.2f. rx=%.2f. ry=%.2f. rz=%.2f. rw=%.2f. --------",sx,sy,sz,srx,sry,srz,srw);
	ROS_INFO("----- end : x=%.2f. y=%.2f. z=%.2f. rx=%.2f. ry=%.2f. rz=%.2f. rw=%.2f. --------",ex,ey,ez,erx,ery,erz,erw);

	d = 0;
	ROS_INFO("-------------- Here F ------------------------");
	while(true) {
		if (fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf\n",&cx,&cy,&cz,&crx,&cry,&crz,&crw,&status) == EOF) {
			break;
		}
		ROS_INFO("----- p=%d : x=%.2f. y=%.2f. z=%.2f. rx=%.2f. ry=%.2f. rz=%.2f. rw=%.2f. st=%.2f. --------",d,cx,cy,cz,crx,cry,crz,crw,status);
		cleanInfo[d][0] = cx;
		cleanInfo[d][1] = cy;
		cleanInfo[d][2] = cz;
		cleanInfo[d][3] = crx;
		cleanInfo[d][4] = cry;
		cleanInfo[d][5] = crz;
		cleanInfo[d][6] = crw;
		cleanInfo[d][7] = 0.0;  // dist to next point
		cleanInfo[d][8] = status;  // 0.0 = same lane. 1.0 = different lane. 
		cleanInfo[d][9] = 0.0;  // 0.0=not clean. 1.0= cleaned
		d++;
	}
	numcleanpoints = d;
	for (int i=0; i<d-1; i++) {
		x = cleanInfo[i][0];
		y = cleanInfo[i][1];
		x1 = cleanInfo[i+1][0];
		y1 = cleanInfo[i+1][1];		
		dx = x1 - x;
		dy = y1 - y;
		ddist = sqrt((dx * dx) + (dy * dy));
		cleanInfo[i][7] = ddist;
		if (cleanInfo[i][8] > 0.0) {
			// turning point
			if (i > 0) {
				x2 = cleanInfo[i-1][0];
				y2 = cleanInfo[i-1][1];
				calcQuat(x2,y2,x,y,&crx,&cry,&crz,&crw);
				cleanInfo[i][3] = crx;
				cleanInfo[i][4] = cry;
				cleanInfo[i][5] = crz;
				cleanInfo[i][6] = crw;
			} 			
		}		
	}
	for (int i=0; i<numcleanpoints; i++) {
		cx = cleanInfo[i][0];
		cy = cleanInfo[i][1];
		cz = cleanInfo[i][2];
		crx = cleanInfo[i][3];
		cry = cleanInfo[i][4];
		crz = cleanInfo[i][5];
		crw = cleanInfo[i][6];
		dx = cleanInfo[i][7];
		status = cleanInfo[i][8];
		ROS_INFO("---- p=%d : x=%.2f. y=%.2f. z=%.2f. rx=%.2f. ry=%.2f. rz=%.2f. rw=%.2f. dd=%.2f. st=%.2f. -----",i,cx,cy,cz,crx,cry,crz,crw,dx,status);
	}
	
	ROS_INFO("-------------------- Test : End of Loading Cleaning Plan --------------------------------");
  fclose(fp);	
}

void diffinTime()
{
  time_t now;	
	time(&now);
	sysrecordtime = difftime(now,mktime(&dashStartTime));
	sysrecordtime = sysrecordtime / 60.0;  // mins
	ROS_INFO("sysrecordtime:%.3f",sysrecordtime);
}

void readTimeDatafromFile() 
{		
	int ret;
	tfp = fopen("/home/rac/catkin_ws/src/htbot/data/time.dat", "r");
  if (tfp == NULL) {
  	ROS_INFO("I couldn't open time.dat for reading.\n");    
  	return;
  }

	ret = fscanf(tfp,"%lf\n",&sysruntime);
	ret = fscanf(tfp,"%lf\n",&sysidletime);
	ret = fscanf(tfp,"%lf\n",&sysdowntime);
	ret = fscanf(tfp,"%lf\n",&systriptime);
	ret = fscanf(tfp,"%d\n",&tm_year);
	ret = fscanf(tfp,"%d\n",&tm_mon);
	ret = fscanf(tfp,"%d\n",&tm_mday);
	ret = fscanf(tfp,"%d\n",&tm_hour);
	ret = fscanf(tfp,"%d\n",&tm_min);
	totaltriptime = systriptime;
	ROS_INFO("System Time : runtime=%.2f. idletime=%.2f. downtime=%.2f. triptime=%.3f. year=%d. mon=%d. day=%d. hr=%d. min=%d",sysruntime,sysidletime,sysdowntime,systriptime,tm_year,tm_mon,tm_mday,tm_hour,tm_min);
	
  fclose(tfp);	
}

void saveTimeData() {
	tfp = fopen("/home/rac/catkin_ws/src/htbot/data/time.dat", "r+");  // r+ : open existing file to r/w
  if (tfp == NULL) {
  	ROS_INFO("I couldn't open time.dat to write.\n");    
  	return;
  }
	
	fprintf(tfp,"%.3f\n",(sysruntime + totalruntime));
	fprintf(tfp,"%.3f\n",sysidletime);
	fprintf(tfp,"%.3f\n",sysdowntime);
	fprintf(tfp,"%.3f\n",totaltriptime);
	fprintf(tfp,"%d\n",tm_year);
	fprintf(tfp,"%d\n",tm_mon);
	fprintf(tfp,"%d\n",tm_mday);
	fprintf(tfp,"%d\n",tm_hour);
	fprintf(tfp,"%d\n",tm_min);
	fclose(tfp);
}

void initialiseTimeData() 
{		
	time_t rawtime;
  //ros::NodeHandle rn;  
		

	//rn.getParam("startTime",startTime);	
  time( &rawtime );
	startTime = 0;
  dashStartTime = *localtime( &rawtime );
	if (startTime != 1) {
		ROS_INFO("Here B");
		dashStartTime.tm_year = tm_year;
		dashStartTime.tm_mon = tm_mon;
		dashStartTime.tm_mday = tm_mday;
		dashStartTime.tm_hour = tm_hour;
		dashStartTime.tm_min = tm_min;
		dashStartTime.tm_sec = 0;
	} else {
		ROS_INFO("Here A");
		tm_year = dashStartTime.tm_year;
		tm_mon = dashStartTime.tm_mon;
		tm_mday = dashStartTime.tm_mday;
		tm_hour = dashStartTime.tm_hour;
		tm_min = dashStartTime.tm_min;
	}
	ros::Time::init();
	initialstartTime = ros::Time::now();
	ROS_INFO("DashBoard Start Time : year=%d. month=%d. day=%d. hr=%d. min=%d",tm_year,tm_mon,tm_mday,tm_hour,tm_min);
	
}

void compute_time()
{
	diffinTime();
	totalTime = ros::Time::now() - initialstartTime;
	totalruntime = totalTime.toSec() / 60.0; // mins
	sysdowntime = sysrecordtime - (sysruntime + totalruntime);
	//sysidletime = (sysruntime + totalruntime) - ((totaltriptime+runningtriptime)/60.0);
}

void publish_sound(int id,int sd, int rsd)
{
	htbot::sound cmd;
	cmd.id = id;
	cmd.startdelay = sd;
	cmd.restartdelay = rsd;
	play_pub.publish(cmd);
	//ROS_INFO("Publish Sound AA");
	return;
}

const char *get_filename_ext(const char *filename) {
    const char *dot = strrchr(filename, '.');
    if(!dot || dot == filename) return "";
    return dot + 1;
}

char *remove_ext(char* myStr, char extSep, char pathSep) {
    char *retStr, *lastExt, *lastPath;

    // Error checks and allocate string.
    if (myStr == NULL) return NULL;
    if ((retStr = (char *)malloc (strlen (myStr) + 1)) == NULL) return NULL;

    // Make a copy and find the relevant characters.
    strcpy (retStr, myStr);
    lastExt = strrchr (retStr, extSep);
    lastPath = (pathSep == 0) ? NULL : strrchr (retStr, pathSep);

    // If it has an extension separator.

    if (lastExt != NULL) {
        // and it's to the right of the path separator.

        if (lastPath != NULL) {
            if (lastPath < lastExt) {
                // then remove it.

                *lastExt = '\0';
            }
        } else {
            // Has extension separator with no path separator.

            *lastExt = '\0';
        }
    }
    // Return the modified string.
    return retStr;
}

void readfiles(void) 
{
	DIR* FD;
  struct dirent* in_file;
  char    *s, *r;
	FILE *entry_file;
	int ret,ret1;
	string ss;

	/* Scanning the in directory */
  if (NULL == (FD = opendir ("/home/rac/catkin_ws/src/htbot/cleanplan"))) 
  {
  	ROS_INFO("----- Test : Failed to open cleanplan directory\n");
    return;
  }

	while ((in_file = readdir(FD))) 
  {
  	// On linux/Unix we don't want current and parent directories
    // If you're on Windows machine remove this two lines   
    if (!strcmp (in_file->d_name, "."))
    	continue;
    if (!strcmp (in_file->d_name, ".."))    
      continue;
    //Open directory entry file for common operation     
    entry_file = fopen(in_file->d_name, "a");
		if (entry_file == NULL)
    {
    	ROS_INFO("---- Test : Failed to open cleanplan file : %s  -------",in_file->d_name);    
      continue;
    }
		ret = strcmp (get_filename_ext(in_file->d_name), "jpeg");
		ret1 = strcmp (get_filename_ext(in_file->d_name), "jpg");
		if ((ret == 0) || (ret1 == 0)) {			
			// check for "map" or "clean"
			s = remove_ext(in_file->d_name,'.','/');
			r = strstr(s,"map");
			if (r != NULL) {
				ROS_INFO("----- Test : JPEG/Map File = %s -------",s);
				ss.assign(s,strlen(s));
				maparray[0] = ss;
			} else {
				r = strstr(s,"clean");
				if (r != NULL) {
					ROS_INFO("----- Test : JPEG/Clean File = %s -------",s);
				}
			}
		} else {
			ret = strcmp (get_filename_ext(in_file->d_name), "dat");
			if ((ret == 0)) {
				ROS_INFO("----- Test : DAT/Cleaning Plan File -------");
			}
		}
		
	}
	if (s != NULL) {
		free(s);
	}
	return;
}

int main(int argc, char **argv)
{
		
    char arr[50],len;
    int count=0;
		const char *cstr ;
		double diff;
		ros::init(argc, argv, "test");
		ros::NodeHandle nh;
		int ret;

		//diff = angles::shortest_angular_distance(1.5,2.5 );
		//ROS_INFO("@@@@@ test 1.5 :  diff=%.3f  @@@@@@@",diff);

		//diff = angles::shortest_angular_distance(1.5,0.5 );
		//ROS_INFO("@@@@@ test -1.5 :  diff=%.3f  @@@@@@@",diff);
		
		std::string s1,s2;
		
		//readfiles();
		loadCleanPlanPath();

    //readTimeDatafromFile();
		//initialiseTimeData();
		
		//sleep(300);
		//compute_time();
		//saveTimeData();
		//play_pub = nh.advertise<htbot::sound>("sound", 5);
		
		//sleep(3);
		//publish_sound(WAFESTART,0,3);
		
		//sleep(4);
		/*
		publish_sound(ESTOPREL,0,0);
		sleep(4);
		publish_sound(BATLOW,0,0);
		sleep(4);
		publish_sound(ESTOP,0,0);
		sleep(4);
		publish_sound(REACHED,0,0);
		sleep(4);
		publish_sound(FINDPATH,0,0);
		sleep(4);
		publish_sound(MOVEOUTDOCKSTATION,0,0);

		sleep(4);
		publish_sound(MOVETODOCK,0,0);
		sleep(4);
		publish_sound(NEXTJOB,0,0);
		sleep(4);
		publish_sound(RETURNTOCHARGE,0,0);
		sleep(4);
		publish_sound(SHUTDOWN,0,0);
		sleep(4);
		*/
		//publish_sound(STOCKER,0,3);
		//sleep(5);
		//publish_sound(WAFESTART,0,3);
		//sleep(5);
		//publish_sound(WAFESTART,0,3);
		//sleep(3);
		//ret = system("rosrun rviz rviz &");
		return 0;
}
