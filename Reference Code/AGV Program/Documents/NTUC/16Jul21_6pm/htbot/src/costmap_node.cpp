/*
 * This is a costmap node 
 */
/* 	History
*		Date Modified :8.11.2018
*		Changes :
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include "boost/algorithm/string.hpp"
#include "htbot/debug.h"
#include <geometry_msgs/Pose.h>
#include "std_msgs/UInt16.h"
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

using namespace std;
using namespace boost::algorithm;

class CMap
{
public:
	CMap(ros::NodeHandle rn);	
	void publish_debug(string s);	
	void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
	void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void costmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
	void printCostMap(void);

	ros::Publisher debug_pub;
	ros::Subscriber pose_sub;
	ros::Subscriber costmap_sub;
	ros::Subscriber costmapUpd_sub;
	FILE *pfp;  // pose data file

	std::string homedir;
	
	double looprate;
	double px,py,pz,prx,pry,prz,prw;
	char costbuf [2600];
	char ncostbuf [50][50];
	int mwidth,mheight;
	bool printflag;

private:
	ros::NodeHandle nh;

};

CMap::CMap(ros::NodeHandle rn):
	looprate(1.0),mwidth(0),mheight(0),printflag(false)
{
	ros::NodeHandle sn(rn);
	debug_pub = sn.advertise<htbot::debug>("debug",100);
	pose_sub = sn.subscribe<geometry_msgs::Pose>("/robot_pose", 1, &CMap::poseCallback,this);
	costmap_sub = sn.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 1, &CMap::costmapCallback,this);
	costmapUpd_sub = sn.subscribe<map_msgs::OccupancyGridUpdate>("/move_base/local_costmap/costmap_updates", 1, &CMap::costmapUpdateCallback,this);
	nh.getParam("home_dir",homedir);
}

void CMap::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{	
	
	ros::Time time;
	double ox,oy;
	int dd,ix;
	
	int xx,yy,x,y;	
	
	mwidth = (int)msg->info.width;
	mheight = (int)msg->info.height;
	time = msg->info.map_load_time;
	ox = msg->info.origin.position.x;
	oy = msg->info.origin.position.y;
	ROS_INFO("Costmap Callback : width=%d. height=%d. x=%.2f. y=%.2f.",mwidth,mheight,ox,oy);
	ix = 0;
	for (x = 0; x < mwidth; x++) {	
    for (y = 0; y < mheight; y++) {
			//dd = (int)msg->data[x+ mwidth * y];
			dd = (int)msg->data[ix++];
			if ((dd == -1)) {
				// unknown
				ncostbuf[x][y] = '?';
			} else {
				if (dd >= 40) {
					// obs
					ncostbuf[x][y] = '*';
				} else {
					// clear
					ncostbuf[x][y] = '-';
				}
			}
		}
	}
	printflag = true;
}

void CMap::costmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
{
	int x,y,w,h;
	int xn,yn,dd,ix;
	x = msg->x;
	y = msg->y;
	w = msg->width;
	h = msg->height;
	if (x < 0 || y < 0) {
    ROS_INFO("negative coordinates, invalid update. x: %d, y: %d", x,y);
    return;
  }
	if ( (mheight == 0) || (mwidth == 0) ) {
		ROS_INFO("Costmap Error : width and height is zero");
		return;
	}
	xn = x + w;
	yn = y + h;
	if (xn > mwidth || x > mwidth || yn > mheight || y > mheight) {
    ROS_INFO("Costmap : received update doesn't fully fit into existing map. x=%d, xn=%d, y=%d, yn=%d. Map is w=%d. height=%d",x,xn,y,yn,mwidth,mheight);
  }
	ROS_INFO("Costmap_Update Callback : x=%d. y=%d. w=%d. h=%d",x,y,w,h);
	ix = 0;
	/*
	for (int i = x; i < xn && i < mwidth; i++) {	
    for (int j = y; j < yn && j < mheight; j++) {
			//dd = (int)msg->data[i + w * j];
			dd = (int)msg->data[ix++];
			if ((dd == -1)) {
				// unknown
				ncostbuf[i][j] = '?';
			} else {
				if (dd >= 30) {
					// obs
					ncostbuf[i][j] = '*';
				} else {
					// clear
					ncostbuf[i][j] = '-';
				}
			}
		}
	}
	*/
}

/*
void CMap::printCostMap(void) {
	int cost [2000];
	int ix;
	string s;
	FILE *fp;

	if ( (mheight == 0) || (mwidth == 0) ) {
		ROS_INFO("Costmap Error : width and height is zero. Cannot print");
		return;
	}
	ix = 0;
	costbuf[ix++] = '0';
	for (int i=0;i<mwidth;i++) {
		costbuf[ix++] = '\n';
		//costbuf[ix++] = (char)i;
		for (int j=0;j<mheight;j++) {
			costbuf[ix++] = ncostbuf[i][j];
		}
	}
	costbuf[ix++] = '2';
	s.assign(costbuf,strlen(costbuf));
	
	printf("%s\n",s.c_str());
	fp = fopen("/home/rac/catkin_ws/src/htbot/data/costmap.dat", "a");
	fprintf(fp,"%s\n",s.c_str());
	fclose(fp);	
}
*/

void CMap::printCostMap(void) {
	char cost [2600];
	int ix,tx,tt;
	string s;
	FILE *fp;

	if ( (mheight == 0) || (mwidth == 0) ) {
		ROS_INFO("Costmap Error : width and height is zero. Cannot print");
		return;
	}
	ix = 0;
	//costbuf[ix++] = '0';
	for (int i=0;i<mwidth;i++) {
		//if (i>0) {
		//	costbuf[ix++] = '\n';
		//}
		//costbuf[ix++] = (char)i;
		for (int j=0;j<mheight;j++) {
			costbuf[ix++] = ncostbuf[i][j];
		}
	}
	//costbuf[ix++] = '2';
	tx = ix;
	tt = 0;
	for (int k=0;k<ix && tx > 0;k++) {
		cost[tt++] = costbuf[--tx];
		if ((k>0) && ((k%mheight) == 0)) {
			cost[tt++]='\n';
		}
	}
	s.assign(costbuf,strlen(cost));
	
	printf("%s\n",s.c_str());
	fp = fopen("/home/rac/catkin_ws/src/htbot/data/costmap.dat", "a");
	fprintf(fp,"%s\n",s.c_str());
	fclose(fp);	
}


void CMap::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	double x,y,z,rz,rw;
	
	px = msg->position.x;
	py = msg->position.y;
	pz = msg->position.z;
	prx = msg->orientation.x;
	pry = msg->orientation.y;
	prz = msg->orientation.z;
	prw = msg->orientation.w;
	//ROS_INFO("Pose : x=%.2f. y=%.2f. rz=%.2f",x,y,rz);
}


void CMap::publish_debug(string s)
{
	htbot::debug deb;
	deb.msg = s;
	debug_pub.publish(deb);
	return;
}


 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "web node");  
  ros::NodeHandle rn;  	
	int cc;	
	ros::Rate loop_rate(10.0);
	CMap cNode(rn);
	sleep(3);
	cc = 0;

	while (true) {  	  	 	  
		cc++;
		if (cc == 20) {
			if (cNode.printflag) {
				cNode.printCostMap();
				cNode.printflag = false;
			}
			cc = 0;
		}
		ros::spinOnce();	
  	loop_rate.sleep();
  }
	
	//ros::MultiThreadedSpinner spinner(4);
  

  //spinner.spin();
  
  return 0;
}


