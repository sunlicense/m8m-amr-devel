/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Modified : 3.12.2014
*		Changes :
*/

//#define ODROID
#define SSMC
//#define RAC

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h> 
#include <fcntl.h>
#include <signal.h>
#include <string>
#include <sys/time.h>
#include <signal.h>
#include "htbot/move.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "boost/algorithm/string.hpp"
#include <tf/transform_datatypes.h>
#include <stdint.h>
#include <sensor_msgs/LaserScan.h>

#define MAX_RANGE 6.0
#define MIN_RANGE 0.05
#define	MAX_IDX	666 //500 //720
#define DEGPIDX 0.36 //0.25
#define RADPIDX 0.00628 //0.00436
#define CIDX 333 //250 //360  // total index for 240 (180) degree scan is 666

#define DEG2RAD(x) ((x)*ANGLE_RES*3.14159/180.0)
#define RAD2DEG(x) ((x)*180.0/3.14159)

using namespace std;

struct sockaddr_in serv_addr; 
int sock, csock, reuse = 1, blocking=1;
char recBuf[40];
char *sendBuff= new char[65536];
int FMPortNumber,FMRobotNum;
string FMAddress;
ros::Subscriber pose_sub;
ros::Subscriber map_sub;
ros::Publisher robot_pub;
ros::Publisher scan_pub;
ros::Publisher pose_pub;
sensor_msgs::LaserScan scan_msg;
geometry_msgs::Pose pose_msg;
double posex,posey,poserz,poserw;
boost::mutex publish_mutex_;
char* fmaddress;
struct timeval trcv, tsnd;
int map_width,map_height,map_size;
char map_data[50000];
double laser_data[MAX_IDX+10];
double yawp;
double Resolution,Window;
uint32_t lseq;
bool poseflag, mapflag;


// prototype
void sigpipe_handler(int sig0);
int connectFM();
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void publish_robot_pos();
void readCmdFromFM();
void sendInfoToFM();
void generateObsMap();

void sigpipe_handler(int sig0)
{
	ROS_INFO("SIGPIPE caught\n");
	connectFM();
}

int connectFM() {
	memset(recBuf, '0',sizeof(recBuf));
	if(sock > 0){
		shutdown(sock,SHUT_RDWR);
		close(sock);
	}
 	if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
  	ROS_INFO(" Error : Could not create socket: %d",sock);
    return -1;
  } 
 
  memset(&serv_addr, '0', sizeof(serv_addr)); 
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(FMPortNumber); 

  if(inet_pton(AF_INET, fmaddress , &serv_addr.sin_addr)<=0) {
  	ROS_INFO("\n inet_pton error occured\n");
    return -1;
  } 
  if( csock = connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
  	ROS_INFO("\n Error : Connect Failed; csock = %d \n", csock);
		return -1;
  } 

	trcv.tv_sec = 10;
	trcv.tv_usec = 0;
	tsnd.tv_sec = 10;
	tsnd.tv_usec = 0;

	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &trcv, sizeof(trcv));
	setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tsnd, sizeof(tsnd));
	ioctl(sock, FIONBIO, (unsigned long*) &blocking);
	return 0;
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	posex = msg->position.x;
	posey = msg->position.y;	
	poserz = msg->orientation.z;
	poserw = msg->orientation.w;
	//ROS_INFO("posex=%.3f",posex);
	poseflag = true;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	double x, y, z,rx,ry,rz,rw;
	int id,dat;
	boost::mutex::scoped_lock lock(publish_mutex_);
	map_width = msg->info.width;
	map_height = msg->info.height;
	x = msg->info.origin.position.x;
	y = msg->info.origin.position.y;
	z = msg->info.origin.position.z;
	rx = msg->info.origin.orientation.x;
	ry = msg->info.origin.orientation.y;
	rz = msg->info.origin.orientation.z;
	rw = msg->info.origin.orientation.w;
	map_size = msg->data.size();
	//memcpy(map_data, msg->data, size);
	for (int i=0;i<map_size;i++) {
		map_data[i] = (char)msg->data[i];
	}
	id = 80 + ((map_height - 80 - 1) * map_width);
	//dat = msg->data[id];
	dat = map_data[id];
	ROS_INFO("Map : Width = %d. Height = %d. Size of Data = %d. Data(80,80) = %d",map_width,map_height,map_size,dat);
	ROS_INFO("Map : x=%.3f. y=%.3f. z=%.3f. rx=%.3f. ry=%.3f. rz=%.3f. rw=%.3f. ",x,y,z,rx,ry,rz,rw);
	mapflag = true;
}

void publish_robot_pos()
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	htbot::move mv;
	mv.x = posex;
	mv.y = posey;
	robot_pub.publish(mv);
	return;
}

void readCmdFromFM() {

}

void sendInfoToFM() {

}
 
void publish_pose() {
	pose_msg.position.x = 1.7422;
	pose_msg.position.y = 2.5862;
	pose_msg.position.z = 0.0;
	pose_msg.orientation.x = 0.0;
	pose_msg.orientation.y = 0.0;
	pose_msg.orientation.z = -0.0049;
	pose_msg.orientation.w = 1.0000;
	pose_pub.publish(pose_msg);
}

void generateObsMap() {
	int pcx,pcy,widx;
	int x1,x2,y1,y2,id,dat,lidx;
	double an,anl,dist;
	tf::Quaternion qp(0.0,0.0,poserz,poserw);
	yawp = tf::getYaw(qp);
	// clear laser data
	for (int i=0;i<MAX_IDX;i++) {
		laser_data[i] = MAX_RANGE;
	}
	// cells of robot pose
	//ROS_INFO("Here A");
	pcx = (int)(posex / Resolution);
	pcy = (int)(posey / Resolution);
	pcy = map_height - pcy;
	widx = (int)(Window / Resolution);
	x1 = pcx - widx;
	x2 = pcx + widx;
	if (x1 < 0) {
		x1 = 0;
	}
	if (x2 > map_width) {
		x2 = map_width;
	}
	y1 = pcy - widx;
	y2 = pcy + widx;
	if (y1 < 0) {
		y1 = 0;
	}
	if (y2 > map_height) {
		y2 = map_height;
	}
	//ROS_INFO(" ================ Start Cycle =======================");
	//ROS_INFO("x=%.3f. y=%.3f. Res=%.3f. widx=%d. x1=%d. x2=%d. y1=%d. y2=%d",posex,posey,Resolution,widx,x1,x2,y1,y2);
	for (int r=y1;r<y2;r++) {
		for (int c=x1;c<x2;c++) {
			id = c + ((map_height - r - 1) * map_width);
			dat = map_data[id];
			//ROS_INFO("Map : dat=%d. r=%d. c=%d",dat,r,c);
			//if (dat > 80) {
			if (dat == -1) {
				// obs. calculate angle of obs.
				if ((r < pcy) && (c > pcx)) {
					// left-top of robot. 1Q. angle is positive rad
					an = atan((pcy - r)*1.0/(c - pcx)*1.0);
				} else {
					if ((r >= pcy) && (c > pcx)) {
						// top-right of robot. 2Q angle is negative rad
						an = -atan((r - pcy)*1.0/(c - pcx)*1.0);		
					}	else {
						if ((r >= pcy) && (c <= pcx)) {
							// bottom-right of robot. 3Q. angle is negative rad
							an = -3.1416 + atan((r - pcy)*1.0/(pcx - c)*1.0);	
						} else {
							// bottom-left of robot. 4Q. angle is positive rad
							an = 3.1416 - atan((pcy-r)*1.0/(pcx - c)*1.0);	
						}
					}		
				}
				// angle/index of obs reference to laser axis
				anl = an - yawp;
				//if ((anl > 1.5709) || (anl < -1.5709)) {
				if ((anl > 2.0923) || (anl < -2.0923)) {
					// infront of robot only
					continue;
				}
				lidx = CIDX + (int)(anl / RADPIDX);
				// dist of obs to laser. assume map axis is pointing along left to right of map
				dist = sqrt((r-pcy)*(r-pcy)*0.0025 + (c-pcx)*(c-pcx)*0.0025);
				//ROS_INFO("***** obs. dist=%.3f. idx=%d  ******",dist,lidx);
				if (dist < laser_data[lidx]) {
					laser_data[lidx] = dist;
				}
			}
		}
	}
	// publis laser scan
	//ROS_INFO("Here B");
	scan_msg.ranges.resize(MAX_IDX);
	//scan_msg.header.seq = lseq++;
	//if (lseq > UINT32_MAX) {
	//	lseq = 0;
	//}
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "/maplaser";
	scan_msg.angle_min = -2.0923; //-1.5708;
  scan_msg.angle_max = 2.0923; //1.5708;
	scan_msg.angle_increment = 4.1846 / MAX_IDX;  //3.1416 / MAX_IDX;
  scan_msg.time_increment = (1.0 / 10.0) / (MAX_IDX);
  scan_msg.range_min = MIN_RANGE;
  scan_msg.range_max = MAX_RANGE;
	for (int i=0;i<MAX_IDX;i++) {
		scan_msg.ranges[i] = laser_data[i];
	}
	//ROS_INFO("Here C");
	scan_pub.publish(scan_msg);
	//ROS_INFO("Here D");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Fleet Manager Server");
  ros::NodeHandle n;
	char buf [30];
	int ret;
	ros::Rate loop_rate(10.0);
	pose_sub = n.subscribe<geometry_msgs::Pose>("/robot_pose", 1 ,poseCallback);
	map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1 ,mapCallback);
	scan_pub = n.advertise<sensor_msgs::LaserScan>("mapscan", 10);
	pose_pub = n.advertise<geometry_msgs::Pose>("/robot_pose", 10);

	n.param<std::string>("FMAddress",FMAddress, "192.168.1.120");
	n.param("FMPortNumber",FMPortNumber,5000);
	n.param("FMRobotNum",FMRobotNum,0);
	n.param("Resolution",Resolution,0.05);
	n.param("Window",Window,3.0);
	poseflag = false;
	mapflag = false;
	fmaddress = const_cast<char*>(FMAddress.c_str());
	/*
	while (true) {
		ret = connectFM();
		if (ret == 0) {
			ROS_INFO("Connected to FM.");
			break;
		}
		ROS_INFO("Failed to connect to FM. reTry...");
		sleep(10);
	}
	*/
	signal(SIGPIPE,sigpipe_handler);
	while (true) {  	  	 			
		//sendInfoToFM();
		//readCmdFromFM();
		//publish_pose();
		if (poseflag && mapflag) {
			generateObsMap();
		}
		ros::spinOnce();	
  	loop_rate.sleep();
  }
	
  //ros::spin();

  return 0;
}



