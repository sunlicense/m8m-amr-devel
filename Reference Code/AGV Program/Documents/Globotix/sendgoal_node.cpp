/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Modified : 13.10.2020
*		Changes :
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "htbot/move.h"
#include "htbot/move_status.h"
#include "htbot/status.h"
#include "boost/algorithm/string.hpp"
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include "htbot/clear.h"
#include "htbot/debug.h"
#include "htbot/sound.h"
#include "htbot/PLAYSOUND.h"
#include "htbot/goal.h"
#include <nav_msgs/Path.h>
#include "htbot/angle.h"
#include "htbot/clear.h"
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace std;

#define STEP 0.05   //0.15
#define DIFF 0.08
#define DIFFRAD 0.025
#define DIA 0.2
#define PI 3.14159
#define ENGAGE 1
#define DISENGAGE 4
#define PREMOVE 9  // pre move
#define POSTMOVE 10
#define TURNTOPATH 20 //
#define TURNTOPATHMOVE 21
#define LOCALISEMOVE 22
#define MAXLP 200
#define MAXPP 50
#define MAXCP 200
#define CLEANPLAN 23
#define LOADCLEANPLAN 24
#define STOPCLEANPLAN 25
#define PAUSECLEANPLAN 26
#define RESUMECLEANPLAN 27
#define SENDCLEANINGREPORT 28
#define CLEANLEFTOVER 29
#define CLEANPLANRAC 30  // cleaning plan using rac global planner
#define RACPATHMOVE 50

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Subscriber move_cmd;
ros::Publisher status_pub;
ros::Publisher debug_pub;
ros::Publisher status_mov;
ros::Publisher pos_pub;
ros::Subscriber movx_sub;
ros::Subscriber pose_sub;
ros::Publisher goalLP_pub;
ros::Publisher play_pub;
ros::Publisher chkplan_pub;
ros::Subscriber cleanPlan_sub;
ros::Subscriber movstat_sub;
ros::Publisher clear_pub;
ros::Publisher event_pub;

double rz,rw,angle;
bool laser_use;
int xcount;
int stop_move;
double fmove,ffmove,mdist,offset,toffset,speed;
bool bootup;
double align_method;
ros::Time goalTime;
bool Cancel_Nav,startGoal,sendGoal;
double px,py,pz,prx,pry,prz,prw;
double gx,gy,gz,grx,gry,grz,grw,igx,igy,cgx,cgy,rpx,rpy;
double ngx,ngy;
boost::mutex mut;
double poseInfo[MAXLP][17];	
double pathInfo[MAXLP][MAXLP][MAXPP][7];
//double cleanInfo[MAXLP][10];
double *cleanInfo;
double cleanStatus[MAXCP][2];
int pathNoPoint[MAXLP][MAXLP];	
std::string pathfile;
int pathmaxpoint,startLP,endLP,idxLP;
bool multiLP,angleTurn, lastLP,lastCP;
double gapdist,gapLimit;
double gxn,gyn;
double checkPlanSum,checkPlanX,checkPlanY,checkPlanXN,checkPlanYN,checkPlanDD;
double checkPlanRX,checkPlanRY,checkPlanRZ,checkPlanRW;
int movedone,moveflag;
bool enableMultiPoint,cleanplan,cpturnpt,recleaning,recleandone,recleannextpt;
int numcleanpoints,cpixs,cpixe,cpcix;
double tpflag,TPGAPLIMIT,CPGAPLIMIT,RCGAPLIMIT,CPLOOKAHEAD,CLOSELIMIT;
double sx,sy,sz,srx,sry,srz,srw;
double ex,ey,ez,erx,ery,erz,erw;
double dsx,dsy,dsz,dsrx,dsry,dsrz,dsrw;
bool racpathhold,nextsegment;

//move_base_msgs::MoveBaseGoal goal;
//MoveBaseClient ac("move_base", true); // move_base_nys

// prototype
void publish_status(string s);
void publish_debug(string s);
void publish_move_status(int stat);
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
void init_LP(void);
void readPathfromFile();
bool GoalLPpub(double x,double y, double rz, double rw);
void publish_sound(int id,int sd,int rsd);
void checkpath(double x,double y);
void cleanPlanCallback(const nav_msgs::Path::ConstPtr& msg);
void movStatusCallback(const htbot::move_status::ConstPtr& msg);
double ptpAngle(void);
double checkangle_NLP();
void publish_clear(void);
void publish_event(string s);
void test(void);
void loadCleanPlanPath();
void checkCleaningStatus();
void CleaningReport(double percent);
bool move(const htbot::move::ConstPtr& msg);
void calcQuat(double sx, double sy,double ex, double ey, double *qx,double *qy, double *qz, double *qw);

// ---------------------------------------------------------------------------------------------//

void calcQuat(double sx, double sy,double ex, double ey, double *qx,double *qy, double *qz, double *qw)
{
	double an,diff;
	tf2::Quaternion quat;
	geometry_msgs::Quaternion qq;
	double x,y,z,w;
	if ( (ex >= sx) && (ey >= sy)) {
		// Q1
		an = atan((ey-sy)/(ex-sx));
	} else {
		if ( (sx >= ex) && (ey >= sy) ) {
			// Q2
			an = atan((ey-sy)/(sx-ex));
			an = PI - an;
		} else {
			if ( (sx >= ex) && (sy >= ey) ) {
				// Q3
				an = atan((sy-ey)/(sx-ex));
				an = -(PI - an);
			} else {
				// Q4
				an = -(atan((sy-ey)/(ex-sx)));
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

void checkCleaningStatus() {
	ros::NodeHandle n; 
	char buf [200];
	string s1;
	double x1,x2,x3,x4,y1,y2,y3,y4,dd;
	int ccnt;

	sprintf(buf,"----- Sgoal : Cleaning Plan Status : Path points=%d ------------",cpcix);
	s1.assign(buf,strlen(buf));
	publish_event(s1);
	for(int i=0;i<cpcix-3;i=i+4) {
		x1 = cleanStatus[i][0];
		y1 = cleanStatus[i][1];
		x2 = cleanStatus[i+1][0];
		y2 = cleanStatus[i+1][1];
		x3 = cleanStatus[i+2][0];
		y3 = cleanStatus[i+2][1];
		x4 = cleanStatus[i+3][0];
		y4 = cleanStatus[i+3][1];
		sprintf(buf,"----- x=%.2f. y=%.2f ----- x=%.2f. y=%.2f ----- x=%.2f. y=%.2f ----- x=%.2f. y=%.2f -----",x1,y1,x2,y2,x3,y3,x4,y4);
		s1.assign(buf,strlen(buf));
		publish_event(s1);
	}
	for(int i=0;i<cpcix;i++) {
		x1 = cleanStatus[i][0];
		y1 = cleanStatus[i][1];
		for(int j=0;j<numcleanpoints;j++) {
			// if (cleanInfo[j][9] > 0.0) {
			if (*(cleanInfo + j*10 + 9) > 0.0) {  
				continue;
			}
			x2 = cleanInfo[j][0];
			y2 = cleanInfo[j][1];
			x3 = x1 - x2;
			y3 = y1 - y2;
			dd = sqrt((x3 * x3) + (y3 * y3));
			if (dd < CLOSELIMIT) {
				cleanInfo[j][9] = 1.0;
				break;
			}
		}
	}
	ccnt = 0;
	publish_event("\n--------- Cleaning Status Report : Start--------------------\n");
	for(int j=0;j<numcleanpoints;j++) {
		x1 = cleanInfo[j][0];
		y1 = cleanInfo[j][1];
		if (cleanInfo[j][9] > 0.0) { 			
			sprintf(buf,"-------- Clean Point Cleaned = %d. x=%.3f y=%.3f ----------",j,x1,y1);			
			ccnt++;
		} else {
			sprintf(buf,"-------- Clean Point Not Cleaned = %d. x=%.3f y=%.3f ----------",j,x1,y1);
		}
		s1.assign(buf,strlen(buf));
		publish_event(s1);
	}
	x1 = (ccnt / (double)numcleanpoints) * 100.0;
	ROS_INFO(" ********************** Percentage Cleaned = %.4f. ccnt=%d. tot=%d ***************",x1,ccnt,numcleanpoints);
	sprintf(buf," ********************** Percentage Cleaned = %.4f. ccnt=%d. tot=%d ***************",x1,ccnt,numcleanpoints);
	s1.assign(buf,strlen(buf));
	publish_event(s1);
	publish_event("\n--------- Cleaning Status Report : End--------------------\n");
	n.setParam("cleaningStatus",30);
	CleaningReport(x1);
}

void publish_clear(void)
{
	htbot::clear cmd;
	cmd.cmd = 1;
	ROS_INFO("-------------------- Sendgoal Node : publise clear Map -----------------");
	clear_pub.publish(cmd);
	return;
}

void publish_event(string s)
{
	htbot::debug status;
	status.msg = s;
	event_pub.publish(status);
	return;
}

double ptpAngle(double nx, double ny, double ox, double oy)
{
	double an;
	if ( (nx >= ox) && (ny >= oy)) {
		// Q1
		an = atan((ny-oy)/(nx-ox));
	} else {
		if ( (ox >= nx) && (ny >= oy) ) {
			// Q2
			an = atan((ny-oy)/(ox-nx));
			an = PI - an;
		} else {
			if ( (ox >= nx) && (oy >= ny) ) {
				// Q3
				an = atan((oy-ny)/(ox-nx));
				an = -(PI - an);
			} else {
				// Q4
				an = -(atan((oy-ny)/(nx-ox)));
			}
		}
	}
	return an;
}

double checkangle_NLP()
{
	double ann,ano,anr;
	ano = ptpAngle(gx,gy,px,py); // find angle to goal
	ann = ptpAngle(ngx,ngy,gx,gy); // find angle to goal to next goal
	anr = angles::shortest_angular_distance(ano, ann);
	return fabs(anr);
}

void checkpath(double x, double y) {

	htbot::goal msg;
	msg.x = x;
	msg.y = y;
	chkplan_pub.publish(msg);
	return;
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

void cleanPlanCallback(const nav_msgs::Path::ConstPtr& plan)
{
	int size;
	double ax,ay,az,arx,ary,arz,arw;
	size = plan->poses.size();
	ROS_INFO(" ---------- SendGoal : Received Cleaning Plan size = %d --------------",size);
		
	for (int i=0;i<size;i++) {
		pathInfo[0][8][i][0] = plan->poses[i].pose.position.x;
		pathInfo[0][8][i][1] = plan->poses[i].pose.position.y;
		pathInfo[0][8][i][2] = plan->poses[i].pose.position.z;
		pathInfo[0][8][i][3] = plan->poses[i].pose.orientation.x;
		pathInfo[0][8][i][4] = plan->poses[i].pose.orientation.y;
		pathInfo[0][8][i][5] = plan->poses[i].pose.orientation.z;
		pathInfo[0][8][i][6] = plan->poses[i].pose.orientation.w;
		ax = plan->poses[i].pose.position.x;
		ay = plan->poses[i].pose.position.y;
		az = plan->poses[i].pose.position.z;
		arx = plan->poses[i].pose.orientation.x;
		ary = plan->poses[i].pose.orientation.y;
		arz = plan->poses[i].pose.orientation.z;
		arw = plan->poses[i].pose.orientation.w;
		ROS_INFO("------ SendGoal : clean plan . x=%.3f. y=%.3f. z=%.3f. --------",ax,ay,az);
	}
	pathNoPoint[0][8] = size;
}

void readPathfromFile() 
{		
	char buf[50];
	int slp,elp,stn,d;
	std::string s1;
	FILE *pfp;
	double tx,ty,tz,rx,ry,rz,rw;
	double tx1,ty1,tz1,rx1,ry1,rz1,rw1;

	pfp = fopen(pathfile.c_str(), "r");
  if (pfp == NULL) {
  	ROS_INFO("----- SendGoal : I couldn't open pathdata.dat for reading. ---------\n");    
  	return;
  }
	for (int i=0;i<MAXLP;i++) {
		for (int j=0;j<MAXLP;j++) {
			pathNoPoint[i][j] = 0;
		}
	}
	
	while(true) {
		if (fscanf(pfp,"%d %d %lf %lf %lf %lf %lf %lf %lf\n",&slp,&elp,&tx,&ty,&tz,&rx,&ry,&rz,&rw) == EOF) {
			break;
		}
		d = pathNoPoint[slp][elp];
		pathInfo[slp][elp][d][0] = tx;
		pathInfo[slp][elp][d][1] = ty;
		pathInfo[slp][elp][d][2] = tz;
		pathInfo[slp][elp][d][3] = rx;
		pathInfo[slp][elp][d][4] = ry;
		pathInfo[slp][elp][d][5] = rz;
		pathInfo[slp][elp][d][6] = rw;

		//tx1 = pathInfo[slp][elp][d][0];
		//ty1 = pathInfo[slp][elp][d][1];
		//tz1 = pathInfo[slp][elp][d][2];
		//rx1 = pathInfo[slp][elp][d][3];
		//ry1 = pathInfo[slp][elp][d][4];
		//rz1 = pathInfo[slp][elp][d][5];
		//rw1 = pathInfo[slp][elp][d][6];		
		//ROS_INFO("---- BotNode : Path %d to %d. pp=%d : x=%.3f.y=%.3f.z=%.3f.rx=%.3f.ry=%.3f.rz=%.3f.rw=%.3f---",slp,elp,d,tx1,ty1,tz1,rx1,ry1,rz1,rw1);
		d++;
		pathNoPoint[slp][elp] = d;
		//pathNoPoint[slp][elp] = 0;
	}
  fclose(pfp);
}

void init_LP(void) 
{
	int d;
	poseInfo[0][0] = 19.8705;
	poseInfo[0][1] = 10.0820;
	poseInfo[0][2] = 0.0000;
	poseInfo[0][3] = 0.0000;
	poseInfo[0][4] = 0.0000;
	poseInfo[0][5] = -0.7039;
	poseInfo[0][6] = 0.7103;

	poseInfo[1][0] = 19.8914;
	poseInfo[1][1] = 8.5017;
	poseInfo[1][2] = 0.0000;
	poseInfo[1][3] = 0.0000;
	poseInfo[1][4] = 0.0000;
	poseInfo[1][5] = -0.5528;
	poseInfo[1][6] = 0.8333;		

	poseInfo[2][0] = 20.5113;
	poseInfo[2][1] = 7.1299;
	poseInfo[2][2] = 0.0000;
	poseInfo[2][3] = 0.0000;
	poseInfo[2][4] = 0.0000;
	poseInfo[2][5] = -0.6773;
	poseInfo[2][6] = 0.7357;

	poseInfo[3][0] = 20.5741;
	poseInfo[3][1] = 6.1238;
	poseInfo[3][2] = 0.0000;
	poseInfo[3][3] = 0.0000;
	poseInfo[3][4] = 0.0000;
	poseInfo[3][5] = -0.6855;
	poseInfo[3][6] = 0.7281;	  
	d = 4;
	poseInfo[d][0] = 20.7400;
	poseInfo[d][1] = 4.3340;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = -0.0583;
	poseInfo[d][6] = 0.9983;
	d = 5;
	poseInfo[d][0] = 22.4864;
	poseInfo[d][1] = 4.1007;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = -0.0010;
	poseInfo[d][6] = 1.0000;
	d = 6;
	poseInfo[d][0] = 24.1984;
	poseInfo[d][1] = 4.1017;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = 0.0451;
	poseInfo[d][6] = 0.9990;
	d = 7;
	poseInfo[d][0] = 26.2632;
	poseInfo[d][1] = 4.2943;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = 0.0037;
	poseInfo[d][6] = 1.0000;
	d = 8;
	poseInfo[d][0] = 34.9979;
	poseInfo[d][1] = 4.0726;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = 0.0083;
	poseInfo[d][6] = 1.0000;
	d = 9;
	poseInfo[d][0] = 38.7000;
	poseInfo[d][1] = 4.1885;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = 0.0034;
	poseInfo[d][6] = 1.0000;
	d = 10;
	poseInfo[d][0] = 43.8565;
	poseInfo[d][1] = 4.2472;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = 0.0013;
	poseInfo[d][6] = 1.0000;
	d = 11;
	poseInfo[d][0] = 51.2168;
	poseInfo[d][1] = 4.3480;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = 0.7066;
	poseInfo[d][6] = 0.7076;
	d = 12;
	poseInfo[d][0] = 51.3009;
	poseInfo[d][1] = 6.8911;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = 0.6862;
	poseInfo[d][6] = 0.7274;
	d = 13;
	poseInfo[d][0] = 51.0974;
	poseInfo[d][1] = 8.3811;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = 0.7123;
	poseInfo[d][6] = 0.7018;
	d = 14;
	poseInfo[d][0] = 51.0946;
	poseInfo[d][1] = 9.5421;
	poseInfo[d][2] = 0.0000;
	poseInfo[d][3] = 0.0000;
	poseInfo[d][4] = 0.0000;
	poseInfo[d][5] = 0.6854;
	poseInfo[d][6] = 0.7282;
}


void publish_status(string s)
{
	htbot::status status;
	status.msg = s;
	status_pub.publish(status);
	return;
}

void publish_debug(string s)
{
	htbot::debug status;
	status.msg = s;
	debug_pub.publish(status);
	return;
}


void publish_move_status(int stat)
{
	htbot::move_status status;
	status.stat = stat;
	status_mov.publish(status);
	return;
}

void movStatusCallback(const htbot::move_status::ConstPtr& msg)
{	
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);		
		if (msg->stat >= 100) {
			moveflag = msg->stat;	
			movedone = 0;
		}
	}
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		px = msg->position.x;
		py = msg->position.y;
		pz = msg->position.z;
		prx = msg->orientation.x;
		pry = msg->orientation.y;
		prz = msg->orientation.z;
		prw = msg->orientation.w;
	}
	//ROS_INFO("------------ Swendgoal : Robot Pose px=%.3f. py=%.3f-----------",px,py);
}

void test(void)
{	
	geometry_msgs::Twist pos;

	ROS_INFO("------Sendgoal : Testing ----------");
	pos.linear.z = 21.0;  //
	pos.linear.x = 1.0;
	pos.angular.z = 1.0;
	pos_pub.publish(pos);
}

bool GoalLPpub(double x,double y, double rz, double rw)
{
	htbot::move cmd;
	
	cmd.x = x;
	cmd.y = y;
	cmd.rz = rz;
	cmd.rw = rw;
		
	goalLP_pub.publish(cmd);
	return true;
}

void CleaningReport(double percent) {
	ros::NodeHandle xn;
	FILE *fp;	
	int ret,d;	
	std::string ss;
	double x,y,cs;

	xn.getParam("CleanPlanReport",ss);
	fp = fopen(ss.c_str(), "w+");

	fprintf(fp,"----------------------Cleaning Report--------------------------------------\n");
	fprintf(fp,"Percentage Cleaned : %.3f\n",percent);	
	fprintf(fp,"---------------------- Points Cleaning Status -----------------------------\n");
	for(int j=0;j<numcleanpoints;j++) {
		x = cleanInfo[j][0];
		y = cleanInfo[j][1];
		cs = cleanInfo[j][9];
		if (cs > 0.0) { 
			fprintf(fp,"Point : x=%3f. y=%3f : YES\n",x,y);
		} else {
			fprintf(fp,"Point : x=%3f. y=%3f : NO \n",x,y);
		}
	}
	fprintf(fp,"---------------------- End of Report -----------------------------\n");
	fclose(fp);
}

void loadCleanPlanPath() {
	ros::NodeHandle xn;
	FILE *fp;	
	int ret,d;	
	std::string cpp,s1,s2;
	double x,y,x1,y1,x2,y2,dx,dy,ddist,status;
	double cx,cy,cz,crx,cry,crz,crw;

	xn.getParam("cleanplanDirectory",s1);
	xn.getParam("cleanplanfile",s2);
	cpp = s1 + s2;
	ROS_INFO("----------- Sendgoal : clean file : %s. ---------------",cpp.c_str());
	fp = fopen(cpp.c_str(), "r");
  if (fp == NULL) {
		ROS_INFO("----- sendGoal : I couldn't load %s for reading Cleaning Plan. -----",cpp.c_str());  	
  	return;
  }
	
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &sx,&sy,&sz,&srx,&sry,&srz,&srw); // start position
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &ex,&ey,&ez,&erx,&ery,&erz,&erw); // end position
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &dsx,&dsy,&dsz,&dsrx,&dsry,&dsrz,&dsrw); // docking station position

	// start of cleaning plan path
	
	d = 0;
	numcleanpoints = 0;
	while(true) {
		if (fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf\n",&cx,&cy,&cz,&crx,&cry,&crz,&crw,&status) == EOF) {
			break;
		}
		d++;
	}
	numcleanpoints = d;
	cleanInfo = (double*)malloc(sizeof(double) * ((numcleanpoints * 10)+1));
	d = 0;
	while(true) {
		if (fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf\n",&cx,&cy,&cz,&crx,&cry,&crz,&crw,&status) == EOF) {
			break;
		}
		//ROS_INFO("----- p=%d : x=%.2f. y=%.2f. z=%.2f. rx=%.2f. ry=%.2f. rz=%.2f. rw=%.2f. st=%.2f. --------",d,cx,cy,cz,crx,cry,crz,crw,status);
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
	calcQuat(sx,sy,cleanInfo[0][0],cleanInfo[0][1],&crx,&cry,&crz,&crw);
	srx = crx;
	sry = cry;
	srz = crz;
	srw = crw;
	cleanInfo[numcleanpoints-1][7] = 0.0;
	x = cleanInfo[numcleanpoints-1][0];
	y = cleanInfo[numcleanpoints-1][1];
	calcQuat(x,y,ex,ey,&crx,&cry,&crz,&crw);
	cleanInfo[numcleanpoints-1][3] = crx;
	cleanInfo[numcleanpoints-1][4] = cry;
	cleanInfo[numcleanpoints-1][5] = crz;
	cleanInfo[numcleanpoints-1][6] = crw;
	ROS_INFO("-------------------- Sendgoal : Cleaning Plan --------------------------------");
	for (int i=0; i<numcleanpoints; i++) {
		cx = cleanInfo[i][0];
		cy = cleanInfo[i][1];
		cz = cleanInfo[i][2];
		crx = cleanInfo[i][3];
		cry = cleanInfo[i][4];
		crz = cleanInfo[i][5];
		crw = cleanInfo[i][6];
		ddist = cleanInfo[i][7];
		status = cleanInfo[i][8];
		ROS_INFO("----- p=%d : x=%.2f. y=%.2f. z=%.2f. rx=%.2f. ry=%.2f. rz=%.2f. rw=%.2f. dd=%.2f. st=%.2f. --------",i,cx,cy,cz,crx,cry,crz,crw,ddist,status);	
	}
	ROS_INFO("-------------------- Sendgoal : End of Cleaning Plan --------------------------------");
  fclose(fp);	
}

bool move(const htbot::move::ConstPtr& msg)
{
	ros::NodeHandle nm;
	geometry_msgs::Twist vel;
	geometry_msgs::Twist pos;
  //MoveBaseClient ac("move_base", true); // move_base_nys
	//move_base_msgs::MoveBaseGoal goal;
  char buf [100];
	string s;
	int cnt;
	ros::Time start_time;
	ros::Time start_delay_time;
	double goal_time;
	bool delay;
	int reached,BackObs;
	double gap,dgap,dx,dy,ddist;
	int ix,ixe,opt;

	opt = msg->opt;
	//ROS_INFO(" SendGoal : SLP=%d. ELP=%d",msg->slp,msg->elp);
	//ac("move_base", true);
	if ((msg->opt >= 70) && (msg->opt < 80)) {
		return true;
	}
	if (msg->opt == ENGAGE) {
		// move straight to charger contact until close	
		ROS_INFO(" SendGoal : Move to Docking Station");
		pos.linear.z = 1.0;
		pos_pub.publish(pos);
		return true;
	}
	if (msg->opt == 2) {
		publish_debug("sendgoal : Move to Docking Station activated Stage 2");
		mdist = ffmove;
		speed = 500.0;
		toffset = offset;
		reached = 0;
		nm.setParam("ReachPosition",reached);
		laser_use = true;
		//sleep(7);
		//publish_move_status(1); // movement completed and ok	
		return true;
	}

	if (msg->opt == DISENGAGE) {
		publish_debug("sendgoal : Move away from Docking Station 0.3m");
		pos.angular.z = 0.0;// 0.0
		pos.linear.x = 0.0; // -0.3
		pos.linear.z = 2.0;
		pos.linear.y = 0.0;  // standard speed
		pos_pub.publish(pos);
		return true;
	}

	if (msg->opt == 5) {
		bootup = true;
		laser_use = true;		
		ROS_INFO("SG 5");		
		return true;
	}
	if (msg->opt == 6) {
		pos.angular.z = 0.0;
		pos.linear.x = 0.0;
		pos.linear.z = 8.0;  // 7.0
		pos.linear.y = 100.0;  // standard speed
		pos_pub.publish(pos);
		return true;
	}
	if (msg->opt == 7) {
		pos.linear.z = align_method; //10.0
		pos_pub.publish(pos);
		return true;
	}
	if (msg->opt == 8) {
		pos.linear.z = 1.0;  //
		pos_pub.publish(pos);
		return true;
	}
	// pre move
	if (msg->opt == PREMOVE) {
		pos.linear.z = 20.0;  //
		pos.linear.x = msg->pd;
		pos.angular.z = msg->pa;
		pos_pub.publish(pos);
		return true;
	}
	// post move
	if (msg->opt == POSTMOVE) {
		pos.linear.z = 20.0;  //
		pos.linear.x = msg->pd;
		pos.angular.z = msg->pa;
		pos_pub.publish(pos);
		return true;
	}
	if (msg->opt == 11) {
		goalTime = ros::Time::now();	
	}
	// turn to path prior to start move
	if (msg->opt == TURNTOPATH) {
		pos.linear.z = 21.0;  //
		pos.linear.x = msg->x;
		pos.angular.z = msg->y;
		pos_pub.publish(pos);
		//ROS_INFO(" ===== Bot_Node Turn to path : x=%.3f. y=%.3f ======",pos.linear.x,pos.angular.z);
		//publish_event("--- SendGoal : Turn to Path ---");
		return true;
	}
	if (msg->opt == TURNTOPATHMOVE) {
		pos.linear.z = 21.5;  //
		pos.linear.x = msg->x;
		pos.angular.z = msg->y;
		pos_pub.publish(pos);
		//ROS_INFO(" ===== Bot_Node 21 : x=%.3f. y=%.3f ======",pos.linear.x,pos.angular.z);
		return true;
	} 

	if (msg->opt == LOCALISEMOVE) {
		pos.linear.z = 22.0;  //
		pos.linear.x = msg->x;
		pos.angular.z = msg->y;
		pos_pub.publish(pos);
		//ROS_INFO(" ===== Bot_Node LOCALISEMOVE ======");
		return true;
	}

	if (msg->opt == LOADCLEANPLAN) {
		loadCleanPlanPath();
		return true;
	}
	
	if (msg->opt == STOPCLEANPLAN) {
		nm.setParam("stopCleaning",true);
		nm.setParam("Cancel_Nav",true);		
		return true;
	}

	if (msg->opt == SENDCLEANINGREPORT) {
		checkCleaningStatus();
		return true;
	}

	if (msg->opt == PAUSECLEANPLAN) {
		pos.linear.z = 100.0;  // set cmd_vel to zero
		pos_pub.publish(pos);
		return true;
	}

	if (msg->opt == RESUMECLEANPLAN) {
		pos.linear.z = 100.1;  //
		pos_pub.publish(pos);
		return true;
	}
	
	if (msg->opt == CLEANLEFTOVER) {
		ROS_INFO("-------- sendgoal : Re-Cleaning Activated ------------");
		recleaning = false;
		for (int i=numcleanpoints-1;i>=0;i--) {
			if (cleanInfo[i][9] == 0.0) {
				cpixe = i;
				recleaning = true;
				break;
			}
		}
		if (!recleaning) {			
			nm.setParam("cleaningStatus",40);
			return true;
		} else {
			gx = cleanInfo[cpixe][0];
			gy = cleanInfo[cpixe][1];
			gz = cleanInfo[cpixe][2];
			grx = cleanInfo[cpixe][3];
			gry = cleanInfo[cpixe][4];
			grz = cleanInfo[cpixe][5];
			grw = cleanInfo[cpixe][6];
			cleanplan = false;
			nm.setParam("cleanplan",cleanplan);
			lastCP = false;
			recleannextpt = false;
		}
	}	

	if (msg->opt == CLEANPLAN) {
		ROS_INFO("-------- sendgoal : Cleaning Plan Activated ------------");				
		ddist = 0.0;
		cpixe = 0;
		lastCP = false;
		for (int i=0;i<numcleanpoints;i++) {
			tpflag = cleanInfo[i][8]; // status
			if (tpflag == 0.0) {
				ddist = ddist + cleanInfo[i][7];
				if (ddist > CPLOOKAHEAD) {
					cpixe = i;
					ROS_INFO("-----------Sendgoal : CP Start dist>2.5 xe=%d -------------",cpixe);
					break;
				}
			} else {
				cpixe = i;
				ROS_INFO("-----------Sendgoal : CP Start. Hit TP xe=%d -------------",cpixe);
				break;
			}
			//cpixe = i;
		}
		if (cpixe == (numcleanpoints - 1)) {
			lastCP = true;
			ROS_INFO("-------------- Sendgoal : Last Cleaning Point -----------------");
		}
		gx = cleanInfo[cpixe][0];
		gy = cleanInfo[cpixe][1];
		gz = cleanInfo[cpixe][2];
		grx = cleanInfo[cpixe][3];
		gry = cleanInfo[cpixe][4];
		grz = cleanInfo[cpixe][5];
		grw = cleanInfo[cpixe][6];
		tpflag = cleanInfo[cpixe][8];
		if (tpflag > 0.0) {
			// it a turning point
			cpturnpt = true;
		} else {
			cpturnpt = false;
		}		
		//cpcix = 0;
		cleanplan = true;		
		nm.setParam("cleanplan",cleanplan);
		//rpx = px; // starting point
		//rpy = py;
		//sprintf(buf,"------Sendgoal : CP Start : xe=%d. xs=%d. gx=%.3f. gy=%.3f. igx=%.3f. igy=%.3f ---------",cpixe,cpixs,gx,gy,igx,igy);
		//s.assign(buf,strlen(buf));
		//publish_event(s);
	}

	gap = 0.3; //msg->gap;
	ix = 0;

	if (opt == 0) {
		ROS_INFO("-------- SendGoal : Normal Move to LP -------------");
		gx = msg->x;
		gy = msg->y;
		gz = msg->z;
		grx = msg->rx;
		gry = msg->ry;
		grz = msg->rz;
		grw = msg->rw;
		multiLP = false;
		lastLP = false;
		publish_event("--- Sendgoal : Start Normal Point to Point Move ---");
	}

	if (opt == RACPATHMOVE) {
		ROS_INFO("-------- SendGoal : RAC Path Move to LP -------------");
		gx = msg->x;
		gy = msg->y;
		gz = msg->z;
		grx = msg->rx;
		gry = msg->ry;
		grz = msg->rz;
		grw = msg->rw;
		multiLP = false;
		lastLP = false;
		publish_event("--- Sendgoal : Start RAC Path Move ---");
		racpathhold = true;
	}

	Cancel_Nav = false;
	start_time = ros::Time::now();
	nm.setParam("navstatus",7);  //navigation started.
	cnt = 0;  // counter for number of retry after abort	
	startGoal = true;
	sendGoal = true;
	//ROS_INFO("----------- Sendgoal : startGoal & sendGoal true ------------");
  return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_to_goal_server");
  ros::NodeHandle n;

	laser_use = false; 
	bootup = false;
	ros::Rate loop_rate(10.0);
	stop_move = 0;
	xcount = 0;
	racpathhold = false;
	init_LP();
	startGoal = false;
	sendGoal = false;
	multiLP = false;
	Cancel_Nav = false;
	MoveBaseClient ac1("move_base", true); // move_base_nys
	move_base_msgs::MoveBaseGoal goal;
	double dx,dy,turnAngle;
	ros::Time abortTime;
	int sstate,nlp,lastOKLP;
	geometry_msgs::Twist pos;
	bool Cancel_Nav,CancelFlag,Lost_Localisation;
	char buf[100];
	string s1;
	double ddist;

	//ROS_INFO("---------SendGoal : Here 0------------");
	move_cmd = n.subscribe<htbot::move>("move", 1000, move);  // move_test to test. move is actual
  status_pub = n.advertise<htbot::status>("feedback",100);
	debug_pub = n.advertise<htbot::debug>("debug",100);
	status_mov = n.advertise<htbot::move_status>("move_status",100);
	pos_pub = n.advertise<geometry_msgs::Twist>("cmd_pos", 1);
	pose_sub = n.subscribe<geometry_msgs::Pose>("/robot_pose", 1,poseCallback);
	goalLP_pub = n.advertise<htbot::move>("goalLP", 20);
	play_pub = n.advertise<htbot::sound>("sound", 5);
	chkplan_pub = n.advertise<htbot::goal>("checkplan",1);
	cleanPlan_sub = n.subscribe<nav_msgs::Path>("/cleanPlan", 1,cleanPlanCallback);
	movstat_sub = n.subscribe<htbot::move_status>("move_status", 100, movStatusCallback);
	clear_pub = n.advertise<htbot::clear>("clearMap", 100);
	event_pub = n.advertise<htbot::debug>("event",100);

	n.getParam("FIRSTMOVE",fmove);
	n.getParam("FINALMOVE",ffmove);
	n.getParam("OFFSET",offset);
	n.getParam("ALIGN_METHOD",align_method);
	n.getParam("path_file",pathfile);
	n.getParam("CPGAPLIMIT",CPGAPLIMIT);
	n.getParam("CPLOOKAHEAD",CPLOOKAHEAD);

	//ROS_INFO("---------SendGoal : Here 1------------");
  //readPathfromFile();
	sstate = 0;
	lastOKLP = 0;
	gapLimit = 0.9;
	angleTurn = false;
	nextsegment = false;
	Cancel_Nav = false;
	n.setParam("Cancel_Nav",Cancel_Nav);
	lastLP = false;
	
	CancelFlag = false;
	//ROS_INFO("---------SendGoal : Here 2------------");
	//MoveBaseClient ac1("move_base", true); // move_base_nys
	//test();
	//sleep(2);
	//test();
	while (true) {  	
		ros::spinOnce();	
  	loop_rate.sleep();

		switch (sstate) {
			case 0: 
				break;
			case 1: // abort
				publish_clear();
				publish_sound(NEEDSPACE,0,0);
				sstate = 10;
				if (lastCP) {
					nlp = 0;
				} else {
					nlp = 1;
				}
				abortTime = ros::Time::now();
				ROS_INFO("--------SendGoal : Need Space idxLP=%d. ----------",idxLP);
				break;
			case 10: 
				if ( ros::Time::now() > (abortTime + ros::Duration(1.0)) ) {					
					if ((cpixe+nlp) < numcleanpoints) {
						ROS_INFO(" ------- SendGoal 10 : cpixe=%d. numcpt=%d ---------------",cpixe,numcleanpoints);
						gxn = cleanInfo[cpixe+nlp][0];
						gyn = cleanInfo[cpixe+nlp][1];
						checkpath(gxn,gyn);
						abortTime = ros::Time::now();
						sstate = 11;
					} else {
						sstate = 101;
						abortTime = ros::Time::now();
						ROS_INFO("--------SendGoal 10 : cpixe+1 exceed numcleanpoints ----------");
					}
				}
				break;
			case 100: 
				if ( ros::Time::now() > (abortTime + ros::Duration(2.0)) ) {
					if ((cpixe+nlp) < numcleanpoints) {
						gxn = cleanInfo[cpixe+nlp][0];
						gyn = cleanInfo[cpixe+nlp][1];
						checkpath(gxn,gyn);
						abortTime = ros::Time::now();
						sstate = 11;
					} else {
						sstate = 1;
						//publish_sound(NEEDSPACE,0,0);
						//nlp = 1;
						ROS_INFO("--------SendGoal 100 : cpixe=%d. nlp=%d----------",cpixe,nlp);
						//abortTime = ros::Time::now();
					}
				}
				break;
			case 101:
				if ( ros::Time::now() > (abortTime + ros::Duration(2.0)) ) {	
					sstate = 1;
				}
				break;
			case 11:
				if ( ros::Time::now() > (abortTime + ros::Duration(0.1)) ) {	
					n.getParam("checkPlanSum",checkPlanSum); 
					n.getParam("checkPlanX",checkPlanX);
					n.getParam("checkPlanY",checkPlanY);
					if (checkPlanSum ==200.0) {						
						movedone = 1;
						moveflag = 0;
						pos.linear.z = 23.0;  //
						pos.linear.x = checkPlanX;
						pos.angular.z = checkPlanY;
						pos_pub.publish(pos);		
						ROS_INFO("--------- Sendgoal 11 : Turn to Path ------------");
						sstate = 12;		
					} else {
						publish_clear();
						publish_sound(NEEDSPACE,0,0);
						ROS_INFO("--------SendGoal 11 : Need Space cpixe=%d. ----------",cpixe);
						if (!lastCP) {
							nlp++;
						}
						sstate = 100;
					}					
					abortTime = ros::Time::now();
				}
				break;
			case 12:
				if (movedone == 0) {		
					if (moveflag == 100) {					
						startGoal = true;		
						cpixe = cpixe + nlp;				
						if (cpixe < numcleanpoints) {						
							gx = cleanInfo[cpixe][0];
							gy = cleanInfo[cpixe][1];
							gz = cleanInfo[cpixe][2];
							grx = cleanInfo[cpixe][3];
							gry = cleanInfo[cpixe][4];
							grz = cleanInfo[cpixe][5];
							grw = cleanInfo[cpixe][6];						
							ROS_INFO(" ------- SendGoal 12 : cpixe=%d. gx=%.3f. gy=%.3f ---------------",cpixe,gx,gy);
							sendGoal = true;
							if (cpixe == numcleanpoints-1) {
								lastCP = true;
							}
							//lastOKLP = idxLP;
							sstate = 0;
							startGoal = true;	
							continue;
						} 
					}
				}
				break;
			case 20:  // turn on the spot
				pos.linear.z = 23.0;  //
				pos.linear.x = gx;
				pos.angular.z = gy;
				pos_pub.publish(pos);	
				abortTime = ros::Time::now();
				movedone = 1;
				moveflag = 0;
				sstate = 21;
				angleTurn = false;
				break;
			case 21:
				if (movedone == 0) {		
					if (moveflag == 100) {
						startGoal = true;
						sstate = 0;
					}
				}
				break;
			case 3:
				if ( ros::Time::now() > (abortTime + ros::Duration(0.3)) ) {
					movedone = 1;
					moveflag = 0;
					pos.linear.z = 23.0;  //
					pos.linear.x = gx;
					pos.angular.z = gy;
					pos_pub.publish(pos);		
					ROS_INFO("--------- Sendgoal 3 : Turn to Next CPoint ------------");
					sstate = 30;		
					abortTime = ros::Time::now();
				}
				break;
			case 30:
				if ( ros::Time::now() > (abortTime + ros::Duration(0.2)) ) {
					if (movedone == 0) {		
						if (moveflag == 100) {
							dx = gx - px;
							dy = gy - py;
							gapdist = sqrt((dx * dx) + (dy * dy));
							movedone = 1;
							moveflag = 0;
							pos.linear.z = 23.5;  //
							pos.linear.x = gapdist;								
							pos_pub.publish(pos);		
							ROS_INFO("--------- Sendgoal 30 : Move to Next CPoint ------------");
							sstate = 31;		
							abortTime = ros::Time::now();
						}
					}
				}
				break;
			case 31:
				if ( ros::Time::now() > (abortTime + ros::Duration(0.3)) ) {
					if (movedone == 0) {		
						if (moveflag == 100) {
							startGoal = true;
							sstate = 0;
							nextsegment = true;
							ROS_INFO("-------------- Sendgoal 31 : Next Segment ---------------------");
						}
					}
				}
				break;
			case 32:
				if ( ros::Time::now() > (abortTime + ros::Duration(0.3)) ) {
					movedone = 1;
					moveflag = 0;
					pos.linear.z = 23.0;  //
					pos.linear.x = gx;
					pos.angular.z = gy;
					pos_pub.publish(pos);		
					ROS_INFO("--------- Sendgoal 32 : Turn to Next CPoint ------------");
					sstate = 33;		
					abortTime = ros::Time::now();
				}
				break;
			case 33:
				if ( ros::Time::now() > (abortTime + ros::Duration(0.2)) ) {
					if (movedone == 0) {		
						if (moveflag == 100) {
							startGoal = true;
							sstate = 0;
							nextsegment = false;		
							sendGoal = true;
							ROS_INFO("-------------- Sendgoal 33 : Clean next Segment ---------------------");
						}
					}
				}
				break;
			case 4:
				if ( ros::Time::now() > (abortTime + ros::Duration(0.2)) ) {
					movedone = 1;
					moveflag = 0;
					pos.linear.z = 23.5;  //
					pos.linear.x = gapdist;
					//pos.angular.z = gy;
					pos_pub.publish(pos);		
					ROS_INFO("--------- Sendgoal 4 : Move closer to Turning Point ------------");
					sstate = 40;		
					abortTime = ros::Time::now();
				}
				break;
			case 40:
				if ( ros::Time::now() > (abortTime + ros::Duration(0.25)) ) {
					if (movedone == 0) {		
						if (moveflag == 100) {
							sstate = 0;		
							startGoal = true;
							abortTime = ros::Time::now();
						}
					}
				}
				break;
		}
  	/* 	  
		if (racpathhold) {
			xcount++;
			if (xcount > 10) {
				xcount = 0;
				racpathhold = false;
			} else {
				continue;
			}
		}
		*/
		if (!startGoal) {
			continue;
		}
		if (sendGoal) {
			goal.target_pose.header.frame_id = "/map";
  		goal.target_pose.header.stamp = ros::Time::now();
  		goal.target_pose.pose.position.x = gx ; //msg->x;
  		goal.target_pose.pose.position.y = gy; //msg->y;
  		goal.target_pose.pose.position.z = gz; //msg->z;
  		goal.target_pose.pose.orientation.x = grx; //msg->rx;
  		goal.target_pose.pose.orientation.y = gry; //msg->ry;
  		goal.target_pose.pose.orientation.z = grz; //msg->rz;
  		goal.target_pose.pose.orientation.w = grw; //msg->rw;  
			GoalLPpub(gx,gy,grz,grw);
			while(!ac1.waitForServer(ros::Duration(10.0))){
  			ROS_INFO("Waited 10sec for the move_base_nys action server to come up");
				//system("gnome-terminal -x aplay ~/sound/honk.wav &");
				break;
 	 		} 
			sendGoal = false;		
 			ac1.sendGoal(goal);
			//sprintf(buf,"--------Sendgoal :  Sending Goal : X : %.3f. Y : %.3f-------------",gx,gy);
			//s1.assign(buf,strlen(buf));
			//publish_event(s1);
			ROS_INFO("--------Sendgoal :  Goal : X : %.3f. Y : %.3f. rz=%.3f. rw=%.3f-------------",gx,gy,grz,grw);
			continue;
		}
				
		n.getParam("Cancel_Nav",Cancel_Nav);
		if (Cancel_Nav) {
			//stop_move = 0;
			//n.setParam("cancelMove",0);
			ac1.cancelAllGoals();
			//ROS_INFO("-------- Sendgoal : Cancel_Nav True. Cancel All Goals -----------");
			publish_event("--------Sendgoal : Cancel_Nav True. Cancel All Goals---------");
			//publish_move_status(6);
			//startGoal = false;
		} 

		if (nextsegment && !lastCP) {
			ROS_INFO("-------- sendgoal : next segment of cleaning plan activated ------------");				
			ddist = 0.0;
			for (int i=cpixe+1;i<numcleanpoints;i++) {
				tpflag = cleanInfo[i][8]; // status
				if (tpflag == 0.0) {
					ddist = ddist + cleanInfo[i][7];
					if (ddist > CPLOOKAHEAD) {
						cpixe = i;
						ROS_INFO("-----------Sendgoal : nextsegment Start dist>2.5 xe=%d -------------",cpixe);
						break;
					}
				} else {
					cpixe = i;
					ROS_INFO("-----------Sendgoal : nextsegment Start. Hit TP xe=%d -------------",cpixe);
					break;
				}
			}
			gx = cleanInfo[cpixe][0];
			gy = cleanInfo[cpixe][1];
			gz = cleanInfo[cpixe][2];
			grx = cleanInfo[cpixe][3];
			gry = cleanInfo[cpixe][4];
			grz = cleanInfo[cpixe][5];
			grw = cleanInfo[cpixe][6];
			tpflag = cleanInfo[cpixe][8];
			if (tpflag > 0.0) {
				// it a turning point
				cpturnpt = true;
			} else {
				cpturnpt = false;
			}		
			if (cpixe == (numcleanpoints - 1)) {
				lastCP = true;
				ROS_INFO("-------------- Sendgoal : Last Cleaning Point -----------------");
			}
			if (ddist < 1.0) {				
				sstate = 3;
			} else {
				sstate = 32;
			}
			abortTime = ros::Time::now();
			startGoal = false;
			continue;
		}
		
		if (cleanplan && !lastCP) {
			dx = gx - px;
			dy = gy - py;
			gapdist = sqrt((dx * dx) + (dy * dy));
			if (!cpturnpt) {				
				// goal is not turning point. go to the next point
				if (gapdist < CPGAPLIMIT) {
					// advance to next point
					if (cpixe < numcleanpoints-1) {
						cpixe++;
						gx = cleanInfo[cpixe][0];
						gy = cleanInfo[cpixe][1];
						gz = cleanInfo[cpixe][2];
						grx = cleanInfo[cpixe][3];
						gry = cleanInfo[cpixe][4];
						grz = cleanInfo[cpixe][5];
						grw = cleanInfo[cpixe][6];
						tpflag = cleanInfo[cpixe][8];
						if (tpflag > 0.0) {
							// next point is a turning point
							cpturnpt = true;
						}
						sendGoal = true;
						continue;
					}
				}
			}
		}		

				
		if(ac1.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    	ROS_INFO("--------Sendgoal : Goal Reached---------");
			if (cleanplan) {
				if (lastCP) {
					publish_move_status(1); // movement completed and ok
					startGoal = false;
					cleanplan = false;
					n.setParam("cleanplan",cleanplan); abortTime = ros::Time::now();
					//startGoal = false;
					//sstate = 3;
					ROS_INFO("--------Sendgoal : Cleaning Completed---------");
				} else {
					nextsegment = true;
					abortTime = ros::Time::now();
					startGoal = false; //true;
					dx = gx - px;
					dy = gy - py;
					gapdist = sqrt((dx * dx) + (dy * dy));
					sstate = 4;
				}
			} else {
				publish_move_status(1); // movement completed and ok
				startGoal = false;
				cleanplan = false;
				n.setParam("cleanplan",cleanplan); 
				publish_event("--- Sendgoal : Goal Reached ---");
			}
			continue;
		} else {
			n.getParam("Cancel_Nav",Cancel_Nav);	
			if(ac1.getState() == actionlib::SimpleClientGoalState::ABORTED) {
				if (Cancel_Nav) {
					startGoal = false;				
					sstate = 0;
					publish_move_status(2);
					cleanplan = false;
				} else {
					if (cleanplan) {
						startGoal = false;				
						sstate = 1;
					} else {
						publish_move_status(2);
						startGoal = false;				
						sstate = 0;
					}
				}
				/*
				if ((multiLP || lastLP) && !Cancel_Nav) {    		
					startGoal = false;				
					sstate = 1;
					ROS_INFO("--------Sendgoal : Aborted Point A---------");
				} else {
					ROS_INFO("--------Sendgoal : Aborted Point B---------");
					n.getParam("Lost_Localisation",Lost_Localisation);
					startGoal = false;
					if (Lost_Localisation) {
						ROS_INFO("--------Sendgoal : Aborted Point C---------");
						publish_move_status(5);
					} else {
						publish_move_status(2); // movement aborted
						ROS_INFO("--------Sendgoal : Aborted Point D---------");
					}
					multiLP = false;
					n.setParam("multiLP",multiLP);
					//n.setParam("moveok",1);
					//publish_debug("Sendgoal : Goal Aborted");
				}
				*/
				ROS_INFO("--------Sendgoal : Goal Aborted---------");
				publish_event("--- Sendgoal : Goal Aborted ---");
				continue;
			} else {
				if(ac1.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    			ROS_INFO("------------------ Sendgoal : preempted ---------------------");			
					//publish_debug("Preempted");
					//n.setParam("moveflag",1);
					publish_move_status(5); // movement cancel
					startGoal = false;
					multiLP = false;
					n.setParam("multiLP",multiLP);
					//ROS_INFO("--------Sendgoal : Goal Preempted---------");
					publish_event("--- Sendgoal : Goal Preempted ---");
					continue;
				} 
			}
		}	
		
  }

  return 0;
}



