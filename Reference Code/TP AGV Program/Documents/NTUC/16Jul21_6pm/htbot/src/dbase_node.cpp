/*
 * This node is to database node 
 */
/* 	History
*		Date Modified :7.1.2015
*		Changes :
*/

//#define ODROID
//#define SSMC
//#define RAC

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include "htbot/status.h"
#include "htbot/srvcmd.h"
#include "htbot/move.h"
#include "htbot/dbcmd.h"
#include "htbot/queue.h"
#include "htbot/mqueue.h"
#include "boost/algorithm/string.hpp"
#include "htbot/sound.h"
#include "htbot/debug.h"
#include "htbot/scanCmd.h"
#include <geometry_msgs/Pose.h>

#define MAXLP 20
#define MAXNG 10
using namespace std;
using namespace boost::algorithm;

class DBNode
{
public:
	DBNode(ros::NodeHandle rn);	
	bool cmdServiceCallback(htbot::dbcmd::Request &req,htbot::dbcmd::Response &res);	
	void readPosefromFile();
	bool sendPose(int);
	void readParamfromFile();
	void setMapFlagToMapMode();
	void setMapFlagToNavMode();
	bool getPose();
	void savePose();
	void storePose(int GN,int LP);	
	void storePoseN(int GN,int LP);
	void saveParamToFile();
	void publish_debug(string s);	
	void initialisation();
	void publish_sound(int id);
	void publish_updategroup(string s);
	void removePose(int GN, int LP);
	void removeAllPose();
	void setMarkedStatus(int gp);
	void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
	bool mqPush(int LP);

	ros::ServiceServer cmd_service;
	ros::Publisher debug_pub;
	ros::Publisher queue_pub;
	ros::Publisher updategroup_pub;
	ros::Publisher scanM_pub;
	ros::Subscriber pose_sub;
	htbot::mqueue mq_srv;
	ros::ServiceClient mq_client;

	string LandingPointString[MAXNG][MAXLP];
	string GroupName[MAXNG];
	int GroupNumLP[MAXNG];
	string LandingPointPasswd[MAXLP];
	int numLP;
	int numG;
	int curGP;
	FILE *pfp;  // pose data file
	FILE *lfp;  // landing points data file
	FILE *lpfp;

	std::string posefile;
	std::string markfile;
	std::string paramfile;	
	std::string adminpwd;
	std::string sdir,homedir;
	double tx,ty,tz;
	double rx,ry,rz,rw;
	double pre_dist,pre_angle,post_dist,post_angle;  // pre=before start. post=after reaching dest.
	double pre_dist1,pre_angle1,post_dist1,post_angle1,align,autostart;
	int marked,omarked;
	double otx,oty,otz;
	double orx,ory,orz,orw;
	double poseInfo[MAXNG][MAXLP][17];	
	int PoseMarkFlag[MAXNG][MAXLP];	
	int mapflag;
	double looprate;
	bool firefox_enabled;
	ros::Publisher play_pub;
	double px,py,pz,prx,pry,prz,prw;

private:
	ros::NodeHandle nh;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::TransformStamped robotpose;
};

DBNode::DBNode(ros::NodeHandle rn):
	rx(0.0),ry(0.0),rz(0.0),rw(0.0),
	tx(0.0),ty(0.0),tz(0.0), marked(0), 
	numLP(0),numG(0),curGP(0),
	looprate(1.0),firefox_enabled(false)
{
	ros::NodeHandle sn(rn);
	cmd_service = nh.advertiseService("dbase_cmd",&DBNode::cmdServiceCallback,this); 
	debug_pub = sn.advertise<htbot::debug>("debug",100);
	queue_pub = sn.advertise<htbot::queue>("queue",100);
	updategroup_pub = sn.advertise<htbot::status>("group",100);
	play_pub = sn.advertise<htbot::sound>("sound", 100);
	scanM_pub = sn.advertise<htbot::scanCmd>("scanCmd", 1);
	pose_sub = sn.subscribe<geometry_msgs::Pose>("/robot_pose", 1, &DBNode::poseCallback,this);
	mq_client = nh.serviceClient<htbot::mqueue>("mqueue");
	//nh.getParam("/dbNode/pose_file",posefile);	
	//nh.getParam("/dbNode/param_file",paramfile);	
	//nh.getParam("/dbNode/posemark_file",markfile);			
	nh.getParam("/dbNode/current_group",curGP);
	nh.getParam("pose_file",posefile);	
	nh.getParam("param_file",paramfile);	
	nh.getParam("posemark_file",markfile);
	nh.getParam("home_dir",homedir);

}

void DBNode::publish_updategroup(string s)
{
	htbot::status status;
	status.msg = s;
	updategroup_pub.publish(status);
	return;
}

void DBNode::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
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

bool DBNode::getPose() 
{
	char buf [100];	
	try
	{
		listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		tf::transformStampedTFToMsg(transform,robotpose);
		tx = robotpose.transform.translation.x;
		ty = robotpose.transform.translation.y;
		tz = robotpose.transform.translation.z;
		rx = robotpose.transform.rotation.x;
		ry = robotpose.transform.rotation.y;
		rz = robotpose.transform.rotation.z;
		rw = robotpose.transform.rotation.w;
		return true;
	}
	catch (tf::TransformException ex)
	{
		ROS_INFO("%s",ex.what());	
		return false;	
	}
}

void DBNode::publish_sound(int id)
{
	htbot::sound cmd;
	cmd.id = id;
	play_pub.publish(cmd);
	return;
}

void DBNode::setMapFlagToMapMode() {
	mapflag = 33;

	//ROS_INFO("Param File : %s",paramfile.c_str());
	lpfp = fopen(paramfile.c_str(), "r+");  // r+ : open existing file to r/w
  if (lpfp == NULL) {
  	ROS_INFO("I couldn't open paramdata.dat to write.\n");    
  	return;
  }
	fprintf(lpfp,"%d\n",mapflag);
	fclose(lpfp);
}

void DBNode::setMapFlagToNavMode() {
	mapflag = 77;

	//ROS_INFO("Param File : %s",paramfile.c_str());
	lpfp = fopen(paramfile.c_str(), "r+");
  if (lpfp == NULL) {
  	ROS_INFO("I couldn't open paramdata.dat to write.\n");    
  	return;
  }
	fprintf(lpfp,"%d\n",mapflag);
	fclose(lpfp);
}

void DBNode::readParamfromFile()
{
	int noLP;
	std::string s1;
	std::string s3;
	char s2[100];
	char s21[100];
	int len;
	char buf[100];
	

	//ROS_INFO("Param File : %s",paramfile.c_str());
	lpfp = fopen(paramfile.c_str(), "r");
  if (lpfp == NULL) {
  	ROS_INFO("I couldn't open paramdata.dat to read.\n");    
  	return;
  }
	// read mapping flag. 1=mapping done. 0=mapping not done
	fscanf(lpfp, "%d\n", &mapflag);	
	//ROS_INFO("mapflag : %d",mapflag);
	nh.setParam("mapflag",mapflag);
	nh.setParam("current_group",curGP);
	//ROS_INFO("Reading Admin Password");
	fgets(s2,100,lpfp);		
	len = strlen(s2);				
	adminpwd.assign(s2,len);
	nh.setParam("AdminPwd",adminpwd);	
	//ROS_INFO("Admin PW : %s",adminpwd.c_str());
	//ROS_INFO("Read Landing Points Info and AMCL");
	fscanf(lpfp, "%d\n", &numG);
	//ROS_INFO("number of Group : %d",numG);	
	nh.setParam("numG",numG);	
	if (numG > 0) {					
		for (int k=0;k<numG;k++) {		
			// read group name		
			fgets(s2,100,lpfp);		
			len = strlen(s2);			
			GroupName[k].assign(s2,len);	
			//ROS_INFO("Group Name %d = %s",k,GroupName[k].c_str());

			// read landing point name
			fscanf(lpfp, "%d\n", &numLP);
			//ROS_INFO("number of LP : %d",numLP);	
			GroupNumLP[k] = numLP;
			for (int j=0;j<numLP;j++) {
				fgets(s2,100,lpfp);		
				len = strlen(s2);	
				LandingPointString[k][j].assign(s2,len);
				//ROS_INFO("LP Str %d = %s",j,LandingPointString[k][j].c_str());
			}		
		}
		readPosefromFile();
	}
	numLP = GroupNumLP[curGP];
	nh.setParam("numLP",numLP);
	fclose(lpfp);	
}


void DBNode::storePose(int GN, int LP) 
{		
	marked = 1;
	poseInfo[GN][LP][0] = tx;
	poseInfo[GN][LP][1] = ty;
	poseInfo[GN][LP][2] = tz;
	poseInfo[GN][LP][3] = rx;
	poseInfo[GN][LP][4] = ry;
	poseInfo[GN][LP][5] = rz;
	poseInfo[GN][LP][6] = rw;	
	
	PoseMarkFlag[GN][LP] = marked;	// 1= marked. 0 = unmarked
	otx = tx;
	oty = ty;
	otz = tz;
	orx = rx;
	ory = ry;
	orz = rz;
	orw = rw;
	omarked = marked;
}

void DBNode::storePoseN(int GN, int LP) 
{		
	poseInfo[GN][LP][0] = px;
	poseInfo[GN][LP][1] = py;
	poseInfo[GN][LP][2] = pz;
	poseInfo[GN][LP][3] = prx;
	poseInfo[GN][LP][4] = pry;
	poseInfo[GN][LP][5] = prz;
	poseInfo[GN][LP][6] = prw;	
	PoseMarkFlag[GN][LP] = 1;	// 1= marked. 0 = unmarked
}

void DBNode::removePose(int GN, int LP) 
{		
	marked = 0;
	poseInfo[GN][LP][0] = 0.0;
	poseInfo[GN][LP][1] = 0.0;
	poseInfo[GN][LP][2] = 0.0;
	poseInfo[GN][LP][3] = 0.0;
	poseInfo[GN][LP][4] = 0.0;
	poseInfo[GN][LP][5] = 0.0;
	poseInfo[GN][LP][6] = 0.0;	
	PoseMarkFlag[GN][LP] = marked;	// 1= marked. 0 = unmarked
}

void DBNode::removeAllPose() 
{	
	int tnLP;
	for (int i=0;i<MAXNG;i++) {
		tnLP = GroupNumLP[i];
		for (int j=0;j<tnLP;j++) {
			poseInfo[i][j][0] = 0.0;
			poseInfo[i][j][1] = 0.0;
			poseInfo[i][j][2] = 0.0;
			poseInfo[i][j][3] = 0.0;
			poseInfo[i][j][4] = 0.0;
			poseInfo[i][j][5] = 0.0;
			poseInfo[i][j][6] = 0.0;
			PoseMarkFlag[i][j] = 0;		
		}
	}
}

void DBNode::initialisation() 
{	
	for (int i=0;i<MAXNG;i++) {
		GroupName[i] = "";
		GroupNumLP[i] = 0;
		for (int j=0;j<MAXLP;j++) {
			poseInfo[i][j][0] = 0.0;
			poseInfo[i][j][1] = 0.0;
			poseInfo[i][j][2] = 0.0;
			poseInfo[i][j][3] = 0.0;
			poseInfo[i][j][4] = 0.0;
			poseInfo[i][j][5] = 0.0;
			poseInfo[i][j][6] = 0.0;
			poseInfo[i][j][7] = 0.0;
			poseInfo[i][j][8] = 0.0;
			poseInfo[i][j][9] = 0.0;
			poseInfo[i][j][10] = 0.0;
			poseInfo[i][j][11] = 0.0;
			poseInfo[i][j][12] = 0.0;
			poseInfo[i][j][13] = 0.0;
			poseInfo[i][j][14] = 0.0;
			poseInfo[i][j][15] = 0.0;
			poseInfo[i][j][16] = 0.0;
			PoseMarkFlag[i][j] = 0;				
			LandingPointString[i][j] = "";
		}
	}
}


void DBNode::savePose() 
{		
	pfp = fopen(posefile.c_str(), "w");  // w : open new file
	lfp = fopen(markfile.c_str(), "w");  // w : open new file
  if (pfp == NULL) {
  	ROS_INFO("I couldn't open posedata.dat for writing.\n");    
  	return;
  }
	if (lfp == NULL) {
  	ROS_INFO("I couldn't open posemark.dat for writing.\n");    
  	return;
  }
	//ROS_INFO("SavePose : numG : %d",numG);
	for (int i=0;i<numG;i++) {
		numLP = GroupNumLP[i];
		//ROS_INFO("SavePose : numlp : %d",numLP);
		for (int j=0;j<numLP;j++) {
			tx = poseInfo[i][j][0];
			ty = poseInfo[i][j][1];
			tz = poseInfo[i][j][2];
			rx = poseInfo[i][j][3];
			ry = poseInfo[i][j][4];
			rz = poseInfo[i][j][5];
			rw = poseInfo[i][j][6];
			pre_dist = poseInfo[i][j][7];
			pre_angle = poseInfo[i][j][8];
			post_dist = poseInfo[i][j][9];
			post_angle = poseInfo[i][j][10];
			pre_dist1 = poseInfo[i][j][11];
			pre_angle1 = poseInfo[i][j][12];
			post_dist1 = poseInfo[i][j][13];
			post_angle1 = poseInfo[i][j][14];
			align = poseInfo[i][j][15];
			autostart = poseInfo[i][j][16];
			marked = PoseMarkFlag[i][j];
			fprintf(pfp,"%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",tx,ty,tz,rx,ry,rz,rw,pre_dist,pre_angle,pre_dist1,pre_angle1,post_dist,post_angle,post_dist1,post_angle1,align,autostart);	
			fprintf(lfp,"%d\n",marked);
		}
	}
  fclose(pfp);	
	fclose(lfp);	
}

void DBNode::readPosefromFile() 
{		
	pfp = fopen(posefile.c_str(), "r");
  if (pfp == NULL) {
  	ROS_INFO("I couldn't open posedata.dat for reading.\n");    
  	return;
  }
	lfp = fopen(markfile.c_str(), "r");
	if (lfp == NULL) {
  	ROS_INFO("I couldn't open posemark.dat for reading.\n");    
  	return;
  }
	for (int i=0;i<numG;i++) {
		numLP = GroupNumLP[i];
		for (int j=0;j<numLP;j++) {
			fscanf(pfp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",&tx,&ty,&tz,&rx,&ry,&rz,&rw,&pre_dist,&pre_angle,&pre_dist1,&pre_angle1,&post_dist,&post_angle,&post_dist1,&post_angle1,&align,&autostart);
			fscanf(lfp,"%d\n",&marked);		
			poseInfo[i][j][0] = tx;
			poseInfo[i][j][1] = ty;
			poseInfo[i][j][2] = tz;
			poseInfo[i][j][3] = rx;
			poseInfo[i][j][4] = ry;
			poseInfo[i][j][5] = rz;
			poseInfo[i][j][6] = rw;
			poseInfo[i][j][7] = pre_dist;				
			poseInfo[i][j][8] = pre_angle;
			poseInfo[i][j][9] = post_dist;				
			poseInfo[i][j][10] = post_angle;
			poseInfo[i][j][11] = pre_dist1;				
			poseInfo[i][j][12] = pre_angle1;
			poseInfo[i][j][13] = post_dist1;				
			poseInfo[i][j][14] = post_angle1;
			poseInfo[i][j][15] = align;
			poseInfo[i][j][16] = autostart;
			PoseMarkFlag[i][j] = marked;	
		}
	}
  fclose(pfp);
	fclose(lfp);		
}

void DBNode::saveParamToFile()
{
	//int mapflag;
	std::string s1;
	//mapflag = 77;
	char buf[100];
	
	lpfp = fopen(paramfile.c_str(), "w");
  if (lpfp == NULL) {
  	ROS_INFO("I couldn't open param_ssmc.dat to write.\n");    
  	return;
  }
	//ROS_INFO("saveParamToFile : numLP : %d.",numLP);
	fprintf(lpfp, "%d\n", mapflag);
	trim(adminpwd);
	fprintf(lpfp,"%s\n",adminpwd.c_str())	;	
	fprintf(lpfp, "%d\n", numG);
	for (int k=0;k<numG;k++) {	
		s1 = GroupName[k];				
		trim(s1);
		fprintf(lpfp,"%s\n",s1.c_str())	;
		//ROS_INFO("GP name : %s\n",s1.c_str())	;
		numLP = GroupNumLP[k];
		fprintf(lpfp, "%d\n", numLP);
		//ROS_INFO("No of LP in Gp : %d\n",numLP)	;
		for (int j=0;j<numLP;j++) {
			s1 = LandingPointString[k][j];
			trim(s1);
			fprintf(lpfp,"%s\n",s1.c_str())	;	
			//ROS_INFO("LP name : %s\n",s1.c_str())	;
		}
	}
	fclose(lpfp);	
}

void DBNode::publish_debug(string s)
{
	htbot::debug deb;
	deb.msg = s;
	debug_pub.publish(deb);
	return;
}

bool DBNode::mqPush(int LP) {
	bool ret;
  ROS_INFO("DB Push . LP : %d",LP);
	mq_srv.request.LP = LP;  		
	mq_srv.request.cmd = 1;
	mq_srv.request.tx = poseInfo[0][LP][0];	
	mq_srv.request.ty = poseInfo[0][LP][1];
	mq_srv.request.tz = poseInfo[0][LP][2];	
	mq_srv.request.rx = poseInfo[0][LP][3];	
	mq_srv.request.ry = poseInfo[0][LP][4];		
	mq_srv.request.rz = poseInfo[0][LP][5];	
	mq_srv.request.rw = poseInfo[0][LP][6];
	mq_srv.request.prd = poseInfo[0][LP][7];	
	mq_srv.request.pra = poseInfo[0][LP][8];	
	mq_srv.request.psd = poseInfo[0][LP][9];
	mq_srv.request.psa = poseInfo[0][LP][10];	
	mq_srv.request.prd1 = poseInfo[0][LP][11];
	mq_srv.request.pra1 = poseInfo[0][LP][12];	
	mq_srv.request.psd1 = poseInfo[0][LP][13];	
	mq_srv.request.psa1 = poseInfo[0][LP][14];
	mq_srv.request.align = poseInfo[0][LP][15];
	mq_srv.request.autostart = poseInfo[0][LP][16];
		
	if (mq_client.call(mq_srv)) {
		if (mq_srv.response.status == 1) {	 
			ret = true;
			//ROS_INFO("DB ok push to move queue");	
		} else {
	    ROS_ERROR("Failed to push to move queue");	
			ret = false;
	 	}
	} else {
		ROS_INFO("DB mq_srv error");
	}
	return ret;
}

void DBNode::setMarkedStatus(int gp)
{
	int markedstatus; 
	ros::NodeHandle xn;

	numLP = GroupNumLP[gp];
	markedstatus = 0;
	for (int i=0;i<numLP;i++) {
		switch (i) {
			case 0:				
				if (PoseMarkFlag[gp][i] > 0) {
					markedstatus = markedstatus | 0x0001;
				}
				break;
			case 1:				
				if (PoseMarkFlag[gp][i] > 0) {
					markedstatus = markedstatus | 0x0002;
				}
				break;
			case 2:				
				if (PoseMarkFlag[gp][i] > 0) {
					markedstatus = markedstatus | 0x0004;
				}
				break;
			case 3:				
				if (PoseMarkFlag[gp][i] > 0) {
					markedstatus = markedstatus | 0x0008;
				}
				break;
			case 4:				
				if (PoseMarkFlag[gp][i] > 0) {
					markedstatus = markedstatus | 0x0010;
				}
				break;
			case 5:				
				if (PoseMarkFlag[gp][i] > 0) {
					markedstatus = markedstatus | 0x0020;
				}
				break;
			case 6:				
				if (PoseMarkFlag[gp][i] > 0) {
					markedstatus = markedstatus | 0x0040;
				}
				break;
			case 7:				
				if (PoseMarkFlag[gp][i] > 0) {
					markedstatus = markedstatus | 0x0080;
				}
				break;
		}
	}
	xn.setParam("Marked_Status",markedstatus);
	return;
}


bool DBNode::cmdServiceCallback(htbot::dbcmd::Request &req,htbot::dbcmd::Response &res)
{
	//ROS_INFO("Dbase Command Services"); 
	int fi,ti;
	char buf [100];
	string s1;
	int readyflag;
	int cmd;
	htbot::scanCmd scmd;
	ros::NodeHandle xn;

	cmd = req.cmd;
  //ROS_INFO("numLP : %d.",numLP);
	switch (cmd) {		
		case 1: // set to navigation mode
			setMapFlagToNavMode();
			//ROS_INFO("Shutting Now..");
			sleep(2);
			system("sudo shutdown -h now");
			break;
		case 10: // Set/Reset Map
			// set mapflag t0 33 and shutdown system
			setMapFlagToMapMode();
			//ROS_INFO("Shutting Now..");
			sleep(2);
			system("sudo shutdown -h now");
			//res.status = 2;
			break;
		case 11: // 
			//ROS_INFO("DB Nav Command 11 : GP : %d. LP : %d",req.GN,req.LP);
			mqPush(req.LP);
			break;
		case 2: // Set/Reset Map
			// set mapflag t0 33 and shutdown system
			//setMapFlagToMapMode();
			//ROS_INFO("Shutting Now..");
			sleep(2);
			system("sudo shutdown -h now");
			//res.status = 2;
			break;
		case 30: // mark landing point			
			if (getPose()) {  // get robot pose.				
				storePose(req.GN,req.LP); // store pose into array if pose ok	
				setMarkedStatus(req.GN);
				xn.setParam("cgp_flag",7);
				//ROS_INFO("Mark Landing Point : %d. Group : %d",req.LP,req.GN);
				//sprintf(buf,"Mark LP : %d. Group : %d",req.LP,req.GN);
			  res.status = 30;
			}			
			//ROS_INFO("Mark Landing Point : %d. Group : %d",req.LP,req.GN);
			//res.status = 30;
			break;		
		case 31: // save map and pose info.
			ROS_INFO("Save Map and Pose Info");				
			savePose();
			saveParamToFile();
			sdir = "rosrun map_server map_saver -f "+homedir+"catkin_ws/src/htbot/maps/docmap";
			sprintf(buf,sdir.c_str());
			//sprintf(buf,"rosrun map_server map_saver -f /home/racnys/catkin_ws/src/htbot/maps/docmap");			
			ROS_INFO("Save Map and Pose Info After");
			system(buf);	
			res.status = 31;
			break;		
		case 32: // request LP Pose Info
			ROS_INFO("DB Request for LP Pose Inof. GN : %d. LP : %d",req.GN,req.LP);	
			res.tx = poseInfo[req.GN][req.LP][0];
			res.ty = poseInfo[req.GN][req.LP][1];
			res.tz = poseInfo[req.GN][req.LP][2];
			res.rx = poseInfo[req.GN][req.LP][3];
			res.ry = poseInfo[req.GN][req.LP][4];
			res.rz = poseInfo[req.GN][req.LP][5];
			res.rw = poseInfo[req.GN][req.LP][6];
			res.prd = poseInfo[req.GN][req.LP][7];
			res.pra = poseInfo[req.GN][req.LP][8];
			res.psd = poseInfo[req.GN][req.LP][9];
			res.psa = poseInfo[req.GN][req.LP][10];
			res.prd1 = poseInfo[req.GN][req.LP][11];
			res.pra1 = poseInfo[req.GN][req.LP][12];
			res.psd1 = poseInfo[req.GN][req.LP][13];
			res.psa1 = poseInfo[req.GN][req.LP][14];
			res.align = poseInfo[req.GN][req.LP][15];
			res.marked = PoseMarkFlag[req.GN][req.LP];
			res.gps = GroupName[req.GN];
			res.lps = LandingPointString[req.GN][req.LP];
			res.status = 32;
			break;	
		case 33: // request  Pose Info
			if (getPose()) {  // get robot pose
				res.rz = rz;
				res.rw = rw;				
			}
			res.status = 33;
			break;	
		case 34: // un mark landing point						
			removePose(req.GN,req.LP);			
			res.status = 34;
			break;
		case 35: // un mark all landing point						
			removeAllPose();			
			res.status = 35;
			break;
		case 36: // mark landing point and save ref laser scan			
			if (getPose()) {  // get robot pose.		
				ROS_INFO("Mark LP and Ref Scan : GP:%d. LP:%d",req.GN,req.LP);		
				storePose(req.GN,req.LP); // store pose into array if pose ok	
				setMarkedStatus(req.GN);
				xn.setParam("cgp_flag",7);
				// save ref laser scan
				sdir = homedir+"catkin_ws/src/htbot/laser/RefScan%d.json";
				sprintf(buf,sdir.c_str(),req.LP);
				//sprintf(buf,"/home/racnys/catkin_ws/src/htbot/laser/RefScan%d.json",req.LP);
				s1.assign(buf,strlen(buf));
				scmd.cmd = 0;
				scmd.file = s1;				
				scanM_pub.publish(scmd);
				//ROS_INFO("Mark Landing Point : %d. Group : %d",req.LP,req.GN);
				//sprintf(buf,"Mark LP : %d. Group : %d",req.LP,req.GN);
			  res.status = 36;
			}			
			//ROS_INFO("Mark Landing Point : %d. Group : %d",req.LP,req.GN);
			//res.status = 30;
			break;
		case 37: // mark landing point and save ref laser scan	
			ROS_INFO("Save Pose Info to Array : 37");	
			poseInfo[req.GN][req.LP][0] = req.tx;
			poseInfo[req.GN][req.LP][1] = req.ty;
			poseInfo[req.GN][req.LP][2] = req.tz;
			poseInfo[req.GN][req.LP][3] = req.rx;
			poseInfo[req.GN][req.LP][4] = req.ry;
			poseInfo[req.GN][req.LP][5] = req.rz;
			poseInfo[req.GN][req.LP][6] = req.rw;
			poseInfo[req.GN][req.LP][7] = req.prd;
			poseInfo[req.GN][req.LP][8] = req.pra;
			poseInfo[req.GN][req.LP][9] = req.psd;
			poseInfo[req.GN][req.LP][10] = req.psa;
			poseInfo[req.GN][req.LP][11] = req.prd1;
			poseInfo[req.GN][req.LP][12] = req.pra1;
			poseInfo[req.GN][req.LP][13] = req.psd1;
			poseInfo[req.GN][req.LP][14] = req.psa1;
			poseInfo[req.GN][req.LP][15] = req.align;				
			//storePose(req.GN,req.LP);
			// save ref laser scan
			ROS_INFO("LPInfo : tx=%.3f,ty=%.3f,rz=%.3f,rw=%.3f",req.tx,req.ty,req.rz,req.rw);
			sdir = homedir+"catkin_ws/src/htbot/laser/RefScan%d.json";
			sprintf(buf,sdir.c_str(),req.LP);
			//sprintf(buf,"/home/racnys/catkin_ws/src/htbot/laser/RefScan%d.json",req.LP);
			s1.assign(buf,strlen(buf));
			scmd.cmd = 0;
			scmd.file = s1;				
			scanM_pub.publish(scmd);
			res.status = 37;			
			break;
		case 38: // save pose info.
			ROS_INFO("Save Pose Info to File : 38");				
			savePose();
			res.status = 38;
			break;
		case 39: // stop sendgoal.
			ROS_INFO("Stop Sendgoal : 39");				
			xn.setParam("/stop_move",1);
			res.status = 39;
			break;
		case 40: // mark landing point and save ref laser scan			
			ROS_INFO("Mark LP and Ref Scan : GP:%d. LP:%d",req.GN,req.LP);		
			storePoseN(req.GN,req.LP); // store pose into array if pose ok	
			setMarkedStatus(req.GN);
			xn.setParam("cgp_flag",7);
			// save ref laser scan
			sdir = homedir+"catkin_ws/src/htbot/laser/RefScan%d.json";
			sprintf(buf,sdir.c_str(),req.LP);
			//sprintf(buf,"/home/racnys/catkin_ws/src/htbot/laser/RefScan%d.json",req.LP);
			s1.assign(buf,strlen(buf));
			scmd.cmd = 0;
			scmd.file = s1;				
			scanM_pub.publish(scmd);
			//ROS_INFO("Mark Landing Point : %d. Group : %d",req.LP,req.GN);
			//sprintf(buf,"Mark LP : %d. Group : %d",req.LP,req.GN);
			res.status = 40;
			break;
		case 50: // get LP Info data			
			res.lps = LandingPointString[req.GN][req.LP];  // get LP String
			res.gps = GroupName[req.GN];
			res.cLP = req.LP;
			res.nLP = GroupNumLP[req.GN];
			res.marked = PoseMarkFlag[req.GN][req.LP];
			setMarkedStatus(req.GN);
			//ROS_INFO("Request for Info. LP : %d. LP Name : %s. Gp Name : %s",req.LP,res.lps.c_str(),res.gps.c_str());
			break;
		case 57: // get LP Info data for group
			//numLP = GroupNumLP[req.GN];			
			//ROS_INFO("get LP Info data for group");
			for (int i=0;i<MAXLP;i++) {
				switch (i) {
					case 0:
						res.lps1 = LandingPointString[req.GN][i];  // get LP String
						res.marked1 = PoseMarkFlag[req.GN][i];
						break;
					case 1:
						res.lps2 = LandingPointString[req.GN][i];  // get LP String
						res.marked2 = PoseMarkFlag[req.GN][i];						
						break;
					case 2:
						res.lps3 = LandingPointString[req.GN][i];  // get LP String
						res.marked3 = PoseMarkFlag[req.GN][i];						
						break;
					case 3:
						res.lps4 = LandingPointString[req.GN][i];  // get LP String
						res.marked4 = PoseMarkFlag[req.GN][i];						
						break;
					case 4:
						res.lps5 = LandingPointString[req.GN][i];  // get LP String
						res.marked5 = PoseMarkFlag[req.GN][i];						
						break;
					case 5:
						res.lps6 = LandingPointString[req.GN][i];  // get LP String
						res.marked6 = PoseMarkFlag[req.GN][i];						
						break;
					case 6:
						res.lps7 = LandingPointString[req.GN][i];  // get LP String
						res.marked7 = PoseMarkFlag[req.GN][i];						
						break;
					case 7:
						res.lps8 = LandingPointString[req.GN][i];  // get LP String
						res.marked8 = PoseMarkFlag[req.GN][i];						
						break;
				}
			}
			//res.lps = LandingPointString[req.GN][req.LP];  // get LP String
			res.gps = GroupName[req.GN];
			//res.cLP = req.LP;
			res.nLP = GroupNumLP[req.GN];
			res.status = 57;
			//res.marked = PoseMarkFlag[req.GN][req.LP];
			//ROS_INFO("Request for Info. LP : %d. LP Name : %s. Gp Name : %s",req.LP,res.lps.c_str(),res.gps.c_str());
			break;
		case 51: // add LP Info data
			//ROS_INFO("GPN : %d. LP : %d", req.GN,req.LP);
			LandingPointString[req.GN][req.LP] = req.lps;  
			GroupName[req.GN] = req.gps;			
			res.status = 51;
			res.gps = GroupName[req.GN];
			res.lps = LandingPointString[req.GN][req.LP];
			numLP = GroupNumLP[req.GN];
			res.nLP = numLP;
			res.nGP = numG;
			if (req.LP >= numLP) {
				// new
				numLP = numLP + 1;
				res.nLP = numLP;
				GroupNumLP[req.GN]=numLP;
			}
			if (req.GN >= numG) {
				// new
				numG = numG + 1;
				res.nGP = numG;	
				nh.setParam("numG",numG);				
			}
			//res.nLP = numLP;
			//ROS_INFO("numG : %d. nLP : %d", res.nGP, res.nLP);
			break;
		case 52: // save admin password
			adminpwd = req.pw;
			nh.setParam("AdminPwd",adminpwd);	
			saveParamToFile();
			res.status = 52;
			break;
		case 53: // save setup data
			saveParamToFile();
			break;
		case 54: // delete rec
			//ROS_INFO("Delete Rec : %d. numLP : %d",req.LP,numLP);
			numLP = GroupNumLP[req.GN];
			if (req.LP == (numLP - 1)) {
				LandingPointString[req.GN][req.LP] = "";  				
				numLP = numLP - 1;
				GroupNumLP[req.GN] = numLP;
			} else {
					if (req.LP < (numLP - 1)) {
						for (int i=req.LP+1;i<numLP;i++) {
							LandingPointString[req.GN][i-1] = LandingPointString[req.GN][i];  
							PoseMarkFlag[req.GN][i-1] = PoseMarkFlag[req.GN][i]; 							
						}
						numLP = numLP - 1;
						GroupNumLP[req.GN] = numLP;
					}
			}
			res.status = 54;
			res.nLP = numLP;
			if (req.LP > 0) {
				res.cLP = req.LP - 1;					
			} else {
				res.cLP = req.LP;
			}			
			res.lps = LandingPointString[req.GN][res.cLP];	
			res.marked = PoseMarkFlag[req.GN][res.cLP];		
			break;
		case 56: // delete group rec
			//ROS_INFO("Delete Group Rec : GPno : %d. numG : %d",req.GN,numG);
			int cGP;
			if (req.GN == (numG - 1)) {
				//ROS_INFO("Delete End Group Rec : GPno : %d. numG : %d ",req.GN,numG);
				GroupName[req.GN] = "";  				
				numG = numG - 1;
				numLP = GroupNumLP[req.GN];
				for (int k=0;k<numLP;k++) {
					LandingPointString[req.GN][k]="";
					PoseMarkFlag[req.GN][k] = 0;
				}
				GroupNumLP[req.GN] = 0;
				cGP = numG-1;
				//ROS_INFO("Delete End Group Rec : cGP : %d. numG : %d ",cGP,numG);
			} else {
					if (req.GN < (numG - 1)) {
						for (int i=req.GN+1;i<numG;i++) {
							GroupName[i-1] = GroupName[i];
							numLP = GroupNumLP[i];
							GroupNumLP[i-1] = numLP;
							for (int j=0;j<numLP;j++) {
								LandingPointString[i-1][j] = LandingPointString[i][j];  
								PoseMarkFlag[i-1][j] = PoseMarkFlag[i][j]; 	
							}						
						}
						numG = numG - 1;
						cGP = req.GN;
					}
			}
			res.status = 56;
			numLP = GroupNumLP[cGP];
			res.nLP = numLP;
			res.cLP = 0;		
			res.nGP = numG;
			res.cGN = cGP;
			res.gps = GroupName[cGP];
			res.lps = LandingPointString[cGP][res.cLP];			
			//ROS_INFO("Exit Delete Group Rec : GPno : %d. numG : %d ",res.cGN,res.nGP);
			break;
		case 55: // switch mode
			if (mapflag == 77) {
				mapflag = 33;
				setMapFlagToMapMode();
				res.status = 33;
			} else {
				mapflag = 77;
				setMapFlagToNavMode();
				res.status = 77;
			}
			nh.setParam("mapflag",mapflag);
			break;
		case 60:			
			curGP = req.GN;
			nh.setParam("current_group",curGP);
			publish_updategroup("Change Group");
			//ROS_INFO("Change current group to %d",curGP);
			nh.setParam("change_group",true);
			res.status = 60;
			break;
		case 77:
			if (!firefox_enabled) {	
				//sprintf(buf,"firefox -new-window \"http://192.168.1.120/navstatus.html\"");
				//system(buf);
				readyflag = 88;
				nh.setParam("RobotReady",readyflag);
				//firefox_enabled = true;
				//publish_debug("Started FireFox");
			}
			break;
	}  
	return true;
}


 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "service node");  
  ros::NodeHandle rn;  		
	DBNode cNode(rn);
	cNode.initialisation();
	sleep(3);
	cNode.readParamfromFile();

	ros::MultiThreadedSpinner spinner(4);
  
	//ROS_INFO("Running DB Service Node...");

  spinner.spin();
  
  return 0;
}


