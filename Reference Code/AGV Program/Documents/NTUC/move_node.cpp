/*
 * This node is to service movement command  
 */
/* 	History
*		Date Modified : 8.5.2015
*		Changes : 
*     31.12.15 : standardise with ssmc
*     20.3.16 : Meiban new requirement. LP = 0 > origin or reference point. 1~9 : Group 1. 10~19 : Gp 2..
*/

//#define ODROID
//#define SSMC
//#define RAC

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/simple_action_client.h>
#include "htbot/Command.h"
#include "htbot/status.h"
#include "htbot/srvcmd.h"
#include "htbot/sendgoal.h"
#include "htbot/move.h"
#include "htbot/move_status.h"
//#include "htbot/dbcmd.h"
#include "htbot/Empty.h"
#include "htbot/queue.h"
#include "htbot/mqueue.h"
#include "htbot/go.h"
#include "boost/algorithm/string.hpp"
#include "std_srvs/Empty.h"
#include "htbot/clear.h"
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "htbot/debug.h"
#include "htbot/sound.h"
#include "htbot/scanCmd.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "htbot/lift.h"

#define MAXLPQ 100
#define MAXSTN 5
#define MAXQN 3
#define CTZ 0.118
#define CTX 0.5
#define NORMAL 0
#define ENGAGE 1
#define DISENGAGE 2
#define MOVETOCHARGE 3

#define STATIONA	0
#define STATIONB	3
#define STATIONC	5
#define STATIOND	7
#define STATIONE	9
#define REACHEDSTN	 77
#define CHARGING 79
#define ZSD 2.0    // zero start dist
#define ZSA 190.0  // zero start angle
#define PREMOVE 9  // pre move
#define POSTMOVE 10 // post move

using namespace std;
using namespace boost::algorithm;

class MovNode
{
public:
	MovNode();	
	bool cmdServiceCallback(htbot::srvcmd::Request &req,htbot::srvcmd::Response &res);	
	int findLPIndex(std::string lpstr);
	bool sendGoalpub(double x,double y, double z, double rx, double ry, double rz, double rw, double pd, double pa, int opt);

	void cmdCallback(const htbot::Command::ConstPtr& msg);	
	void movCallback(const htbot::move_status::ConstPtr& msg);
	void passCallback(const htbot::status::ConstPtr& msg);

	double calposeDist(int id1, int id2);	

	void pathplan(int nLP);
	void moveToLP(int nGP, int nLP,string sLP);
	void moveToLPpub(int nLP);
	bool moveToLPn(int nGP, int nLPPtr);
	void moveToLPnPre(int nGP, int nLP);
	bool getPose();

	void publish_status(string s);
	void publish_debug(string s);
	void publish_queue(void);
	void publish_go(int cmd);
	void moveLPLoop();
	void publish_changeMove(int stat);
	void publish_move_complete(string s);
	void publish_fButton(unsigned short cmd);
	void publish_sound(int id);
	void publish_arrive(string lps);
	
	bool dbRequest(int cmd, int GP, int LP);
	bool dbRequestS(std::string lps);
	bool clearCostmap(void);
	bool checkPasswd();
	void publish_clear(void);
	void keepCurrentPose(void);
	bool checkPlan(double x,double y, double z, double rx, double ry, double rz, double rw, double tol);
	void initialise(void);
	void voltCallback(const std_msgs::Float32::ConstPtr& msg);
	void currCallback(const std_msgs::Float32::ConstPtr& msg);
	void usoundCallback(const std_msgs::Float32::ConstPtr& msg);
	void buttonCallback(const std_msgs::UInt16::ConstPtr& msg);
	void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
	void publish_pose(void);
	void scanMatchAlign(int cmd);
	void initialpose(void);
	bool mqPop(void);
	void cancel_goal(void);
	// ntuc
	void publish_lift(int cmd, int cfloor, int dfloor,int inuse);
	void ntucCallback(const htbot::lift::ConstPtr& msg);
	

	ros::ServiceServer cmd_service;
	htbot::mqueue web_srv;
	ros::ServiceClient web_client;
	ros::ServiceClient clearcostmap;
	ros::ServiceClient makeplan;
	htbot::mqueue mq_srv;
	ros::ServiceClient mq_client;

	ros::Publisher status_pub;
	ros::Publisher debug_pub;
	ros::Publisher queue_pub;
	ros::Publisher go_pub;
	ros::Publisher move_pub;
	ros::Publisher movex_pub;
	ros::Publisher clear_pub;
	ros::Subscriber movstat_sub;
	ros::Subscriber pass_sub;
	ros::Publisher move_complete_pub;
	ros::Publisher pos_pub;
	ros::Publisher play_pub;
	ros::Subscriber pose_sub;
	ros::Publisher scanM_pub;
	ros::Publisher ipose_pub;
	ros::Publisher arrive_pub;
	// ntcu
	ros::Publisher lift_pub;
	ros::Subscriber lift_sub;
	// Uno 
	ros::Subscriber button_sub;
	ros::Subscriber volt_sub;
	ros::Subscriber curr_sub;
	ros::Subscriber usound_sub;
	ros::Publisher fButton_pub;
	ros::Publisher cancel_pub;
	bool button_mode; // true=navigation. false=mapping

	int numLP;
	int curLoc;
	int mstate,nstate;
	int abort_retry;

	double tx,ty,tz;
	double rx,ry,rz,rw;
	double pre_dist,pre_angle,post_dist,post_angle;
	double pre_dist1,pre_angle1,post_dist1,post_angle1;
	double start_dist, start_angle,start_dist1, start_angle1;
	double otx,oty,otz;
	double orx,ory,orz,orw;
	double call_time,voltage,charging_voltage,work_voltage,lowpower_voltage;
	double current,usound;
	int moveLPLoopQ[MAXLPQ];
	string LPLoopQString[MAXLPQ];
	string LPString[MAXLPQ];
	string sstring;
	std::string homedir;
	double gx,gy,grz,grw,px,py,prz,prw;
	
	int currentLP,nextLP,lastnLP,noLPIQ;
	bool nflag;
	int mapflag;
	int count;
	double pathDist;	
	double looprate;
	
	bool passwdreq,sendgoalready;
	int moveflag,movedone;
	string passwd,inputpw;
	string lastsLP,cGPName,cLPName;
	bool navReady;
	bool startNav;
	bool clearMapFlag;
	bool cmdFlag; // true = LP to move (targetGP, targetLP)
	bool fromToflag; // true = from. false = to.
	bool loadflag,unloadflag,startbot,botrun,measure_cstn;  // 
	int targetGP,targetLP;
	ros::Time clearmap_start_time,dock_time,lap_time,refresh_time;
	double clear_time;
	bool low_power,estop,low_power_flag;
	int charging_type,alignment_type;
	double delay;
	double align,autostart;  // autostart = 0.0 > no autostart. 1.0 > autostart
	int noIQ;	
	double ProfileMoveObsDist;
	// landing points queue
	double qposeInfo[MAXLPQ][15];	
	// ntuc
	bool ntucTest,NTUCINTERNALTEST,liftDoorOpen;
	int ntucstatus; // 1=lift arrived door opened.2=lift reached door opened
	int ntucfloor;
	int ntucLobbyDoor; //1=door open request received. 2=Door open activated.
	int ntucDoorOpenRequestCnt;
	double ntucDistToLobbyDoorOutside,ntucDistToLobbyDoorInside;

private:
	ros::NodeHandle nh;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::TransformStamped robotpose;
	boost::mutex mut;	
};

MovNode::MovNode():
	rx(0.0),ry(0.0),rz(0.0),rw(0.0),moveflag(0),movedone(0),clearMapFlag(false),noLPIQ(0),fromToflag(true),
	tx(0.0),ty(0.0),tz(0.0),passwdreq(false),sendgoalready(true),low_power(false),charging_type(0),
	currentLP(0),nextLP(0),mstate(0),curLoc(99),count(0),lastnLP(0),navReady(false),startNav(false),
	looprate(1.0),cmdFlag(false),targetGP(0),targetLP(0),clear_time(0.0),measure_cstn(false),
	loadflag(false),unloadflag(false),startbot(false),abort_retry(12),botrun(false),alignment_type(0),
	voltage(0.0),charging_voltage(27.0),lowpower_voltage(24.0),work_voltage(27.0),start_dist(2.0),
	start_angle(190.0),start_dist1(2.0),start_angle1(190.0),low_power_flag(false),estop(false),
  ntucTest(false),ntucstatus(0),ntucLobbyDoor(0),NTUCINTERNALTEST(false),liftDoorOpen(false),
	ntucDistToLobbyDoorOutside(1.5),ntucDistToLobbyDoorInside(1.5)
{
	ros::NodeHandle nh;
	cmd_service = nh.advertiseService("move_service",&MovNode::cmdServiceCallback,this); 
	movstat_sub = nh.subscribe<htbot::move_status>("move_status", 100, &MovNode::movCallback,this);
	pass_sub = nh.subscribe<htbot::status>("barcode", 100, &MovNode::passCallback,this);
	status_pub = nh.advertise<htbot::status>("feedback",100);
	debug_pub = nh.advertise<htbot::debug>("debug",100);
	queue_pub = nh.advertise<htbot::queue>("queue",100);
	go_pub = nh.advertise<htbot::go>("go",100);
	move_complete_pub = nh.advertise<htbot::status>("move_completed",100);
	movex_pub = nh.advertise<htbot::move_status>("move_change",100);
	play_pub = nh.advertise<htbot::sound>("sound", 5);
	move_pub = nh.advertise<htbot::move>("move", 100);
	web_client = nh.serviceClient<htbot::mqueue>("web_cmd");
	mq_client = nh.serviceClient<htbot::mqueue>("mqueue");
	clearcostmap = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	clear_pub = nh.advertise<htbot::clear>("clearMap", 100);
	makeplan = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
	pose_sub = nh.subscribe<geometry_msgs::Pose>("/robot_pose", 1, &MovNode::poseCallback,this);
	cancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

	nh.getParam("clear_time",clear_time);
	nh.getParam("charging_type",charging_type);
	nh.getParam("Abort_Retry",abort_retry);
	nh.getParam("charging_voltage",charging_voltage); // move to dock only if voltage above this
	nh.getParam("lowpower_voltage",lowpower_voltage); // trigger low power.
	nh.getParam("work_voltage",work_voltage);// can do job above this voltage
	nh.getParam("alignment_type",alignment_type);

	nh.getParam("ntucDistToLobbyDoorOutside",ntucDistToLobbyDoorOutside);
	nh.getParam("ntucDistToLobbyDoorInside",ntucDistToLobbyDoorInside);

	//ROS_INFO("***** Charging Voltage : %.3f *****",charging_voltage);
	pos_pub = nh.advertise<geometry_msgs::Twist>("cmd_pos", 1);
	scanM_pub = nh.advertise<htbot::scanCmd>("scanCmd", 1);
	ipose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
	arrive_pub = nh.advertise<std_msgs::String>("arrive",1);

  // Uno
	button_sub = nh.subscribe<std_msgs::UInt16>("button", 100, &MovNode::buttonCallback,this);
	fButton_pub = nh.advertise<std_msgs::UInt16>("fbutton",1);
	volt_sub = nh.subscribe<std_msgs::Float32>("voltage", 100, &MovNode::voltCallback,this);
	curr_sub = nh.subscribe<std_msgs::Float32>("current", 100, &MovNode::currCallback,this);
	usound_sub = nh.subscribe<std_msgs::Float32>("usound", 100, &MovNode::usoundCallback,this);

	// NTUC Lift Operation
	lift_pub = nh.advertise<htbot::lift>("to_lift",100);
	lift_sub = nh.subscribe<htbot::lift>("fr_lift", 100, &MovNode::ntucCallback,this);
	
	// nh.getParam("home_dir",homedir);
	nh.param<std::string>("home_dir",homedir,"/home/rac-tprf/");
}

void MovNode::publish_lift(int cmd, int cfloor, int dfloor,int inuse)
{
	htbot::lift msg;
	msg.cmd = cmd;
	msg.cfloor = cfloor;
	msg.dfloor = dfloor;
	msg.inuse = inuse;
	lift_pub.publish(msg);
	return;
}

void MovNode::ntucCallback(const htbot::lift::ConstPtr& msg)
{
	//ros::NodeHandle rn; 
	int cmd,cfloor,dfloor;

	cmd = msg->cmd; 
	//ROS_INFO("----------- from lift : cmd = %d ----------------",cmd);
	switch (cmd) {
		case 10: // lift arrived at start level and lift door is kept opened. cfloor =m
			ntucstatus = 10;
			ntucfloor = msg->cfloor;
			liftDoorOpen = true;
			ROS_INFO("----------- from lift : cmd = %d ----------------",cmd);
			break;
		case 11: // lift arrived at end level and lift door is kept opened. dfloor =m
			ntucstatus = 11;
			ntucfloor = msg->dfloor;
			liftDoorOpen = true;
			ROS_INFO("----------- from lift : cmd = %d ----------------",cmd);
			break;
		case 21: // Door open status. dooropen = 0/1
			//ntucstatus = 21;
			//if (msg->dooropen == 1) {
			//	liftDoorOpen = true;
			//} else {
			//	liftDoorOpen = false;
			//}
			break;
		case 22: // Door close status. doorclose = 0/1
			//ntucstatus = 22;
			//if (msg->doorclose == 1) {
			//	liftDoorOpen = false;
			//} else {
			//	liftDoorOpen = true;
			//}
			break;
		case 100:  // lift arrived. door opened
			//ntucstatus = 100;
			//ntucfloor = msg->dfloor;
			break;
		case 101:
			//ntucstatus = 101;
			//ntucfloor = msg->dfloor;
			break;
		case 102:
			//ntucstatus = 102;
			break;
	}
}

void MovNode::initialise(void) 
{
	web_srv.request.GN = 0;  
	web_srv.request.LP = 0;  		
	web_srv.request.cmd = 57;

	if (web_client.call(web_srv)) {
		if (web_srv.response.status == 57) {	
			LPString[0] =  web_srv.response.lps1;
			LPString[1] =  web_srv.response.lps2;
			LPString[2] =  web_srv.response.lps3;
			LPString[3] =  web_srv.response.lps4;
		} else {
	    ROS_ERROR("Failed to call web service : Move Node");	
	 	}
	}
}

void MovNode::cancel_goal(void)
{
	//MoveBaseClient ac("move_base", true); // move_base
	//while(!ac.waitForServer(ros::Duration(5.0))){
  //	ROS_INFO("Waiting for the move_base action server to come up : web_node");
  //}
	//ac.cancelGoal();
	actionlib_msgs::GoalID msg;
	msg.stamp = ros::Time::now();
	msg.id = "CancelGoal";
	cancel_pub.publish(msg);
	return;
}

void MovNode::initialpose() 
{
	geometry_msgs::PoseWithCovarianceStamped ipose;
	geometry_msgs::Quaternion quat;

	//ROS_INFO("initialpose..");
	ipose.header.frame_id = "map";
	ipose.header.stamp = ros::Time::now();
	ipose.pose.pose.position.x = tx;
	ipose.pose.pose.position.y = ty;
	ipose.pose.pose.position.z = tz;
	ipose.pose.pose.orientation.x = rx;
	ipose.pose.pose.orientation.y = ry;
	ipose.pose.pose.orientation.z = rz;
	ipose.pose.pose.orientation.w = rw;
	ipose.pose.covariance[0] = 1e-3;
	ipose.pose.covariance[7] = 1e-3;
	ipose.pose.covariance[14] = ipose.pose.covariance[21] = ipose.pose.covariance[28] = 1e9;
	ipose.pose.covariance[35] = 1e-3;
	ipose_pub.publish(ipose);
}

bool MovNode::mqPop() {
	bool ret;

	mq_srv.request.cmd = 2;
		
	if (mq_client.call(mq_srv)) {
		if (mq_srv.response.status == 1) {	 
			tx = mq_srv.response.tx;   	
			ty = mq_srv.response.ty;
			tz = mq_srv.response.tz;
			rx = mq_srv.response.rx;   	
			ry = mq_srv.response.ry;
			rz = mq_srv.response.rz;
			rw = mq_srv.response.rw;
			pre_dist = mq_srv.response.prd;
			pre_angle = mq_srv.response.pra;
			post_dist = mq_srv.response.psd;
			post_angle = mq_srv.response.psa;
			pre_dist1 = mq_srv.response.prd1;
			pre_angle1 = mq_srv.response.pra1;
			post_dist1 = mq_srv.response.psd1;
			post_angle1 = mq_srv.response.psa1;
			align = mq_srv.response.align;
			autostart = mq_srv.response.autostart;
			targetLP = mq_srv.response.LP;
			targetGP = 0; 
			cLPName =  mq_srv.response.lps;
			ROS_INFO("------- MovNode : Align : %.3f --------------",align);
			//cmdFlag = true;
			ret = true;
		} else {
	    ROS_ERROR("Failed to pop to move queue");	
			ret = false;
	 	}
	}
	return ret;
}


void MovNode::buttonCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	//ros::NodeHandle rn; 
	int nGP, nLP, butNo;
	//bool ok;
	geometry_msgs::Twist pos;
	bool ok;
	char buf [100];
	string s;

	butNo = msg->data; 
	//rn.getParam("current_group",nGP);
	//ROS_INFO("Button_Node : Button Code Received = %d",butNo);
	//sprintf(buf,"Button_Node : Button Code Received = %d",butNo);
	//s.assign(buf,strlen(buf));
	//publish_debug(s);

	switch (butNo) {
/*
		case 1:
			targetGP = 0;
			targetLP = 1;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;
		case 2:
			targetGP = 0;
			targetLP = 2;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;
		case 3:
			targetGP = 0;
			targetLP = 3;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;
		case 4:
			targetGP = 0;
			targetLP = 4;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;
		case 5:
			targetGP = 0;
			targetLP = 5;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;
		case 6:
			targetGP = 0;
			targetLP = 6;
			//ROS_INFO("GP : %d. LP : %d",targetGP,targetLP);
			cmdFlag = true;
			break;		
		case 13: // robot charging now. Stop robot
			// stop robot.
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.0;  // 7.0
			pos_pub.publish(pos);
			publish_status("Docking Activated");
			publish_debug("Move Node : Docking Achieved.Stop Robot");
			break;
*/
		case 15:
			break;
		case 155:
			// stop robot.
			if (!estop) {
				pos.angular.z = 0.0;
				pos.linear.x = 0.0;
				pos.linear.z = 7.8;  // 7.0
				pos_pub.publish(pos);
				publish_status("Emergency Stop Activated");
				publish_debug("Move Node : Emergency Stop Activated");
				estop = true;
				//publish_sound(15);		
				//sleep(2);
			}
			break;
		case 16:
			if (estop) {
				pos.angular.z = 0.0;
				pos.linear.x = 0.0;
				pos.linear.z = 7.9;  // 7.0
				pos_pub.publish(pos);
				publish_status("Emergency Stop Released");
				publish_debug("Move Node : Emergency Released");
				estop = false;
			}
			break;
		case 32:
			ROS_INFO("Cancelled Move");
			publish_sound(17);  // navigation aborted
			cancel_goal();
			break;
		case 300:
			ntucLobbyDoor = 1;
			break;
		case 302:
			ntucLobbyDoor = 2;
			break;
	}

	return; 
}


void MovNode::keepCurrentPose(void)
{
	otx = tx;
	oty = ty;
	otz = tz;
	orx = rx;
	ory = ry;
	orz = rz;
	orw = rw;
}

void MovNode::publish_sound(int id)
{
	htbot::sound cmd;
	cmd.id = id;
	play_pub.publish(cmd);
	//ROS_INFO("Publish Sound AA");
	return;
}

void MovNode::publish_fButton(unsigned short cmd)
{
	std_msgs::UInt16 msg;
	msg.data = cmd;
	fButton_pub.publish(msg);
	return;
}

void MovNode::publish_arrive(string lps)
{
	std_msgs::String msg;
	msg.data = lps;
	arrive_pub.publish(msg);
	return;
}

void MovNode::voltCallback(const std_msgs::Float32::ConstPtr& msg)
{
	ros::NodeHandle xn; 
	{ 
		boost::mutex::scoped_lock lock(mut);
		voltage = msg->data;		
	}
	//ROS_INFO("\n Move_Node : Voltage : %.3f \n",voltage);
	if ((voltage < lowpower_voltage) && (!low_power)) {
		// low power
		low_power = true;
		//publish_debug("Move Node : Lower Power Activated");
		publish_status("Lower Power Activated");
		//xn.setParam("current_station",99);  // reference point
		//publish_move_comple// =============================  //te("update stations");
	} else {
		if ((voltage > work_voltage) && (low_power)) {
			low_power = false;
			//publish_debug("Move Node : Power Restored");
			publish_status("Transporter Sufficiently Charged");
		}
	}
	return;
}

void MovNode::currCallback(const std_msgs::Float32::ConstPtr& msg)
{
	ros::NodeHandle xn; 
	char buf [100];
	string s;

	{ 
		boost::mutex::scoped_lock lock(mut);
		current = msg->data;		
	}
	if (current > 3.0) {
		sprintf(buf,"Current Usage > 3.0A : %.3f",current);
		s.assign(buf,strlen(buf));
		publish_debug(s);
	}
	
	return;
}

void MovNode::usoundCallback(const std_msgs::Float32::ConstPtr& msg)
{
	ros::NodeHandle xn; 
	{ 
		boost::mutex::scoped_lock lock(mut);
		usound = msg->data;		
	}

	return;
}

void MovNode::publish_go(int cmd)
{
	htbot::go gmsg;
	gmsg.cmd = cmd;
	go_pub.publish(gmsg);
	return;
}

void MovNode::publish_move_complete(string s)
{
	htbot::status status;
	status.msg = s;
	move_complete_pub.publish(status);  // subscribed by index.html
	publish_status("Transporter Ready for Job");
	publish_fButton(targetLP);
	publish_arrive(cLPName);
	return;
}

void MovNode::publish_clear(void)
{
	htbot::clear cmd;
	cmd.cmd = 1;
	//ROS_INFO("Move_Node : publise clear Map");
	publish_debug("Move_Node : Publish Clear Map" );
	clear_pub.publish(cmd);
	return;
}

bool MovNode::checkPlan(double x,double y, double z, double rx, double ry, double rz, double rw, double tol) {
	
	nav_msgs::GetPlan srv;
	bool ret;
		
	ret = false;

	srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = otx;
  srv.request.start.pose.position.y = oty;
	srv.request.start.pose.position.z = otz;
	srv.request.start.pose.orientation.x = orx;
	srv.request.start.pose.orientation.y = ory;
	srv.request.start.pose.orientation.z = orz;
  srv.request.start.pose.orientation.w = orw;

	srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = x;
  srv.request.goal.pose.position.y = y;
	srv.request.goal.pose.position.z = z;
	srv.request.goal.pose.orientation.x = rx;
	srv.request.goal.pose.orientation.y = ry;
	srv.request.goal.pose.orientation.z = rz;
	srv.request.goal.pose.orientation.w = rw;
	srv.request.tolerance = tol;
	if (makeplan.call(srv)) {
		if (srv.response.plan.poses.empty()) {
			//publish_debug("Move_Node : Got empty plan" );
  	 	//ROS_INFO("Move_Node : Got empty plan");
			ret = false;
    } else {
			//publish_debug("Move_Node : Found Plan" );
			ret = true;
		}
 	} else {
		//publish_debug("Move_Node : Failed to call make plan service");
  	//ROS_INFO("Move_Node : Failed to call make plan service");
  }
	return ret;
}

bool MovNode::clearCostmap(void) {
	
	std_srvs::Empty srv;
	bool ret;
		
	ret = false;
	if (clearcostmap.call(srv)) {
		//publish_debug("Cleared Costmaps");
		ret = true;		
	} else {
		//publish_debug("Cleared Costmaps Failed");
		ret = false;
	}
	return ret;
}

void MovNode::movCallback(const htbot::move_status::ConstPtr& msg)
{
	//ROS_INFO("Move Status");
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		moveflag = msg->stat;	
		movedone = 0;
	}
}

void MovNode::passCallback(const htbot::status::ConstPtr& msg)
{
	//ROS_INFO("Password");
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		inputpw = msg->msg;	
	}
}

void MovNode::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	double x,y,z,rz,rw;
	
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		px = msg->position.x;
		py = msg->position.y;
		prz = msg->orientation.z;
		prw = msg->orientation.w;
	}
	//ROS_INFO("Pose : x=%.2f. y=%.2f. rz=%.2f",x,y,rz);
}

void MovNode::publish_pose(void)
{
	double x,y,z,rz,rw;	
	char buf [100];
	string s;
	
	sprintf(buf,"Pose : px=%.3f.py=%.3f.prz=%.3f.prw=%.3f",px,py,prz,prw);
	s.assign(buf,strlen(buf));
	publish_debug(s);
}

void MovNode::scanMatchAlign(int cmd) {
	htbot::scanCmd scmd;
	ros::NodeHandle nm;
	char buf [100];
	string s,sdir;

	//sdir = homedir + "catkin_ws/src/htbot/laser/RefScan%d.json";
	sdir = homedir + "catkin_ws/src/htbot/laser/" + cLPName + ".json";
	scmd.cmd = cmd;
	//sprintf(buf,"/home/racnys/catkin_ws/src/htbot/laser/RefScan%d.json",targetLP);
	sprintf(buf,sdir.c_str(),targetLP);
	s.assign(buf,strlen(buf));
	scmd.file = s;
	scmd.lp = targetLP;
	scmd.gp = targetGP;
	scanM_pub.publish(scmd);
}

bool MovNode::sendGoalpub(double x,double y, double z, double rx, double ry, double rz, double rw, double pd, double pa, int opt)
{
	char buf [100];
	string s;
	ros::NodeHandle nm;
	htbot::move cmd;
	
	cmd.x = x;
	cmd.y = y;
	cmd.z = z;
	cmd.rx = rx;
	cmd.ry = ry;
	cmd.rz = rz;
	cmd.rw = rw;
	cmd.pd = pd;
	cmd.pa = pa;
	cmd.opt = opt;
	
	gx = x; 
	gy = y;
	grz = rz;

	/*
	if (opt == 9) {
		movedone = 0;
		moveflag = 1;
		return true;
	}
	*/

	if (opt == 0) {
		sprintf(buf,"SendGoal : gx=%.3f. gy=%.3f. grz=%.3f.",gx,gy,grz);
		s.assign(buf,strlen(buf));
		publish_debug(s);
	}	
	
	move_pub.publish(cmd);
	//ROS_INFO("Move_Node : Move Command Published to move");
		
	return true;
}

bool MovNode::getPose() 
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

int MovNode::findLPIndex(std::string lpstr) 
{
	int ret;
	string s1,s2;
	//ret = -99;
	s2 = lpstr;
	char cp[100];
	const char *p;
	int id,num;

	num = 4;
	trim(s2);
	//ROS_INFO("LPS :  %s",s2.c_str());
	for (int i=0;i<MAXSTN;i++) {
		s1 = LPString[i];
		trim(s1);
		//ROS_INFO("Array S :  %s",s1.c_str());
		if (s2.compare(s1) == 0) {
			num = i;
			break;
		}
	}
	//ROS_INFO("Num :  %d",num);
	return num;
}

bool MovNode::dbRequest(int cmd, int GP, int LP) {
	bool ret;

	web_srv.request.GN = GP;  
	web_srv.request.LP = LP;  		
	web_srv.request.cmd = cmd;
			
	//ROS_INFO("dbReequest : %d. LP : %d ",cmd,LP);
		
	if (web_client.call(web_srv)) {
		//ROS_INFO("Received reply from dbase Server");
		if (web_srv.response.status == 32) {	 
			//ROS_ERROR("call dbase service OK");	 		 
			tx = web_srv.response.tx;   	
			ty = web_srv.response.ty;
			tz = web_srv.response.tz;
			rx = web_srv.response.rx;   	
			ry = web_srv.response.ry;
			rz = web_srv.response.rz;
			rw = web_srv.response.rw;
			pre_dist = web_srv.response.prd;
			pre_angle = web_srv.response.pra;
			post_dist = web_srv.response.psd;
			post_angle = web_srv.response.psa;
			pre_dist1 = web_srv.response.prd1;
			pre_angle1 = web_srv.response.pra1;
			post_dist1 = web_srv.response.psd1;
			post_angle1 = web_srv.response.psa1;
			//passwd = db_srv.response.pw;
			cGPName = web_srv.response.gps;
			cLPName = web_srv.response.lps;
			ret = true;
		} else {
	    ROS_ERROR("Failed to call web service");	
			ret = false;
	 	}
	}
	return ret;
}

bool MovNode::dbRequestS(std::string lps) {
	bool ret;
		
	web_srv.request.cmd = 30;
	web_srv.request.lps = lps;		

	if (web_client.call(web_srv)) {
		//ROS_INFO("Received reply from dbase Server");
		if (web_srv.response.status == 30) {	 
			//ROS_ERROR("call dbase service OK");	 		 
			tx = web_srv.response.tx;   	
			ty = web_srv.response.ty;
			tz = web_srv.response.tz;
			rx = web_srv.response.rx;   	
			ry = web_srv.response.ry;
			rz = web_srv.response.rz;
			rw = web_srv.response.rw;
			pre_dist = web_srv.response.prd;
			pre_angle = web_srv.response.pra;
			post_dist = web_srv.response.psd;
			post_angle = web_srv.response.psa;
			pre_dist1 = web_srv.response.prd1;
			pre_angle1 = web_srv.response.pra1;
			post_dist1 = web_srv.response.psd1;
			post_angle1 = web_srv.response.psa1;
			targetLP = web_srv.response.LP;
			cLPName = web_srv.response.lps;
			ret = true;
		} else {
	    ROS_ERROR("Failed to call web service");	
			ret = false;
	 	}
	} else {
		ret = false;
	}
	return ret;
}


bool MovNode::checkPasswd() {
	bool ret;
	//ROS_INFO("Check Passwd");
	publish_status("Waitng for Password..");
	trim(inputpw);
	trim(passwd);
	if (inputpw.compare(passwd) == 0) {
		ret = true;
	} else {
		ret = false;
	}
	return ret;
}

bool MovNode::moveToLPn(int nGP, int nLP)
{
	bool ret;
	//ROS_INFO("moveToLPn");
	if (curLoc == nLP) {
		moveflag = 1;
		movedone = 0;
		ret = true;
	}

	//if (nLP == 0) {
	//	publish_fButton(77);
	//}
	
	if (dbRequest(32,nGP,nLP)) {
		//publish_status("Moving to Call Station : "+cLPName);
		publish_status("Moving to Destination : "+cLPName);
		publish_debug("Move Node : Moving to Destination : "+cLPName);		
		//ROS_INFO("Moving to Call Station : %s",cLPName.c_str());
		ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
		moveflag = 0;  // set by send_goal
		movedone = 1;
		curLoc = nLP;
		ret = true;
	} else {
		ROS_INFO("Error in DB Request : from");
		ret = false;
	}
	return ret;
}

void MovNode::moveToLPnPre(int nGP, int nLP)
{
	bool ret;
	
	/*
	if (curLoc == nLP) {
		moveflag = 1;
		movedone = 0;
		return;
	}
	*/
	//ROS_INFO("moveToLPnPre");
	if (dbRequest(32,nGP,nLP)) {
		publish_status("PreMove");
		//publish_status("Moving to Destination : "+cLPName);	
		//ROS_INFO("Moving to Call Station : %s",cLPName.c_str());
		ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist,start_angle,PREMOVE);
		moveflag = 0;  // set by send_goal
		//curLoc = nLP;
		ret = true;
	} else {
		ROS_INFO("Error in DB Request : Premove");
		ret = false;
	}

}


void MovNode::moveToLP(int nGP, int nLP, string sLP)
{
	int nidx,cnt;
	char buf[100];
	string s,s1;
	bool ret;

	ROS_INFO("moveToLP");
	cnt = 0;
	lastnLP = nLP;
	if (nLP == MAXLPQ) {
		// move to charging point
		publish_status("Moving to Charging Area");
		//publish_debug("Move Node : Moving to Charging Area");
		ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,MOVETOCHARGE);
		moveflag = 0;  // set by send_goal
		curLoc = nLP;
	} else {
		if (nLP == (MAXLPQ-1)) {
			// move to charger contact
			//publish_debug("Move Node : Moving to Charging Contact");
			publish_status("Moving to Charger Contact" );
			ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,ENGAGE);
			moveflag = 0;  // set by send_goal
		} else {
			if (nLP == (MAXLPQ-2)) {
				// move away from charger
				//publish_debug("Move Node : Moving Away from Charger Contact");
				publish_status("Moving Away from Charger Contact");
				ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,DISENGAGE);
				moveflag = 0;  // set by send_goal
			} else {
				publish_status("Moving to : "+sLP);
				//publish_debug("Move Node : Moving to : "+sLP);
				//clearCostmap();
				if (dbRequest(32,nGP,nLP)) {
					//ROS_INFO("Moving to %s",sLP.c_str());
					ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
					inputpw = "xxx";
					moveflag = 0;  // set by send_goal
					//mstate = 1;
					curLoc = nLP;
				} else {
					ROS_INFO("Error in DB Request");
				}
			}
		}
	}
}


void MovNode::publish_status(string s)
{
	htbot::status status;
	status.msg = s;
	sstring = s;
	status_pub.publish(status);
	return;
}

void MovNode::publish_debug(string s)
{
	htbot::debug status;
	status.msg = s;
	debug_pub.publish(status);
	return;
}

void MovNode::publish_changeMove(int stat)
{
	htbot::move_status msg;
	msg.stat = stat;
	movex_pub.publish(msg);
	return;
}

void MovNode::publish_queue(void)
{
	string sf,st;
	int cLP,nLP,idx,nq,lp;

	//ROS_INFO("Publish Queue");
	sf = "";
	st = "";
	//htbot::status qmsg;
	htbot::queue qmsg;
	cLP = currentLP;
	nLP = nextLP;
	nq = noLPIQ;
	idx = 0;
	//ROS_INFO("cLP : %d. nLP : %d. nq : %d",cLP,nLP,nq);
	if (nq > 0) {
		while(cLP != nLP) {
			lp = moveLPLoopQ[cLP];
			if (lp >= 200) {
				// to
				st = LPLoopQString[cLP];
				switch (idx) {
					case 0:						
						qmsg.tLP1 = st;
						break;
					case 1:
						qmsg.tLP2 = st;
						break;
					case 2:
						qmsg.tLP3 = st;
						break;
				}	
				idx++;
			} else {
				// from
				sf = LPLoopQString[cLP];
				switch (idx) {
					case 0:						
						qmsg.fLP1 = sf;
						break;
					case 1:
						qmsg.fLP2 = sf;
						break;
					case 2:
						qmsg.fLP3 = sf;
						break;
				}			
			}			
			cLP++;
			if (cLP == MAXLPQ) {
				cLP = 0;
			}
			//idx++;
		}
	} else {
		// empty queue
		nq = 0;
		qmsg.fLP1 = "";
		qmsg.fLP2 = "";
		qmsg.fLP3 = "";
		qmsg.tLP1 = "";
		qmsg.tLP2 = "";
		qmsg.tLP3 = "";
	} 
	qmsg.noQ = nq;
	queue_pub.publish(qmsg);
	return;
}

bool MovNode::cmdServiceCallback(htbot::srvcmd::Request &req,htbot::srvcmd::Response &res)
{
	//ROS_INFO("Movement Services"); 
	int fi,ti;
	char buf [100];
	int readyflag;
	int cmd;
	bool ret;
	ros::NodeHandle xn; 
	geometry_msgs::Twist pos;
	string sdir;
	cmd = req.cmd;
  
	switch (cmd) {
		case 1: // 			
			ROS_INFO("Nav Command 1 : from fLP to tLP");			
			if (low_power) {
				res.status = -1;			
				break;
			}
			if (noLPIQ == MAXQN) {
				// full
				res.status = 0;
				break;
			}
			fi = findLPIndex(req.fromLP);
			ti = findLPIndex(req.toLP);
			ROS_INFO("From id : %d. To id : %d",fi,ti);
			//ROS_INFO("From LP = %s.",req.fromLP.c_str());	
			targetGP = req.cGP;
			LPLoopQString[nextLP] = req.fromLP;
			moveLPLoopQ[nextLP++] = fi+100;  // 100 > from
			
			if (nextLP == MAXLPQ) {
				nextLP = 0;
			}
			if (fi != ti) {
				LPLoopQString[nextLP] = req.toLP;
				moveLPLoopQ[nextLP++] = ti+200;  // 200 > to
				if (nextLP == MAXLPQ) {
					nextLP = 0;
				}
			}
			noLPIQ = noLPIQ + 1;
			//ROS_INFO("nextLP = %d. currentLP = %d. noLPIQ = %d",nextLP,currentLP,noLPIQ);	
			publish_queue();
			//ROS_INFO("End");
			res.status = 1;			
			break;
		case 11: // 			
			ROS_INFO("Nav Command 11 : GP : %d. LP : %d",req.cGP,req.cLP);
			//publish_debug("Move Node : Nav Cmd 11");
			//ti = findLPIndex(req.toLP);			
			targetGP = req.cGP;
			targetLP = req.cLP;
			cmdFlag = true;
			publish_fButton(100+targetLP);
			//moveLPLoopQ[nextLP++] = ti;	
			res.s1="";
			res.s2="";						
			res.status = 11;			
			break;		
		case 12: // trigger goals
			//ROS_INFO("Trigger Goal in Move Service");
			moveLPLoop();
			break;
		case 2: // Set/Reset Map
			// set mapflag t0 33 and shutdown system
			
			//setMapFlagToMapMode();
			//ROS_INFO("Shutting Now..");
			sleep(2);
			system("sudo shutdown -h now");
			res.status = 2;
			break;
		case 3:
			//ROS_INFO("Testing Clear Costmap");
			publish_debug("Move Node : Testing Clear Costmap");
			publish_clear();	
			res.status = 3;		
			break;
		case 31:
			//ROS_INFO("Testing GetMap for Global Costmap");
			sdir = homedir+"remap.sh";
			publish_debug("Move Node : Testing GetMap for Global Costmap..");
			//system("/home/racnys/remap.sh");
			system(sdir.c_str());
			res.status = 31;
			break;
		case 4:
			// low power
			low_power = true;
			//publish_debug("Move Node : Lower Power Activated");
			publish_status("Lower Power Activated");
			//xn.setParam("current_station",99);  // reference point
			//publish_move_complete("update stations");
			res.status = 4;
			break;
		case 41:
			// robot has power now
			low_power = false;
			//publish_debug("Move Node : Power Restored");
			publish_status("Transporter Sufficiently Charged");
			res.status = 41;
			break;
		case 42:
			// testing laser
			ret = sendGoalpub(0.0,0.0,0.0,0.0,0.0,0.0,0.0,ZSD,ZSA,1);	
			//publish_debug("Move Node : Activate Laser. Move to charge");
			res.status = 42;
			break;
		case 43:
			// testing laser. return to ref
			ret = sendGoalpub(0.0,0.0,0.0,0.0,0.0,0.0,0.0,ZSD,ZSA,2);	
			//publish_debug("Move Node : Activate Laser. Move to charge 2");
			res.status = 43;
			break;
		case 44:
			// test robot.
			pos.angular.z = 60.0;
			pos.linear.x = 5.0;
			pos.linear.y = 500.0;
			pos.linear.z = 0.0;  // 7.0
			pos_pub.publish(pos);
			//publish_debug("Move Node : Test Robot");
			res.status = 45;
			break;
		case 45:
			// stop robot.
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.0;  // 7.0
			pos_pub.publish(pos);
			//publish_debug("Move Node : Stop Robot");
			res.status = 45;
			break;
		case 46: // transporter loaded and ready to move to destination
			startNav = true;
			break;
		case 47: // transporter unloaded and ready to move to call station for next job
			unloadflag = false;
			break;
		case 48:
			//ROS_INFO("Move Node : Start Robot Pressed");
			startNav = true;
			publish_debug("Move Node : Move Triggered");
			//ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,5); // measure stn size
			break;
		case 49:
			measure_cstn = true;
			break;
		case 50: // rotate 180 deg
			pos.linear.z = 9.0; 
			pos_pub.publish(pos);
			break;
		case 51:  // move straight
			ROS_INFO("move_node : move straight");
			pos.linear.z = 9.5;  
			pos_pub.publish(pos);
			break;
		case 52: // rotate 180 deg
			pos.linear.z = 9.1; 
			pos_pub.publish(pos);
			break;
		case 53:  // move straight
			pos.linear.z = 9.6;  
			pos_pub.publish(pos);
			break;
		case 54:  // move straight
			publish_sound(3);
			break;
		case 55:  // move straight
			publish_sound(13);
			break;
		case 56:  // test estop activation
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.8;  // 7.0
			pos_pub.publish(pos);
			publish_status("Emergency Stop Activated");
			publish_debug("Move Node : Emergency Stop Activated");
			estop = true;
			publish_sound(15);		
			sleep(2);
			break;
		case 57:  // test estop release
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.9;  // 7.0
			pos_pub.publish(pos);
			publish_status("Emergency Stop Released");
			publish_debug("Move Node : Emergency Released");
			estop = false;
			break;
		case 58:
			initialpose();
			res.status = 58;	
			break;
		case 59:
			ROS_INFO("Test Move robot");
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 45.0;  // 7.0
			pos_pub.publish(pos);
			break;
		case 60:  // foot3
			ROS_INFO("Change footprint to 0.3");
			system("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap /home/agv/catkin_ws/src/htbot/config/footprint3.yaml");
			system("rosrun dynamic_reconfigure dynparam load /move_base/local_costmap /home/agv/catkin_ws/src/htbot/config/footprint3.yaml");
			break;
		case 61:  // foot45
			ROS_INFO("Change footprint to 0.45");
			system("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap /home/agv/catkin_ws/src/htbot/config/footprint4.yaml");
			system("rosrun dynamic_reconfigure dynparam load /move_base/local_costmap /home/agv/catkin_ws/src/htbot/config/footprint4.yaml");
			break;
	}  
	return true;
}

void MovNode::moveLPLoop()
{
	int cLP,nLP,mLP,nGP,nLPIQ;
	string sLP, sGP;
	int readyflag;
	bool start,ret,change_group;
	ros::NodeHandle rn;  
	double secs;
	char buf[100];
	string s1,sdir;
	int LeftLaser,RightLaser;
	geometry_msgs::Twist pos;

	//while(ros::ok())
	
	//ROS_INFO("Move Loop");
	//publish_queue();
	start = false;
	change_group = false;
	switch (mstate) {
		case 0: // no movement state								
			{  // lock scope
				boost::mutex::scoped_lock lock(mut);
				nGP = targetGP;
				nLP = nextLP;
				cLP = currentLP;
				nLPIQ = noLPIQ;
			}			
			if (estop) {  // estop
				break;
			}
			//ROS_INFO("------- Move : mstate = %d ----------",mstate);
			rn.getParam("noIQ",noIQ);
			rn.getParam("ntucTest",ntucTest);
			if (ntucTest) {
				ROS_INFO("------- Move : ntucTest ----------");
			}
			// wait for main laser to be up.
			if (!botrun) {
				rn.getParam("RobotReady",readyflag);
				if (readyflag == 77) {
					botrun = true;
					publish_fButton(88);  // robot ready to move
					if (dbRequestS("POWER")) {  // pre move i
						start_dist = pre_dist;
						start_angle = pre_angle;
						start_dist1 = pre_dist1;
						start_angle1 = pre_angle1;
						ROS_INFO("Stdist : %.3f. stangle : %.3f Stdist1 : %.3f. stangle1 : %.3f",start_dist,start_angle,start_dist1,start_angle1);
						publish_sound(1);
						initialpose();
					} else {
						start_dist = 2.0;
						start_angle = 190.0;
						start_dist1 = 2.0;
						start_angle1 = 190.0;
					}
				}
				//ROS_INFO("botrun");
				//movedone = 1;
				//ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,5);
				//mstate = 104;
				//clearmap_start_time = ros::Time::now();
				//count = 0;
				break;
			}
			if (ntucTest) {
				ROS_INFO("------------- ntuc Test Activated -------------");
				//publish_sound(22); // request to open Lobby Door
				//publish_fButton(301);  // instruct arduino to open door to lift lobby
				//rn.setParam("lookforObs",1);  // 
				//rn.getParam("NTUCINTERNALTEST",NTUCINTERNALTEST);
				if (dbRequestS("WIRECUT")) {  // pre move i
					movedone = 1;
					mstate = 1;
					nstate = 6;
					ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist,start_angle,NORMAL);
					sdir = "gnome-terminal -x "+homedir+"ssmc.sh &";
					system(sdir.c_str());
				} else {
					ROS_INFO("---------- ntuc : Fail to get Wirecut ------------------");
				}
				//ntucDoorOpenRequestCnt = 0;
				clearmap_start_time = ros::Time::now();
				break;
			}
			// low power. complete current job and go to charging station.
			/*
			if (low_power && botrun) {
			//if (low_power)  {
				
				publish_sound(16);
				ROS_INFO("-------------- Move : Low Power---------------");
				if (dbRequestS("POWER")) {  // pre move i
					publish_fButton(99); // low power, moving to reference point
					publish_status("Lower Power. Moving to Charging Station");
					movedone = 1;
					mstate = 102;
					low_power_flag = true;
					ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist,start_angle,PREMOVE);
					clearmap_start_time = ros::Time::now();
				} else {
					publish_status("Lower Power. No Charging Station");
				}
				
				break;
			}
			*/
			// ssmc button
			//if ((cmdFlag)) {
			if ((noIQ > 0) && botrun) {
				//ROS_INFO("Here");
				if (mqPop()) {
					ROS_INFO("pop ok");
					movedone = 1;
					if (autostart == 1.0) {
						mstate = 102;
						//cmdFlag = false;
						//moveToLPnPre(targetGP,targetLP);
						//ROS_INFO("Premove 1");	
						//publish_sound(19);	
						sdir = "gnome-terminal -x "+homedir+"ssmc.sh &";
						system(sdir.c_str());
						// system("gnome-terminal -x /home/agv/ssmc.sh &");
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist,start_angle,PREMOVE);
						clearmap_start_time = ros::Time::now();
					} else {
						mstate = 100; // no autostart. wait for trigger to start
						sprintf(buf,"Moving to Destination : %s. Waiting for Trigger to Satrt",cLPName.c_str());
						s1.assign(buf,strlen(buf));
						publish_status(s1);
						publish_go(1);
					}
				} else {
					ROS_INFO("pop fail");
				}
				break;
			}		
			
			break;
		case 100:
			//ROS_INFO("wait for trigger");
			if (startNav) { // trigger to start move of transporter
				startNav = false;
				mstate = 102;
				publish_status("Moving to Destination : "+cLPName);
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist,start_angle,PREMOVE);
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 102:  // pre move
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						//ROS_INFO("Premove 2");
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist1,start_angle1,PREMOVE);
						mstate = 103;
						clearmap_start_time = ros::Time::now();
					} else {
						if (moveflag == 5) {
							// cancelled move.
							//publish_clear();
							publish_sound(99);								
							clearmap_start_time = ros::Time::now();							
							mstate = 107;
						}
					}
				} else {
					if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.0)) ) {
						//publish_sound(99);
						//publish_sound(19);
						clearmap_start_time = ros::Time::now();		
					}
				}
			}
			break;
		case 103:  // move
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						ROS_INFO("Normal Move to Goal");
						publish_status("Moving to Destination : "+cLPName);
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						//publish_sound(19);
						mstate = 1;		
						movedone = 1;	
						clearmap_start_time = ros::Time::now();		
					}	else {
						if (moveflag == 5) {
							// cancelled move.
							//publish_clear();
							publish_sound(99);
							clearmap_start_time = ros::Time::now();							
							mstate = 107;
						}
					}		
				} else {
					if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.0)) ) {
						//publish_sound(99);
						//publish_sound(19);
						clearmap_start_time = ros::Time::now();		
					}
				}
			}
			break;		
		case 104:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						botrun = true;
						rn.getParam("LeftLaser",LeftLaser);	
						rn.getParam("RightLaser",RightLaser);
						rn.getParam("RobotReady",readyflag);
						if ((LeftLaser == 1) && (RightLaser == 1) && (readyflag == 77)) {
							publish_sound(1);
							mstate = 0;
						}
						//ROS_INFO("case 104");
					}
				}
				count++;
				if (count >= 10) {
					publish_sound(15); 
					//system("/home/racnys/restartusb.sh");
					botrun = true;
					publish_sound(15);
					count = 0;
					mstate = 0;
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 105:
			if (movedone == 0) {
				if (moveflag == 1) {
					movedone = 1;
					mstate = 1;
					//ROS_INFO("moveToLPn : 3");
					moveToLPn(targetGP,targetLP);	
				}
			}
			break;
		
		case 1: // movement state
			//move to from LP
			if (movedone == 0) {
				if (moveflag == 1) {
					mstate = 10;
					clearmap_start_time = ros::Time::now();		
					publish_sound(99);						
				} else {
					if (moveflag == 2) {
						//ROS_INFO("Movement Aborted");				
						mstate = 31;
						count = 0;
						publish_debug("Movement Aborted. Try again");
						publish_sound(14); // robot need space
						clearmap_start_time = ros::Time::now();
					} else {
						if (moveflag == 5) {
							// cancelled move.
							//publish_clear();
							publish_sound(99);
							clearmap_start_time = ros::Time::now();							
							mstate = 107;
						} else {
							publish_debug("Something Very Wrong. Need Help..");
							mstate = 7;
						}
					}
				}
			} else {
				if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.0)) ) {
					//publish_sound(99);
					//publish_sound(19);
					clearmap_start_time = ros::Time::now();		
				}
			}
			break; 
		case 10:
			// check if time delay up
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				publish_status("Reached Destination. Performing Alignment");
				//ROS_INFO("Reached Dest.Clear Map");
				publish_clear();  // shorten time to clear obs map
				clearmap_start_time = ros::Time::now();
				ROS_INFO("------- MovNode : Align : %.3f ----------",align);
				align = 2.0;
				if (align == 0) {  // no alignment
					ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,9);
					mstate = 36;
					ROS_INFO("------- MovNode : No Alignment ----------");
				} else {					
					publish_pose();
					if (align == 1.0) {  // dx,dy
						ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,7);
						mstate = 36;
						ROS_INFO("------- MovNode : dy dx Alignment ----------");
					} else {
						if (align == 2.0) {  // scan match
							scanMatchAlign(1);
							mstate = 106;
							ROS_INFO("------- MovNode : Scan Match ----------");
						}
					}
					
				}
				movedone = 1;				
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 106:
			// check if time delay up
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						ROS_INFO("Scan Align 2nd time");
						publish_pose();						
						scanMatchAlign(3);
						movedone = 1;
						mstate = 36;
						clearmap_start_time = ros::Time::now();
					}
				}
			}
			break;
		case 107:  // %%%
			// check if time delay up
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				publish_clear();
				publish_status("Navigation Aborted.");			
				publish_sound(17);		
				mstate = 108;				
			}
			break;
		case 108: // %%%
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(3.0)) ) {
				rn.setParam("navstatus",0);
				rn.setParam("lookforObs",0);
				start_dist = 2.0;
				start_angle = 190.0;
				start_dist1 = 2.0;
				start_angle1 = 190.0;
				publish_status("Transporter Ready for Job");
				publish_fButton(132);
				mstate = 0;		
			}
			break;
		case 11:
			//if ( ros::Time::now() > (clearmap_start_time + ros::Duration(clear_time)) ) {
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(delay)) ) {
				sdir = homedir + "remap.sh";
				//system("/home/racnys/remap.sh");	
				system(sdir.c_str());
				publish_debug("Restore Global Costmap with Static Map");
				mstate = nstate;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 12:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				//publish_status("Reached Station : "+cLPName);	
				if (!fromToflag) {
					// complete TO LP				
					publish_go(2); // unload robot
					unloadflag = true;
					loadflag = false;
					mstate = 13;	
				} else {
					publish_go(1); // load robot
					unloadflag = false;
					loadflag = true;
					mstate = 14;	
				}
				//clearmap_start_time = ros::Time::now();
				lap_time = ros::Time::now();
				refresh_time = ros::Time::now();
				call_time = 3.0;
			}
			break;
		case 13:
			if (!unloadflag) {
				mstate = 15;
				publish_status("Job Done. Getting Ready for Next Job");	
				//clearmap_start_time = ros::Time::now();
			}
			if ( ros::Time::now() > (lap_time + ros::Duration(call_time)) ) {
				//publish_status("Please Unload Job");	
				publish_sound(11);
				lap_time = ros::Time::now();
				call_time = 15.0;
			}
			if ( ros::Time::now() > (refresh_time + ros::Duration(4.0)) ) {
				publish_go(2); // unload robot
				refresh_time = ros::Time::now();
			}
			break;
		case 14:
			if (!loadflag) {
				mstate = 15;
				publish_status("Job Loaded. Getting Ready to move to next Destination");	
				//clearmap_start_time = ros::Time::now();
			}
			if ( ros::Time::now() > (lap_time + ros::Duration(call_time)) ) {
				//publish_status("Please Load Job");
				publish_sound(10);
				lap_time = ros::Time::now();
			  call_time = 15.0;
			}
			if ( ros::Time::now() > (lap_time + ros::Duration(4.0)) ) {
				publish_go(1); // unload robot
				lap_time = ros::Time::now();
			}
			break;
		case 15:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {	
				//publish_move_complete("move completed");	
				publish_status("Transporter Ready For Job");
				start_dist = pre_dist;
				start_angle = pre_angle;
				start_dist1 = pre_dist1;
				start_angle1 = pre_angle1;
				mstate = 0;			
				publish_move_complete("move completed");	
				rn.setParam("navstatus",0);
				rn.setParam("lookforObs",0);
				if (ntucTest) {
					mstate = nstate;
				}
			}
			break;
		case 150:  // post move
			//system("/home/racnys/remap.sh");	
			//sdir = homedir + "remap.sh";
			//system(sdir.c_str());
			//publish_debug("Restore Global Costmap with Static Map");
			clearmap_start_time = ros::Time::now();
			movedone = 1;
			//publish_debug("post move start");
			//rn.setParam("navstatus",0);
			ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,post_dist,post_angle,POSTMOVE);
			mstate = 151;
			break;
		case 151:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,post_dist1,post_angle1,POSTMOVE);
						mstate = 152;
						clearmap_start_time = ros::Time::now();
					}
				}
			}
			break;
		case 152:
			if (movedone == 0) {		
				if (moveflag == 1) {
					if (low_power_flag) {
						if (charging_type == 0) {
							// move to docking station
							//rn.setParam("navstatus",0);
							movedone = 1;
							publish_status("Move to Docking Station");
							ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,1);
							mstate = 22;
						} else {
							// hot swap
							mstate = 22;
						}
					} else {
						publish_status("Wait... Preparing for next Job");
						//nstate = 15;		
						delay = clear_time;
						mstate = 15;
					}
				}
			}
			break;			
		case 16:
			if (movedone == 0) {
				if ((moveflag == 7) || (moveflag == 1)) {
					publish_debug("Bootup Scan Stn Size Done");
					mstate = 17;		
					clearmap_start_time = ros::Time::now();		
					//botrun = true;
				}
			}
			break;
		case 18:
			if (movedone == 0) {
				if ((moveflag == 7) || (moveflag == 1)) {
					mstate = 19;							
				}
			}
			break;
		case 19:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(3.0)) ) {	
				//publish_debug("Measure Charging Stn Size Done");
				mstate = 0;				
			}
			break;
		case 2: // change group
			if (movedone == 0) {
				if (moveflag == 1) {
					mstate = 9;
					count = 0;					
				}
			}			
			break;
		case 20: // reference point
			if (movedone == 0) {
				if (moveflag == 1) {
					publish_pose();
					if (align == 1.0) {
						ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,7); 
						mstate = 201;
					} else {
						if (align == 2.0) {
							scanMatchAlign(1);
							mstate = 200;
						}
					}
					//publish_clear();	
					clearmap_start_time = ros::Time::now();			
					//mstate = 201;
					movedone = 1; //***
					//movedone = 0;  // *** test only
					//moveflag = 1;  // *** test only
					//publish_toggleButton(CHARGING);			
				} else {
					if (moveflag == 2) {
						//ROS_INFO("Movement Aborted");				
						mstate = 40;
						count = 0;
						//publish_debug("Movement Aborted. Try again");
						publish_sound(14); // robot need space
						clearmap_start_time = ros::Time::now();
					}
				}
			}			
			break;
		case 200:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						//ROS_INFO("Scan Align 2nd time");
						publish_pose();						
						scanMatchAlign(3);
						movedone = 1;
						mstate = 201;
						clearmap_start_time = ros::Time::now();
					}
				}
			}
			break;
		case 201:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						//publish_debug("All Alignment Done");
						clearmap_start_time = ros::Time::now();
						publish_clear();	
						mstate = 202;
					}
				}
			}
			break;
		case 202:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(clear_time)) ) {
				sdir = homedir+"remap.sh";
				//system("/home/racnys/remap.sh");	
				system(sdir.c_str());	
				//publish_debug("Restore Global Costmap with Static Map");
				mstate = 28;
				clearmap_start_time = ros::Time::now();
				publish_status("Transporter Charging ....");
			}
			break;
		case 21: //
			if (movedone == 0) {
				if (moveflag == 1) {
					//publish_debug("Moving to Docking Station Stage 2");
					ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,6);
					mstate = 22;
					movedone = 1;
					clearmap_start_time = ros::Time::now();		
					dock_time = clearmap_start_time; //ros::Time::now();						
				}
				if (moveflag == 7) {
					// abort docking. cannot find stn
					mstate = 26;
				}
			}	
			break;
		case 22: // Reach Docking Station for hot swap robot
			if (movedone == 0) {
				if (moveflag == 1) {
					if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
						publish_status("Transporter Charging ....");
						mstate = 29;
					}					
				}
			}			
			break;
		case 23: // Reach Docking Station
			if (!low_power) {
				// robot charged
				if (charging_type == 0) {
					//publish_debug("Moving Away from Docking Station 0.3m");
					ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,4);
					mstate = 24;
					movedone = 1;
					count = 0;
				}
				if (charging_type == 1) {  // swap or wire charging
					rn.setParam("current_station",88);
					publish_move_complete("update stations");
					mstate = 0;										
				}	
			}		
			break;
		case 24:
			if (movedone == 0) {
				if (moveflag == 1) {
					moveToLPn(nGP,0);	
					//ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,NORMAL);
					mstate = 25;
					movedone = 1;
					count = 0;
				}
			}			
			break;
		case 25: // Reach Reference point
			if (movedone == 0) {
				if (moveflag == 1) {
					rn.setParam("current_station",88);  // reference point and ready 
					publish_move_complete("update stations");
					mstate = 0;
				}
			}			
			break;
		case 26:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(clear_time)) ) {
				//publish_clear(); tt
				mstate = 25;
				movedone = 0;
				moveflag = 1;
			}	
			break;
		case 28:  // delay and go to next state
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(11.0)) ) {		
				targetLP = 0;	
				//publish_move_complete("update stations");
				count = 0;
				mstate = 29;	
				publish_move_complete("update stations");
			}
			break;
		case 29:
			if ((!low_power) && (low_power_flag)) {
				//rn.setParam("current_station",88);  // refnstateerence point and ready 
				//publish_move_complete("update stations");
				mstate = 290;		
				//publish_move_complete("update stations");
				publish_status("Transporter Charged.");
				//ROS_INFO("FULL");
				publish_fButton(199);				
				low_power_flag = false;
			}
			break;
		case 290:
			rn.getParam("noIQ",noIQ);
			if (noIQ > 0) {
				movedone = 1;
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,4);
				publish_status("Moving Away from Docking Station");
				mstate = 291;
			}	
			break;
		case 291: // 
			if (movedone == 0) {
				if (moveflag == 1) {
					//publish_status("Move Node : Turning Away from Docking Station");
					//publish_debug("Move Node : OK. Ready for next Job");
					mstate = 0;
				}
			}			
			break;				
		case 3:
			//ROS_INFO("Move Node : Move to Group LP=0");
			rn.getParam("current_group",nGP);
			if (dbRequest(32,nGP,0)) {						
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);						
				moveflag = 0;  // set by send_goal
				movedone = 1;
				mstate = 4;
				rn.setParam("current_station",0);
			} else {
				ROS_INFO("Error in DB Request");
			}
			break;
		case 30:
			{  // lock scope
				boost::mutex::scoped_lock lock(mut);
				nGP = targetGP;
				nLP = nextLP;
				cLP = currentLP;
				nLPIQ = noLPIQ;
			}
						
			if ((cLP != nLP) && (!low_power)) {
				// robot has job to do
				//publish_status("Moving Away from Docking Station");
				//publish_debug("Moving Away from Docking Station 0.3m");
				//ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,4);
				mstate = 0;
				movedone = 1;
				count = 0;			
				curLoc = 99;	
			}
			break;
		case 31: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(3.0)) ) {	
				count++;
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				movedone = 1;
				mstate = 32;
				//publish_sound(19);
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 32:
			if (movedone == 0) {
				if (moveflag == 1) {
					mstate = 10;		
					publish_sound(99);			
					clearmap_start_time = ros::Time::now();
				} else {
					if (moveflag == 2) {
						//ROS_INFO("Movement Aborted");							
						//publish_debug("Movement Aborted. Try again");
						publish_sound(14); // robot need space
						//rn.setParam("clearObs",77);
						if (count >=20) {
							mstate = 107;
							count = 0;		
							publish_sound(99);
							clearmap_start_time = ros::Time::now();	
						} else {
							if ((count == 5) || (count == 10) || (count == 17)) {
								mstate = 33;
								clearmap_start_time = ros::Time::now();	
							} else {						
								mstate = 31;					
								//mstate = 33;	
								clearmap_start_time = ros::Time::now();
							}
						}
					} else {
						if (moveflag == 5) {
							// cancelled move.
							//publish_clear();
							publish_sound(99);
							clearmap_start_time = ros::Time::now();							
							mstate = 107;
						} else {
							publish_debug("Something Very Wrong. Need Help..");
							mstate = 7;
						}
					}
				}
			}	else {
				if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.0)) ) {
					//publish_sound(99);
					//publish_sound(19);
					clearmap_start_time = ros::Time::now();		
				}
			}
			break;
		case 33:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				publish_clear();
				//count = 0;
				mstate = 34;
				//mstate = 31;
				//publish_status("Reached Destination : "+cLPName);	
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 34:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(clear_time)) ) {
				//system("/home/racnys/remap.sh");
				sdir = homedir+"remap.sh";
				system(sdir.c_str());	
				//publish_debug("Restore Global Costmap with Static Map");
				//count = 0;
				mstate = 35;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 35:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(9.0)) ) {	
				mstate = 31;				
			}
			break;
		case 36:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						mstate = 150; //11;
						//publish_debug("All Alignment Done");
						//publish_status("Alignment Done. Doing Post Move Positioning");
						clearmap_start_time = ros::Time::now();
						//publish_clear();
					}
				}
			}
			break;
		case 4: 			
			if (movedone == 0) {
				if (moveflag == 1) {
					mstate = 80;
					count = 0;					
				}
			}			
			break;
		case 40:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(3.0)) ) {	
				count++;
				moveToLPn(nGP,0);	
				//ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,NORMAL);
				movedone = 1;
				mstate = 41;
			}
			break;
		case 41:
			if (movedone == 0) {
				if (moveflag == 1) {
					mstate = 20;		
					//mstate = 93;		
					movedone = 0;	
					moveflag = 1;
					clearmap_start_time = ros::Time::now();
				} else {
					if (moveflag == 2) {
						//ROS_INFO("Movement Aborted");	
						//publish_debug("Movement Aborted. Try again");
						publish_sound(14); // robot need space
						//rn.setParam("clearObs",77);
						if (count >= 12) {
							mstate = 93;
							count = 0;		
							clearmap_start_time = ros::Time::now();	
						} else {				
							if ((count == 1) || (count == 3) || (count == 11)) {
								mstate = 42;
								clearmap_start_time = ros::Time::now();	
							} else {						
								mstate = 40;					
								//mstate = 41;	
								clearmap_start_time = ros::Time::now();
							}		
							//mstate = 40;					
							//clearmap_start_time = ros::Time::now();
						}
					}
				}
			}
			break;
		case 42:
			// check if time delay up
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				publish_clear();
				mstate = 43;
				//mstate = 40;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 43:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(clear_time)) ) {
				//system("/home/racnys/remap.sh");
				sdir = homedir+"remap.sh";
				system(sdir.c_str());
				//publish_debug("Restore Global Costmap with Static Map");
				mstate = 44;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 44:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(10.0)) ) {	
				mstate = 40;				
			}
		case 5:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {				
				if (ntucLobbyDoor == 1) {
					mstate = 50;
					ntucDoorOpenRequestCnt = 0;
					ROS_INFO("------------- Lobby Door Command Received. ----------------");
				} else {
					publish_fButton(301);
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 5) {
						publish_sound(29); //Lobby Door Error. Command not recevied
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 50:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {				
				if (ntucLobbyDoor == 2) {
					mstate = 51;
					ntucDoorOpenRequestCnt = 0;
					ntucLobbyDoor = 0;
					ROS_INFO("------------- Lobby Door Command Activated. ----------------");
				} else {
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 5) {
						publish_sound(30); //Lobby Door Error. Door Controller Error
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 51:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if ((ProfileMoveObsDist > ntucDistToLobbyDoorOutside)) {
					// door opened
					publish_sound(23);// Lobby Door Opened	
					mstate = 52;
					rn.setParam("lookforObs",0);
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 52:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				// move to lift entrance.				
				if (!NTUCINTERNALTEST) {
					pos.linear.z = 46.0;  // 7.0
					pos_pub.publish(pos);
					movedone = 1;
				} else {
					movedone = 0;
					moveflag = 1;
				}
				clearmap_start_time = ros::Time::now();	
				ROS_INFO("------------- AGV Moving to front of Lift Door ----------------");
				mstate = 53;
			}
			break;
		case 53:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						// request for lift
						publish_sound(24);// Request for Lift	
						ROS_INFO("------------- AGV Requesting for Lift  ----------------");
						publish_lift(0,1,0,1);
						//clearmap_start_time = ros::Time::now();	
						mstate = 54;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 54:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if ((ntucstatus == 10) && (liftDoorOpen)) {
					// robot move into lift
					ROS_INFO("------------- AGV Checking if Lift Door is Opened ----------------");
					rn.setParam("lookforObs",1);			
					ntucstatus = 0;
					mstate = 55;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 55:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.5) {
					// door opened
					ROS_INFO("------------- Lift Door Opened ----------------");
					publish_sound(25);// Lift Door Opened	
					mstate = 56;
					rn.setParam("lookforObs",0);					
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 56:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				// robot move into lift
				publish_sound(26);// robot move into lift
				if (!NTUCINTERNALTEST) {
					pos.linear.z = 46.1;  // 7.0
					pos_pub.publish(pos);
					movedone = 1;
				} else {
					movedone = 0;
					moveflag = 1;
				}			
				clearmap_start_time = ros::Time::now();	
				ROS_INFO("------------- AGV Moving into Lift ----------------");
				mstate = 57;
			}
			break;
		case 57:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_lift(1,1,0,1);  // robot in lift
						clearmap_start_time = ros::Time::now();	
						ROS_INFO("------------- AGV Inside Lift ----------------");
						mstate = 58;
					}
				}
			}
			break;
		case 58:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				publish_lift(2,1,7,1);  // lift go to 7th floor
				clearmap_start_time = ros::Time::now();	
				ROS_INFO("------------- AGV Sent instruction to go to 7 floor ----------------");
				mstate = 59;
			}
			break;
		case 59:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if ((ntucstatus == 11) && (liftDoorOpen)) {
					// lift reached destination
					ROS_INFO("------------- AGV Checking if Lift Door is opened ----------------");
					rn.setParam("lookforObs",1);								
					ntucstatus = 0;
					mstate = 501;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 501:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.5) {
					// door opened
					ROS_INFO("------------- Lift Door Opened----------------");
					publish_sound(25);// Lift Door Opened	
					mstate = 502;
					rn.setParam("lookforObs",0);					
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 502:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				// robot move out of lift
				publish_sound(27);// robot move out of lift at 7th Floor
				if (!NTUCINTERNALTEST) {
					pos.linear.z = 46.2;  // 7.0
					pos_pub.publish(pos);
					movedone = 1;
				} else {
					movedone = 0;
					moveflag = 1;
				}							
				ROS_INFO("------------- AGV Moving out of Lift at 7th Floor ----------------");
				clearmap_start_time = ros::Time::now();	
				mstate = 503;
			}
			break;
		case 503:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_lift(3,7,1,0);  // robot outside lift
						ROS_INFO("------------- AGV outside of Lift at 7th Floor ----------------");
						clearmap_start_time = ros::Time::now();	
						mstate = 504;
					}
				}
			}
			break;
		case 504:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(5.0)) ) {
				publish_sound(24);// Request for Lift	
				ROS_INFO("------------- AGV Request for Lift at Return to Ground Floor ----------------");
				publish_lift(0,7,1,1);
				clearmap_start_time = ros::Time::now();	
				mstate = 505;
			}
			break;
		case 505:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if ((ntucstatus == 10) && (liftDoorOpen))  {
					// robot move into lift
					ROS_INFO("------------- Checking if Lift Door is opened at 7th Floor ----------------");
					rn.setParam("lookforObs",1);									
					ntucstatus = 0;
					mstate = 506;
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 506:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.5) {
					// door opened
					ROS_INFO("------------- Lift Door is Opened ----------------");
					publish_sound(25);// Lift Door Opened	
					mstate = 507;
					rn.setParam("lookforObs",0);					
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 507:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				// robot move into lift
				publish_sound(26);// robot move into lift
				if (!NTUCINTERNALTEST) {
					pos.linear.z = 46.1;  // 7.0
					pos_pub.publish(pos);
					movedone = 1;
				} else {
					movedone = 0;
					moveflag = 1;
				}		
				ROS_INFO("------------- AGV Moving into Lift at 7th Floor ----------------");
				clearmap_start_time = ros::Time::now();	
				mstate = 508;
			}
			break;
		case 508:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_lift(1,7,1,1);  // robot in lift				
						ROS_INFO("------------- AGV inside Lift at 7th Floor ----------------");		
						mstate = 509;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 509:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				publish_lift(2,1,1,1);  // lift go to ground floor
				ROS_INFO("------------- AGV Request Lift to go to Ground Floor ----------------");
				clearmap_start_time = ros::Time::now();	
				mstate = 510;
			}
			break;
		case 510:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if ((ntucstatus == 11) && (liftDoorOpen)) {
					// lift reached destination
					rn.setParam("lookforObs",1);								
					ntucstatus = 0;
					mstate = 511;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 511:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.5) {
					// door opened
					ROS_INFO("------------- AGV Checking if Door is opened at ground Floor ----------------");
					publish_sound(25);// Lift Door Opened	
					mstate = 512;
					rn.setParam("lookforObs",0);					
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 512:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				// robot move into lift
				publish_sound(27);// robot move out of lift
				if (!NTUCINTERNALTEST) {
					pos.linear.z = 46.2;  // 7.0
					pos_pub.publish(pos);
					movedone = 1;
				} else {
					movedone = 0;
					moveflag = 1;
				}		
				ROS_INFO("---------- AGV Moving Out of Lift at Ground Floor ----------------");							
				clearmap_start_time = ros::Time::now();	
				mstate = 513;
			}
			break;
		case 513:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_lift(3,8,1,0);  // robot outside lift			
						ROS_INFO("---------- AGV Outside of Lift at 8th Floor ----------------");					
						mstate = 0;
						ntucTest = false;
						rn.setParam("ntucTest",ntucTest);
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 6:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_sound(22); // request to open Lobby Door
						publish_fButton(301);  // instruct arduino to open door to lift lobby
						rn.setParam("lookforObs",1);  // 
						mstate = 60;
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 60:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {				
				if (ntucLobbyDoor == 1) {
					mstate = 61;
					ntucDoorOpenRequestCnt = 0;
					ROS_INFO("------------- Lobby Door Command Received. ----------------");
				} else {
					publish_fButton(301);
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 20) {
						publish_sound(29); //Lobby Door Error. Command not recevied
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 61:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {				
				if (ntucLobbyDoor == 2) {
					mstate = 62;
					ntucDoorOpenRequestCnt = 0;
					ntucLobbyDoor = 0;
					ROS_INFO("------------- Lobby Door Command Activated. ----------------");
				} else {
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 25) {
						publish_sound(30); //Lobby Door Error. Door Controller Error
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 62:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				ROS_INFO("------- MoveNode : ProfileMoveObsDist = %.3f. --------------",ProfileMoveObsDist);
				if ((ProfileMoveObsDist > 1.4)) {
					// door opened
					publish_sound(23);// Lobby Door Opened	
					//if (dbRequestS("CNC")) {  // move to lift entrance
					//	movedone = 1;
					//	nstate = 63;
					//	mstate = 1;
					//	ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist,start_angle,NORMAL);
					//	ROS_INFO("------------- AGV Moving to front of Lift Door ----------------");
					//} else {
					//	ROS_INFO("---------- ntuc : Fail to get CNC ------------------");
					//}			
					mstate = 620;		
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 620:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.5)) ) {
				if (dbRequestS("CNC")) {  // move to lift entrance
					movedone = 1;
					nstate = 63;
					mstate = 1;
					ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist,start_angle,NORMAL);
					ROS_INFO("------------- AGV Moving to front of Lift Door ----------------");
				} else {
					ROS_INFO("---------- ntuc : Fail to get CNC ------------------");
				}	
			}
			break;
		case 63:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						// request for lift
						publish_sound(24);// Request for Lift	
						ROS_INFO("------------- AGV Requesting for Lift  ----------------");
						publish_lift(0,1,0,1);
						//clearmap_start_time = ros::Time::now();	
						mstate = 64;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 64:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if ((ntucstatus == 10) && (liftDoorOpen)) {
					// robot move into lift
					ROS_INFO("------------- AGV Checking if Lift Door is Opened ----------------");
					rn.setParam("lookforObs",1);			
					ntucstatus = 0;
					mstate = 65;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 65:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				ROS_INFO("------- MoveNode : ProfileMoveObsDist = %.3f. --------------",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.4) {
					// door opened
					ROS_INFO("------------- Lift Door Opened ----------------");
					publish_sound(25);// Lift Door Opened	
					mstate = 66;
					rn.setParam("lookforObs",0);					
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 66:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				// robot move into lift
				publish_sound(26);// robot move into lift
				if (!NTUCINTERNALTEST) {
					pos.linear.z = 46.1;  // 7.0
					pos_pub.publish(pos);
					movedone = 1;
				} else {
					movedone = 0;
					moveflag = 1;
				}			
				clearmap_start_time = ros::Time::now();	
				ROS_INFO("------------- AGV Moving into Lift ----------------");
				mstate = 67;
			}
			break;
		case 67:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_lift(1,1,0,1);  // robot in lift
						clearmap_start_time = ros::Time::now();	
						ROS_INFO("------------- AGV Inside Lift at level 1 ----------------");
						mstate = 68;
					}
				}
			}
			break;
		case 68:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				publish_lift(2,1,11,1);  // lift go to 11th floor
				clearmap_start_time = ros::Time::now();	
				ROS_INFO("------------- AGV Sent instruction to go to 11 floor ----------------");
				mstate = 69;
			}
			break;
		case 69:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if ((ntucstatus == 11) && (liftDoorOpen)) {
					// lift reached destination
					ROS_INFO("------------- AGV Checking if Lift Door is opened ----------------");
					rn.setParam("lookforObs",1);								
					ntucstatus = 0;
					mstate = 600;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 600:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				ROS_INFO("------- MoveNode : ProfileMoveObsDist = %.3f. --------------",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.4) {
					// door opened
					ROS_INFO("------------- Lift Door Opened----------------");
					publish_sound(25);// Lift Door Opened	
					mstate = 601;
					rn.setParam("lookforObs",0);					
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 601:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				// robot move out of lift
				publish_sound(27);// robot move out of lift at 7th Floor
				if (!NTUCINTERNALTEST) {
					pos.linear.z = 46.2;  // 7.0
					pos_pub.publish(pos);
					movedone = 1;
				} else {
					movedone = 0;
					moveflag = 1;
				}							
				ROS_INFO("------------- AGV Moving out of Lift at 11th Floor ----------------");
				clearmap_start_time = ros::Time::now();	
				mstate = 602;
			}
			break;
		case 602:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_lift(3,11,1,0);  // robot outside lift
						ROS_INFO("------------- AGV outside of Lift at 11th Floor ----------------");
						clearmap_start_time = ros::Time::now();	
						mstate = 603;
					}
				}
			}
			break;
		case 603:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(60.0)) ) {
				publish_sound(24);// Request for Lift	
				ROS_INFO("------------- AGV Request for Lift at 11 fl to go to 8th Floor ----------------");
				publish_lift(0,11,8,1);
				clearmap_start_time = ros::Time::now();	
				mstate = 604;
			}
			break;
		case 604:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if ((ntucstatus == 10) && (liftDoorOpen))  {
					// robot move into lift
					ROS_INFO("------------- Checking if Lift Door is opened at 11th Floor ----------------");
					rn.setParam("lookforObs",1);									
					ntucstatus = 0;
					mstate = 605;
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 605:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				ROS_INFO("------- MoveNode : ProfileMoveObsDist = %.3f. --------------",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.4) {
					// door opened
					ROS_INFO("------------- Lift Door is Opened at 11th floor ----------------");
					publish_sound(25);// Lift Door Opened	
					mstate = 606;
					rn.setParam("lookforObs",0);					
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 606:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				// robot move into lift
				publish_sound(26);// robot move into lift
				if (!NTUCINTERNALTEST) {
					pos.linear.z = 46.1;  // 7.0
					pos_pub.publish(pos);
					movedone = 1;
				} else {
					movedone = 0;
					moveflag = 1;
				}		
				ROS_INFO("------------- AGV Moving into Lift at 11th Floor ----------------");
				clearmap_start_time = ros::Time::now();	
				mstate = 607;
			}
			break;
		case 607:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_lift(1,11,1,1);  // robot in lift				
						ROS_INFO("------------- AGV inside Lift at 11th Floor ----------------");		
						mstate = 608;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 608:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				publish_lift(2,1,8,1);  // lift go to ground floor
				ROS_INFO("------------- AGV Request Lift to go to 8th Floor ----------------");
				clearmap_start_time = ros::Time::now();	
				mstate = 609;
			}
			break;
		case 609:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if ((ntucstatus == 11) && (liftDoorOpen)) {
					// lift reached destination
					rn.setParam("lookforObs",1);			
					ROS_INFO("------------- AGV Lift at 8th Floor ----------------");					
					ntucstatus = 0;
					mstate = 610;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 610:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				ROS_INFO("------- MoveNode : ProfileMoveObsDist = %.3f. --------------",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.1) {
					// door opened
					ROS_INFO("------------- AGV Checking if Door is opened at 8th Floor ----------------");
					publish_sound(25);// Lift Door Opened	
					mstate = 611;
					rn.setParam("lookforObs",0);					
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 611:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				// robot move into lift
				publish_sound(27);// robot move out of lift
				if (!NTUCINTERNALTEST) {
					pos.linear.z = 46.2;  // 7.0
					pos_pub.publish(pos);
					movedone = 1;
				} else {
					movedone = 0;
					moveflag = 1;
				}		
				ROS_INFO("---------- AGV Moving Out of Lift at 8th Floor ----------------");							
				clearmap_start_time = ros::Time::now();	
				mstate = 612;
			}
			break;
		case 612:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_lift(3,1,1,0);  // robot outside lift			
						ROS_INFO("---------- AGV Outside of Lift at Ground Floor ----------------");					
						mstate = 0;
						ntucTest = false;
						rn.setParam("ntucTest",ntucTest);
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 7:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(3.0)) ) {	
				publish_sound(15); // nav aborted		
				clearmap_start_time = ros::Time::now();					
			}
			break;
		case 8:	
			count++;
			if (count == 2) {	
				publish_move_complete("move completed");	
				//ROS_INFO("mstate 8");		
				//publish_clear();tt
			}
			if (count >= 28) {				
				count = 0;
				mstate = 0;				
			}
			break;
		case 80:	
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {			
				count = 0;
				mstate = 10;
				publish_move_complete("move completed");	
				publish_status("Reached Station");
				// send 11 reached stn A to toggle_led	
				//publish_toggleButton(REACHEDSTN);		
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 9:	
			count++;
			if (count == 24) {	
				//ROS_INFO("mstate 9");		
				//publish_clear();tt
			}
			if (count >= 28) {				
				count = 0;
				mstate = 3;
			}
			break;
		case 90:	
			count++;
			if (count == 2) {			
				count = 0;
				mstate = 3;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 91:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {	
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);				
				mstate = 92;
				moveflag = 0;  // set by send_goal		
				movedone = 1;
				//publish_debug("Move : Alignment");
			}
			break;
		case 92: // movement state
			//move to from LP
			if (movedone == 0) {
				if (moveflag == 1) {
					mstate = 10;
					count = 0;		
					clearmap_start_time = ros::Time::now();								
				} else {
					if (moveflag == 2) {
						//ROS_INFO("Movement Aborted");				
						mstate = 31;
						count = 0;
						//publish_debug("Movement Aborted. Try again");
						clearmap_start_time = ros::Time::now();
					} else {
						if (moveflag == 5) {
							// stop moving to charging area and do other task.
							mstate = 0;
						} else {
							//publish_debug("Something Very Wrong. Need Help..");
							mstate = 7;
						}
					}
				}
			}
			break; 
		case 93: // reference point
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(3.0)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_pose();
						ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,7);
						//publish_debug("Move : Doing alignment");
						publish_clear();
						//publish_move_complete("move completed");
						curLoc = 99;
						mstate = 95;
						movedone = 1;
						clearmap_start_time = ros::Time::now();		
					} else {
						if (moveflag == 2) {
							//ROS_INFO("Movement Aborted");				
							mstate = 40;
							count = 0;
							//publish_debug("Movement Aborted. Try again");
							clearmap_start_time = ros::Time::now();
						}
					}
				}		
			}	
			break;
		case 94: // reference point
			if (movedone == 0) {
				if (moveflag == 1) {
					publish_pose();
					ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,7);
					//publish_debug("Move : Doing alignment");
					//publish_clear();
					//publish_move_complete("move completed");
					curLoc = 0;
					mstate = 95;
					movedone = 1;
					clearmap_start_time = ros::Time::now();		
				} else {
					if (moveflag == 2) {
						//ROS_INFO("Movement Aborted");				
						mstate = 40;
						count = 0;
						//publish_debug("Movement Aborted. Try again");
						clearmap_start_time = ros::Time::now();
					}
				}
			}			
			break;
		case 95: // reference point
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(clear_time)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						//system("/home/racnys/remap.sh");
						sdir = homedir+"remap.sh";
						system(sdir.c_str());
						//publish_debug("Restore Global Costmap with Static Map");					
						mstate = 96;		
						clearmap_start_time = ros::Time::now();
					} 
				}			
			}
			break;
		case 96:  // 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(11.0)) ) {				
				mstate = 97;
			}
			break;
		case 97:
			if (movedone == 0) {		
				publish_move_complete("move completed");	
				publish_status("Reached Destination Aligned");
				start_dist = pre_dist;
				start_angle = pre_angle;
				count = 0;
				mstate = 22;			
				publish_move_complete("move completed");	
			}
			break;
	}
		
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move  node");  
  ros::NodeHandle rn;  		
	MovNode mNode;
	//int readyflag, mapflag;
	int readyflag;
	int cc;
	int startmove;
	double odd;

	//ros::MultiThreadedSpinner spinner(4);
	ros::Rate loop_rate(10.0);
	while (true) {
		mNode.publish_status("Checking if Robot is Ready..");
		rn.getParam("RobotReady",readyflag);
		if ((readyflag == 77) || (readyflag == 88)) {
			mNode.publish_status("Robot is Ready. Checking if Nav Mode");
			//ROS_INFO("Move_Node : Robot is Ready");
			rn.getParam("mapflag",mNode.mapflag);
			if (mNode.mapflag == 77) {
				mNode.publish_status("Nav Mode. Running Move Node");
				//ROS_INFO("Move_Node : Running Move Node");
				break;
			}
		}
		sleep(1);
	}
	//sleep(1);
  //ROS_INFO("Navigation Mode. Running Movement Node...");
	//mNode.publish_debug("Move Node : Navigation Mode. Running Movement Node...");
	cc = 0;
	//rn.setParam("current_station",88); // reference
	//rn.setParam("botrunflag",0); // robot not running yet
	
	sleep(1);
	mNode.initialise();  // setup LPString
  while (true) {  	  	 	
  	//ROS_INFO("MoveLoop");
		mNode.moveLPLoop();  
		cc++;
		if (cc == 10) {
			//mNode.publish_queue();
			mNode.publish_status(mNode.sstring);
			//mNode.publish_pose();
			cc = 0;
			//rn.getParam("ProfileMoveObsDist",odd);
			//ROS_INFO("--------- MoveNode : odd=%.3f. -----------",odd);
		}
		ros::spinOnce();	
  	loop_rate.sleep();
	}
  return 0;
}


