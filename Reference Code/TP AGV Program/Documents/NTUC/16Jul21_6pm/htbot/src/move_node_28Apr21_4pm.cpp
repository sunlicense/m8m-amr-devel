/*
 * This node is to service movement command  
 */
/* 	History
*		Date Modified : 22.4.2021:1030AM
*		Changes : 
*     31.12.15 : standardise with ssmc
*     20.3.16 : Meiban new requirement. LP = 0 > origin or reference point. 1~9 : Group 1. 10~19 : Gp 2..
*/

//#define ODROID
//#define SSMCMOVETOCHARGE
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
#include "htbot/PLAYSOUND.h"
#include "htbot/goal.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "htbot/DYNA.h"
#include "htbot/dyna.h"
#include "htbot/lift.h"
#include "htbot/agv_status.h"

#define MAXLPQ 150
#define MAXSTN 5
#define MAXQN 3
#define CTZ 0.118
#define CTX 0.5
#define NORMAL 0
#define ENGAGE 1
#define DISENGAGE 4
#define MOVETOCHARGE 3
#define RACPATHMOVE 50
#define MAXPP 200
#define CLEANPLAN 23
#define LOADCLEANPLAN 24

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
#define TURNTOPATH 20 //
#define TURNTOPATHMOVE 21 //
#define LOCALISEMOVE 22 //  
#define MAXVP 100
#define DOCKING_UNDOCKING 203
#define STATIONL 9
#define GOBUTTON  210
#define AMPHRCONV 0.00006944  // 0.25 sec divided by 3600
#define AMPHRPERCENT 3.33 //2.7778  // (amphr * 100) / 36ah

#define PI 3.141593

// ntuc
#define NORMAL_MOVEMENT 103
#define LIFTAREA_TO_LOBBYAREA	2000
#define LIFTAREA_TO_LAUNDRYAREA	2050
#define LIFTAREA_TO_LIFTAREA	2350
#define LOBBYAREA_TO_LAUNDRYAREA	2100
#define LOBBYAREA_TO_LIFTAREA	2150
#define LAUNDRYAREA_TO_LIFTAREA	2200
#define LAUNDRYAREA_TO_LOBBYAREA	2250

#define ASKLIFTTOCOME 0
#define AGVINLIFT 1
#define LIFTGOTO	2
#define AGVOUTSIDE	3
#define	KITCHEN 8

#define TROLLET1WIDTH 0.7
#define TROLLET2WIDTH 1.2
#define TROLLET3WIDTH 1.4

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
	void publish_event(string s);
	void publish_queue(void);
	void publish_go(int cmd);
	void checkpath(double x,double y);
	void moveLPLoop();
	void publish_changeMove(int stat);
	void publish_move_complete(string s);
	void publish_fButton(unsigned short cmd);
	void publish_sound(int id,int sd,int rsd);
	void publish_arrive(string lps);
	
	bool dbRequest(int cmd, int GP, int LP);
	bool dbRequestS(std::string lps);
	bool dbRequestSP(std::string lps);
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
	double findBatteryEnergy(double volt);
	void readVoltDatafromFile();
	double ptpAngle(double tx, double ty);
	void calcQuat(double sx, double sy,double ex, double ey);
	void checkDockLP(std::string lps);
	void loadStartEndDockPosition();
	void moveToStartPosition();
	void moveToEndPosition();
	void moveToSegEndPosition();
	void dynaReconfig(int pm, int iV, double dV, string sV);
	void checkLocInPath(void);
	void closeGapToGoal(void);
	void prepareLocInPath(int idx);
	// ntuc
	void publish_lift(int cmd, int cfloor, int dfloor,int inuse,int beacon);
	void ntucCallback(const htbot::lift::ConstPtr& msg);
	void liftMechanism(bool updown);
	void publish_ntuc_status(int cmd, int status, int location, int b_level, int complete);
	bool planning(void);
	void restoreLP(void);
	int findLocation(int lp);
	int findLiftLocation(int lp);
	double findtrolleywidth(int type);
	int checkFloor(int lp);
	
	ros::ServiceServer cmd_service;
	htbot::mqueue web_srv;
	ros::ServiceClient web_client;
	ros::ServiceClient clearcostmap;
	ros::ServiceClient makeplan;
	htbot::mqueue mq_srv;
	ros::ServiceClient mq_client;

	ros::Publisher status_pub;
	ros::Publisher debug_pub;
	ros::Publisher event_pub;
	ros::Publisher queue_pub;
	ros::Publisher go_pub; 
	ros::Publisher chkplan_pub;
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
	// Uno 
	ros::Subscriber button_sub;
	ros::Subscriber volt_sub;
	ros::Subscriber curr_sub;
	ros::Subscriber usound_sub;
	ros::Publisher fButton_pub;
	ros::Publisher cancel_pub;
	ros::Publisher dyna_pub;
	bool button_mode; // true=navigation. false=mapping
	// ntuc
	ros::Publisher lift_pub;
	ros::Subscriber lift_sub;
	ros::Publisher ntucstatus_pub;

	int numLP;
	int curLoc;
	int mstate,nstate;
	int abort_retry;
	int currentLocation;

	double tx,ty,tz;
	double rx,ry,rz,rw;
	double pre_dist,pre_angle,post_dist,post_angle;
	double pre_dist1,pre_angle1,post_dist1,post_angle1;
	double start_dist, start_angle,start_dist1, start_angle1;
	double otx,oty,otz;
	double orx,ory,orz,orw;
	double call_time,voltage,charging_voltage,work_voltage,lowpower_voltage,avgvolt,energy,avgenergy;
	double current,usound;
	int moveLPLoopQ[MAXLPQ];
	string LPLoopQString[MAXLPQ];
	string LPString[MAXLPQ];
	string sstring;
	std::string homedir;
	double gx,gy,grz,grw,px,py,prz,prw;
	double checkPlanSum,checkPlanX,checkPlanY,checkPlanXN,checkPlanYN,checkPlanDD;
	double checkPlanRX,checkPlanRY,checkPlanRZ,checkPlanRW;
	int currentLP,nextLP,lastnLP,noLPIQ;
	bool nflag;
	int mapflag;
	int count;
	double pathDist;	
	double looprate;
	double txm,tym,tzm;
	double rxm,rym,rzm,rwm;
	double pre_distm,pre_anglem,post_distm,post_anglem;
	double pre_dist1m,pre_angle1m,post_dist1m,post_angle1m;
	double alignm,funcm,trolleym,trolleydistm;
	
	bool midPoint,domidPoint,Re_Localisation,init_localisation,Localised_Ready,Cancel_Nav,restart_localisation;
	bool Lost_Localisation,ZeroReference,testglobalplanner;
	double spx,spy,spz,sprx,spry,sprz,sprw;  // start position
	double epx,epy,epz,eprx,epry,eprz,eprw;  // end position
	double dpx,dpy,dpz,dprx,dpry,dprz,dprw;  // docking position
	bool Change_Nav,normalPath;
	
	bool passwdreq,sendgoalready;
	int moveflag,movedone;
	string passwd,inputpw;
	string lastsLP,cGPName,cLPName,voltfile,cLPNamem;
	bool navReady;
	bool startNav;
	bool clearMapFlag;
	bool cmdFlag; // true = LP to move (targetGP, targetLP)
	bool fromToflag; // true = from. false = to.
	bool loadflag,unloadflag,startbot,botrun,measure_cstn;  // 
	int targetGP,targetLP,otargetLP,targetLPm;
	ros::Time clearmap_start_time,dock_time,lap_time,refresh_time;
	double clear_time;
	bool low_power,estop,low_power_flag,completedocking;
	int charging_type,alignment_type;
	double delay;
	double align,func;  // func = 0.0 > no lifting of trolley. 1.0 > lifting of trolley. 2.0 > check loading bay
	double trolley,trolleydist;
	int numLPInQ;
	int readynav;
	double waitStationTime,waitBuzzTime,waitExtraTime;
	bool wait15m,wait15mflag,moveOn,WNmoveOn,WNwait15m;
	int navstatus;
	bool cancel_ops,stuck,dockLP;
	bool botCharging,checkLocalisation,AutoReference;
	double charging_AH,current_offset;
	int destPoint; // 0=normal LP. 1=cleaning start position. 2=Re-Direct. 
	int npos,alignCount;
	double pathInfo[MAXPP][8];
	int noPP,PPPtr,curPPtr,nextPPtr;  // curPPtr = robot near point in cleaning path. nextPPtr = next point in the cleaning path that robot can move to
	bool cleanplanmove,racplanMove,racplanMoveObs,MOVEREADY,ONCEMOVEREADY;
	// landing points queue
	double qposeInfo[MAXLPQ][15];	
	// ntuc
	bool ntucTest,NTUCINTERNALTEST,liftDoorOpen,agvjackUp,LoadingBayStatus,NTUCNav;
	int ntucstatus; // 1=lift arrived door opened.2=lift reached door opened
	int ntucfloor;
	int ntucLobbyDoor; //1=door open request received. 2=Door open activated.
	int ntucDoorOpenRequestCnt;
	double ntucDistToLobbyDoorOutside,ntucDistToLobbyDoorInside;
	int ntucjobstatus; // 1=ready for next job. 2=job completed but not ready. 3=not ready
	bool jackdowncmd,jackupcmd;
	FILE *vfp;  
	double perInfo[MAXVP],voltInfo[MAXVP];
	int numVP;
	bool UPJACK,redirect;
	double ProfileMoveObsDist;
	int currentfloor;

private:
	ros::NodeHandle nh;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::TransformStamped robotpose;
	boost::mutex mut;	
};

MovNode::MovNode():
	rx(0.0),ry(0.0),rz(0.0),rw(0.0),moveflag(0),movedone(0),clearMapFlag(false),noLPIQ(0),fromToflag(true),
	tx(0.0),ty(0.0),tz(0.0),passwdreq(false),sendgoalready(true),low_power(false),charging_type(0),readynav(0),
	currentLP(0),nextLP(0),mstate(0),curLoc(99),count(0),lastnLP(0),navReady(false),startNav(false),waitStationTime(1.0),
	looprate(1.0),cmdFlag(false),targetGP(0),targetLP(0),clear_time(0.0),measure_cstn(false),moveOn(false),wait15m(false),
	loadflag(false),unloadflag(false),startbot(false),abort_retry(12),botrun(false),alignment_type(0),wait15mflag(false),
	voltage(0.0),charging_voltage(27.0),lowpower_voltage(24.0),work_voltage(27.0),start_dist(2.0),otargetLP(0),charging_AH(0.0),
	start_angle(190.0),start_dist1(2.0),start_angle1(190.0),low_power_flag(false),estop(false),avgvolt(0.0),botCharging(false),
	current_offset(0.0),WNmoveOn(false),WNwait15m(false),cancel_ops(false),domidPoint(false),stuck(false),destPoint(0),
	Re_Localisation(false),init_localisation(false),Localised_Ready(false),restart_localisation(false),Lost_Localisation(false),
	Cancel_Nav(false),checkLocalisation(false),AutoReference(false),ZeroReference(false),testglobalplanner(false),PPPtr(0),
	cleanplanmove(false),racplanMove(false),racplanMoveObs(false),NTUCNav(true),currentLocation(13),MOVEREADY(false),
	ntucTest(false),ntucstatus(0),ntucLobbyDoor(0),NTUCINTERNALTEST(false),liftDoorOpen(false),ntucjobstatus(3),
	ntucDistToLobbyDoorOutside(1.5),ntucDistToLobbyDoorInside(1.5),agvjackUp(false),LoadingBayStatus(true),
	ONCEMOVEREADY(false),jackdowncmd(false),jackupcmd(false),UPJACK(false),redirect(false),
  currentfloor(1)
{
	ros::NodeHandle nh;
	cmd_service = nh.advertiseService("move_service",&MovNode::cmdServiceCallback,this); 
	movstat_sub = nh.subscribe<htbot::move_status>("move_status", 100, &MovNode::movCallback,this);
	pass_sub = nh.subscribe<htbot::status>("barcode", 100, &MovNode::passCallback,this);
	status_pub = nh.advertise<htbot::status>("feedback",100);
	debug_pub = nh.advertise<htbot::debug>("debug",100);
	event_pub = nh.advertise<htbot::debug>("event",100);
	queue_pub = nh.advertise<htbot::queue>("queue",100);
	go_pub = nh.advertise<htbot::goal>("go",100);
	chkplan_pub = nh.advertise<htbot::goal>("checkplan",1);
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
	ROS_INFO("***** Charging Voltage : %.3f *****",charging_voltage);
	pos_pub = nh.advertise<geometry_msgs::Twist>("cmd_pos", 1);
	scanM_pub = nh.advertise<htbot::scanCmd>("scanCmd", 1);
	ipose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
	arrive_pub = nh.advertise<std_msgs::String>("arrive",1);
	dyna_pub = nh.advertise<htbot::dyna>("reconfigure",100);

  // Uno
	button_sub = nh.subscribe<std_msgs::UInt16>("button", 100, &MovNode::buttonCallback,this);
	fButton_pub = nh.advertise<std_msgs::UInt16>("fbutton",1);
	volt_sub = nh.subscribe<std_msgs::Float32>("voltage", 100, &MovNode::voltCallback,this);
	curr_sub = nh.subscribe<std_msgs::Float32>("current", 100, &MovNode::currCallback,this);
	usound_sub = nh.subscribe<std_msgs::Float32>("usound", 100, &MovNode::usoundCallback,this);

	// NTUC Lift Operation
	lift_pub = nh.advertise<htbot::lift>("to_lift",100);
	lift_sub = nh.subscribe<htbot::lift>("fr_lift", 100, &MovNode::ntucCallback,this);
	ntucstatus_pub = nh.advertise<htbot::agv_status>("to_fleet", 100);
	
	nh.getParam("home_dir",homedir);
	nh.getParam("volt_file",voltfile);
	nh.getParam("readynav",readynav);
	nh.getParam("waitStationTime",waitStationTime);
	nh.getParam("waitBuzzTime",waitBuzzTime);
	nh.getParam("waitExtraTime",waitExtraTime);
	nh.getParam("current_offset",current_offset);
	nh.getParam("domidPoint",domidPoint);
	nh.getParam("checkLocalisation",checkLocalisation);
	nh.setParam("Cancel_Nav",Cancel_Nav);
	nh.setParam("NTUCNav",NTUCNav);
}

double MovNode::findtrolleywidth(int type) {
	double width;
	switch(type) {
		case 1:
			width = (TROLLET1WIDTH / 2.0) - 0.03;
			break;
		case 2:
			width = (TROLLET2WIDTH / 2.0) - 0.03;
			break;
		case 3:
			width = (TROLLET3WIDTH / 2.0) - 0.03;
			break;
	}
	return width;
}

int MovNode::checkFloor(int lp) {
	int cfl;
	if ((lp == 0) || (lp == 1) || ((lp >= 50) && (lp <90))) {
		cfl = 1;
	} else {
		if ((lp >= 3) && (lp <=11)) {
			cfl = lp;
		} else {
			cfl = currentfloor;
		}
	}
	ROS_INFO("\n----------- MovNode clf : Floor=%d. --------------\n",cfl);
	return cfl;
}

int MovNode::findLiftLocation(int lp) {
	int liftfloor;

	
	switch(lp) {
		case 0:
		case 1:
		case 73:
		case 74:
		case 75:
		case 76:
		case 77:
		case 78:
		case 79:
			liftfloor = 1;
			break;
		case 51:
		case 52:
			liftfloor = 1;
			break;
		case 3: // Ward 3
			liftfloor = 3;
			break;
		case 4: // Ward 4
			liftfloor = 4;
			break;
		case 5: // Ward 5
			liftfloor = 5;
			break;
		case 6: // Ward 6
			liftfloor = 6;
			break;
		case 7: // Ward 7
			liftfloor = 7;
			break;
		case 8: // Ward 8
			liftfloor = 8;
			break;
		case 9: // Ward 9
			liftfloor = 9;
			break;
		case 10: // Ward 10
			liftfloor = 10;
			break;
		case 11: // Ward 11
			liftfloor = 11;
			break;
	}
	ROS_INFO("------------- Find Floor Level : floor = %d. --------------",liftfloor);
	return liftfloor;
}

int MovNode::findLocation(int LP) 
{

	switch(LP) {
		case 0:
		case 1:
		case 73:
		case 74:
		case 75:
		case 76:
		case 77:
		case 78:
		case 79:
			currentLocation = 13;
			break;
		case 51:
		case 52:
			currentLocation = 12;
			break;
		case 3: // Ward 3
			currentLocation = 3;
			break;
		case 4: // Ward 4
			currentLocation = 4;
			break;
		case 5: // Ward 5
			currentLocation = 5;
			break;
		case 6: // Ward 6
			currentLocation = 6;
			break;
		case 7: // Ward 7
			currentLocation = 7;
			break;
		case 8: // Ward 8
			currentLocation = 8;
			break;
		case 9: // Ward 9
			currentLocation = 9;
			break;
		case 10: // Ward 10
			currentLocation = 10;
			break;
		case 11: // Ward 11
			currentLocation = 11;
			break;
	}
	return currentLocation;
}


void MovNode::publish_lift(int cmd, int cfloor, int dfloor,int inuse, int beacon)
{
	htbot::lift msg;
	msg.cmd = cmd;
	msg.cfloor = cfloor;
	msg.dfloor = dfloor;
	msg.inuse = inuse;
	msg.beacon = beacon;
	lift_pub.publish(msg);
	ROS_INFO("---- Lift Cmd : cmd=%d. cf=%d. df=%d. beacon=%d. -------",cmd,cfloor,dfloor,beacon);
	return;
}

void MovNode::liftMechanism(bool updown)
{
	if (updown) {
		publish_fButton(602);  // instruct arduino to up lift mechanism
		ROS_INFO("------------ liftmechanism : up lift -------------");
		jackupcmd = true;
	} else {
		publish_fButton(603);  // instruct arduino to down lift mechanism
		ROS_INFO("------------ liftmechanism : down lift -------------");
		jackdowncmd = true;
	}
}

void MovNode::ntucCallback(const htbot::lift::ConstPtr& msg) 
{
	//ros::NodeHandle rn; 
	int cmd,cfloor,dfloor;

	cmd = msg->cmd; 
	ROS_INFO("----------- from lift : cmd = %d ----------------",cmd);
	switch (cmd) {
		case 10: // lift arrived at start level and lift door is kept opened. cfloor =m
			ntucstatus = 10;
			ntucfloor = msg->cfloor;
			liftDoorOpen = true;
			ROS_INFO("----------- from lift : Lift ready for AGV to enter. cmd = %d ----------------",cmd);
			break;
		case 11: // lift arrived at end level and lift door is kept opened. dfloor =m
			ntucstatus = 11;
			ntucfloor = msg->dfloor;
			liftDoorOpen = true;
			ROS_INFO("----------- from lift : Lift ready for AGV to exit. cmd = %d ----------------",cmd);
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

void MovNode::closeGapToGoal(void) {
	geometry_msgs::Twist pos;
	double dx,dy,dd;
	dx = tx - px;
	dy = ty - py;
	dd = sqrt((dx * dx) + (dy * dy));
	pos.linear.x = dd;
	pos.linear.z = 42.0;  // 7.0
	pos_pub.publish(pos);
}

void  MovNode::prepareLocInPath(int idx) {
	double x,y,z,rx,ry,rz,rw;
	char buf[200];
	string s1;
	
	tx = pathInfo[idx][0];
	ty = pathInfo[idx][1];
	tz = 0.0;
	if (idx < (noPP-1)) {
		x = pathInfo[idx+1][0];
		y = pathInfo[idx+1][1];
		calcQuat(tx,ty,x,y);
		rx = checkPlanRX;
		ry = checkPlanRY;
		rz = checkPlanRZ;
		rw = checkPlanRW;
	} else {
		rx = 0.0;
		ry = 0.0;
		rz = 1.0;
		rw = 0.0;
	}
	sprintf(buf,"--- prepareLocInPath : tx=%.2f. ty=%.2f. ---",tx,ty);
	s1.assign(buf,strlen(buf));
	publish_event(s1);
}

void  MovNode::checkLocInPath(void) {
	int id,ir;
	double x,y,dd,dx,dy,mdx;
	char buf[200];
	string s1;
	ros::NodeHandle nn; 
	// locate closest to which point in path
	mdx = 200.0;
	ir = PPPtr;
	for (id = PPPtr;id<noPP;id++) {
		x = pathInfo[id][0];
		y = pathInfo[id][1];
		dx = x - px;
		dy = y - py;
		dd = sqrt((dx * dx) + (dy * dy));
		if ((dd < mdx)) {
			mdx = dd;
			ir = id;
		}
	}
	
	//if (mdx > 1.5) {
	//	publish_event("--- checkLocInPath : Error. Distance of AGV to Path is > 1.5m. ---");
	//}	

	curPPtr = ir;
	//if (curPPtr < (noPP-1))  {
		//x = pathInfo[curPPtr+1][0];
		//y = pathInfo[curPPtr+1][1]; 
		//nn.setParam("racplanMoveX",x);
		//nn.setParam("racplanMoveY",y);
		
	//}
	//sprintf(buf,"--- checkLocInPath : PPPtr=%d. curPPtr = %d. px=%.2f. py=%.2f. nx=%.2f. ny=%.2f---",PPPtr,curPPtr,px,py,x,y);
	sprintf(buf,"--- checkLocInPath : PPPtr=%d. curPPtr = %d. px=%.2f. py=%.2f. ---",PPPtr,curPPtr,px,py);
	s1.assign(buf,strlen(buf));
	publish_event(s1);
}

void MovNode::checkpath(double x, double y) {

	htbot::goal msg;
	msg.x = x;
	msg.y = y;
	chkplan_pub.publish(msg);
	ROS_INFO("\n---------- MovNode : checkpath tx=%.3f. ty=%.3f ---------------------\n",x,y);
	return;
}

void MovNode::calcQuat(double sx, double sy,double ex, double ey)
{
	double an,yawp,diff;
	tf2::Quaternion quat;
	geometry_msgs::Quaternion qq;
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
	checkPlanRX = qq.x;
	checkPlanRY = qq.y;
	checkPlanRZ = qq.z;
	checkPlanRW = qq.w;
	return;
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

	ROS_INFO("----------- MoveNode : initialpose point 1");
	ipose.header.frame_id = "map";
	ipose.header.stamp = ros::Time::now();
	ipose.pose.pose.position.x = tx;
	ipose.pose.pose.position.y = ty;
	ipose.pose.pose.position.z = tz;
	ipose.pose.pose.orientation.x = rx;
	ipose.pose.pose.orientation.y = ry;
	ipose.pose.pose.orientation.z = rz;
	ipose.pose.pose.orientation.w = rw;
	ipose.pose.covariance[0] = 0.55;//1e-3;
	ipose.pose.covariance[7] = 0.55;//1e-3;
	ipose.pose.covariance[14] = ipose.pose.covariance[21] = ipose.pose.covariance[28] = 1e9;
	ipose.pose.covariance[35] = 0.55; //1e-3;
	ipose_pub.publish(ipose);
	ROS_INFO("-------MovNode : Initialpose . x=%.3f.  y=%.3f. ---------",tx,ty);
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
			func = mq_srv.response.func;
			trolley = mq_srv.response.trolley;
			trolleydist = mq_srv.response.trolleydist;
			targetLP = mq_srv.response.LP;
			cLPName =  mq_srv.response.lps;
			ret = true;
		} else {
	    ROS_ERROR("Failed to pop to move queue");	
			ret = false;
	 	}
	} else {
		ret = false;
		targetLP = 99;
	}
	return ret;
}


void MovNode::buttonCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	ros::NodeHandle bn; 
	int nGP, nLP, butNo;
	//bool ok;
	geometry_msgs::Twist pos;
	bool ok;
	char buf [100];
	string s;
	int ret;

	butNo = msg->data; 
	//rn.getParam("current_group",nGP);
	ROS_INFO("Move_Node : Button Code Received = %d",butNo);
	//sprintf(buf,"Button_Node : Button Code Received = %d",butNo);
	//s.assign(buf,strlen(buf));
	//publish_debug(s);

	switch (butNo) {
		case 13: // robot charging now. Stop robot
			// stop robot.
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.0;  // 7.0
			pos_pub.publish(pos);
			publish_status("Docking Activated");
			publish_debug("Move Node : Docking Achieved.Stop Robot");
			botCharging = true;
			charging_AH = 0.0;
			break;

		case 15:
			// stop robot.
			if (!estop) {
				pos.angular.z = 0.0;
				pos.linear.x = 0.0;
				pos.linear.z = 7.8;  // 7.0
				pos_pub.publish(pos);
				
				//publish_status("Emergency Stop Activated");
				//publish_event("Emergency Stop Activated");
				//publish_debug("Move Node : Emergency Stop Activated");
				estop = true;
				//err_estop = "Emergency Stop Activated";
				ret = system("~/offallsound.sh &");
				bn.getParam("navstatus",navstatus);
				if (navstatus == 7) {
					cancel_goal();	
					cancel_ops = true;		
					ntucjobstatus = 4;					
				} else {
					ntucjobstatus = 3;						
				}
				liftMechanism(false); // down lift mechanism		
				publish_ntuc_status(1,3,otargetLP,(int)(avgenergy),ntucjobstatus);
				publish_sound(ESTOP,3,0);	
			}
			break;
		case 16:
			if (estop) {
				pos.angular.z = 0.0;
				pos.linear.x = 0.0;
				pos.linear.z = 7.9;  // 7.0
				pos_pub.publish(pos);
				//publish_status("Emergency Stop Released");
				//publish_event("Emergency Stop Released");
				//publish_debug("Move Node : Emergency Released");
				//err_estop = "";
				ntucjobstatus = 1; 
				publish_ntuc_status(1,3,otargetLP,(int)(avgenergy),ntucjobstatus);
				publish_sound(ESTOPREL,0,0);
				estop = false;
			}
			break;
		case 20:  // shutdown
			//ROS_INFO(" Recieved Shutdown from Button");
			//system("gnome-terminal -x aplay ~/sound/cannon_x.wav &");
			publish_sound(SHUTDOWN,0,0);	
			//publish_fButton(120); // inform audrino
			//publish_fButton(120); // inform audrino
			//system("sudo chmod +x /sbin/shutdown");
			//sleep(2);
			//system("sudo shutdown -h now");
			publish_event("Transporter Shutdown Activated");
			bn.setParam("shutdown",1);
			//shutdown_activated = true;
			break;
		case 21:  // robot move on
			moveOn = true;
			break;
		case 32:
			if (!cancel_ops) {
				bn.getParam("navstatus",navstatus);
				if (navstatus == 7) {		
					//system("gnome-terminal -x aplay ~/sound/Abort.wav &");
					publish_sound(NAVABORT,0,0);	
					//publish_event("Transporter Navigation Cancelled");
					cancel_goal();						
					cancel_ops = true;					
				} else {
					//publish_fButton(132);  // cancelled move
					//publish_fButton(132);  // cancelled move
				}
			}
			break;
		case 300:
			ntucLobbyDoor = 1;
			break;
		case 302:
			ntucLobbyDoor = 2;
			break;
		case 600: // command received
			break;
		case 604: // lift completed			
			if (jackupcmd) {
				agvjackUp = true;
				jackupcmd = false;
				ROS_INFO("---------- Button : Jack is UP.----------------");
			}
			if (jackdowncmd) {
				agvjackUp = false;
				jackdowncmd = false;
				ROS_INFO("---------- Button : Jack is Down.----------------");
			}
			break;
		case 605: // lift failed.
			//agvjackUp = false;
			ROS_INFO("---------- Button : Jack is Failed.----------------");
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

void MovNode::publish_sound(int id,int sd, int rsd)
{
	htbot::sound cmd;
	cmd.id = id;
	cmd.startdelay = sd;
	cmd.restartdelay = rsd;
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

void MovNode::publish_ntuc_status(int cmd, int status, int lp, int b_level, int complete)
{
	htbot::agv_status msg;
	msg.cmd = cmd;
	msg.status = status;
	msg.location = findLocation(lp);
	msg.b_level = b_level;
	msg.complete = complete;
	msg.lastLP = lp;
	ntucstatus_pub.publish(msg);
	//ROS_INFO("------ MovNode ntuc status : cmd=%d. status=%d. lp=%d. complete=%d. -------",cmd,status,lp,complete);
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
	if (avgvolt < 15.0) {
		avgvolt = voltage;
	}
	avgvolt = avgvolt + voltage;
	avgvolt = avgvolt / 2.0;
	energy = energy + findBatteryEnergy(avgvolt);
	energy = energy / 2.0;
	//if (current > 0.0) {
	//	energy = energy / 2.0;
	//}
	avgenergy = energy;
	xn.setParam("avgenergy",avgenergy);
	xn.setParam("avgvolt",avgvolt);
	//ROS_INFO("VoltCallback Here 2");
	if ((avgenergy < lowpower_voltage) && (!low_power)) {
		// low power
		low_power = true;
		//ROS_INFO("Battery Power Low Activated");
		//publish_status("Lower Power Activated");
		//publish_event("Battery Power Low Activated");
	} else {
		if ((avgenergy > work_voltage) && (completedocking)) {
			low_power = false;
			publish_status("Transporter Sufficiently Charged");
			//publish_event("Transporter Battery Sufficiently Charged. Ready for Job");
			completedocking = false;
		}
	}
	
	//ROS_INFO("VoltCallback exit");	 
	return;
}

void MovNode::readVoltDatafromFile() 
{		
	double per,volt;
	int j;
	ROS_INFO("----------------- MoveNode : start readVoltDatafromFile ---------------------------");
	vfp = fopen(voltfile.c_str(), "r");
  if (vfp == NULL) {
  	ROS_INFO("I couldn't open voltdata.dat for reading.\n");    
  	return;
  }
	j = 0;

	while(true) {
		if (fscanf(vfp,"%lf %lf\n",&per,&volt) == EOF) {
			break;
		}
		perInfo[j] = per;
		voltInfo[j] = volt;
		j++;
	}
	numVP = j;
	//ROS_INFO("readvoltfile numVP=%d",numVP);
	//for (int i=0;i<numVP;i++) {
	//	ROS_INFO("Bat : per=%.3f. volt=%.3f",perInfo[i],voltInfo[i]);
	//}
  fclose(vfp);	
}

double MovNode::findBatteryEnergy(double volt) 
{
	double per;
	int i;
	/*
	if (botCharging || (current < 0.0)) {
		per = charging_AH * AMPHRPERCENT;						
		for (i=0;i<numVP;i++) {
			if (((per + avgenergy)*0.01) < perInfo[i]) {
				avgvolt = voltInfo[i];			
				break;	
			}
		}		
		//avgenergy = avgenergy+per;
		//ROS_INFO("#################  curAH=%.3f. per=%.3f. volt=%.3f. avgenergy=%.3f  ###################",charging_AH,per,avgvolt,avgenergy);
		charging_AH = 0.0;
		return per;
	}
	*/
	if (volt < voltInfo[0]) {
		return 0.0;
	} else {
		if (volt > voltInfo[numVP-1]) {
			return 100.0;
		}
	}
	for (i=0;i<numVP;i++) {
		if (volt <= voltInfo[i]) {
			per = perInfo[i];
			//ROS_INFO(" Volt=%.3f. per=%.3f",volt,per);
			return (per * 100.0);
		} 
	}	

}

void MovNode::currCallback(const std_msgs::Float32::ConstPtr& msg)
{
	ros::NodeHandle xn; 
	char buf [100];
	string s;
	double cur;

	{ 
		boost::mutex::scoped_lock lock(mut);
		current = msg->data;		
	}
	current = current + current_offset;
	if (current < 0.0) {
		cur = -current;
		charging_AH = charging_AH + (cur * AMPHRCONV);
	} else {
		botCharging = false;
		charging_AH = 0.0;
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
	//publish_fButton(targetLP);
	ROS_INFO("\n----------- MovNode Move Completed : OLP=%d. NLP=%d -----------\n",otargetLP,targetLP);
	otargetLP = targetLP;
	currentfloor = checkFloor(otargetLP);
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
	char buf [100];
	string s;
	
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		if (msg->stat < 100) {
			moveflag = msg->stat;	
			movedone = 0;
		}
	}
	//ROS_INFO("------------ MovNode : Move Status : %d. Movedaone : %d ---------- ", moveflag,movedone);
	//sprintf(buf,"-------------- MovNode : Move Status = %d. mstate=%d -----------",moveflag,mstate);
	//s.assign(buf,strlen(buf));
	//publish_event(s); 
}

void MovNode::passCallback(const htbot::status::ConstPtr& msg)
{
	//ROS_INFO("Password");domidPoint
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		inputpw = msg->msg;	
	}
	ROS_INFO("PassCall");	 
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
	//ROS_INFO("PoseCall : x=%.2f. y=%.2f. rz=%.2f",x,y,rz);
	//if (racplanMove) {
		// find location in path and next point to clean. curPPtr
		//checkLocInPath();
	//}
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
	ROS_INFO("------------- MovNode : scanMatchAlign : %d. --------------",cmd);
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
	
	if ((opt == PREMOVE) || (opt == POSTMOVE) ) {
		if ((pd >= ZSD) && (pa >= ZSA)) {
			movedone = 0;
			moveflag = 1;
			return true;
		}
	}
	if (opt == TURNTOPATH) {
		if (stuck) {
			opt = TURNTOPATHMOVE;
			stuck = false;
			//ROS_INFO("----- MoveNode sendgoal TURNTOPATHMOVE : x=%.3f. y=%.3f ----------",x,y);
		}		
	}
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

	cmd.slp = otargetLP;
	cmd.elp = targetLP;
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
			align = web_srv.response.align;
			cGPName = web_srv.response.gps;
			cLPName = web_srv.response.lps;
			targetLP = web_srv.response.LP;
			func = web_srv.response.func;
			trolley = web_srv.response.trolley;
			trolleydist = web_srv.response.trolleydist;
			ret = true;
		} else {
	    ROS_ERROR("Failed to call web service");	
			ret = false;
	 	}
	}
	return ret;
}
void MovNode::checkDockLP(std::string lps) {
	std::string s1;

	s1 = "DOCK";
	trim(lps);
	if (s1.compare(lps) == 0) {
		dockLP = true;
		//publish_event("--- LP is Docking Station ---");
	} else {
		dockLP = false;
		//publish_event("--- LP is NOT Docking Station ---");
	}
}

bool MovNode::dbRequestS(std::string lps) {
	bool ret;
	std::string s1;
		
	web_srv.request.cmd = 30;
	web_srv.request.lps = lps;	
	// check if going to Docking station	
	trim(lps);
	s1 = "DOCK";
	if (s1.compare(lps) == 0) {
		dockLP = true;
	} else {
		dockLP = false;
	}
	ROS_INFO("---------- MoveNode : dbRequestS. Point : %s. -------",lps.c_str());
	if (web_client.call(web_srv)) {
		ROS_INFO("--------- MoveNode Received reply from dbase Server");
		if (web_srv.response.status == 30) {	 
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
			align = web_srv.response.align;
			func = web_srv.response.func;
			trolley = web_srv.response.trolley;
			trolleydist = web_srv.response.trolleydist;
			ret = true;
		} else {
	    ROS_INFO("------------- MovNode : Failed to call web service. -------------");				
			ret = false;
	 	}
	} else {
		ret = false;
		ROS_INFO("-------- MoveNode : dbRequestS Error---------");
	}
	ROS_INFO("-------- MoveNode : dbRequestS Exit ---------");
	return ret;
}

void MovNode::restoreLP(void) {
	tx = txm;
	ty = tym;
	tz = tzm;
	rx = rxm;
	ry = rym;
	rz = rzm;
	rw = rwm;
	pre_dist = pre_distm;
	pre_angle = pre_anglem;
	post_dist = post_distm;
	post_angle = post_anglem;
	pre_dist1 = pre_distm;
	pre_angle1 = pre_anglem;
	post_dist1 = post_dist1m;
	post_angle1 = post_angle1m;
	targetLP = targetLPm;
	cLPName = cLPNamem;
	align = alignm;
	trolley = trolleym;
	trolleydist = trolleydistm;
	func = funcm;
	ROS_INFO("---- RestoreLP : tx=%.2f. ty=%.2f. pd=%.2f. pa=%.2f. pd1=%.2f. pa1=%.2f. lp=%d. ---",tx,ty,post_dist,post_angle,post_dist1,post_angle1,targetLP);
}

bool MovNode::dbRequestSP(std::string lps) {
	bool ret;
	std::string s1;
		
	web_srv.request.cmd = 30;
	web_srv.request.lps = lps;	
	// check if going to Docking station	
	trim(lps);
	s1 = "DOCK";
	if (s1.compare(lps) == 0) {
		dockLP = true;
	} else {
		dockLP = false;
	}

	if (web_client.call(web_srv)) {
		if (web_srv.response.status == 30) {	 
			txm = tx;
			tym = ty;
			tzm = tz;
			rxm = rx;
			rym = ry;
			rzm = rz;
			rwm = rw;
			pre_distm = pre_dist;
			pre_anglem = pre_angle;
			post_distm = post_dist;
			post_anglem = post_angle;
			pre_dist1m = pre_dist1;
			pre_angle1m = pre_angle1;
			post_dist1m = post_dist1;
			post_angle1m = post_angle1;
			targetLPm = targetLP;
			cLPNamem = cLPName;
			alignm = align;
			funcm = func;
			trolleym = trolley;
			trolleydistm = trolleydist;

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
			align = web_srv.response.align;
			func = web_srv.response.func;
			trolley = web_srv.response.trolley;
			trolleydist = web_srv.response.trolleydist;
			ret = true;
		} else {
	    ROS_ERROR("Failed to call web service");	
			ret = false;
	 	}
	} else {
		ret = false;
		ROS_INFO("-------- MoveNode : dbRequestSP Error---------");
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

bool MovNode::planning(void) {
	//nstate = 0;
	int ret;
	ros::NodeHandle rn;
	rn.getParam("NTUCNav",NTUCNav);
	if (!NTUCNav) {
		movedone = 1;
		mstate = 103;
		//destPoint = 0;
		//redirect = false;
		//publish_sound(SSMC,0,0);	
		//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist,start_angle,PREMOVE);
		clearmap_start_time = ros::Time::now();
		//rn.setParam("startnav",1);
		//cancel_ops = false;						
		//ROS_INFO("----------- MovNode : Start Move : LP x=%.2f. y=%.2f.  -----------------------",tx,ty);
		return true;
	}
	ROS_INFO("--------- Planning : otarget=%d. target=%d. ------------",otargetLP,targetLP);
	if ((otargetLP > 1) && (otargetLP <= 49) ) {
		// LIFTAREA
		if ((targetLP >= 50) && (targetLP <= 69) ) {
			// LIFTAREA to LOBBYAREA
			mstate = LIFTAREA_TO_LOBBYAREA;
		} else {
			if ((targetLP >= 70) && (targetLP <= 89) ) {
				// LIFTAREA to LAUNDRYAREA
				mstate = LIFTAREA_TO_LAUNDRYAREA;
			} else {
				if ((targetLP == 0) || (targetLP == 1)) {
					// going to Reference or Docking stn
					mstate = LIFTAREA_TO_LAUNDRYAREA;
				} else {
					if ((targetLP >= 3) && (targetLP <= 11)) {
						if (currentfloor != checkFloor(targetLP)) {
							mstate = LIFTAREA_TO_LIFTAREA;
						} else {
							mstate = NORMAL_MOVEMENT;
						}
					} else {
						mstate = NORMAL_MOVEMENT;
					}
				}
			}
		}
	} else {
		if ((otargetLP >= 50) && (otargetLP <= 69) ) {
			// LOBBYAREA
			if ((targetLP >= 2) && (targetLP <= 49) ) {
				// LOBBYAREA to LIFTAREA
				mstate = LOBBYAREA_TO_LIFTAREA;
			} else {
				if ((targetLP >= 70) && (targetLP <= 89) ) {
					// LOBBYAREA to LAUNDRYAREA
					mstate = LOBBYAREA_TO_LAUNDRYAREA;
				} else {
					// LOBBYAREA to LOBBYAREA
					if ((targetLP == 0) || (targetLP == 1)) {
						// going to Reference or Docking stn
						mstate = LOBBYAREA_TO_LAUNDRYAREA;
					} else {
						mstate = NORMAL_MOVEMENT;
					}
				}
			}
		} else {
			if ((otargetLP >= 70) && (otargetLP <= 89) || ((otargetLP >= 0) && (otargetLP < 2)) ) {
				// LAUNDRYAREA
				if ((targetLP >= 2) && (targetLP <= 49) ) {
					// LAUNDRYAREA to LIFTAREA
					mstate = LAUNDRYAREA_TO_LIFTAREA;
				} else {
					if ((targetLP >= 50) && (targetLP <= 69) ) {
						// LAUNDRYAREA to LOBBYAREA
						mstate = LAUNDRYAREA_TO_LOBBYAREA;
					} else {
						mstate = NORMAL_MOVEMENT;
						destPoint = 0;
						redirect = false;
					}
				}
			} else {			
				ROS_INFO("--------- MovNode Planning : Error in old targetLP --------------------");
			}
		}
		if ((targetLP == 0) || (targetLP == 1)) {
			ntucjobstatus = 3; 
			publish_ntuc_status(1,5,targetLP,(int)(avgenergy),ntucjobstatus);
		} else {
			ntucjobstatus = 3; 
			publish_ntuc_status(1,1,targetLP,(int)(avgenergy),ntucjobstatus);
		}
	}	
	return true;
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

	//ROS_INFO("moveToLP");
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

void MovNode::publish_event(string s)
{

	htbot::debug status;
	status.msg = s;
	event_pub.publish(status);
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
	int rt;
  
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
			//publish_fButton(100+targetLP);
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
			rt = system("sudo shutdown -h now");
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
			rt = system(sdir.c_str());
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
		case 50: // rotate left 
			pos.linear.z = 9.0; 
			pos_pub.publish(pos);
			break;
		case 51:  // move forward
			ROS_INFO("move_node : move straight");
			pos.linear.z = 9.5;  
			pos_pub.publish(pos);
			break;
		case 52: // rotate right 180 deg
			pos.linear.z = 9.1; 
			pos_pub.publish(pos);
			break;
		case 53:  // move back
			pos.linear.z = 9.6;  
			pos_pub.publish(pos);
			break;
		case 54:  // special
			pos.linear.z = 9.3;  
			pos_pub.publish(pos);
			break;
		case 55:  // cancel move
			//ROS_INFO("Cancelled Move");
			xn.getParam("navstatus",navstatus);
			//publish_sound(NAVABORT,0,0);  // navigation aborted
			if (navstatus == 7) {	
				cancel_goal();
				cancel_ops = true;
				ROS_INFO("------------Move Node : Cancelled Move -------------");
				//publish_sound(NAVABORT,0,0);
			}
			break;
		case 56:  // test estop activation
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.8;  // 7.0
			pos_pub.publish(pos);
			publish_status("Emergency Stop Activated");
			publish_debug("Move Node : Emergency Stop Activated");
			estop = true;
			publish_sound(ESTOP,0,0);		
			sleep(2);
			break;
		case 57:  // test estop release
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.9;  // 7.0
			pos_pub.publish(pos);
			publish_status("Emergency Stop Released");
			publish_debug("Move Node : Emergency Released");
			publish_sound(ESTOPREL,0,0);		
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
			rt = system("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap /home/agv/catkin_ws/src/htbot/config/footprint3.yaml");
			rt = system("rosrun dynamic_reconfigure dynparam load /move_base/local_costmap /home/agv/catkin_ws/src/htbot/config/footprint3.yaml");
			break;
		case 61:  // foot45
			ROS_INFO("Change footprint to 0.45");
			rt = system("rosrun dynamic_reconfigure dynparam load /move_base/global_costmap /home/agv/catkin_ws/src/htbot/config/footprint4.yaml");
			rt = system("rosrun dynamic_reconfigure dynparam load /move_base/local_costmap /home/agv/catkin_ws/src/htbot/config/footprint4.yaml");
			break;
		case 62:  //new alignment
			pos.linear.z = 9.7;  
			pos_pub.publish(pos);
			break;
	}  
	//ROS_INFO("Move Service Exit");	 
	return true;
}

void MovNode::loadStartEndDockPosition() {
	ros::NodeHandle xn;
	FILE *fp;	
	int ret,i;	
	std::string cpp,s1,s2;
	//double dx,dy,dz,drx,dry,drz,drw;
	double px,py,pz,prx,pry,prz,prw,status;
	char buf[200];

	xn.getParam("cleanplanDirectory",s1);
	xn.getParam("cleanplanfile",s2);		
	cpp = s1 + s2;
	fp = fopen(cpp.c_str(), "r");
  if (fp == NULL) {
		ROS_INFO("----- Move_Node : I couldn't load %s for reading Docking Station Position. -----",cpp.c_str());
  	sprintf(buf,"--- Move_Node : I couldn't load %s for reading Docking Station Position. ---",cpp.c_str());   
		s1.assign(buf,strlen(buf)); 
		publish_event(s1);
  	return;
  }
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &spx,&spy,&spz,&sprx,&spry,&sprz,&sprw); // start position
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &epx,&epy,&epz,&eprx,&epry,&eprz,&eprw); // end position
	ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", &dpx,&dpy,&dpz,&dprx,&dpry,&dprz,&dprw); // docking station position
	//ROS_INFO("--- MoveNode :Start Pos : %.3f %.3f %.3f %.3f %.3f %.3f %.3f ----",spx,spy,spz,sprx,spry,sprz,sprw);
	//ROS_INFO("--- MoveNode :End Pos : %.3f %.3f %.3f %.3f %.3f %.3f %.3f ----",epx,epy,epz,eprx,epry,eprz,eprw);
	
	noPP = 0;
	while(true) {
		if (fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf\n",&px,&py,&pz,&prx,&pry,&prz,&prw,&status) == EOF) {
			break;
		}
		pathInfo[noPP][0] = px;
		pathInfo[noPP][1] = py;
		pathInfo[noPP][2] = px;
		pathInfo[noPP][3] = prx;
		pathInfo[noPP][4] = pry;
		pathInfo[noPP][5] = prz;
		pathInfo[noPP][6] = prw;
		pathInfo[noPP][7] = status; // 0.0=normal. 1.0=segment marker. segment end point
		i=noPP;
		ROS_INFO("--- PP=%d : x=%.3f. y=%.3f. Status=%.3f ----",i,pathInfo[i][0],pathInfo[i][1],pathInfo[i][7]);
		noPP++;
		
	}
	
	calcQuat(spx,spy,pathInfo[0][0],pathInfo[0][1]);
	sprx = checkPlanRX;
	spry = checkPlanRY;
	sprz = checkPlanRZ;
	sprw = checkPlanRW;
	
	ROS_INFO("--- MoveNode :Start Pos : %.3f %.3f %.3f %.3f %.3f %.3f %.3f ----",spx,spy,spz,sprx,spry,sprz,sprw);
	ROS_INFO("--- MoveNode :End Pos : %.3f %.3f %.3f %.3f %.3f %.3f %.3f ----",epx,epy,epz,eprx,epry,eprz,eprw);
	//ROS_INFO("------ MovNode : Start : x=%.2f. y=%.2f. rz=%.3f. rw=%.3f. --------",spx,spy,sprz,sprw);
  fclose(fp);	
}

void MovNode::moveToStartPosition() {
	tx = spx;
	ty = spy;
	tz = spz;
	rx = sprx;
	ry = spry;
	rz = sprz;
	rw = sprw;
	destPoint = 1;
	redirect = true;
}

void MovNode::moveToEndPosition(void) {
	tx = epx;
	ty = epy;
	tz = epz;
	rx = eprx;
	ry = epry;
	rz = eprz;
	rw = eprw;
	destPoint = 1;
	redirect = true;
}

void MovNode::moveToSegEndPosition() {
	ros::NodeHandle nh;
	string s1;
	char buf[200];

	nh.getParam("PPPtr",PPPtr);
	curPPtr = PPPtr;
	//ROS_INFO("------ moveToSegEndPosition : Start PPPtr=%d. ------------",PPPtr);
	for (int i=PPPtr;i<noPP;i++) {
		if (pathInfo[i][7] == 1.0) {
			tx = pathInfo[i][0];
			ty = pathInfo[i][1];
			tz = pathInfo[i][2];
			rx = pathInfo[i][3];
			ry = pathInfo[i][4];
			rz = pathInfo[i][5];
			rw = pathInfo[i][6];
			//ROS_INFO("------ moveToSegEndPosition : End PPPtr=%d. ------------",i);
			//sprintf(buf,"--- moveToSegEndPosition : Start PPPtr = %d. End PPPtr = %d. X=%.3f. Y=%.3f ---",PPPtr,i,tx,ty);
			//s1.assign(buf,strlen(buf));
			//publish_event(s1);
			break;
		}
	}
	return;
}

void MovNode::moveLPLoop()
{
	int cLP,nLP,mLP,nGP,nLPIQ;
	string sLP, sGP;
	int readyflag;
	bool start,ret,clearObs;
	ros::NodeHandle rn;  
	double secs;
	char buf[200];
	string s1,sdir;
	int LeftLaser,RightLaser,rt;
	double ptpAn;
	double nnx,nny;
	int nPPPtr;
	geometry_msgs::Twist pos;
	
	//Cancel_Nav = false;
	start = false;
	//ROS_INFO("moveLPLoop : Running mstate = %d*********************** ",mstate);
	switch (mstate) {
		case 0: // no movement state					
			if (estop) {  // estop
				break;
			}
			rn.getParam("numLPInQ",numLPInQ);			
			rn.getParam("UPJACK",UPJACK);	
			if (!botrun) {
				rn.getParam("RobotReady",readyflag);
				//ROS_INFO("Move : RobotReady = %d",readyflag);
				if (readyflag == 77) {
					botrun = true;					
					ROS_INFO("----------- MovNode 0 : Motor Ready ----------");
					if (dbRequestS("REF")) {					
						start_dist = pre_dist;
						start_angle = pre_angle;
						start_dist1 = pre_dist1;
						start_angle1 = pre_angle1;
						ROS_INFO("----- MovNode 0 : initial pose set -----");
						initialpose();
					} else {
						start_dist = 2.0;
						start_angle = 190.0;
						start_dist1 = 2.0;
						start_angle1 = 190.0;
					}					
					clearmap_start_time = ros::Time::now();
					if (checkLocalisation) {
						mstate = 109; 
					} else {
						mstate = 101;
					}
					normalPath = false;
					rn.setParam("normalPath",normalPath);
					liftMechanism(false);
				}
				break;
			}
			
			//ROS_INFO("MoveNode Here B");
			// low power. complete current job and go to charging station.
			if ((low_power) && botrun) {
				publish_sound(BATLOW,0,0);
				ROS_INFO("***** Move_node : Low Power");
				if (dbRequestS("DOCK")) {  // pre move i
					//publish_fButton(99); // low power, moving to reference point
					low_power_flag = true;
					planning();
					clearmap_start_time = ros::Time::now();					
				} else {
					ROS_INFO("------Lower Power. No Charging Station");
				}				
				break;
			}
			if ((numLPInQ > 0) && botrun) {
				if (mqPop()) {
					planning();
					break;
				} 
				ONCEMOVEREADY = true;
			} else {
				if (botrun) {
					if (!ONCEMOVEREADY) {
						if (MOVEREADY) {
							if ( ros::Time::now() > (clearmap_start_time + ros::Duration(5.0) )) {
								ntucjobstatus = 0;
								publish_ntuc_status(1,1,targetLP,(int)(avgenergy),ntucjobstatus);
								//publish_lift(KITCHEN,currentfloor,0,0,1);
								//publish_lift(ASKLIFTTOCOME,1,0,1,0);
								clearmap_start_time = ros::Time::now();
							}
						}
					}
				}
			}
			if (UPJACK) {
				UPJACK = false;
				publish_fButton(301);
			}
			break;			
		case 109:  // check for localisation
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.0) )) {
				//initialpose();
				publish_sound(CHECKLOCALISE,0,0);
				ROS_INFO("------------- MovNode : initial checking localisation ---------");
				//publish_event("----------- MoveNode : initial checking localisation ------");
				//rn.setParam("init_localisation",true);				
				mstate = 1090;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 1090:  // check for localisation
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(8.0) )) {
				rn.getParam("Localised_Ready",Localised_Ready);
				if (Localised_Ready) {
					mstate = 0;
					publish_sound(LOCALISEREADY,0,0);
					ROS_INFO("------------- MovNode 1090 : Robot Localised and Ready ---------");
					publish_event("----------- MoveNode 1090 : Robot Localised and Ready. Go mstate 0 ------");
					//rn.setParam("init_localisation",false);
					rn.setParam("Localised_Ready",false);
					Localised_Ready = false;
					rn.setParam("PoseReady",true);
					MOVEREADY = true;
					rn.setParam("MOVEREADY",MOVEREADY);	
					ZeroReference = false;
					rn.setParam("ZeroReference",ZeroReference);
				}
			}
			break;
		case 1091:  // check for re-localisation
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.0) )) {
				rn.getParam("Localised_Ready",Localised_Ready);
				if (Localised_Ready) {
					mstate = 0;
					publish_sound(LOCALISEREADY,0,0);
					ROS_INFO("------------- MovNode 1091 : Robot Re-Localised and Ready ---------");
					publish_event("----------- MoveNode 1091 : Robot Re-Localised and Ready. Go mstate 0 ------");
					rn.setParam("Localised_Ready",false);
					rn.setParam("restart_localisation",false);
					restart_localisation = false;
					Localised_Ready = false;
					rn.setParam("PoseReady",true);
				}
			}
			break;
		case 1092:  // check for re-localisation
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.0) )) {
				rn.setParam("amcl_ready",true);
				Re_Localisation = false;
				rn.setParam("Re_Localisation",false);		
				mstate = 0;
			}
			break;
		case 1093:  // check for localisation
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(5.0) )) {					
				//publish_sound(CHECKLOCALISE,2,0);
				ROS_INFO("------------- MovNode 1093 : initial checking localisation ---------");
				//publish_event("----------- MoveNode : initial checking localisation ------");	
				mstate = 1090;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 101:  // pre move
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(7.0)) ) {
				//rn.setParam("init_localisation",false);
				rn.setParam("PoseReady",true);
				MOVEREADY = true;
				rn.setParam("MOVEREADY",MOVEREADY);	
				ROS_INFO("------------ MovNode 101 : Move Ready --------------------");
				ntucjobstatus = 0;
				publish_ntuc_status(1,1,targetLP,(int)(avgenergy),ntucjobstatus); 
				mstate = 0;
				clearmap_start_time = ros::Time::now();
			}
		case 102:  // pre move
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist1,start_angle1,PREMOVE);
						mstate = 310; //1030; // 103
						clearmap_start_time = ros::Time::now();
						ROS_INFO("----------- MovNode 102 : PreMove 2.  -----------------------"); 
					} 
				} 
			}
			break;		
		case 103:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				movedone = 1;
				mstate = 102;
				destPoint = 0;
				redirect = false;
				publish_sound(SSMC,0,0);	
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist,start_angle,PREMOVE);
				clearmap_start_time = ros::Time::now();
				rn.setParam("startnav",1);
				cancel_ops = false;
				ROS_INFO("----------- MovNode 103 : PreMove 1.  -----------------------"); 
				ntucjobstatus = 3; 
				publish_ntuc_status(1,1,targetLP,(int)(avgenergy),ntucjobstatus); 
			}
			break;
		case 1: // movement state
			//move to from LP
			if (movedone == 0) {
				if (moveflag == 1) {
					mstate = 10;
					clearmap_start_time = ros::Time::now();							
				} else {
					if ((moveflag == 2) || (moveflag == 5)) {		
						ROS_INFO("------- MovNode : Moveflag=2/5. ------------");				
						rn.getParam("Cancel_Nav",Cancel_Nav);								
						//rn.getParam("racplanMove",racplanMove);
						if (racplanMove) {
							publish_sound(NEEDSPACE,0,5);
							//checkLocInPath();
							ROS_INFO("------- MovNode : racplanMove. ------------");	
							if (racplanMoveObs) {
								mstate = 63;
								publish_event("--- MoveNode : racplanMoveObsAbort ---");
							} else {
								mstate = 6;
								publish_event("--- MoveNode : racplanMoveAbort ---");
							}
							clearmap_start_time = ros::Time::now();	
						} else {
							ROS_INFO("------- MovNode : Abort Normal Move. ------------");								
							if (Cancel_Nav) {
								ROS_INFO("------- MovNode : Move Cancelled. Go to 108 ------------");	
								publish_sound(STOPMOVESOUND,0,0);
								//publish_event("--- MoveNode : Cancel_Nav Occurred 2 ---");			
								Cancel_Nav = false;
								rn.setParam("Cancel_Nav",Cancel_Nav);		
								mstate = 108;		
								publish_sound(NAVABORT,0,0);	
							} else {
								publish_clear();
								ROS_INFO("------------------------ MovNode : Abort Clear. navfn/NavfnROS ----------------------");	
								publish_sound(NEEDSPACE,0,5);
								//publish_clear();
								stuck = true;
								mstate = 310; //317; //310;
								count = 0;
								clearmap_start_time = ros::Time::now();	
							}		
						}				
					} else {
						publish_event("--- Something Very Wrong 1. Need Help ---");
						ROS_INFO("----------- Something Very Wrong 1. Need Help ---------");
						mstate = 9;
					}
				}
			} 
			break; 
		case 10:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				publish_sound(STOPMOVESOUND,0,0);	
				checkDockLP(cLPName);
				ROS_INFO("-------- MoveNode : Reached Destination. Performing Alignment ----------");
				publish_clear();  // shorten time to clear obs map
				rn.setParam("amcl_monitor",false);
				rn.getParam("alignCount",alignCount);
				publish_move_complete("move completed");
				//clearmap_start_time = ros::Time::now();
				//if ((align == 0) || (destPoint == 1)) {  // no alignment
				if ((align == 0)) {  // no alignment
					//publish_event("--- MovNode : Going yo state 15 ----");
					mstate = 15;
				} else {					
					publish_pose();
					if (align == 1.0) {  // dx,dy
						ret = sendGoalpub(0.0,0.0,CTZ,0.0,0.0,0.0,1.0,ZSD,ZSA,7);
						mstate = 36;
					} else {
						if (align == 2.0) {  // scan match							
							scanMatchAlign(1);// 3
							mstate = 36;
							ROS_INFO("---------- MoveNode : First Scan Match 2.0. mstate=%d ---------",mstate);
						} else {							
							scanMatchAlign(1);// 3
							mstate = 106;
							ROS_INFO("---------- MoveNode : First Scan Match 3.0. mstate=%d  ---------",mstate);
						}
					}
					
				}
				movedone = 1;				
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 106:
			// check if time delay up
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						ROS_INFO("----------- MoveNode : Scan Align 2nd time --------------------");
						publish_pose();						
						scanMatchAlign(3);//1
						movedone = 1;
						mstate = 36; //1060; //36;
						clearmap_start_time = ros::Time::now();
					}
				}
			}
			break;
		case 1060:
			// check if time delay up
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						//ROS_INFO("Scan Align 3rd time");
						publish_pose();						
						scanMatchAlign(3);// 3
						movedone = 1;rn.setParam("normalPath",normalPath);
						mstate = 36;
						clearmap_start_time = ros::Time::now();
					}
				}
			}
			break;
		case 108: // %%%
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				rn.setParam("navstatus",0);
				rn.setParam("lookforObs",0);
				rn.setParam("BackObs",0);
				rn.setParam("BackObs1",0);
				rn.setParam("amcl_monitor",false);
				start_dist = 2.0;
				start_angle = 190.0;
				start_dist1 = 2.0;
				start_angle1 = 190.0;
				publish_status("Transporter Ready for Job");
				//publish_fButton(132);
				rn.getParam("Lost_Localisation",Lost_Localisation);
				publish_sound(STOPMOVESOUND,0,0);				
				if (Lost_Localisation) {
					mstate = 1080;
					publish_event("----------- MoveNode 108 : To mstate = 1080 for re-start localization ------");
					clearmap_start_time = ros::Time::now();
				} else {
					mstate = 0;		
					ROS_INFO("---------- MoveNode 108 : To mstate = 0 ---------------");
					publish_event("----------- MoveNode 108 : To mstate = 0 ------");
				}
			}
			break;
		case 1080:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.0)) ) {
				rn.getParam("restart_localisation",restart_localisation);
				if (restart_localisation) {
					Lost_Localisation = false;
					rn.setParam("Lost_Localisation",Lost_Localisation);
					mstate = 0;
					otargetLP = 1;
				}
				clearmap_start_time = ros::Time::now();
				publish_sound(LOSTLOCALISE,0,5);
				//publish_event("----------- MoveNode 1080 : Waiting for Re-Starting of Localisation  ------");
			}
			break;
		case 15:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {	
				publish_status("------ MovNode 15 : Transporter Ready For Job ---------");
				ROS_INFO("---------------------- MovNode 15 : Transporter Checking function ---------------");
				start_dist = pre_dist;
				start_angle = pre_angle;
				start_dist1 = pre_dist1;
				start_angle1 = pre_angle1;
				//publish_move_complete("move completed");
				rn.setParam("navstatus",0);
				rn.setParam("lookforObs",0);
				rn.setParam("startnav",0);		
				if (func == 0.0) {
					// non pickup or drop LP
					mstate = 152;
				} else {
					if (func == 1.0) {
						// pickup trolley LP
						mstate = 16;
					} else {
						if (func == 2.0) {
							// drop trolley LP
							mstate = 17;
						} else {
							ROS_INFO("----------- MovNode 15 : Undefined function -----------------");
							mstate = 0;
						}
					}
				}			
				clearmap_start_time = ros::Time::now();
			}
			break;
		/*
		case 15:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {	
				publish_status("------ MovNode 15 : Transporter Ready For Job ---------");
				ROS_INFO("---------------------- MovNode 15 : Transporter Ready For Job ---------------");
				start_dist = pre_dist;
				start_angle = pre_angle;
				start_dist1 = pre_dist1;
				start_angle1 = pre_angle1;
				publish_move_complete("move completed");
				rn.setParam("navstatus",0);
				rn.setParam("lookforObs",0);
				rn.setParam("startnav",0);					
				if (readynav == 0) {
					mstate = 0;
				} else {
					if (destPoint == 1) {						
						mstate = nstate;
						ROS_INFO("------- MovNode 15 : destPoint 1. mstate=%d  --------",mstate);
						destPoint = 0;
					} else {
						if (destPoint == 2) {
							mstate = 156;
							ROS_INFO("-------- MovNode 15 : destPoint 2  -----");
						} else {
							if (destPoint == 0) {
								mstate = 156;
								ROS_INFO("--------- MovNode 15 : destPoint 0  -----");
							} else {
								mstate = 156;
								ROS_INFO("--------- MovNode 15 : destPoint 3 or more  -----");
							}
							
						}
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
			*/
		case 150:  // post move
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				clearmap_start_time = ros::Time::now();
			 	movedone = 1;
			 	ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,post_dist,post_angle,POSTMOVE);
			 	//publish_event("--- MovNode : PostMove 1 ---");
			 	ROS_INFO("--- MovNode 150 : PostMove 1 pdist=%.2f. pangle=%.2f. ---",post_dist,post_angle);
			 	mstate = 151;
			}
			break;
		case 151:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,post_dist1,post_angle1,POSTMOVE);
						//publish_event("--- MovNode : PostMove 2 ---");
						ROS_INFO("--- MovNode 151 : PostMove 2 pdist=%.2f. pangle=%.2f. ---",post_dist1,post_angle1);
						mstate = 155;// 152
						clearmap_start_time = ros::Time::now();
					}
				}
			}
			break;
		case 152:
			if (dockLP) {
				publish_sound(MOVETODOCK,0,0);
				movedone = 1;
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,ENGAGE);
				mstate = 22;
				clearmap_start_time = ros::Time::now();
			} else {
				if (redirect) {
					mstate = nstate;
					redirect = false;
				} else {
					mstate = 0;
				}
			}
			break;
		case 155:
			if (movedone == 0) {		
				if (moveflag == 1) {
					mstate = 15;
					clearmap_start_time = ros::Time::now();
				}
			}
			break;	
		case 156:
			//if (targetLP == 1) {
			if (dockLP) {
				//publish_event("--- 156 : Docking Station ---");
				publish_sound(MOVETODOCK,0,0);
				//publish_fButton(DOCKING_UNDOCKING);
				movedone = 1;
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,ENGAGE);
				mstate = 22;
				clearmap_start_time = ros::Time::now();
			} else {
				delay = clear_time;
				clearmap_start_time = ros::Time::now();
				if (func == 0.0) {
					// nothing to do
					if (destPoint == 2) {
						mstate = nstate;
						ROS_INFO("-------- MovNode 156 : func = 0.0 destPoint = 2. --------------");
					} else {
						mstate = 0;
						ROS_INFO("-------- MovNode 156 : func = 0.0 destPoint != 2. --------------");
					}
				} else {
					if (func == 1.0) {
						// activate lifting mechanism
						ROS_INFO("------------MovNode 156 : activate lifting mechanism. ------------");
						liftMechanism(true);
						mstate = 157;
					} else {
						if (func == 2.0) {
							// de-activate lift mechanism
							ROS_INFO("------------MovNode 156 : De-activate lifting mechanism. ------------");
							liftMechanism(false);
							mstate = 158;
						} else {
							if (func == 3.0) {
								// check loading bay
								ROS_INFO("------------MovNode 156 : Check LoadingBay. ------------");
								mstate = 159;
							} else {
								// do nothing
								if (destPoint == 2) {
									mstate = nstate;
								} else {
									mstate = 0;
								}
							}
						}
					}
				}
			}
			break;
		case 157:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (agvjackUp) {
					if (destPoint == 2) {
						mstate = nstate;
					} else {
						mstate = 0;
					}
					ntucjobstatus = 3; // not ready
					publish_ntuc_status(1,1,otargetLP,(int)(avgenergy),ntucjobstatus);
				}
			}
			break;
		case 158:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (!agvjackUp) {
					if (destPoint == 2) {
						mstate = nstate;
						ROS_INFO("-------------- MovNode 158 : mstate nstate. ------------------");
					} else {
						mstate = 0;
						ROS_INFO("-------------- MovNode 158 : mstate 0. ------------------");
					}
					if (low_power) {
						ntucjobstatus = 2; 
						publish_ntuc_status(1,1,otargetLP,(int)(avgenergy),ntucjobstatus);
					} else {
						ntucjobstatus = 1; 
						publish_ntuc_status(1,1,otargetLP,(int)(avgenergy),ntucjobstatus);
					}
				}
			}
			break;
		case 159:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("LoadingBayStatus",LoadingBayStatus);
				if (!LoadingBayStatus) {
					if (destPoint == 2) {
						mstate = nstate;
					} else {
						mstate = 0;
					}
					//mstate = 0;
				} else {
					ROS_INFO("----------- MovNode 159 : Loading Bay Not Empty ------------");
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 16: // pickup trolley after postmove
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				// jack down to be sure
				liftMechanism(false);				
				mstate = 160;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 160:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (!agvjackUp) {
					mstate = 161;			
					ROS_INFO("------------- MovNode 160 : Jack is down.------------");
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 161:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				// special move into trolley
				movedone = 1;
				moveflag = 0;
				pos.linear.z = 42.1; //
				pos.linear.x = trolley; //  trolley type
				pos.angular.z = findtrolleywidth((int)trolley); // trolley width
				pos_pub.publish(pos);
				mstate = 162;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 162:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						liftMechanism(true);				
						mstate = 163;				
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 163:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (agvjackUp) {
					ROS_INFO("----------- MovNode 163 : Jack is Up. -----------");
					mstate = 164;			
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 164:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				// reverse L dist
				movedone = 1;
				moveflag = 0;
				pos.linear.z = 42.2; // 
				pos.angular.z = -trolleydist; // reverse trolley dist
				pos_pub.publish(pos);
				mstate = 165;
				clearmap_start_time = ros::Time::now();
				ROS_INFO("----------- MovNode 164 : Bring Trolley out with reversing. -----------");
			}
			break;
		case 165:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						liftMechanism(false);				
						mstate = 166;				
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 166:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (!agvjackUp) {
					ROS_INFO("-------- MovNode 166: Jack is down -----------");
					mstate = 167;			
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 167:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				// rotate 90
				movedone = 1;
				moveflag = 0;
				pos.linear.z = 42.0; // 
				pos.angular.z = 90.0;
				pos_pub.publish(pos);
				mstate = 168;
				ROS_INFO("------------- MovNode 167 : Rotate 90 antiClockwise. -----------");
			}
			break;
		case 168:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						liftMechanism(true);				
						mstate = 169;				
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 169:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (agvjackUp) {
					mstate = 1690;			
					ROS_INFO("---------- MovNode 169 : Jack is Up. -------------");
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 1690:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (redirect) {
				//if (destPoint == 2) {
					mstate = nstate;
					redirect = false;
				} else {
					mstate = 0;
					ROS_INFO("----------- MovNode 1690 : PickUp Completed. -----------");
				}
				ntucjobstatus = 3; // not ready
				publish_ntuc_status(1,1,targetLP,(int)(avgenergy),ntucjobstatus);
			}
			break;
		case 17: // drop trolley LP
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				// jack down
				liftMechanism(false);				
				mstate = 170;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 170:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (!agvjackUp) {
					mstate = 171;			
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 171:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				// rotate -90
				movedone = 1;
				moveflag = 0;
				pos.linear.z = 42.0; // 
				pos.angular.z = -90.0;
				pos_pub.publish(pos);
				mstate = 172;
			}
			break;
		case 172:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						liftMechanism(true);				
						mstate = 173;				
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 173:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (agvjackUp) {
					mstate = 174;			
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 174:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				// reverse L dist
				movedone = 1;
				moveflag = 0;
				pos.linear.z = 42.2; // 
				pos.angular.z = trolleydist; // move trolley dist
				pos_pub.publish(pos);
				mstate = 175;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 175:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						liftMechanism(false);				
						mstate = 176;				
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 176:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (!agvjackUp) {
					mstate = 177;			
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 177:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				// reverse L dist
				movedone = 1;
				moveflag = 0;
				pos.linear.z = 42.2; // 
				pos.angular.z = -trolleydist; // move trolley dist
				pos_pub.publish(pos);
				mstate = 178;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 178:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						//if (destPoint == 2) {
						if (redirect) {
							mstate = nstate;
							redirect = false;
							ROS_INFO("-------------- MovNode 178 : mstate nstate. ------------------");
						} else {
							mstate = 0;
							ROS_INFO("-------------- MovNode 178 : mstate 0. ------------------");
						}
						if (low_power) {
							ntucjobstatus = 2; 
							publish_ntuc_status(1,1,otargetLP,(int)(avgenergy),ntucjobstatus);
						} else {
							ntucjobstatus = 1; 
							publish_ntuc_status(1,1,otargetLP,(int)(avgenergy),ntucjobstatus);
						}
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 22: // Reach Docking Station for hot swap robot
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {					
						publish_status("Docked. Transporter Charging ....");
						//publish_event("--- Docking Completed. Charging now ---");
						ROS_INFO("Docked. Transporter Charging ....");
						if (low_power) {
							mstate = 29;
							ROS_INFO("low power. going to state 29. avgenergy = %.3f",avgenergy);
						} else {
							mstate = 27;
							ROS_INFO("going to state 27. avgenergy = %.3f",avgenergy);
						}
						completedocking = true;
						clearmap_start_time = ros::Time::now();	
					}					
				}
			}		
			break;
		case 27:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.0)) ) {	
				if ((avgenergy > work_voltage)) {  // work_voltage = 30%
					mstate = 290;	
					ROS_INFO("Transporter has sufficient energy.Ready for next Job");
					publish_status("Transporter has sufficient energy.Ready for next Job");
					//publish_fButton(199);				
					ntucjobstatus = 1; 
					publish_ntuc_status(1,5,otargetLP,(int)(avgenergy),ntucjobstatus);
				}
			}
			break;
		case 29:
			if ((!low_power) && (low_power_flag)) {
				mstate = 290;		
				publish_status("Transporter Charged.");
				ROS_INFO("Low Power : Transporter Charged.");
				//publish_fButton(199);				
				low_power_flag = false;
				ntucjobstatus = 1; 
				publish_ntuc_status(1,5,otargetLP,(int)(avgenergy),ntucjobstatus);
			}
			break;
		case 290:
			rn.getParam("numLPInQ",numLPInQ);
			if ((numLPInQ > 0) && !low_power) {
				movedone = 1;
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,DISENGAGE);
				//publish_sound(STOPMOVESOUND,0,0);
				//publish_event("--- MovNode : Moving away from Docking Station ---");
				mstate = 291;
				clearmap_start_time = ros::Time::now();	
			}	
			break;
		case 291: // 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						mstate = 0;
					}
				}		
			}	
			break;				
		case 310: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {	
				rn.getParam("Cancel_Nav",Cancel_Nav);	
				if (Cancel_Nav) {
					mstate = 0;
					stuck = false;										
					Cancel_Nav = false;
					rn.setParam("Cancel_Nav",Cancel_Nav);
					ROS_INFO(" ============== Move_Node 310 : Cancel_Nav. state = 0 =========================");
				} else {
					rn.setParam("checkPlanSum",0.0);
					normalPath = true;
					rn.setParam("normalPath",normalPath);  
					checkpath(tx,ty);
					mstate = 311; //316; //311;
					ROS_INFO(" ============== Move_Node 310 : Check path =========================");
					clearmap_start_time = ros::Time::now();
				}
				count++;
			}
			break;
		case 311: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {	
				rn.getParam("checkPlanSum",checkPlanSum); 
				if (checkPlanSum == 200.0) {
					ROS_INFO(" ============== Move_Node 311 : TurnToPath =========================");
					if (stuck) {
						ROS_INFO(" -------------  MoveNode : Stuck ...................");						
						mstate = 316;
						stuck = false;
					} else {
						mstate = 313;
					}
					rn.getParam("checkPlanX",checkPlanX);
					rn.getParam("checkPlanY",checkPlanY);  
					movedone = 1;	
					ret = sendGoalpub(checkPlanX,checkPlanY,tz,rx,ry,rz,rw,ZSD,ZSA,TURNTOPATH); // TURNTOPATH					
				} else {
					mstate = 310; //312;					
				}				 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 312:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						if (count > 3) {
							mstate = 108; 
							publish_sound(NAVABORT,0,5);
						} else {
							mstate = 310;
						}
						clearmap_start_time = ros::Time::now();
					}
				}
			}
			break;
		case 313: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {		
						ROS_INFO(" ========== Move_Node 313 : Normal Move Again ==========");
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						mstate = 1; //1; //314; //1;  //32		
						movedone = 1;						
						moveflag = 0;
						rn.setParam("BackObs",1);		
						rn.setParam("amcl_monitor",true);				
						clearmap_start_time = ros::Time::now();														
					}
				} 
			}
			break;
		case 314: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (normalPath) {
					dynaReconfig(global_planner,1,0.0,"navfn/NavfnROS");
					normalPath = false;
					rn.setParam("normalPath",normalPath);					
				}
				mstate = 1;  //32		
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 315:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						//dynaReconfig(global_costmap_inflation_radius,1,0.72,"");
						rn.setParam("racPlanCall",true);
						mstate = 313; 
						movedone = 0;
						moveflag = 1;
						clearmap_start_time = ros::Time::now();	
					}
				}
			}
			break;
		case 316:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_clear();
						mstate = 313; 
						movedone = 0;
						moveflag = 1;
						clearmap_start_time = ros::Time::now();	
					}
				}
			}
			break;
		case 319:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				//dynaReconfig(global_costmap_inflation_radius,1,0.72,"");				
				mstate = 313; 
				stuck = false;
				publish_clear();
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 317:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.8)) ) {
				mstate = 310; 
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 318:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.8)) ) {
				mstate = 1; 
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 32:
			dynaReconfig(global_planner,1,0.0,"rac_planner/RacPlanner");
			clearmap_start_time = ros::Time::now();
			mstate = 320;
			break;
		case 320:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				ROS_INFO(" ========== Move_Node 320 : Normal Move using RACPlanner ==========");
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				mstate = 1;  //32		
				movedone = 1;						
				moveflag = 0;
				rn.setParam("BackObs",1);		
				rn.setParam("amcl_monitor",true);				
				clearmap_start_time = ros::Time::now();	 
				dynaReconfig(global_planner,1,0.0,"navfn/NavfnROS"); 
			}
			break;
		case 36:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						mstate = 150;
						ROS_INFO("----------- MovNode : 36. -----------------");
						//clearmap_start_time = ros::Time::now();
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 5:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {	
				moveToStartPosition();
				mstate = 310;		
				nstate = 50;	
				cleanplanmove = true;
				rn.setParam("cleanplanmove",cleanplanmove);
				//publish_event("--- MovNode : Moving to Cleaning Start Position ---");
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 50:  // at start position
			// change global planner			
			dynaReconfig(global_planner,1,0.0,"rac_planner/RacPlanner");
			//publish_event("--- MovNode : Change to RAC Global Planner ---");
			//ROS_INFO("----- MovNode : Change Global Planner ---");
			clearmap_start_time = ros::Time::now();
			//movedone = 0;	
			//moveflag = 1;
			mstate = 507;
			break;
		case 501:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				rn.getParam("PPPtr",PPPtr);							
				//ROS_INFO("--- MovNode 501 : Turn to first Point in Path. PPPtr = %d. -----",PPPtr);
				movedone = 1;	
				moveflag = 0;
				ret = sendGoalpub(pathInfo[PPPtr][0],pathInfo[PPPtr][1],tz,rx,ry,rz,rw,ZSD,ZSA,TURNTOPATH);
				mstate = 502;
				//ROS_INFO("--- MovNode 501 : PPPtr=%d. Turn X=%.3f. Y=%.3f. ---",PPPtr,pathInfo[PPPtr][0],pathInfo[PPPtr][1]);
				sprintf(buf,"--- MovNode 501 : Turn to first Point in Path. PPPtr = %d. X=%.3f. Y=%.3f. ---",PPPtr,pathInfo[PPPtr][0],pathInfo[PPPtr][1]);
				s1.assign(buf,strlen(buf));
				publish_event(s1);					
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 502: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.15)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {		
						//rn.getParam("PPPtr",PPPtr);
						publish_event("--- MovNode : Move along path ---");
						ROS_INFO("--- MovNode 502 : Move along Path. PPPtr = %d. -----",PPPtr);	
						nh.setParam("racPlanCall",true);						
						moveToSegEndPosition();
						destPoint = 1;
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						mstate = 1;  //32		
						movedone = 1;						
						moveflag = 0;
						nstate = 508; //505; //503;	
						//rn.setParam("BackObs",1);		
						//rn.setParam("amcl_monitor",true);				
						clearmap_start_time = ros::Time::now();		
					}
				} 
			}
			break;
		case 503:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				//publish_event("--- MovNode : Check if End of Cleaning Plan ---");
				//ROS_INFO("-----  MoveNode : 503 . Check if End of Path Plan------------");				
				racplanMove = false;
				rn.setParam("racplanMove",false);
				rn.getParam("cleanplanmove",cleanplanmove);
				if (!cleanplanmove) {
					mstate = 504;
				} else {
					mstate = 509;
					//ROS_INFO("-----  MoveNode : 503 . Next Path ------------");
					//publish_event("--- MovNode 503 : Next portion of Cleaning Plan  ---");
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 504:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				publish_event("--- MovNode : Reach Cleaning End Position. Stop Cleaning ---");
				dynaReconfig(global_planner,1,0.0,"navfn/NavfnROS"); // change back to global planner
				mstate = 0;
				destPoint = 0;
			}
			break;	
		case 505:  // 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				publish_event("--- MovNode 505  : Close Gap ---");
				ROS_INFO("--- MovNode 503 : Close Gap -----");	
				movedone = 1;						
				moveflag = 0;
				closeGapToGoal();
				mstate = 506;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 506: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {		
						mstate = 503;  //32		
						clearmap_start_time = ros::Time::now();		
					}
				} 
			}
			break;
		case 507: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {				
				mstate = 509;		
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 508: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.05)) ) {
				mstate = 503;  //32		
				clearmap_start_time = ros::Time::now();		 
			}
			break;
		case 509: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				//publish_event("--- MovNode 509 : Move along path ---");
				//ROS_INFO("--- MovNode 509 : Move along Path. PPPtr = %d. -----",PPPtr);	
				nh.setParam("racPlanCall",true);						
				moveToSegEndPosition();
				destPoint = 1;
				racplanMove = true;
				rn.setParam("racplanMove",true);				
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				mstate = 1;  //32		
				movedone = 1;						
				moveflag = 0;
				nstate = 503; //508; //505; //503;	
				//rn.setParam("BackObs",1);		
				//rn.setParam("amcl_monitor",true);				
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 6:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {	
				dynaReconfig(global_planner,1,0.0,"navfn/NavfnROS");	
				checkLocInPath();  // find location in path
				nPPPtr = curPPtr + 1;
				mstate = 60;		
				nstate = 61;
				clearObs = false;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 60:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {	
				// find location of obstacle
				nnx = pathInfo[nPPPtr][0];
				nny = pathInfo[nPPPtr][1];
				checkpath(nnx,nny);
				mstate = nstate;		
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 61:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {	
				nh.getParam("checkPlanSum",checkPlanSum); 
				if (!clearObs) {
					if (checkPlanSum == 200.0) {		
						nPPPtr++;
						if (nPPPtr >= (noPP-1)) {
							mstate = 69;  // end of path
						} else {
							mstate = 60;
							nstate = 61;
						}
					} else {
						// found location of obs
						publish_event("--- MovNode : Found Location of Obs ---");
						nPPPtr++;
						if (nPPPtr >= (noPP-1)) {
							mstate = 69;  // end of path
						} else {
							clearObs = true;
							mstate = 60;
							nstate = 61;
						}
					}
				} else {
					if (checkPlanSum == 200.0) {
						// found location after obs
						publish_event("--- MovNode : Found Location after Obs ---");
						mstate = 62;
					} else {
						nPPPtr++;
						if (nPPPtr >= (noPP-1)) {
							mstate = 69;  // end of path
						} else {
							mstate = 60;
							nstate = 61;
						}
					}
				}
				clearmap_start_time = ros::Time::now();		
			}
			break;
		case 62:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				// prepare location to move to
				prepareLocInPath(nPPPtr);						
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				mstate = 310;
				//movedone = 1;						
				//moveflag = 0;
				destPoint = 1;
				nstate = 64;
				racplanMoveObs = true;
			}
			break;
		case 63:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {
				if (nPPPtr < (noPP-1)) {
					nPPPtr++;
					mstate = 62;
				} else {
					mstate = 69;
				}
				clearmap_start_time = ros::Time::now();		
			}
			break;
		case 64:
			// Return to path after obstacle
			publish_event("--- MovNode : Back to Path ---");
			racplanMoveObs = false;
			nh.setParam("cleanplanmoveObs",true); 
			nh.setParam("PPPtr",nPPPtr); 			
			mstate = 50;
			break;
		case 69:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(3.0)) ) {				
				mstate = 504;
				clearmap_start_time = ros::Time::now();
			}			
			break;
		case 7:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {	
				moveToStartPosition();
				mstate = 310;		
				nstate = 70;					
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 70:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {	
				ret = sendGoalpub(0,0,0,0,0,0,0,ZSD,ZSA,CLEANPLAN);
				mstate = 71;	
				movedone = 1;						
				moveflag = 0;
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 71:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						ROS_INFO("---------------- MoveNode 71 : Cancel Goal ---------------------");
						cancel_goal();
						mstate = 72;
						movedone = 0;						
						moveflag = 1;
						clearmap_start_time = ros::Time::now();		
					}
				}
			}
			break;
		case 72:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.5)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						ROS_INFO("-------- MovNode 72 : Move to End Point ----------");
						moveToEndPosition();
						mstate = 310;
						nstate = 73;
						clearmap_start_time = ros::Time::now();		
					}
				}
			}
			break;
		case 73:  // at end position
			ROS_INFO("-------- MovNode 73 : At End Position. Cleanplanmove completed ----------");
			clearmap_start_time = ros::Time::now();
			mstate = 0;
			//cleanplanmove = false;
			//rn.setParam("cleanplanmove",cleanplanmove);	
			break;
		case 74:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.1)) ) {	
				mstate = 0;  //32	
				cleanplanmove = false;
				rn.setParam("cleanplanmove",cleanplanmove);	
				ROS_INFO("------------- MoveNode 73 : Cleanplanmove completed ---------------");
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 9:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(3.0)) ) {	
				publish_sound(NAVABORT,0,0); // nav aborted		
				clearmap_start_time = ros::Time::now();					
			}
			break;
		case 2000: // LIFTAREA_TO_LOBBYAREA
			mstate = 2300;
			nstate = 2001;
			break;
		case 2001: // Go to AGV Lift LP
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						if (dbRequestSP("LFLD")) {
							movedone = 1;
							destPoint = 1;
							redirect = true;
							mstate = 310; // 
							nstate = 2002;
							ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
							//clearmap_start_time = ros::Time::now();
						} else {
							ROS_INFO("---------MovNode 2001 :  LFLD Error -----------");
						}
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2002: // Call for Lift.
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_sound(ASKLIFT,0,0);
						ROS_INFO("-------------MovNode 2002 : AGV Requesting for Lift  ----------------");
						publish_lift(ASKLIFTTOCOME,currentfloor,0,1,0);
						mstate = 2003;						
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2003:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if ((ntucstatus == 10) && (liftDoorOpen)) {
					// robot move into lift
					ntucstatus = 0;
					liftDoorOpen = false;
					ROS_INFO("-------------MovNode 2003 :  AGV Move into lift ----------------");
					//publish_sound(MOVEINTOLIFT,0,0);			
					//if (dbRequestS("LFIN")) {
					//	movedone = 1;
					//	destPoint = 1;
					//	mstate = 310; // 
					//	nstate = 2004;
					//	ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
					//} else {
					//	ROS_INFO("--------- MovNode 2003 : LFIN  Error. -----------");
					//}
					mstate = 2004;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2004:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {
					mstate = 2005;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2005:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				publish_sound(MOVEINTOLIFT,0,0);			
				if (dbRequestS("LFIN")) {
					movedone = 1;
					destPoint = 1;
					redirect = true;
					mstate = 310; // 
					nstate = 2006;
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				} else {
					ROS_INFO("--------- MovNode 2005 : LFIN  Error. -----------");
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2006:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						// AGV turn around inside lift
						publish_lift(AGVINLIFT,currentfloor,0,1,0);  // robot in lift
						liftMechanism(false); // lower jack						
						mstate = 2007;		
					}
				}		
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2007:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (!agvjackUp) {
					// AGV rotate 180.0
					movedone = 1;
					moveflag = 0;
					pos.linear.z = 42.0; // 
					pos.angular.z = 180.0;
					pos_pub.publish(pos);
					mstate = 2008;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2008:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						liftMechanism(true); // raise jack
						//clearmap_start_time = ros::Time::now();		
						publish_sound(ASKLIFT,0,0);
						publish_lift(LIFTGOTO,1,checkFloor(targetLP),1,0);  // ask for lift to ground floor
						mstate = 2009;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;	
		case 2009:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (agvjackUp) {
					if ((ntucstatus == 10) && (liftDoorOpen)) {
						ntucstatus = 0;
						liftDoorOpen = false;
						mstate = 2010;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}			
			break;
		case 2010:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {
					mstate = 2011;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2011:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				publish_sound(MOVEOUTOFLIFT,0,0);			
				if (dbRequestS("LFLBD")) {
					movedone = 1;
					destPoint = 1;
					redirect = true;
					mstate = 310; // 
					nstate = 2012;
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				} else {
					ROS_INFO("--------- MovNode 2007 : LFLBD  Error. -----------");
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;		
		case 2012: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_lift(AGVOUTSIDE,1,1,0,0);
						publish_fButton(301);
						publish_sound(REQUESTLOBYDOOROPEN,0,0);
						ROS_INFO("--------- MovNode 2012 : Request Lobby Door Open. -----------");
						//clearmap_start_time = ros::Time::now();	
						ntucDoorOpenRequestCnt = 0;
						mstate = 2013;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2013:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {				
				if (ntucLobbyDoor == 1) {
					mstate = 2014;
					ntucDoorOpenRequestCnt = 0;
					ROS_INFO("------------- MovNode 2013 : Lobby Door Command Received. ----------------");
				} else {
					publish_fButton(301);
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 20) {
						publish_sound(LOBBYDOORCMDERROR,0,0); //Lobby Door Error. Command not recevied
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2014:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {				
				if (ntucLobbyDoor == 2) {
					mstate = 2015;
					ntucDoorOpenRequestCnt = 0;
					ntucLobbyDoor = 0;
					publish_sound(LOBBYDOOROPENED,0,0);
					ROS_INFO("-------------MovNode 2010 : Lobby Door Command Activated. ----------------");
				} else {
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 25) {
						publish_sound(LOBBYDOORCONTROLERROR,0,0); //Lobby Door Error. Door Controller Error
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2015:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {
					mstate = 2016;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2016:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				restoreLP();   // restore final LP				
				movedone = 1;
				destPoint = 2;
				redirect = true;
				mstate = 310; // 		
				nstate = 2017;		
				//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);			
				clearmap_start_time = ros::Time::now();		
			}
			break;
		case 2017:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						mstate = 2018;	
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2018:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (!agvjackUp) {
					if (dbRequestS("LBRA")) {
						mstate = 2300; // 
						nstate = 2019;
					} else {
						ROS_INFO("---------MovNode 2018 :  Lift Rest Area LP  Error. -----------");
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2019:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						destPoint = 0;
						redirect = false;
						//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						mstate = 310;
						clearmap_start_time = ros::Time::now();
					} 
				} 
			}
			break;
		case 2050: // LIFTAREA_TO_LAUNDRYAREA
			mstate = 2300;
			nstate = 2051;
			break;
		case 2051:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						if (dbRequestSP("LFLD")) {
							movedone = 1;
							destPoint = 1;
							redirect = true;
							mstate = 310; // 
							nstate = 2052;
							//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
							//clearmap_start_time = ros::Time::now();
						} else {
							ROS_INFO("--------- MovNode 2051 : LFLD Error -----------");
						}
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2052:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_sound(ASKLIFT,0,0);
						ROS_INFO("-------------MovNode 2052 : AGV Requesting for Lift  ----------------");
						publish_lift(ASKLIFTTOCOME,currentfloor,0,1,0);
						mstate = 2053;
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2053:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if ((ntucstatus == 10) && (liftDoorOpen)) {
					// robot move into lift
					ntucstatus = 0;
					liftDoorOpen = false;
					mstate = 2054;
					ROS_INFO("------------- MovNode 2053 : AGV Move into lift ----------------");
					//publish_sound(MOVEINTOLIFT,0,0);			
					//if (dbRequestS("LFIN")) {
					//	movedone = 1;
					//	destPoint = 1;
					//	mstate = 310; // 
					//	nstate = 2054;
					//	ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
					//} else {
					//	ROS_INFO("---------MovNode 2053 : Inside Lift LP  Error. -----------");
					//}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2054:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {
					mstate = 2055;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2055:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				publish_sound(MOVEINTOLIFT,0,0);			
				if (dbRequestS("LFIN")) {
					movedone = 1;
					destPoint = 1;
					redirect = true;
					mstate = 310; // 
					nstate = 2056;
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				} else {
					ROS_INFO("---------MovNode 2055 : Inside Lift LP  Error. -----------");
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2056:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						// AGV turn around inside lift
						publish_lift(AGVINLIFT,currentfloor,0,1,0);  // robot in lift
						liftMechanism(false); // lower jack						
						mstate = 2057;		
					}
				}		
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2057:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (!agvjackUp) {
					// AGV rotate 180.0
					movedone = 1;
					moveflag = 0;
					pos.linear.z = 42.0; // 
					pos.angular.z = 180.0;
					pos_pub.publish(pos);
					mstate = 2058;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2058:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						liftMechanism(true); // raise jack
						clearmap_start_time = ros::Time::now();		
						publish_sound(ASKLIFT,0,0);
						publish_lift(LIFTGOTO,1,checkFloor(targetLP),1,0);  // ask for lift to ground floor
						mstate = 2059;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2059:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (agvjackUp) {
					if ((ntucstatus == 11) && (liftDoorOpen)) {
						ntucstatus = 0;
						liftDoorOpen = false;
						mstate = 2060;
						//publish_sound(MOVEOUTOFLIFT,0,0);			
						//if (dbRequestS("LFLBD")) {
						//	movedone = 1;
						//	destPoint = 1;
						//	mstate = 310; // 
						//	nstate = 2058;
						//	ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						//} else {
						//	ROS_INFO("--------- MovNode 2057 : LFLBD  Error. -----------");
						//}
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;	
		case 2060:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {
					mstate = 2061;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2061:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				publish_sound(MOVEOUTOFLIFT,0,0);			
				if (dbRequestS("LFLBD")) {
					movedone = 1;
					destPoint = 1;
					redirect = true;
					mstate = 310; // 
					nstate = 2062;
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				} else {
					ROS_INFO("--------- MovNode 2061 : LFLBD  Error. -----------");
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;	
		case 2062: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_lift(AGVOUTSIDE,1,0,0,0);
						publish_fButton(301);
						publish_sound(REQUESTLOBYDOOROPEN,0,0);							
						ntucDoorOpenRequestCnt = 0;
						mstate = 2063;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2063:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {				
				if (ntucLobbyDoor == 1) {
					mstate = 2064;
					ntucDoorOpenRequestCnt = 0;
					ROS_INFO("-------------MovNode 2063 : Lobby Door Command Received. ----------------");
				} else {
					publish_fButton(301);
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 20) {
						publish_sound(LOBBYDOORCMDERROR,0,0); //Lobby Door Error. Command not recevied
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2064:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {				
				if (ntucLobbyDoor == 2) {
					mstate = 2065;
					ntucDoorOpenRequestCnt = 0;
					ntucLobbyDoor = 0;
					publish_sound(LOBBYDOOROPENED,0,0);
					ROS_INFO("------------- MovNode 2064 : Lobby Door Command Activated. ----------------");
				} else {
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 25) {
						publish_sound(LOBBYDOORCONTROLERROR,0,0); //Lobby Door Error. Door Controller Error
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2065:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				if (dbRequestS("LBLB")) {
					movedone = 1;
					destPoint = 1;
					redirect = true;
					mstate = 310; // 
					nstate = 2066;
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				} else {
					ROS_INFO("--------- MovNode 2061 : LoadingBay_lobby LP  Error. -----------");
				}			
			}
			break;
		case 2066: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						rn.getParam("LoadingBayStatus",LoadingBayStatus);
						if (!LoadingBayStatus) {
							mstate = 2067;
						}						
						//clearmap_start_time = ros::Time::now();
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2067:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				restoreLP();   // restore final LP				
				movedone = 1;
				destPoint = 2;
				redirect = true;
				mstate = 310; // 				
				nstate = 2068;
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);			
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2068:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						mstate = 2069;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2069:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (!agvjackUp) {
					if (dbRequestS("REF")) {
						mstate = 2300; // 
						nstate = 2070;
					} else {
						ROS_INFO("---------  MovNode 2069 : REF   Error. -----------");
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2070:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						destPoint = 0;
						redirect = false;
						//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						mstate = 310;
						clearmap_start_time = ros::Time::now();
					} 
				} 
			}
			break;
		case 2100: // LOBBYAREA_TO_LAUNDRYAREA
			mstate = 2300;
			nstate = 2101;
			break;
		case 2101:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						if (dbRequestSP("LBLB")) {
							movedone = 1;
							destPoint = 1;
							redirect = true;
							mstate = 310; // 
							nstate = 2102;
							//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);							
						} else {
							ROS_INFO("--------- MovNode 2101 : LoadingBay_lobby LP  Error -----------");
						}
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;		
		case 2102: 
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						rn.getParam("LoadingBayStatus",LoadingBayStatus);
						if (!LoadingBayStatus) {
							mstate = 2103;
						}												
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2103:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				restoreLP();   // restore final LP				
				movedone = 1;
				destPoint = 2;
				redirect = true;
				mstate = 310; // 				
				nstate = 2104;
				//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);	
				clearmap_start_time = ros::Time::now();			
			}
			break;
		case 2104:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						mstate = 2105;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2105:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (!agvjackUp) {
					if (dbRequestS("REF")) {
						mstate = 2300; // 
						nstate = 2106;
					} else {
						ROS_INFO("---------MovNode 2105 : Lift Rest Area LP  Error. -----------");
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2106:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						destPoint = 0;
						redirect = false;
						//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						mstate = 310;
						clearmap_start_time = ros::Time::now();
					} 
				} 
			}
			break;
		case 2150: // LOBBYAREA_TO_LIFTAREA
			mstate = 2300;
			nstate = 2151;
			break;
		case 2151:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						if (dbRequestSP("LBLD")) {
							movedone = 1;
							destPoint = 1;
							redirect = true;
							mstate = 310; // 
							nstate = 2152;
							//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);							
						} else {
							ROS_INFO("---------MovNode 2151 : Outside_lobby LP  Error -----------");
						}
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2152:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_fButton(301);
						publish_sound(REQUESTLOBYDOOROPEN,0,0);
						clearmap_start_time = ros::Time::now();	
						ntucDoorOpenRequestCnt = 0;
						mstate = 2153;
					}
				}
			}
			break;
		case 2153:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {				
				if (ntucLobbyDoor == 1) {
					mstate = 2154;
					ntucDoorOpenRequestCnt = 0;
					ROS_INFO("-------------MovNode 2153 : Lobby Door Command Received. ----------------");
				} else {
					publish_fButton(301);
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 20) {
						publish_sound(LOBBYDOORCMDERROR,0,0); //Lobby Door Error. Command not recevied
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2154:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {				
				if (ntucLobbyDoor == 2) {
					mstate = 2155;
					ntucDoorOpenRequestCnt = 0;
					ntucLobbyDoor = 0;
					ROS_INFO("-------------MovNode 2154 : Lobby Door Command Activated. ----------------");
				} else {
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 25) {
						publish_sound(LOBBYDOORCONTROLERROR,0,0); //Lobby Door Error. Door Controller Error
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2155:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (dbRequestS("LFLD")) {
					movedone = 1;
					destPoint = 1;
					redirect = true;
					mstate = 310; // 
					nstate = 2156;
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);					
				} else {
					ROS_INFO("---------MovNode 2155 : Outside Lift LP Error -----------");
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2156:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_sound(ASKLIFT,0,0);
						ROS_INFO("-------------MovNode 2156 : AGV Requesting for Lift  ----------------");
						publish_lift(ASKLIFTTOCOME,1,0,1,0);
						mstate = 2157;						
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2157:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if ((ntucstatus == 10) && (liftDoorOpen)) {
					// robot move into lift
					ntucstatus = 0;
					liftDoorOpen = false;
					mstate = 2158;
					//ROS_INFO("------------- AGV Move into lift ----------------");
					//publish_sound(MOVEINTOLIFT,0,0);			
					//if (dbRequestS("LFIN")) {
					//	movedone = 1;
					//	destPoint = 1;
					//	mstate = 310; // 
					//	nstate = 2158;
					//	//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
					//} else {
					//	ROS_INFO("---------MovNode 2157 : Inside Lift LP  Error. -----------");
					//}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2158:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {
					mstate = 2159;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2159:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				publish_sound(MOVEINTOLIFT,0,0);			
				if (dbRequestS("LFIN")) {
					movedone = 1;
					destPoint = 1;
					redirect = true;
					mstate = 310; // 
					nstate = 2160;
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				} else {
					ROS_INFO("---------MovNode 2159 : Inside Lift LP  Error. -----------");
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2160:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						// AGV turn around inside lift
						publish_lift(AGVINLIFT,1,0,1,0);  // robot in lift
						liftMechanism(false); // lower jack						
						mstate = 2161;				
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2161:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (!agvjackUp) {
					// AGV rotate 180.0
					movedone = 1;
					moveflag = 0;
					pos.linear.z = 42.0; // 
					pos.angular.z = 180.0;
					pos_pub.publish(pos);
					mstate = 2162;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2162:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						liftMechanism(true); // raise jack
						clearmap_start_time = ros::Time::now();		
						publish_sound(ASKLIFT,0,0);
						publish_lift(LIFTGOTO,1,checkFloor(targetLP),1,0);  
						mstate = 2163;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;		
		case 2163:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (agvjackUp) {
					if ((ntucstatus == 11) && (liftDoorOpen)) {
						ntucstatus = 0;
						liftDoorOpen = false;
						mstate = 2164;
						//publish_sound(MOVEOUTOFLIFT,0,0);			
						//restoreLP();   // restore final LP				
						//movedone = 1;
						//destPoint = 2;
						//mstate = 310; // 		
						//nstate = 2162;		
						//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);		
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2164:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {
					mstate = 2165;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2165:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				publish_sound(MOVEOUTOFLIFT,0,0);			
				restoreLP();   // restore final LP				
				movedone = 1;
				destPoint = 2;
				redirect = true;
				mstate = 310; // 		
				nstate = 2166;		
				//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);		
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2166:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						publish_lift(AGVOUTSIDE,currentfloor,0,0,0);
						mstate = 2167;		
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2167:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (!agvjackUp) {
					if (dbRequestS("LFRA")) {
						mstate = 2300; // 
						nstate = 2168;
						publish_lift(KITCHEN,currentfloor,0,0,currentfloor);
					} else {
						ROS_INFO("---------MovNode 2163 :  Lift Rest Area LP  Error. -----------");
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2168:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						destPoint = 0;
						redirect = false;
						//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						mstate = 310;
						clearmap_start_time = ros::Time::now();
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2200: // LAUNDRYAREA_TO_LIFTAREA
			ROS_INFO("----------- MovNode 2200: LAUNDRYAREA_TO_LIFTAREA ---------------");
			mstate = 2300; // pre-move
			nstate = 2201;
			break;
		case 2201:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						if (dbRequestSP("LULB")) {
							ROS_INFO("---------MovNode 2201 : Move To LULB  -----------");
							movedone = 1;
							destPoint = 1;
							redirect = true;
							mstate = 310; // 
							nstate = 2202;
							//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);							
						} else {
							ROS_INFO("---------MovNode 2201 : LULB LP  Error -----------");
						}
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2202:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						ROS_INFO("---------MovNode 2202 : Checking loadingBay Status  -----------");
						rn.getParam("LoadingBayStatus",LoadingBayStatus);
						if (!LoadingBayStatus) {
							mstate = 2203;
							ROS_INFO("---------MovNode 2202 : loadingBay Clear  -----------");
						}												
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2203:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (dbRequestS("LBLD")) {
					ROS_INFO("---------MovNode 2203 : Move To LBLD  -----------");
					movedone = 1;
					destPoint = 1; // mstate = nstate
					redirect = true;
					mstate = 310; // 
					nstate = 2204;
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
					//clearmap_start_time = ros::Time::now();
				} else {
					ROS_INFO("---------MovNode 2203 : Outside LBLD  Error -----------");
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2204:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_fButton(301);// request to open lobby door						
						publish_sound(REQUESTLOBYDOOROPEN,0,0);
						ROS_INFO("-------------MovNode 2204 : Request to open Lobby Door. ----------------");
						//clearmap_start_time = ros::Time::now();	
						ntucDoorOpenRequestCnt = 0;
						mstate = 2205;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2205:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {				
				if (ntucLobbyDoor == 1) {
					mstate = 2206;
					ntucDoorOpenRequestCnt = 0;
					ROS_INFO("-------------MovNode 2205 : : Lobby Door Command Received. ----------------");
				} else {
					publish_fButton(301);
					ROS_INFO("-------------MovNode 2205 : Request to open Lobby Door Again. ----------------");
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 20) {
						publish_sound(LOBBYDOORCMDERROR,0,0); //Lobby Door Error. Command not recevied
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2206:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {				
				if (ntucLobbyDoor == 2) {
					mstate = 2207;
					ntucDoorOpenRequestCnt = 0;
					ntucLobbyDoor = 0;
					publish_sound(LOBBYDOOROPENED,0,0);
					ROS_INFO("-------------MovNode 2206 : Lobby Door Command Activated. ----------------");
				} else {
					ntucDoorOpenRequestCnt++;
					if (ntucDoorOpenRequestCnt > 25) {
						publish_sound(LOBBYDOORCONTROLERROR,0,0); //Lobby Door Error. Door Controller Error
						ntucDoorOpenRequestCnt = 0;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2207:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {
					mstate = 2208;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2208:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				if (dbRequestS("LFLD")) {
					movedone = 1;
					destPoint = 1;
					redirect = true;
					mstate = 310; // 
					nstate = 2209;
					ROS_INFO("---------MovNode 2208 : Go to LFLD -----------");
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
					//clearmap_start_time = ros::Time::now();
				} else {
					ROS_INFO("---------MovNode 2208 : Outside Lift LP Error -----------");
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2209:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_sound(ASKLIFT,0,0);
						ROS_INFO("------------- MovNode 2208 :  AGV Requesting for Lift  ----------------");
						publish_lift(ASKLIFTTOCOME,1,0,1,0);
						mstate = 2210;
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2210:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if ((ntucstatus == 10) && (liftDoorOpen)) {
					// robot move into lift
					ntucstatus = 0;
					liftDoorOpen = false;
					//ROS_INFO("-------------MovNode 2209 : AGV Move into lift ----------------");								
					//if (dbRequestS("LFIN")) {
					//	movedone = 1;
					//	destPoint = 1;
					//	mstate = 310; // 
					//	nstate = 2210;
					//	//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
					//	publish_sound(MOVEINTOLIFT,0,0);
					//} else {
					//	ROS_INFO("---------MovNode 2209 : Inside Lift LP  Error. -----------");
					//}
					mstate = 2211;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2211:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {
					mstate = 2212;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2212:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(3.0)) ) {
				ROS_INFO("-------------MovNode 2212 : AGV Move into lift ----------------");								
				if (dbRequestS("LFIN")) {
					movedone = 1;
					destPoint = 1;
					redirect = true;
					mstate = 310; // 
					nstate = 2213;
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
					publish_sound(MOVEINTOLIFT,0,0);
				} else {
					ROS_INFO("---------MovNode 2212 : Inside Lift LP  Error. -----------");
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2213:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						// AGV turn around inside lift
						//publish_sound(ASKLIFT,0,0);	
						publish_lift(AGVINLIFT,1,0,1,0);  // robot in lift
						liftMechanism(false); // lower jack
						ROS_INFO("---------- MovNode 2213 : Lower Jack. ----------");
						//clearmap_start_time = ros::Time::now();
						mstate = 2214;			
					}
				}	
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2214:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (!agvjackUp) {
					// AGV rotate 180.0
					ROS_INFO("---------- MovNode 2214 : Turn around inside lift. ----------");
					movedone = 1;
					moveflag = 0;
					pos.linear.z = 42.0; // 
					pos.angular.z = 180.0;
					pos_pub.publish(pos);
					mstate = 2215;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2215:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						liftMechanism(true); // raise jack
						ROS_INFO("---------- MovNode 2215 : Raise Jack. ----------");
						//restoreLP();
						//clearmap_start_time = ros::Time::now();		
						//publish_sound(ASKLIFT,0,0);
						//publish_lift(LIFTGOTO,1,checkFloor(targetLP),1,0);  
						mstate = 2216;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2216:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {	
				if (agvjackUp) {
					ROS_INFO("---------- MovNode 2216 : Ask for lift. ----------");
					restoreLP();
					//clearmap_start_time = ros::Time::now();		
					publish_sound(ASKLIFT,0,0);
					publish_lift(LIFTGOTO,1,checkFloor(targetLP),1,0);  
					mstate = 2217;
					liftDoorOpen = false;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2217:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				if ((ntucstatus == 11) && (liftDoorOpen)) {
					liftDoorOpen = false;
					ntucstatus = 0;
					mstate = 2218;
					//publish_sound(MOVEOUTOFLIFT,0,0);		
					//ROS_INFO("---------- MovNode 2217 : Move out of Lift to Ward 3. ----------");	
					//restoreLP();   // restore final LP				
					//movedone = 1;
					//destPoint = 2;
					//redirect = true;
					//mstate = 310; // 		
					//nstate = 2218;		
					//liftMechanism(false); // testing only
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);		
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2218:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {					
					publish_sound(MOVEOUTOFLIFT,0,0);		
					ROS_INFO("---------- MovNode 2218 : Move out of Lift to Ward 3. ----------");	
					//restoreLP();   // restore final LP				
					movedone = 1;
					destPoint = 2;
					redirect = true;
					mstate = 310; // 		
					nstate = 2219;		
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2219:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						publish_lift(AGVOUTSIDE,currentfloor,0,0,0);
						publish_lift(KITCHEN,currentfloor,0,0,1);
						ROS_INFO("---------- MovNode 2219 : AGV outside of Lift. ----------");
						mstate = 2220;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2220:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.5)) ) {
				if (!agvjackUp) {
					if (dbRequestS("LFRA")) {
						//publish_lift(KITCHEN,currentfloor,0,0,1);			
						ROS_INFO("--------- MovNode 2220 :   going to Lift Rest Area. -----------");			
						mstate = 2300; // 
						nstate = 2221;
					} else {
						ROS_INFO("--------- MovNode 2216 :   Lift Rest Area LP  Error. -----------");
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2221:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						destPoint = 0;
						redirect = false;
						//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						mstate = 310;
						//clearmap_start_time = ros::Time::now();
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2250: // LAUNDRYAREA_TO_LOBBYAREA
			mstate = 2300;
			nstate = 2251;
			break;
		case 2251:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						if (dbRequestSP("LULB")) {
							movedone = 1;
							destPoint = 1;
							mstate = 310; // 
							nstate = 2252;
							ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);							
						} else {
							ROS_INFO("--------- MovNode 2251 : LoadingBay_laundry LP  Error -----------");
						}
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2252:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						rn.getParam("LoadingBayStatus",LoadingBayStatus);
						if (!LoadingBayStatus) {
							mstate = 2253;
						}												
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2253:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				restoreLP();   // restore final LP				
				movedone = 1;
				destPoint = 2;
				mstate = 310; // 		
				nstate = 2254;		
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);		
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2254:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						mstate = 2255;						
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2255:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.3)) ) {
				if (!agvjackUp) {
					if (dbRequestS("LBRA")) {
						mstate = 2300; // 
						nstate = 2256;
					} else {
						ROS_INFO("---------MovNode 2255 :  Lift Rest Area LP  Error. -----------");
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2256:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						destPoint = 0;
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						mstate = 310;
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2300:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				movedone = 1;
				mstate = 2301;
				destPoint = 0;
				//publish_sound(SSMC,0,0);	
				ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist,start_angle,PREMOVE);
				clearmap_start_time = ros::Time::now();
				cancel_ops = false;
				rn.setParam("startnav",1);				
				ROS_INFO("----------- MovNode 2300 : PreMove1 :  -----------------------"); 
			}
			break;
		case 2301:  // pre move
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,start_dist1,start_angle1,PREMOVE);
						mstate = nstate;				
						ROS_INFO("----------- MovNode 2301 : PreMove2 :  -----------------------"); 		
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;		
		case 2350: // LIFTAREA_TO_LIFTAREA
			ROS_INFO("----------- MovNode : LIFTAREA_TO_LIFTAREA ---------------");
			mstate = 2300; // pre-move
			nstate = 2351;
			break;
		case 2351:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						if (dbRequestSP("LFLD")) {
							movedone = 1;
							destPoint = 1;
							redirect = true;
							mstate = 310; // 
							nstate = 2352;
							//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
							//clearmap_start_time = ros::Time::now();
						} else {
							ROS_INFO("--------- MovNode 2051 : LFLD Error -----------");
						}
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2352:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						publish_sound(ASKLIFT,0,0);
						ROS_INFO("-------------MovNode 2352 : AGV Requesting for Lift  ----------------");
						publish_lift(ASKLIFTTOCOME,currentfloor,0,1,0);
						mstate = 2353;
					}
				}
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2353:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if ((ntucstatus == 10) && (liftDoorOpen)) {
					// robot move into lift
					ntucstatus = 0;
					liftDoorOpen = false;
					mstate = 2354;
					ROS_INFO("------------- MovNode 2353 : AGV Move into lift ----------------");
					//publish_sound(MOVEINTOLIFT,0,0);			
					//if (dbRequestS("LFIN")) {
					//	movedone = 1;
					//	destPoint = 1;
					//	mstate = 310; // 
					//	nstate = 2054;
					//	ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
					//} else {
					//	ROS_INFO("---------MovNode 2053 : Inside Lift LP  Error. -----------");
					//}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2354:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {
					mstate = 2355;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2355:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				publish_sound(MOVEINTOLIFT,0,0);			
				if (dbRequestS("LFIN")) {
					movedone = 1;
					destPoint = 1;
					redirect = true;
					mstate = 310; // 
					nstate = 2356;
					//ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
				} else {
					ROS_INFO("---------MovNode 2355 : Inside Lift LP  Error. -----------");
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2356:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						// AGV turn around inside lift
						publish_lift(AGVINLIFT,currentfloor,0,1,0);  // robot in lift
						liftMechanism(false); // lower jack						
						mstate = 2357;		
					}
				}		
				clearmap_start_time = ros::Time::now();
			}
			break;
		case 2357:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (!agvjackUp) {
					// AGV rotate 180.0
					movedone = 1;
					moveflag = 0;
					pos.linear.z = 42.0; // 
					pos.angular.z = 180.0;
					pos_pub.publish(pos);
					mstate = 2358;
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2358:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						liftMechanism(true); // raise jack
						clearmap_start_time = ros::Time::now();		
						publish_sound(ASKLIFT,0,0);
						restoreLP();
						publish_lift(LIFTGOTO,1,checkFloor(targetLP),1,0);  // ask for lift to ground floor
						mstate = 2359;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2359:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {
				if (agvjackUp) {
					if ((ntucstatus == 11) && (liftDoorOpen)) {
						ntucstatus = 0;
						liftDoorOpen = false;
						mstate = 2360;
						//publish_sound(MOVEOUTOFLIFT,0,0);			
						//if (dbRequestS("LFLBD")) {
						//	movedone = 1;
						//	destPoint = 1;
						//	mstate = 310; // 
						//	nstate = 2058;
						//	ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						//} else {
						//	ROS_INFO("--------- MovNode 2057 : LFLBD  Error. -----------");
						//}
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;			
		case 2360:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(1.0)) ) {
				rn.getParam("ProfileMoveObsDist",ProfileMoveObsDist);
				if (ProfileMoveObsDist > 1.0) {					
					publish_sound(MOVEOUTOFLIFT,0,0);		
					ROS_INFO("---------- MovNode 2360 : Move out of Lift to Ward . ----------");	
					//restoreLP();   // restore final LP				
					movedone = 1;
					destPoint = 2;
					redirect = true;
					mstate = 310; // 		
					nstate = 2361;		
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2361:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.5)) ) {	
				if (movedone == 0) {
					if (moveflag == 1) {		
						publish_lift(AGVOUTSIDE,currentfloor,0,0,0);
						publish_lift(KITCHEN,currentfloor,0,0,1);
						ROS_INFO("---------- MovNode 2361 : AGV outside of Lift. ----------");
						mstate = 2362;
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2362:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(2.5)) ) {
				if (!agvjackUp) {
					if (dbRequestS("LFRA")) {
						//publish_lift(KITCHEN,currentfloor,0,0,1);			
						ROS_INFO("--------- MovNode 2362 :   going to Lift Rest Area. -----------");			
						mstate = 2300; // 
						nstate = 2363;
					} else {
						ROS_INFO("--------- MovNode 2362 :   Lift Rest Area LP  Error. -----------");
					}
				}
				clearmap_start_time = ros::Time::now();	
			}
			break;
		case 2363:
			if ( ros::Time::now() > (clearmap_start_time + ros::Duration(0.2)) ) {
				if (movedone == 0) {
					if (moveflag == 1) {
						movedone = 1;
						destPoint = 0;
						redirect = false;
						ret = sendGoalpub(tx,ty,tz,rx,ry,rz,rw,ZSD,ZSA,NORMAL);
						mstate = 310;
						//clearmap_start_time = ros::Time::now();
					} 
				} 
				clearmap_start_time = ros::Time::now();
			}
			break;
		
	}
	//ROS_INFO("MovNode Switch End");

}
 
void MovNode::dynaReconfig(int pm, int iV, double dV, string sV)
{
	htbot::dyna dyna;
	dyna.paramid = pm;
	dyna.intValue = iV;
	dyna.doubleValue = dV;
	dyna.strValue = sV;
	dyna_pub.publish(dyna);
	//ROS_INFO("---------- dynaReconfig_int %d --------------",param);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move  node");  
  ros::NodeHandle rn;  		
	MovNode mNode;
	int readyflag;
	int cc,c1,c2,c3,ret;
	int startmove;
	bool mready,chkready;

	//ros::MultiThreadedSpinner spinner(4);
	ros::Rate loop_rate(10.0);
	//ROS_INFO("--- Move_Node : Testing reconfigure of global planner ---");
	//mNode.dynaReconfig(global_planner,1,0.0,"rac_planner/RacPlanner");
	//ROS_INFO("--- Move_Node : Testing Done reconfigure of global planner ---");
	//ROS_INFO("Move_Node : Before sleep 5 *********************** ");
	sleep(5);
	//ROS_INFO("Move_Node : After sleep 5 *********************** ");
	while (true) {
		mNode.publish_status("Checking if Robot is Ready..");
		rn.getParam("RobotReady",readyflag);
		//ROS_INFO("MovNode : RobotReady = %d",readyflag);
		if ((readyflag == 77) || (readyflag == 88)) {
			mNode.publish_status("Robot is Ready. Navigation Mode");
			break;
		}
		sleep(1);
	}
	mNode.readVoltDatafromFile();
	//ROS_INFO("------------- Move_Node : After Reading Voltdata .  Running *********************** ");
	cc = 0;
	c1 = 0;
	c2 = 0;
	c3 = 0;
	//ROS_INFO("-------------- MovNode : Testintg Lift command -------------");
	//mNode.publish_lift(AGVINLIFT,1,0,1,0);
	chkready = false;
	mready = false;
  while (true) {  
		//ROS_INFO("MovNode loop 1");	  	 	
		mNode.moveLPLoop();  
		//ROS_INFO("MovNode loop 2");	
		cc++;
		c1++;
		c3++;
		if (c1 == 2) {
			//if (mNode.racplanMove) {
			//	mNode.checkLocInPath();
			//}
			c1 = 0;
		}
		if (cc == 10) {
			//mNode.publish_queue();
			mNode.publish_status(mNode.sstring);
			cc = 0;
		}
		if (!chkready) {
			rn.getParam("MOVEREADY",mready);
			if (mready) {
				c2++;
				if (c2 > 20) {
					chkready = true;
					mready = false;
					//ret = system("firefox -new-window -reload http://127.0.0.1/index.html &");
				}
			}				
		} else {
			c3++;
			if (c3 > 50) {
				c3 = 0; // 
				//mNode.publish_ntuc_status(4,1,mNode.otargetLP,(int)(mNode.avgenergy),mNode.ntucjobstatus);
			}
		}
		
		//ROS_INFO("MovNode loop 3");	
		ros::spinOnce();	
		//ROS_INFO("MovNode loop 4");	
  	loop_rate.sleep();
		//ROS_INFO("MovNode loop 5");	
  }

  return 0;
}


