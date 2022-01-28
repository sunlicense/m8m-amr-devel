/*
 * Node that listens to cmd_vel and cmd_pos msgs
 * and publishes the robot odometry
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "boost/algorithm/string.hpp"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "htbot/bot2wheel_driver.h"
#include "htbot/sound.h"
#include "htbot/status.h"
#include "htbot/clear.h"
#include "htbot/move_status.h"
#include "htbot/debug.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "htbot/move.h"
#include "htbot/odom.h"
#include "htbot/angle.h"
#include "htbot/goal.h"
#include "htbot/stat_speed.h"
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>

// #include "techx_mcu/mcu.h"
//#include "transporter/LaptopChargeStatus.h"
using namespace std;
//using namespace boost::algorithm;

#define ROBOTREADY_UNO 70
//#define ODROID
#define SSMC
//#define RAC
#define ZDIST 2.0
#define ZANGLE 190.0
#define PI 3.141593
#define PI2 6.28319
#define PI_2 1.5707965
#define MAXLP 50

class BotNode
{
public:
	BotNode();
	Bot2Wheel robot;
	double cur_x,cur_y,cur_theta;
	double linear_, angular_, joylinear_, joyangular_,cmdvel_offset;
	double plinear,pangular,pspeed,mstop_time;
	bool errorState, cmd_estopped, estopped, resetting, docking;
	//void mcuCallback(const techx_mcu::mcu::ConstPtr& msg);
	//void laptopBattCallback(const techx_bot::LaptopChargeStatus::ConstPtr& msg);
	//void estopCallback(const std_msgs::Bool::ConstPtr& msg);
	void velCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void posCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void joyCallback(const geometry_msgs::Twist::ConstPtr& msg);
	bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
	void publish_odom();
	void restartRobot();
	void watchdog();
	void mainLoop();
	void publish_sound(int id,int startdelay,int restartdelay);
	void publish_debug(string s);
	void publish_clear(void);
	void publish_move_status(int stat);
	void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
	void moveCallback(const htbot::move::ConstPtr& msg);
	void goalCallback(const htbot::goal::ConstPtr& msg);
	void calAlignAngle();
	void publish_podom(double dist, double angle);
	void buttonCallback(const std_msgs::UInt16::ConstPtr& msg);
	void publish_status(string s);
	double calcAn(void);
	void publish_speed(double lin, double ang);
	bool findPathtoEscape();
	bool checkfront();
	void currCallback(const std_msgs::Float32::ConstPtr& msg);
	void cancel_goal(void);
	void ptpmoveT(double gx,double gy);
	//void ptpmoveS(double gx,double gy);
	void ptpmoveS(void);
	void ptpmoveN(void);
	double ptpAngle(void);
	void newMoveAngle();
	void newMoveDist();
	void getNextPt(void);
	void getNextPtN(int idx);
	void initMove(void);
	void newMoveN();
	void alignRobot();
	bool checkPlan(double x,double y, double z, double rx, double ry, double rz, double rw, double tol);
	void findNewPathForward(void);
	void ptpAlignMove();
	bool testCheckPlan();
	void modify_cmdvel();
	bool playsound,alignDistReached,alignAngleReached;
	
	ros::Time last_cmd_time, last_joycmd_time;
	ros::Time honk_time;
	ros::Time clearMap_time,escapeTime, escapeBuzzTime,escapeNextTime,playTime,waitTime;
	double rate;
	ros::Publisher play_pub;
	ros::Publisher clear_pub;
	ros::Publisher status_mov;
	ros::Subscriber pose_sub;
	ros::Subscriber move_cmd;
	ros::Subscriber goal_cmd;
	ros::Publisher status_pub;
	ros::Publisher chkplan_pub;
	void publish_toggleButton(unsigned short cmd);
	int navstatus;
	int hstate,pstate,mcstate;
	bool stop,estop_pressed,estop_pressed_flag;
	int count,pcount,ocount;
	bool startPMove,escape,escapeNext,escapeTurn,escapePos;;
	double fspeedtime;
	double rotate_angle,move_distance;
	double gx,gy,grz,grw,px,py,pz,prx,pry,prz,prw,targetDist,slowDist,slowRatio,alignAngle,alignDist,frontObsslowRatio;
	double alignX,alignY,alignDX,alignDY;
	double alignAX,alignAY,alignT,alignTD;
	double dockdist,dockspeed,docksspeed,dockrdist,frontobsdist,dockrangle;
	bool checkDist;
	double oddist, odangle;  // odom
	//int profile_move_flag;
	bool estop,reach,reachTurn,checkfrontflag,findpathflag,waitflag,findnewpath;
	int escape_cnt;
	double oprz,oprw,sprz,sprw,syaw,obsdist,obsddist;
	double obdist[150];
	int rcount;
	double inc_angle;
	bool firstpt,secondpt,nearflag;
	int firstcnt,secondcnt;
	double yawf,yaws,dyaw;
	double yawp,yawg,yawStart,yawAlign;
	double pidAngle,pidAngleS,pidDist,maxAngleVel,maxDistVel,dAngleLimit,dDistLimit,switchDist;
	double rePlanTime,constVel,redVel,termVel;
	double stopDist;
	bool stopAngleMove,stopDistMove,stopMove,stopAlignRobot;
	double opx,opy,distStart,angleToGoal;
	double leftlaserobsdist,rightlaserobsdist,mapobsdist;
	double current;
	bool nearLP,navDone;
	double cancelDist;
	bool ptpFlag;
	double ptpDist;
	double nposeInfo[MAXLP][19];	
	int cmovePtr,emovePtr;
	bool multiptMove,rotateFirst,stopFlag,checkPlanDone;
	int pointInfo;
	double checkPlanSum;
	ros::Time stopTime;
	ros::ServiceClient makeplan;

private:
	ros::NodeHandle nh,ph;
	double cov_x, cov_y, cov_th;
	double cov_vx, cov_vy, cov_vth;
	int laptop_min_charge;

	geometry_msgs::Pose2D prevPose;
	nav_msgs::Odometry odom;
	//ros::Subscriber mcu_sub, laptop_sub, estop_sub, vel_sub, pos_sub, joyvel_sub;
	ros::Subscriber vel_sub, pos_sub, joyvel_sub;
	ros::Publisher odom_pub;
	ros::Publisher podom_pub;		
	ros::Publisher queue_pub;
	ros::Publisher debug_pub;
	ros::Publisher toggleButton_pub;
	ros::Subscriber button_sub;
	ros::Publisher velstat_pub;
	ros::Subscriber curr_sub;
	ros::Publisher cancel_pub;

	ros::ServiceServer reset_odom_srv;
	ros::Time current_time, last_time, delay_time,mc_time;
	boost::mutex publish_mutex_;
};

BotNode::BotNode():
	cur_x(0.0),cur_y(0.0),cur_theta(0.0),cmdvel_offset(1.0),startPMove(false),docking(false),estop_pressed_flag(false),
	linear_(0.0),angular_(0.0), joylinear_(0.0),joyangular_(0.0),navstatus(0),stop(false),estop_pressed(false),
	errorState(false), cmd_estopped(false), estopped(false), resetting(false),hstate(0),pstate(0),reach(false),
	alignAngle(0.0),checkDist(false),escape(false),escapeNext(false),escapeTurn(false),ocount(0), ph("~"),
	sprz(9.0),sprw(9.0),checkfrontflag(false),findpathflag(false),playsound(false),cancelDist(0.0),ptpFlag(false),
	leftlaserobsdist(2.5),rightlaserobsdist(2.5),mapobsdist(2.5),nearLP(false),navDone(false),findnewpath(false),
	mcstate(0)
{
	//mcu_sub = nh.subscribe<techx_mcu::mcu>("mcu", 1, &BotNode::mcuCallback,this);
	//laptop_sub = nh.subscribe<techx_bot::LaptopChargeStatus>("laptop_charge", 1, &BotNode::laptopBattCallback,this);
	//estop_sub = nh.subscribe<std_msgs::Bool>("cmd_estop", 1, &BotNode::estopCallback,this);
	vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 100, &BotNode::velCallback,this);
	pos_sub = nh.subscribe<geometry_msgs::Twist>("cmd_pos", 100, &BotNode::posCallback,this);
	joyvel_sub = nh.subscribe<geometry_msgs::Twist>("joycmd_vel", 100, &BotNode::joyCallback,this);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
	podom_pub = nh.advertise<htbot::odom>("podom",100);
	reset_odom_srv = nh.advertiseService("reset_odom", &BotNode::resetOdom, this);
	play_pub = nh.advertise<htbot::sound>("sound", 1);
	debug_pub = nh.advertise<htbot::debug>("debug",100);
	clear_pub = nh.advertise<htbot::clear>("clearMap", 100);
	toggleButton_pub = nh.advertise<std_msgs::UInt16>("toggle_button",100);
	status_mov = nh.advertise<htbot::move_status>("move_status",100);
	pose_sub = nh.subscribe<geometry_msgs::Pose>("/robot_pose", 1, &BotNode::poseCallback,this);
	goal_cmd = nh.subscribe<htbot::goal>("goal", 1, &BotNode::goalCallback,this); 
	chkplan_pub = nh.advertise<htbot::goal>("checkplan", 100);
	button_sub = nh.subscribe<std_msgs::UInt16>("button", 100, &BotNode::buttonCallback,this);
	status_pub = nh.advertise<htbot::status>("feedback",100);
	velstat_pub = nh.advertise<htbot::stat_speed>("speed", 100);
	curr_sub = nh.subscribe<std_msgs::Float32>("current", 100, &BotNode::currCallback,this);
	cancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
	move_cmd = nh.subscribe<htbot::move>("move", 1, &BotNode::moveCallback,this); 
	makeplan = nh.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
	
	nh.param("laptop_min_charge",laptop_min_charge, 10);
	nh.param("odom_pub_rate", rate, 10.0);
	nh.param("estopped", estopped, false);
	nh.param("mstop_time", mstop_time, 4.0);
	nh.param("RotateAngle",rotate_angle,90.0);
	nh.param("MoveDistance",move_distance,2.5);
	nh.param("targetDist",targetDist,0.05);
	nh.param("slowDist",slowDist,0.2);
	nh.param("slowRatio",slowRatio,0.5);
	nh.param("Docking_Distance",dockdist,0.3);
	nh.param("Docking_Speed",dockspeed,0.05);
	nh.param("Docking_SlowSpeed",docksspeed,0.03);
	nh.param("Docking_Retract_Distance",dockrdist,0.5); 
	nh.param("Docking_Retract_Angle",dockrangle,180.0);
	nh.param("Front_Obs_Distance",frontobsdist,1.0);
	nh.param("FrontObsslowRatio",frontObsslowRatio,0.75);

	ph.param("cov_x",cov_x, 1e-3);
	ph.param("cov_y",cov_y, 1e-3);
	ph.param("cov_th",cov_th, 1e-3);
	ph.param("cov_vx",cov_vx, 1e-3);
	ph.param("cov_vy",cov_vy, 1e-3);
	ph.param("cov_vth",cov_vth, 1e-3);

	ph.param<std::string>("LEFT_MOTOR",robot.LEFT_MOTOR, "/dev/ttyUSB0");
	ph.param<std::string>("RIGHT_MOTOR",robot.RIGHT_MOTOR, "/dev/ttyUSB1");

	ph.param("PBAUDL",robot.pbaudL,4);
	ph.param("EBAUDL",robot.ebaudL,4);
	ph.param("CHANGEBAUDRATEL",robot.cbaudrateL,0);
	ph.param("PBAUDR",robot.pbaudR,4);
	ph.param("EBAUDR",robot.ebaudR,4);
	ph.param("CHANGEBAUDRATER",robot.cbaudrateR,0);				

	ph.param("REFLECT",robot.REFLECT,1);
	ph.param("AXLE_LEN",robot.AXLE_LEN,0);
	ph.param("WHEEL_DIA",robot.WHEEL_DIA,0);
	ph.param("GEAR_RATIO",robot.GEAR_RATIO,0);
	ph.param("STEPS_PER_REV",robot.STEPS_PER_REV,0);
	ph.param("wheel_circum_correction",robot.wheel_circum_correction,1.);
	ph.param("wheel_base_correction",robot.wheel_base_correction,1.);
	ph.param("odom_angular_scale_correction",robot.odom_angular_scale_correction,1.);

	ph.param("CurrentRegulatorPGain",robot.CurrentRegulatorPGain,0);
	ph.param("CurrentRegulatorIGain",robot.CurrentRegulatorIGain,0);
	ph.param("VelocityPGain",robot.VelocityPGain,0);
	ph.param("VelocityIGain",robot.VelocityIGain,0);
	ph.param("PositionPGain",robot.PositionPGain,0);
	ph.param("PositionIGain",robot.PositionIGain,0);
	ph.param("PositionDGain",robot.PositionDGain,0);

	ph.param("PositionProfileAcceleration",robot.ProfileAcceleration,0);
	ph.param("PositionProfileDeceleration",robot.ProfileDeceleration,0);
	ph.param("PositionProfileVelocity",robot.ProfileVelocity,0);
	ph.param("MaxProfileVelocity",robot.MaxProfileVelocity,0);
	ph.param("MaxProfileVelocityMD",robot.MaxProfileVelocityMD,0);
	ph.param("MaxFollowError",robot.MaxFollowError,0);
	ph.param("PositionWindow",robot.PositionWindow,0);
	ph.param("PositionWindowTime",robot.PositionWindowTime,0);

	ph.param("MotorMaxContinuousCurrent",robot.MotorMaxContinuousCurrent,0);
	ph.param("ThermalTimeConstantWinding",robot.ThermalTimeConstantWinding,0);

	nh.param("pidAngle",pidAngle,0.35);
	nh.param("pidAngleS",pidAngleS,0.8);
	nh.param("pidDist",pidDist,0.6);
	nh.param("maxAngleVel",maxAngleVel,0.35);
	nh.param("maxDistVel",maxDistVel,0.6);
	nh.param("dAngleLimit",dAngleLimit,0.02);
	nh.param("dDistLimit",dDistLimit,0.07);
	nh.param("switchDist",switchDist,0.15);
	nh.param("stopDist",stopDist,0.3);
	nh.param("rePlanTime",rePlanTime,10.0);
	nh.param("constVel",constVel,0.6);
	nh.param("redVel",redVel,0.3);
	nh.param("termVel",termVel,0.1);

	current_time = last_time = last_cmd_time = ros::Time::now();
	//ROS_INFO("cov_vth : %.4f",cov_vth);
} // BotNode constructor

void BotNode::initMove(void) {
	
	nposeInfo[0][0] = 3.6;  // p1
	nposeInfo[0][1] = 3.75;
	nposeInfo[0][17] = 0; // 1=end pt
	nposeInfo[0][18] = 1; // 0=straight to next pt. 1=need to rotate to next pt.
	nposeInfo[1][0] = 6.05;  // p2
	nposeInfo[1][1] = 2.5;
	nposeInfo[1][17] = 0;
	nposeInfo[1][18] = 1; // 0=straight to next pt. 1=need to rotate to next pt.
	nposeInfo[2][0] = 7.95;  // p3
	nposeInfo[2][1] = 3.5;
	nposeInfo[2][17] = 0;
	nposeInfo[2][18] = 1; // 0=straight to next pt. 1=need to rotate to next pt.
	nposeInfo[3][0] = 10.5208;  // Stn2  
	nposeInfo[3][1] = 2.1414;
	nposeInfo[3][5] = 0.9993;
	nposeInfo[3][6] = 0.03618;
	nposeInfo[3][17] = 1;
	nposeInfo[3][18] = 0;
	nposeInfo[4][0] = 8.95;  // p1
	nposeInfo[4][1] = 3.75;
	nposeInfo[4][17] = 0;
	nposeInfo[4][18] = 0; // 0=straight to next pt. 1=need to rotate to next pt.
	nposeInfo[5][0] = 7.3781;  // StnL  
	nposeInfo[5][1] = 5.2766;
	nposeInfo[5][5] = -0.0280;
	nposeInfo[5][6] = 0.9996;
	nposeInfo[5][17] = 1;
	nposeInfo[5][18] = 0;
	cmovePtr = 0;
	emovePtr = 6;
	
}

bool BotNode::checkPlan(double x,double y, double z, double rx, double ry, double rz, double rw, double tol) {
	
	nav_msgs::GetPlan srv;
	bool ret;
	double xx,yy,zz,rxx,ryy,rzz,rww;
	double ox,oy,dd,pd,ddt,an;
	int len;
		
	ret = false;

	srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = px;
  srv.request.start.pose.position.y = py;
	srv.request.start.pose.position.z = pz;
	srv.request.start.pose.orientation.x = prx;
	srv.request.start.pose.orientation.y = pry;
	srv.request.start.pose.orientation.z = prz;
  srv.request.start.pose.orientation.w = prw;

	srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = x;
  srv.request.goal.pose.position.y = y;
	srv.request.goal.pose.position.z = z;
	srv.request.goal.pose.orientation.x = rx;
	srv.request.goal.pose.orientation.y = ry;
	srv.request.goal.pose.orientation.z = rz;
	srv.request.goal.pose.orientation.w = rw;
	srv.request.tolerance = tol;
	dd = 0.0;
	ddt = 0.0;
	//ROS_INFO("------ CPlan px=%.3f. py=%.3f. pz=%.3f. prx=%.3f. pry=%.3f. prz=%.3f. prw=%.3f. -------",px,py,pz,prx,pry,prz,prw);
	//ROS_INFO("------ CPlan x=%.3f. y=%.3f. z=%.3f. rx=%.3f. ry=%.3f. rz=%.3f. rw=%.3f. tol=%.3f -----",x,y,z,rx,ry,rz,rw,tol);
	if (makeplan.call(srv)) {
		if (srv.response.plan.poses.empty()) {
			//publish_debug("Move_Node : Got empty plan" );
  	 	ROS_INFO("------ CheckPlan : Empty Plan ------");
			ret = false;
    } else {
			//publish_debug("Move_Node : Found Plan" );
			//len = sizeof(srv.response.plan.poses);
			len = srv.response.plan.poses.size();
			for (int j=0;j<len;j++) {
				xx = srv.response.plan.poses[j].pose.position.x;			
				yy = srv.response.plan.poses[j].pose.position.y;
				rzz = srv.response.plan.poses[j].pose.orientation.z;		
				rww = srv.response.plan.poses[j].pose.orientation.w;
				//an = ptpAngleN(px,py,xx,yy);
				pd = sqrt(((xx-px) * (xx-px)) + ((yy-py) * (yy-py)));
				if (j>0) {
					dd = sqrt(((xx-ox) * (xx-ox)) + ((yy-oy) * (yy-oy)));					
					ddt = ddt + dd;
				}
				ROS_INFO("---- CPlan %d : x=%.3f. y=%.3f. rz=%.3f. rw=%.3f. dd=%.3f. pd=%.3f. ddt=%.3f  -----",j+1,xx,yy,rzz,rww,dd,pd,ddt);
				ox = xx;
				oy = yy;
			}
			ROS_INFO("----- CPlan Total dist=%.3f ------",ddt);
			ret = true;
		}
 	} else {
		//publish_debug("Move_Node : Failed to call make plan service");
  	ROS_INFO("------ CheckPlan : Failed to call make plan service ----------");
  }
	return ret;
}

/*
bool BotNode::checkPlan(double x,double y, double z, double rx, double ry, double rz, double rw, double tol) {
	
	nav_msgs::GetPlan srv;
	bool ret;
		
	ret = false;

	srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = px;
  srv.request.start.pose.position.y = py;
	//srv.request.start.pose.position.z = pz;
	//srv.request.start.pose.orientation.x = prx;
	//srv.request.start.pose.orientation.y = pry;
	//srv.request.start.pose.orientation.z = prz;
  //srv.request.start.pose.orientation.w = prw;
	srv.request.start.pose.orientation.w = 1.0;

	srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = x;
  srv.request.goal.pose.position.y = y;
	//srv.request.goal.pose.position.z = z;
	//srv.request.goal.pose.orientation.x = rx;
	//srv.request.goal.pose.orientation.y = ry;
	//srv.request.goal.pose.orientation.z = rz;
	//srv.request.goal.pose.orientation.w = rw;
	srv.request.goal.pose.orientation.w = 1.0;
	srv.request.tolerance = tol;
	ROS_INFO(" --------- checkPlan A -----------");
	if (makeplan.call(srv)) {
		if (srv.response.plan.poses.empty()) {
			//publish_debug("Move_Node : Got empty plan" );
  	 	ROS_INFO("BtNode_Node : Got empty plan");
			ret = false;
    } else {
			ROS_INFO("BtNode_Node : Found Plan" );
			ret = true;
		}
 	} else {
		//publish_debug("Move_Node : Failed to call make plan service");
  	ROS_INFO("BtNode_Node : Failed to call make plan service");
  }
	return ret;
}
*/
bool BotNode::testCheckPlan() {
	
	nav_msgs::GetPlan srv;
	bool ret;
		
	ret = false;
	ROS_INFO(" --------- testcheckPlan A -----------");
	srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = 1.7284;
  srv.request.start.pose.position.y = 1.9136;
	//srv.request.start.pose.position.z = pz;
	//srv.request.start.pose.orientation.x = prx;
	//srv.request.start.pose.orientation.y = pry;
	//srv.request.start.pose.orientation.z = 0.7115;
  //srv.request.start.pose.orientation.w = 0.7027;
	//srv.request.start.pose.orientation.w = 1.0;
	  
	srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = 4.5327;
  srv.request.goal.pose.position.y = 4.2259;
	//srv.request.goal.pose.position.z = z;
	//srv.request.goal.pose.orientation.x = rx;
	//srv.request.goal.pose.orientation.y = ry;
	//srv.request.goal.pose.orientation.z = -0.0106;
	//srv.request.goal.pose.orientation.w = 0.9999;
	//srv.request.goal.pose.orientation.w = 1.0;
	//srv.request.tolerance = 1.5;
	ROS_INFO(" --------- testcheckPlan B -----------");
	if (makeplan.call(srv)) {
		if (srv.response.plan.poses.empty()) {
			//publish_debug("Move_Node : Got empty plan" );
  	 	ROS_INFO("BtNode_Node : Got empty plan");
			ret = false;
    } else {
			ROS_INFO("BtNode_Node : Found Plan" );
			ret = true;
		}
 	} else {
		//publish_debug("Move_Node : Failed to call make plan service");
  	ROS_INFO("BtNode_Node : Failed to call make plan service");
  }
	return ret;
}

void BotNode::moveCallback(const htbot::move::ConstPtr& msg)
{

	if (msg->opt == 0) {
		gx = msg->x;
		gy = msg->y;
		grz = msg->rz;
		grw = msg->rw;
	}
}

double BotNode::ptpAngle(void)
{
	double an;
	if ( (gx >= px) && (gy >= py)) {
		// Q1
		an = atan((gy-py)/(gx-px));
	} else {
		if ( (px >= gx) && (gy >= py) ) {
			// Q2
			an = atan((gy-py)/(px-gx));
			an = PI - an;
		} else {
			if ( (px >= gx) && (py >= gy) ) {
				// Q3
				an = atan((py-gy)/(px-gx));
				an = -(PI - an);
			} else {
				// Q4
				an = -(atan((py-gy)/(gx-px)));
			}
		}
	}
	return an;
}

//void BotNode::ptpmoveS(double ggx,double ggy)
void BotNode::ptpmoveS()
{
	double av,lv,dd,an,diff,angle,idist;
	
	//tf::Quaternion qg(0.0,0.0,grz,grw);
	//yawg = tf::getYaw(qg);
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	tf::Quaternion qg(0.0,0.0,grz,grw);
	yawg = tf::getYaw(qg);
	//gx = ggx;
	//gy = ggy;
  // calculate angle to goal(gx,gy) from robot position(px,py)
	an = ptpAngle();
	diff = angles::shortest_angular_distance(yawp, an);
	//angle = 57.2958 * diff;
	dd = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
	
	if (dd < 0.07) {
		linear_ = 0.0;
		angular_ = 0.0;
		robot.moveVelocity(linear_,angular_);
		ptpFlag = false;		
		diff = angles::shortest_angular_distance(yawp, yawg);
		angle = 57.2958 * diff;
		robot.moveAngle(angle);	
		//robot.setVelMode(); 
		estopped = false;
		return;
	}
	//lv = 0.6 * (dd/ptpDist);
	lv = 0.6 * (dd/1.0);
	av = 0.35 * (diff);
	if (lv > 0.6) {
		lv = 0.6;
	}
	if (av > 0.35) {
		av = 0.35;
	}
	linear_ = lv;
	angular_ = av;
	//ROS_INFO("ptpmoveS : diff = %.3f. Dist = %.2f.lv=%.2f. av=%.2f",diff,dd,linear_,angular_);
	robot.moveVelocity(linear_,angular_);
	
}

void BotNode::ptpmoveN()
{
	double av,lv,dd,an,diff,angle,idist;
	
	//tf::Quaternion qg(0.0,0.0,grz,grw);
	//yawg = tf::getYaw(qg);
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	tf::Quaternion qg(0.0,0.0,grz,grw);
	yawg = tf::getYaw(qg);
	//gx = ggx;
	//gy = ggy;
  // calculate angle to goal(gx,gy) from robot position(px,py)
	an = ptpAngle();
	diff = angles::shortest_angular_distance(yawp, an);
	//angle = 57.2958 * diff;
	dd = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
	
	if (dd < 0.07) {
		linear_ = 0.0;
		angular_ = 0.0;
		//robot.moveVelocity(linear_,angular_);
		//ptpFlag = false;		
		//diff = angles::shortest_angular_distance(yawp, yawg);
		//angle = 57.2958 * diff;
		//robot.moveAngle(angle);	
		//robot.setVelMode(); 
		//estopped = false;
		return;
	}
	//lv = 0.6 * (dd/ptpDist);
	lv = 0.4 * (dd/0.5);
	av = 0.25 * (diff);
	if (lv > 0.4) {
		lv = 0.4;
	}
	if (av > 0.25) {
		av = 0.25;
	}
	linear_ = lv;
	angular_ = av;
	//ROS_INFO("ptpmoveS : diff = %.3f. Dist = %.2f.lv=%.2f. av=%.2f",diff,dd,linear_,angular_);
	//robot.moveVelocity(linear_,angular_);
	
}

void BotNode::ptpmoveT(double ggx,double ggy)
{
	double av,lv,dd,an,diff,angle,idist;
	
	//tf::Quaternion qg(0.0,0.0,grz,grw);
	//yawg = tf::getYaw(qg);
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	gx = ggx;
	gy = ggy;
  // calculate angle to goal(gx,gy) from robot position(px,py)
	if ( (gx >= px) && (gy >= py)) {
		// Q1
		an = atan((gy-py)/(gx-px));
	} else {
		if ( (px >= gx) && (gy >= py) ) {
			// Q2
			an = atan((gy-py)/(px-gx));
			an = PI - an;
		} else {
			if ( (px >= gx) && (py >= gy) ) {
				// Q3
				an = atan((py-gy)/(px-gx));
				an = -(PI - an);
			} else {
				// Q4
				an = -(atan((py-gy)/(gx-px)));
			}
		}
	}
	diff = angles::shortest_angular_distance(yawp, an);
	angle = 57.2958 * diff;
	ptpDist = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
	dd = idist;
	ROS_INFO("ptpmove : ptpAngle = %.3f. Robot Angle = %.3f. diff = %.3f. ptpDist = %.2f",an,yawp,diff,ptpDist);
	robot.moveAngle(angle);	
	//robot.setVelMode(); 
	estopped = true;
	ptpFlag = true;
}

void BotNode::cancel_goal(void)
{
	//MoveBaseClient ac("move_base", true); // move_base
	//while(!ac.waitForServer(ros::Duration(5.0))){
  //	ROS_INFO("Waiting for the move_base action server to come up : web_node");
  //}
	//ac.cancelGoal();
	ros::NodeHandle nn;
	actionlib_msgs::GoalID msg;
	msg.stamp = ros::Time::now();
	msg.id = "CancelGoal";
	cancel_pub.publish(msg);
	nn.setParam("navstatus",0);
	return;
}

void BotNode::currCallback(const std_msgs::Float32::ConstPtr& msg)
{

	{ 
		boost::mutex::scoped_lock lock(publish_mutex_);
		current = msg->data;		
	}
	
	return;
}

void BotNode::publish_speed(double lin, double ang)
{
	htbot::stat_speed velst;
	velst.linear = lin;
	velst.angular = ang;
	velstat_pub.publish(velst);
	return;
}

bool BotNode::checkfront()
{
	double tdl,tdt,inc;
	double adiff,an;
	int tdi;
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	return true;
	if (!checkfrontflag) {
		adiff = (angles::shortest_angular_distance(yaws, yawf));
		//adiff = adiff / 2.0;
		if (adiff > 0.0) {
			escapePos = true;
		} else {
			escapePos = false;
			adiff = -adiff;
		}
		//dyaw = adiff;
		dyaw = dyaw * 0.95;
		//ROS_INFO("Check Front : dyaw=%.3f.",dyaw);
		checkfrontflag = true;
	}
	if (escapePos) {
		angular_ = 0.05; // 0.13
	} else {
		angular_ = -0.05; // -0.13
	}
	an = fabs(angles::shortest_angular_distance(yawp, yawf));
	//ROS_INFO("chkfront: angular=%.3f. an=%.3f. yawp=%.3f. yawf=%.3f.dyaw=%.3f",angular_,an,yawp,yawf,dyaw);
	if (an < dyaw) {
		angular_ = 0.0;
		return true;
	}
	return false;
}

/*
bool BotNode::checkfront_org()
{
	double tdl,tdt,yawp,inc;
	double adiff;
	int tdi;
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	
	if (!checkfrontflag) {
		tdl = 0.0;
		tdi = 0;
		for (int i=0;i<15;i++) {
			tdt = obdist[i];
			if (tdt > tdl) {
				tdl = tdt;
				tdi = i;
			}
		}
		for (int i=45;i<60;i++) {
			tdt = obdist[i];
			if (tdt > tdl) {
				tdl = tdt;
				tdi = i;
			}
		}		
		if (tdi < 15) {
			escapePos = true;
			inc = (tdi+1) * 0.095;
		} else {
			escapePos = false;			
			tdi = 59 - tdi;
			inc = (tdi+1) * 0.095;
		}
		ROS_INFO("Check Front : tdi=%d. obsdist=%.3f. inc=%.3f.",tdi,tdl,inc);
		checkfrontflag = true;
	}
	if (escapePos) {
		angular_ = 0.25;
	} else {
		angular_ = -0.25;
	}
	adiff = fabs(angles::shortest_angular_distance(yawp, syaw));
	if ((adiff > 0.01) && (adiff >= inc)){
		angular_ = 0.0;
		return true;
	}
	return false;
}
*/

bool BotNode::findPathtoEscape()
{
	double adiff,bdiff;
	ros::NodeHandle nn;
	int moreopen,mapdirection;
	double hLDist,vLDist, hRDist,vRDist, vTDist;

	nn.getParam("ProfileMoveObsDist",obsdist);
	nn.getParam("MapObsDist",mapobsdist);
	nn.getParam("LeftLaserObsDist",leftlaserobsdist);
	nn.getParam("RightLaserObsDist",rightlaserobsdist);
	ROS_INFO("******* ObsDist:%.3f. LeftLaserObsDist:%.3f. RightLaserObsDist:%.3f. MapObsDist:%.3f. ************",obsdist,leftlaserobsdist,rightlaserobsdist,mapobsdist);
	
	//ROS_INFO("Start find Path : Angular=%.3f. obsdist=%.3f. obsddist=%.3f",angular_,obsdist,obsddist);
	//if (obsdist <= 0.02) {
	//	 	angular_ = 0.0;
	//		ROS_INFO(" @@@@@@@@@@@@@@@@  obsdist < 0.03  @@@@@@@@@@@@@@@@@@@@@@@@");
	//		return true;
	//}
	
	if (sprz == 9.0) {
		nn.getParam("MapDirectionToSearch",mapdirection);
		nn.getParam("DirectionToSearch",moreopen);		
		ROS_INFO("************** mapdirection:%d. moreopen:%d. ****************",mapdirection,moreopen);
		sprz = prz;
		sprw = prw;
		oprz = sprz;
		oprw = sprw;
		tf::Quaternion sp(0.0,0.0,sprz,sprw);
		syaw = tf::getYaw(sp);
		rcount = 0;
		inc_angle = 3.1; //1.57; // 90 deg
		// combine the sensors input
		if (leftlaserobsdist < 0.3) {
			moreopen--;
		}
		if (rightlaserobsdist < 0.3) {
			moreopen++;
		}
		moreopen = moreopen + mapdirection;
		escapePos = false;
		if (moreopen < 0) {
			angular_ = -0.3;  // rotate right
			escapePos = false;
		} else {
			if (moreopen > 0) {
				angular_ = 0.3;  // rotate left
				escapePos = true;
			}
		}
		//ROS_INFO("find After Moreopen. angular=%.3f",angular_);
		/*
		if (mapdirection < 0) {
			angular_ = -0.3;  // rotate right
			escapePos = false;
		} else {
			if (mapdirection > 0) {
				angular_ = 0.3;  // rotate left
				escapePos = true;
			}
		}
		
		//ROS_INFO("find After Map. angular=%.3f",angular_);
		//ROS_INFO("findPathtoEscape : LLD=%.3f. RLD=%.3f",leftlaserobsdist,rightlaserobsdist);
		if (leftlaserobsdist < rightlaserobsdist) {
			if (leftlaserobsdist < 0.3) {
				angular_ = -0.3;
				escapePos = false;
			}
		} else {
			if (leftlaserobsdist > rightlaserobsdist) {
				if (rightlaserobsdist < 0.3) {
					angular_ = 0.3;
			  	escapePos = true;
				}
			}
		}
		*/
		//ROS_INFO("find After Lasser : LLD=%.3f. RLD=%.3f. mopen=%d. mapopen=%d.an=%.3f",leftlaserobsdist,rightlaserobsdist,moreopen,mapdirection,angular_);
		/*
		if (obsddist > 0.0) {
			angular_ = 0.1;
			escapePos = true;
		} else {
			angular_ = -0.1;
			escapePos = false;
		}
		*/
		yawf = syaw;
		firstpt = false;
		secondpt = false;
		//ROS_INFO("Start findpath : syaw=%.3f. angular=%.3f. ",syaw,angular_);
	}
	if (escapePos) {
		angular_ = 0.3;
		//vTDist = vLDist;
	} else {
		angular_ = -0.3;
		//vTDist = vRDist;
	}
	//ROS_INFO("prz=%.3f. prw=%.3f. oprz=%.3f. oprw=%.3f",prz,prw,oprz,oprw);
	tf::Quaternion qp(0.0,0.0,prz,prw);
	//tf::Quaternion qo(0.0,0.0,oprz,oprw);
	yawp = tf::getYaw(qp);
	
	//if (yawp < 0.0) {
	//	yawp = 6.2832 - yawp;
	//}
	//yawo = tf::getYaw(qo);
	//if (yaw0 < 0.0) {
	//	yawo = 6.2832 - yawo;
	//}
	//ROS_INFO("prz=%.3f. prw=%.3f. oprz=%.3f. oprw=%.3f. yawp=%.3f. yawo=%.3f",prz,prw,oprz,oprw,yawp,yawo);
	
	adiff = fabs(angles::shortest_angular_distance(yawp, syaw));
	//ROS_INFO("adiff=%.3f. inc_angle=%.3f",adiff,inc_angle);
	if (adiff > 0.01) {
		if (adiff <= inc_angle) {
			//obdist[rcount] = obsdist;
			//ROS_INFO("findPath: obsdist=%.3f. mapobsdist=%.3f. adiff=%.3f, an=%.3f",obsdist,mapobsdist,adiff,angular_);
			//rcount++;
			//if (obsdist > 0.35) {
			if ((obsdist > 0.45) && (mapobsdist > 0.45) && (rightlaserobsdist > 0.6) && (leftlaserobsdist > 0.6)) {
				// stop;
				//angular_ = 0.0;
				//sprz = 9.0;
				//sprw = 9.0;
				if (!firstpt) {
					firstpt = true;
					//firstcnt = rcount;
					yawf = yawp;
					//ROS_INFO("findPath First. yawf=%.3f.",yawf);
				}		
				if (firstpt && !secondpt) {
					bdiff = fabs(angles::shortest_angular_distance(yawp, yawf));
					//if ((obsdist < 0.3) || (bdiff > 0.25)) {  // 0.27rad = 15 deg
					if ((bdiff > 0.25)) {  // 0.27rad = 15 deg
						secondpt = true;
						//secondcnt = rcount;
						yaws = yawp;
						dyaw = bdiff;
						checkfrontflag = false;
						angular_ = 0.0;
						//ROS_INFO("findPath A complete. yaws=%.3f. angular=%.3f. bdiff=%.3f",yaws,angular_,bdiff);														
						sprz = 9.0;	
						return true;
					}
				}				
			} else {
				if (firstpt) {
					firstpt = false;
					secondpt = false;
				}
			}
			//oprz = prz;
			//oprw = prw;
		} else {
			yaws = yawp;
			angular_ = 0.0;
			sprz = 9.0;
			//ROS_INFO("findPath Hit Limit. yaws=%.3f. angular=%.3f",yaws,angular_);			
			return true;
		}
	}
	//oprz = prz;
	//oprw = prw;
	//ROS_INFO("Exit find Path : Angular=%.3f",angular_);
	return false;
}



void BotNode::publish_status(string s)
{
	htbot::status status;
	status.msg = s;
	status_pub.publish(status);
	return;
}


void BotNode::goalCallback(const htbot::goal::ConstPtr& msg)
{
	ros::NodeHandle nm;
	int cmd;
	double an;
	
	cmd = msg->cmd;

	if (cmd == 0) {
		tf::Quaternion qp(0.0,0.0,prz,prw);
		yawp = tf::getYaw(qp);
		yawStart = yawp + (msg->pa * 0.0174533);  // rotate right -> -ve
		pstate = 30;
		stopAngleMove = false;
		return;
	}
	if (cmd == 1) {
		opx = px;
		opy = py;
		distStart = msg->pd;
		pstate = 31;
		stopDistMove = false;
		return;
	}
	if (cmd == 2) {
		gx = msg->x;
		gy = msg->y;
		grz = msg->rz;
		grw = msg->rw;
		tf::Quaternion qp(0.0,0.0,prz,prw);
		yawp = tf::getYaw(qp);
		tf::Quaternion qg(0.0,0.0,grz,grw);
		yawg = tf::getYaw(qg);
		yawStart = ptpAngle();
		distStart = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
		an = angles::shortest_angular_distance(yawp, yawStart);
		ROS_INFO(" Align : dist=%.3f. yawp=%.3f. start=%.3f",distStart,yawp,yawStart);
		if (fabs(an) > PI_2) {
			yawStart = yawStart + PI;
			if (yawStart > PI) {
				yawStart = yawStart - PI2;
			} 
			distStart = - distStart;
		}		
		
		opx = px;
		opy = py;
		stopDistMove = false;
		stopAngleMove = false;
		ROS_INFO(" Align2 : dist=%.3f. yawp=%.3f. start=%.3f",distStart,yawp,yawStart);
		pstate = 32;
		return;
	}
	if (cmd == 3) {
		cmovePtr = msg->startidx;
		emovePtr = msg->lastidx;
		getNextPt();
		stopMove = false;
		pstate = 33;
		return;
	}
	if (cmd == 4) {
		gx = msg->x;
		gy = msg->y;
		grz = msg->rz;
		grw = msg->rw;
		tf::Quaternion qg(0.0,0.0,grz,grw);
		yawg = tf::getYaw(qg);
		ptpDist = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
		stopAlignRobot = false;
		alignDistReached = false;
		alignAngleReached = false;
		pstate = 34;
		return;
	}
	
}

void BotNode::getNextPt(void) {
	
	if (cmovePtr <= emovePtr) {
		getNextPtN(cmovePtr);
		cmovePtr++;		
		//ROS_INFO(" ************  gx=%.3f. gy=%.3f. ptpDist=%.3f. AngleToGoal=%.3f  *************",gx,gy,ptpDist,angleToGoal);
	}
}

void BotNode::getNextPtN(int idx) {
	int endflag;
	double an;
	gx = nposeInfo[idx][0];
	gy = nposeInfo[idx][1];
	
	pointInfo = nposeInfo[idx][18];
	endflag = nposeInfo[idx][17];
	if ((pointInfo == 1) || (endflag == 1)) { // need to rotate to next point
		ptpDist = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));		
	} else {
		ptpDist = 100.0; // no slow down
	} 
	ROS_INFO(" ------------------------------------------------------------");
	an = ptpAngle();
	angleToGoal = angles::shortest_angular_distance(yawp, an);
	if (fabs(angleToGoal) > 0.1) {
		rotateFirst = true;
		ROS_INFO(" Rotate First ");
	}else {
		rotateFirst = false;
	}

	if (endflag == 1) { // end point
		ROS_INFO("  Final LP ");
		grz = nposeInfo[idx][5];
		grw = nposeInfo[idx][6];
		tf::Quaternion qg(0.0,0.0,grz,grw);
		yawg = tf::getYaw(qg);
		multiptMove = false;		
	} else {
		multiptMove = true;
		ROS_INFO(" Intermediate LP ");
	}
	//ptpDist = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
	
	ROS_INFO(" getNextPtN :  gx=%.3f. gy=%.3f. AngleToGoal=%.3f. ptpDist=%.3f.",gx,gy,angleToGoal,ptpDist);
	ROS_INFO(" ------------------------------------------------------------");
}

void BotNode::findNewPathForward(void) {
	ros::NodeHandle nm;
	bool ret;
	htbot::goal msg;

	ROS_INFO("----- Find New Path. cptr=%d. eptr=%d ------- ",cmovePtr,emovePtr);
	if (cmovePtr <= emovePtr) {
		msg.x = nposeInfo[cmovePtr][0];
		msg.y = nposeInfo[cmovePtr][1];
		ROS_INFO("--- findNewPathForward Next Location : x=%.3f. y=%.3f -----",msg.x,msg.y);
		chkplan_pub.publish(msg);
		nm.setParam("checkPlanDone",false);
	}
}

void BotNode::newMoveAngle()
{
	double av,diff;
	
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);

  // calculate angle to goal(gx,gy) from robot position(px,py)
	diff = angles::shortest_angular_distance(yawp, yawStart);
	//ROS_INFO("Angle Move. diff=%.3f",diff);
	//av = pidAngle * (diff);
	//if (av > maxAngleVel) {
	//	av = maxAngleVel;
	//}
	//av = maxAngleVel;

	av = maxAngleVel * fabs(diff);
	//if (av > maxAngleVel) {
	//	av = maxAngleVel;
	//}
	if (av > 0.5) {
		av = 0.5;
	}
	if (av < 0.03) {
		av = 0.03;
	}
	if (diff > 0.0) {
		angular_ = av;
	} else {
		angular_ = -av;;
	}
	linear_ = 0.0;
	//ROS_INFO("Angle Move. diff=%.3f. speed=%.3f",diff,angular_);
	if (fabs(diff) < dAngleLimit) {
		angular_ = 0.0;
		stopAngleMove = true;
		//ROS_INFO("Angle Move Stopped. diff=%.3f",diff);
	}
	errorState = !robot.moveVelocity(linear_,angular_);
}

void BotNode::newMoveDist()
{
	double lv,dd,an,av,diff,andiff;
				
	dd = sqrt(((px-opx) * (px-opx)) + ((py-opy) * (py-opy)));
	diff = fabs(fabs(distStart) - dd);
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	an = ptpAngle();
	//andiff = angles::shortest_angular_distance(yawp, an);
	//ROS_INFO("Dist Move. dd=%.3f. diff=%.3f",dd,diff);
	lv = maxDistVel * diff;
	//if (lv > maxDistVel) {
	//	lv = maxDistVel;
	//}
	//if (lv < 0.05) {
	//	lv = 0.05;
	//}
	if (lv > 0.1) {
		lv = 0.1;
	}
	if (distStart > 0.0) {
		linear_ = lv;
		andiff = angles::shortest_angular_distance(yawp, an);
	} else {
		linear_ = -lv;
		andiff = angles::shortest_angular_distance(yawp+PI, an);
	}
	//ROS_INFO("Speed diff = %.3f. lv=%.3f",diff,linear_);
	//angular_ = 0.0;
	av = 0.4 * fabs(andiff);  //0.38
	if (av > 0.4) {
		av = 0.4;
	}		
		
	if (andiff > 0.0) {
		angular_ = av;
	} else {
		angular_ = -av;
	}
	if (diff < dDistLimit) {
		linear_ = 0.0;
		stopDistMove = true;
		//ROS_INFO("Dist Move Stopped. diff=%.3f",diff);
	}
	errorState = !robot.moveVelocity(linear_,angular_);
}

void BotNode::newMoveN()
{
	double av,lv,dd,an,diff,angle,idist,xdist;
	ros::NodeHandle nn;
	
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	an = ptpAngle();
	diff = angles::shortest_angular_distance(yawp, an);
	dd = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
		
	if (rotateFirst) {
		av = maxAngleVel * fabs(diff);
		if (av > maxAngleVel) {
			av = maxAngleVel;
		}
		if (av < 0.1) {
			av = 0.1;
		}
		if (diff > 0.0) {
			angular_ = av;
		} else {
			angular_ = -av;;
		}
		linear_ = 0.0;
		//ROS_INFO("======= Angle Move. diff=%.3f. speed=%.3f ==========",diff,angular_);
		if (fabs(diff) < 0.03) {
			angular_ = 0.0;
			rotateFirst = false;
			//ROS_INFO("-------- RotateFirst Stopped. diff=%.3f ------------",diff);
		}
	} else {
		nn.getParam("ProfileMoveObsDist",obsdist);
		nn.getParam("LeftLaserObsDist",leftlaserobsdist);
		nn.getParam("RightLaserObsDist",rightlaserobsdist);
		xdist = 5.0;	
		if (obsdist < xdist) {
			xdist = obsdist;
		}
		if (leftlaserobsdist < xdist) {
			xdist = leftlaserobsdist;
		}
		if (rightlaserobsdist < xdist) {
			xdist = rightlaserobsdist;
		}
		if (xdist < stopDist) {
			//ROS_INFO(" ============== XDist = %.3f =============",xdist);
			linear_ = 0.0;
			angular_ = 0.0;
			publish_sound(23,0,3);
			if (!stopFlag) {
				stopFlag = true;
				stopTime = ros::Time::now();
			} else {
				if ( ros::Time::now() > (stopTime + ros::Duration(rePlanTime)) ) {
					findnewpath = true;
					publish_sound(31,0,3);
					linear_ = 0.0;
					angular_ = 0.0;
					//stopMove = true;
				}
			}
			errorState = !robot.moveVelocity(linear_,angular_);
			return;
		} else {
			if (stopFlag) {
				stopFlag = false;
			}
			findnewpath = false;
		}
		if (ptpDist == 100.0) {
			lv = maxDistVel;
			//if (dd < (switchDist+0.1)) {
			//	if (lv > 0.25) {
			//		lv = 0.25;
			//	}
			//}
		} else {
			//if (dd > 0.5) {
			//	lv = pidDist * (dd/ptpDist);
			//} else {
			//	lv = 0.8 * pidDist * (dd/ptpDist);
			//}
			lv = pidDist * (dd/ptpDist);
			if (lv > maxDistVel) {
				lv = maxDistVel;
			} else {
				if (lv < 0.08) {
					lv = 0.08;
				}
			}
		} 
		av = 0.4 * fabs(diff);  //0.38
		if (av > 0.4) {
			av = 0.4;
		}		
		
		if (diff > 0.0) {
			angular_ = av;
		} else {
			angular_ = -av;
		}
		if (multiptMove) {
			if (dd < switchDist) {
				getNextPt();
			}
		} else {
			//if (dd < (switchDist+0.1)) {
			//	if (lv > 0.1) {
			//		lv = 0.1;
			//	}
			//}
		}
		linear_ = lv;
		if (dd < dDistLimit) {
			ROS_INFO("********** Reached Dist Limit. diff=%.3f. dd=%.3f. linear=%.3f. angular=%.3f *******",diff,dd,linear_,angular_);
			linear_ = 0.0;
			angular_ = 0.0;
			stopMove = true;
			waitTime = ros::Time::now();
			if (!multiptMove) {
				stopAlignRobot = false;
			}			
		}
		
		//ROS_INFO("********** Straight Move : diff=%.3f. dd=%.3f. linear=%.3f. angular=%.3f *******",diff,dd,linear_,angular_);
	}
	errorState = !robot.moveVelocity(linear_,angular_);
}


void BotNode::ptpAlignMove()
{
	double av,lv,dd,an,diff,angle,idist,xdist;
	ros::NodeHandle nn;
	
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	an = ptpAngle();
	diff = angles::shortest_angular_distance(yawp, an);
	dd = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
	if ((dd < dDistLimit) || (alignDistReached)) {
		diff = angles::shortest_angular_distance(yawp, yawg);
		if ((fabs(diff) < 0.03)) {
			alignAngleReached = true;
			stopAlignRobot = true;
			linear_ = 0.0;
			angular_ = 0.0;
			errorState = !robot.moveVelocity(linear_,angular_);
			ROS_INFO("********** Reached and Aligned to ptpAlign Destination  *******");
			return;
		}
	}
	// check for obstacle
	nn.getParam("ProfileMoveObsDist",obsdist);
	nn.getParam("LeftLaserObsDist",leftlaserobsdist);
	nn.getParam("RightLaserObsDist",rightlaserobsdist);
	xdist = 5.0;	
	if (obsdist < xdist) {
		xdist = obsdist;
	}
	if (leftlaserobsdist < xdist) {
		xdist = leftlaserobsdist;
	}
	if (rightlaserobsdist < xdist) {
		xdist = rightlaserobsdist;
	}
	if (xdist < stopDist) {
		//ROS_INFO(" ============== XDist = %.3f =============",xdist);
		linear_ = 0.0;
		angular_ = 0.0;
		publish_sound(23,0,3);
		return;
	} 

	av = maxAngleVel * fabs(diff);
	if (av > maxAngleVel) {
		av = maxAngleVel;
	}
	//if (av < 0.1) {
	//	av = 0.1;
	//}
	if (diff > 0.0) {
		angular_ = av;
	} else {
		angular_ = -av;;
	}
	//if ((fabs(diff) < 0.03)) {
	//	angular_ = 0.0;
		//if (alignDistReached) {
		//	alignAngleReached = true;
		//}
	//	ROS_INFO("-------- Reached Angle Limit. diff=%.3f ------------",diff);
	//}
	lv = pidDist * (dd/ptpDist);
	if (lv > 0.1) {
		lv = 0.1;
	} 
	if ((fabs(diff) > 0.2)) {
		lv = 0.02;
	}
	linear_ = lv;
	if ((dd < dDistLimit) || (alignDistReached)) {
	//if ((dd < dDistLimit)) {
		ROS_INFO("********** Reached Dist Limit. dd=%.3f.  *******",dd);
		linear_ = 0.0;		
		//angular_ = 0.0;
		alignDistReached = true;
		//publish_sound(23,0,3);
	}
	errorState = !robot.moveVelocity(linear_,angular_);
}


void BotNode::alignRobot()
{
	double av,diff;
	
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	tf::Quaternion qg(0.0,0.0,grz,grw);
	yawg = tf::getYaw(qg);
  
	diff = angles::shortest_angular_distance(yawp, yawg);
	av = maxAngleVel * fabs(diff);
	if (av > maxAngleVel) {
		av = maxAngleVel;
	}
	if (av < 0.08) {
		av = 0.08;
	}
	if (diff > 0.0) {
		angular_ = av;
	} else {
		angular_ = -av;;
	}
	if (fabs(diff) < dAngleLimit) {
		angular_ = 0.0;
		stopAlignRobot = true;
		ROS_INFO("Alignment Move Stopped. diff=%.3f",diff);
	}
	errorState = !robot.moveVelocity(linear_,angular_);
}

void BotNode::buttonCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	ros::NodeHandle rn; 
	int nGP, nLP, butNo;
	//bool ok;
	geometry_msgs::Twist pos;
	bool ok;
	char buf [100];
	string s;

	butNo = msg->data; 
	//ROS_INFO("BT_Node : Button Code Received = %d",butNo);

	switch (butNo) {

		case 13: // robot charging now. Stop robot
			// stop robot.
			//startPMove = false;
			//publish_debug("Stop Robot");
			robot.quickstop = true;
			sprintf(buf,"BotNode : Stop Robot"); 
			s.assign(buf,strlen(buf));
			publish_debug(s);
			publish_status("Docking Activated");
			break;
		case 15:
			//estop_pressed = true;
			//ROS_INFO("estop pressed");
			//publish_sound(15,0,0);
			//estop = true;
			//rn.setParam("estop",estop);
			//publish_status("Emergency Stop Activated");
			//publish_debug("Move Node : Emergency Stop Activated");			
			break;
		case 16:
			//if (estop) {
			//	estop_pressed = false;
			//	ROS_INFO("estop released");	
			//	publish_status("Emergency Stop Released");
			//	publish_debug("Move Node : Emergency Released");
			//	estop = false;
			//}
			break;

	}
	return;
}

void BotNode::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	px = msg->position.x;
	py = msg->position.y;
	pz = msg->position.z;
	prx = msg->orientation.x;
	pry = msg->orientation.y;
	prz = msg->orientation.z;
	prw = msg->orientation.w;

	//poseAvailable = true;
	//ROS_INFO("BNPose : x=%.3f. y=%.3f. rz=%.3f",px,py,prz);
}

void BotNode::publish_podom(double dist, double angle)
{
	htbot::odom odom;
	odom.dist = dist;
	odom.angle = angle;
	podom_pub.publish(odom);
	return;
}

void BotNode::publish_move_status(int stat)
{
	htbot::move_status status;
	status.stat = stat;
	status_mov.publish(status);
	return;
}

void BotNode::publish_toggleButton(unsigned short cmd)
{
	std_msgs::UInt16 msg;
	msg.data = cmd;
	toggleButton_pub.publish(msg);
	return;
}

void BotNode::publish_debug(string s)
{
	htbot::debug status;
	status.msg = s;
	debug_pub.publish(status);
	return;
}

void BotNode::publish_sound(int id,int startdelay,int restartdelay)
{
	htbot::sound cmd;
	cmd.id = id;
	cmd.startdelay = startdelay;
	cmd.restartdelay = restartdelay;
	play_pub.publish(cmd);
	//ROS_INFO("Publish Sound AA");
	return;
}

double BotNode::calcAn(void)
{
	double yd;
	double ang_diff;

	tf::Quaternion qg(0.0,0.0,grz,grw);
	tf::Quaternion qp(0.0,0.0,prz,prw);

	yawg = tf::getYaw(qg);
	yawp = tf::getYaw(qp);
	ang_diff = angles::shortest_angular_distance(yawp, yawg);
	//ROS_INFO("\n============ BTNode : ang_diff = %.2f. =============\n",ang_diff);
	return ang_diff;
}

void BotNode::publish_clear(void)
{
	htbot::clear cmd;
	cmd.cmd = 1;
	//ROS_INFO("Move_Node : publise clear Map");
	publish_debug("bot_Node : Publish Clear Map" );
	clear_pub.publish(cmd);
	return;
}

void BotNode::restartRobot()
{
	int tries = 0;

	nh.setParam("MotorError",1);
	if (!resetting)
		resetting = true;
	else return;
	
	while(ros::ok() && !robot.reset())
	{
		ROS_ERROR("Failed to contact EPOS, going to try restarting EPOS, tries = %i...", tries+1);
		sleep(1);
		tries++;
		if (tries>10)
		{
			tries = 0;
			robot.shutDown();
			robot.start();
		}
	}
	resetting = errorState = false;
	nh.setParam("MotorError",0);
}

bool BotNode::resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	nh.param("starting_x", cur_x, 0.);
	nh.param("starting_y", cur_y, 0.);
	nh.param("starting_theta", cur_theta, 0.);
	return true;
}

void BotNode::joyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	joylinear_ = msg->linear.x;
	joyangular_= msg->angular.z;
	//if ((joylinear_ == 0.0) && (joyangular_ == 0.0) ) {
	//	estopped = false;
	//} else {
	//	estopped = true;
	//}
	last_joycmd_time = ros::Time::now();
	//ROS_INFO("Move joycallbk at vx [%f] & w [%f]", joylinear_, joyangular_);
}

void BotNode::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	//angular_= cmdvel_offset * msg->angular.z;
	//linear_ = cmdvel_offset * msg->linear.x;
	robot.mangular =  msg->angular.z;
	robot.mlinear =  msg->linear.x;
	//ROS_INFO("Vel : Move at vx [%f] & w [%f] & offset [%.6f]", linear_, angular_,cmdvel_offset);
	last_cmd_time = ros::Time::now();
}

void BotNode::calAlignAngle() {
	double yd,yda;
	char buf[100];
	string s1;

	tf::Quaternion qg(0.0,0.0,grz,grw);
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawg = tf::getYaw(qg);
	yawp = tf::getYaw(qp);
	yd = yawg - yawp;
	if (yd >= M_PI) {
		yd -= 2.0 * M_PI;
	} else {
		if (yd < -M_PI) {
			yd += 2.0 * M_PI;
		}
	}
	yda = (180.0 * yd) / M_PI;
	sprintf(buf,"Quat : AlignAngle(rad) : %0.3f. AlignAngle(deg) : %0.3f.",yd,yda);
	s1.assign(buf,strlen(buf));
	publish_debug(s1);
}

void BotNode::posCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	
	//bool st;
	double dd,trz,dx,dy,pa,ga,xa,ya;
	double aa,aan,da,dan,dat,dant;
	char buf[100];
	string s1;
	bool front;

	plinear = msg->linear.x;
	pangular = msg->angular.z;
	pspeed = msg->linear.y;

	// *************** scan matching alignment : polar *************
	if (msg->linear.z == 35.0) {
		dx = msg->linear.x;
		dy = msg->linear.y * 0.85;
		da = msg->angular.z;
		pa = (da * 180.0) / 3.14159;
		dd = sqrt((dx * dx) + (dy * dy));
		// calculate axis of goal and robot position
		if ((dx >= 0.0) && (dy >= 0.0)) {
			// 1st Q : anticlockwise
			ya = ((atan(dy/dx) * 180.0) / 3.14159);  // from x axis
			alignX = ya - pa; // reverse dx
			alignDX = -dd;
			alignAngle = -ya;
		} else {
			if ((dx < 0.0) && (dy >= 0.0)) {
				// 2nd Q : anticlockwise
				ya = ((atan(dy/-dx) * 180.0) / 3.14159);   // from x axis
				alignX = -(ya + pa);  // forward dx
				alignDX = dd;
				alignAngle = ya;
			} else {
				if ((dx < 0.0) && (dy < 0.0)) {
					// 3rd Q : clockwise
					ya = ((atan(-dy/-dx) * 180.0) / 3.14159);
					alignX = ya - pa; // forward dx
					alignDX = dd;
					alignAngle = -ya;
				} else {
					// 4th Q : clockwise
					ya = ((atan(-dy/dx) * 180.0) / 3.14159);
					alignX = -(ya + pa);  // reverse dx
					alignDX = -dd;
					alignAngle = ya;
				}
			}
		}
		sprintf(buf,"Align35: outx=%.3f. outy=%.3f. outa=%.3f.",msg->linear.x,msg->linear.y,msg->angular.z);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		sprintf(buf,"Align35: alignX=%.3f. alignDX=%.3f. alignAngle=%.3f",alignX,alignDX,alignAngle);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		startPMove = true;
		//pstate = 9;
		pstate = 110;
		return;
	}

	if (msg->linear.z == 35.5) {
		pstate = 90;
		return;
	}

	if (msg->linear.z == 40.0) {  // scan matching alignment		
		if ((grz < 0.0)) {
			// 4th quadrant
			ga = 360.0 - (asin(-grz) * 114.59);
		} else {
			if (grw < 0.0) {
				// 3rd quad
				ga = 360.0 - (asin(grz) * 114.59);
			} else {
				// 1st and 2nd quad
				ga = (asin(grz) * 114.59);
			}
		}
		// convert prz to angle 0~360
		if ((prz < 0.0)) {
			// 4th quadrant
			pa = 360.0 - (asin(-prz) * 114.59);
		} else {
			if (prw < 0.0) {
				// 3rd quad
				pa = 360.0 - (asin(prz) * 114.59);
			} else {
				// 1st and 2nd quad
				pa = (asin(prz) * 114.59);
			}
		}
		//dx = gx-px;
		//dy = gy-py;
		dx = msg->linear.x;
		dy = msg->linear.y;
		if ((pa < 90.0) || (pa > 270.0)) {
			front = true;
		} else {
			front = false;
		}
		if ((dx > 0.015) || (dx < -0.015)) {  // 0.01
			if ((pa < 90.0) || (pa > 270.0)) {
				xa = 0.0;
				ya = 0.0;
				alignDX = -dx;
			} else {
				xa = 180.0;
				ya = 180.0;
				alignDX = -dx;
			}
			alignX = xa - pa;
			if (alignX > 180.0) {
				alignX = alignX - 360.0;
			} else {
				if (alignX < -180.0) {
					alignX = 360.0 + alignX;
				}
			}
		} else {
			ya = pa;
			alignDX = 0.0;
		}
		if ((dy > 0.015) || (dy < -0.015)) {  //0.01
			if ((ya < 180.0) && (ya > 0.0)) {
				xa = 90.0;				
				if (front) {
					alignDY = -dy;
				} else {
					alignDY = dy;
				}
			} else {
				xa = 270.0;			
				if (front) {	
					alignDY = dy;
				} else {
					alignDY = -dy;
				}
			}
			alignY = xa - ya;
			if (alignY > 180.0) {
				alignY = alignY - 360.0;
			} else {
				if (alignY < -180.0) {
					alignY = 360.0 + alignY;
				}
			}
		} else {
			xa = ya;
			alignDY = 0.0;
		}
		alignAngle = ga - xa;				
		if (alignAngle > 180.0) {
			alignAngle = alignAngle - 360.0;
		} else {
			if (alignAngle < -180.0) {
				alignAngle = 360.0 + alignAngle;
			}
		}
		sprintf(buf,"Align30: outx=%.3f. outy=%.3f. outa=%.3f.",msg->linear.x,msg->linear.y,msg->angular.z);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		sprintf(buf,"Align30A: dx=%.3f. alignX=%.3f. dy=%.3f. alignY=%.3f. alignAngle=%.3f",alignDX,alignX,alignDY,alignY,alignAngle);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		startPMove = true;
		//pstate = 9;
		pstate = 91;
		//pstate = 100;
		return;
	}

	// *************** scan matching alignment : dx and dy *************
	if (msg->linear.z == 30.0) {
		dx = msg->linear.x;
		dy = msg->linear.y;
		da = msg->angular.z;
		pa = -(da * 180.0) / 3.14159;
		alignDX = -dx;
		alignX = pa;
		alignDY = dy;
		alignY = -90.0;
		alignAngle = 90.0;
		sprintf(buf,"Align30: outx=%.3f. outy=%.3f. outa=%.3f.",msg->linear.x,msg->linear.y,msg->angular.z);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		sprintf(buf,"Align30: dx=%.3f. alignX=%.3f. dy=%.3f. alignY=%.3f. alignAngle=%.3f",alignDX,alignX,alignDY,alignY,alignAngle);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		startPMove = true;
		//pstate = 9;
		pstate = 100;
		return;
	}

	if (msg->linear.z == 0.0) {
		// normal state m/c move
		startPMove = true;
		docking = true;
		sprintf(buf,"posCall-startPM = true : Linear : %0.6f. Angle : %0.6f. Speed : %0.6f.",plinear,pangular,pspeed);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		return;
	}
	if (msg->linear.z == 1.0) {
		// move straight to docking station
		//publish_debug("BTNode : Move forward to Dock");
		startPMove = true;
		pstate = 11;
		return;
	}
	if (msg->linear.z == 2.0) {
		// move away from docking station
		//publish_debug("BTNode : Move away from Dock");
		startPMove = true;
		pstate = 13;
		return;
	}
	if (msg->linear.z == 7.0) {
		// stop robot halfway
		startPMove = false;
		//publish_debug("Stop Robot");
		robot.quickstop = true;
		//sprintf(buf,"BotNode : Stop Robot"); 
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
    //ROS_INFO("btnode : stop robot");
		return;
	}
	if (msg->linear.z == 7.5) {
		// stop robot halfway
		startPMove = false;
		//publish_debug("Stop Robot");
		robot.quickstop = true;
		//sprintf(buf,"BotNode : Stop Robot"); 
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
		estopped = true;
		return;
	}
	if (msg->linear.z == 7.6) {
		// cancel operation		
		robot.freeMotor();
		publish_debug("Motors Released");
		publish_sound(22,0,0);  // motors released
		return;
	}
	if (msg->linear.z == 7.7) {	
		while(!robot.reset())
		{
			ROS_INFO("EPOS not ready. Trying again in 1 second....");
			sleep(1);
		}
		publish_debug("Motors Re-Engaged");
		publish_sound(22,0,0);  // motors re-engaged
		return;
	}
	if (msg->linear.z == 7.8) { // estop activated. reduce start velocity
		// slow down re-start speed
		//publish_sound(22,0,0);
		//robot.moveVelocity(0.0,0.0);
		//usleep(800000);
		//sleep(1.5);
		estop_pressed = true;
		//robot.moveVelocity(0.0,0.0);
		//usleep(100000);
		//robot.freeMotor();
		//ROS_INFO("BotNode :estop pressed");
		//if (navstatus == 7) {
		//	publish_sound(15,0,0);
		//}
		//publish_sound(22,0,0);
		return;
	}
	if (msg->linear.z == 7.9) { // estop release. 
		robot.moveVelocity(0.0,0.0);
		usleep(100000);
		estop_pressed = false;
		//publish_sound(18,0,0);
		ROS_INFO("BotNode estop released");	
		last_cmd_time = ros::Time::now();
		return;
	}
	if (msg->linear.z == 8.0) {
		startPMove = true;
		docking = true;
		//sprintf(buf,"BotNode : Stage 2 -> moving to dock"); 
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
		return;
	}
	if (msg->linear.z == 9.0) {
		boost::mutex::scoped_lock lock(publish_mutex_);
		startPMove = true;
		pstate = 7;
		//publish_debug("Rotate Anti Clockwise");
		return;
	}
	if (msg->linear.z == 9.1) {
		boost::mutex::scoped_lock lock(publish_mutex_);
		startPMove = true;
		pstate = 71;
		//publish_debug("Rotate Clockwise");
		return;
	}
	if (msg->linear.z == 9.5) {
		boost::mutex::scoped_lock lock(publish_mutex_);
		startPMove = true;
		pstate = 8;
		//publish_debug("Move Straight Forward");
		return;
	}
	if (msg->linear.z == 9.6) {
		boost::mutex::scoped_lock lock(publish_mutex_);
		startPMove = true;
		pstate = 81;
		//publish_debug("Move Straight Back");
		return;
	}
	if (msg->linear.z == 9.7) {
		boost::mutex::scoped_lock lock(publish_mutex_);
		startPMove = true;
		pstate = 1020;
		//new align move
		return;
	}
	if (msg->linear.z == 10.0) {
		//startPMove = true;
		//pstate = 9;
		// dd = 2[asin(grz) - asin(prz)]*180 / pi
		//trz = sin(atan((gy - py)/(gx - px)) / 2.0);
		// convert grz to angle 0~360
		if ((grz < 0.0)) {
			// 4th quadrant
			ga = 360.0 - (asin(-grz) * 114.59);
		} else {
			if (grw < 0.0) {
				// 3rd quad
				ga = 360.0 - (asin(grz) * 114.59);
			} else {
				// 1st and 2nd quad
				ga = (asin(grz) * 114.59);
			}
		}
		// convert prz to angle 0~360
		if ((prz < 0.0)) {
			// 4th quadrant
			pa = 360.0 - (asin(-prz) * 114.59);
		} else {
			if (prw < 0.0) {
				// 3rd quad
				pa = 360.0 - (asin(prz) * 114.59);
			} else {
				// 1st and 2nd quad
				pa = (asin(prz) * 114.59);
			}
		}
		//alignAngle = (114.59 * (asin(grz) - asin(prz)));
		//alignAngle = ga - pa;				
		
		//alignAngle = (114.59 * (asin(trz) - asin(prz)));
		dx = gx-px;
		dy = gy-py;
		if ((dx > 0.025) || (dx < -0.025)) {  // 0.01
			if ((pa < 90.0) || (pa > 270.0)) {
				xa = 0.0;
				ya = 0.0;
				alignDX = dx;
			} else {
				xa = 180.0;
				ya = 180.0;
				alignDX = -dx;
			}
			alignX = xa - pa;
			if (alignX > 180.0) {
				alignX = alignX - 360.0;
			} else {
				if (alignX < -180.0) {
					alignX = 360.0 + alignX;
				}
			}
		} else {
			ya = pa;
			alignDX = 0.0;
		}
		if ((dy > 0.025) || (dy < -0.025)) {  //0.01
			if ((ya < 180.0) && (ya > 0.0)) {
				xa = 90.0;				
				alignDY = dy;
			} else {
				xa = 270.0;				
				alignDY = -dy;
			}
			alignY = xa - ya;
			if (alignY > 180.0) {
				alignY = alignY - 360.0;
			} else {
				if (alignY < -180.0) {
					alignY = 360.0 + alignY;
				}
			}
		} else {
			xa = ya;
			alignDY = 0.0;
		}
		alignAngle = ga - xa;				
		if (alignAngle > 180.0) {
			alignAngle = alignAngle - 360.0;
		} else {
			if (alignAngle < -180.0) {
				alignAngle = 360.0 + alignAngle;
			}
		}
		//alignDY = dy;
		//calAlignAngle();
		alignDist = sqrt((dx * dx) + (dy * dy));
		sprintf(buf,"Align10: trz=%.3f. prz=%.3f. ga=%.3f. pa=%.3f. align=%.3f. Dist=%.3f",trz,prz,ga,pa,alignAngle,alignDist);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		sprintf(buf,"AlignXY10: dx=%.3f. alignX=%.3f. dy=%.3f. alignY=%.3f",alignDX,alignX,alignDY,alignY);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		startPMove = true;
		//pstate = 9;
		if (dx < 0.0) {
			dx = -dx;
		}
		if (dy < 0.0) {
			dy = -dy;
		}
		if ((dx > 0.65) || (dy > 0.65)) {
			pstate = 90;
		} else {
			pstate = 91;
		}
		return;
	}
	if (msg->linear.z == 11.0) {
		if ((grz < 0.0)) {
			// 4th quadrant
			ga = 360.0 - (asin(-grz) * 114.59);
		} else {
			if (grw < 0.0) {
				// 3rd quad
				ga = 360.0 - (asin(grz) * 114.59);
			} else {
				// 1st and 2nd quad
				ga = (asin(grz) * 114.59);
			}
		}
		// convert prz to angle 0~360
		if ((prz < 0.0)) {
			// 4th quadrant
			pa = 360.0 - (asin(-prz) * 114.59);
		} else {
			if (prw < 0.0) {
				// 3rd quad
				pa = 360.0 - (asin(prz) * 114.59);
			} else {
				// 1st and 2nd quad
				pa = (asin(prz) * 114.59);
			}
		}
		dx = gx-px;
		dy = gy-py;
		dd = sqrt((dx * dx) + (dy * dy));
		if ( ((dx > -0.035) && (dx < 0.035 )) && ((dy > -0.035) && (dy < 0.035 )) ) {
			alignAngle = ga - pa;				
			if (alignAngle > 180.0) {
				alignAngle = alignAngle - 360.0;
			} else {
				if (alignAngle < -180.0) {
					alignAngle = 360.0 + alignAngle;
				}
			}
			pstate = 9;
			startPMove = true;
			sprintf(buf,"Align11O: dx=%.3f. dy=%.3f. dd=%.3f. prz=%.3f. ga=%.3f. pa=%.3f. alignAngle=%.3f",dx,dy,dd,prz,ga,pa,alignAngle);
			s1.assign(buf,strlen(buf));
			publish_debug(s1);
			return;
		}
		// calc aa and aan
		if ((dx > 0.0) && (dy > 0.0)) {
			// 1st Q
			aa = 57.296 * atan(dy/dx);
		} else {
			if ((dx < 0.0) && (dy > 0.0)) {
				//2nd Q
				aa = 90.0 + (57.296 * atan(-dx/dy));
			} else {
				if ((dx < 0.0) && (dy < 0.0)) {
					//Q3
					aa = 270.0 - (57.296 * atan(-dx/-dy));
				} else {
					//Q4
					aa = 270.0 + (57.296 * atan(dx/-dy));
				}
			}
		}
		if (aa >= 180.0) {
			aan = aa - 180.0;
		} else {
			aan = aa + 180.0;
		}
		// cal alignT
		da = aa - pa;
		if (da >= 180.0) {
			da = da - 360.0;
		} else {
				if (da < -180) {
					da = 360.0 + da;
				}
		}
		dan = aan - pa;
		if (dan >= 180.0) {
			//dan = 360.0 - dan;
			dan = dan - 360.0;
		} else {
				if (dan < -180) {
					dan = 360.0 + dan;
				}
		}
		dat = da;
		if (dat < 0.0) {
			dat = -dat;
		} 
		dant = dan;
		if (dant < 0.0) {
			dant = -dant;
		}
		if (dat >= dant) {
			alignT = dan;
			alignTD = -dd;
			xa = aan;
		} else {
			alignT = da;
			alignTD = dd;
			xa = aa;
		}		
		alignAngle = ga - xa;				
		if (alignAngle > 180.0) {
			alignAngle = alignAngle - 360.0;
		} else {
			if (alignAngle < -180.0) {
				alignAngle = 360.0 + alignAngle;
			}
		}		
		sprintf(buf,"Align11: dx=%.2f. dy=%.2f. dd=%.2f. aa=%.2f. aan=%.2f.",dx,dy,dd,aa,aan);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		sprintf(buf,"Align11a: aT=%.2f. aTD=%.2f. ga=%.2f. pa=%.2f. alignAngle=%.2f",alignT,alignTD,ga,pa,alignAngle);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		startPMove = true;
		pstate = 95;
		return;
	}
	if (msg->linear.z == 20.0) {
		// pre/post move		
		//publish_debug("Start Pre or Post Move");		
		pstate = 83;
		return;
	}

	if (msg->linear.z == 45.0) {  // point to point move		
		ptpmoveS();	
		return;
	}
}



void BotNode::publish_odom()
{
	if(errorState) return;
	ros::NodeHandle nn;
	//int navstatus;
	//ROS_INFO("publish odom");
	// store the current pose
	nn.getParam("navstatus",navstatus);
	prevPose.x = cur_x;
	prevPose.y = cur_y;
	prevPose.theta = cur_theta;

	// calculate the new pose
	double distance, angle;
	
	errorState = !robot.getDisplacement(&distance, &angle);
	if(errorState)
	{
		ROS_ERROR("getDisplacement failed in errorState.");
		return;
	}
	publish_podom(distance,angle);
} //void BotNode::publish_odom()

/*
void BotNode::publish_odom()
{
	if(errorState) return;
	ros::NodeHandle nn;
	//int navstatus;
	//ROS_INFO("publish odom");
	// store the current pose
	nn.getParam("navstatus",navstatus);
	prevPose.x = cur_x;
	prevPose.y = cur_y;
	prevPose.theta = cur_theta;

	// calculate the new pose
	double distance, angle;
	
	errorState = !robot.getDisplacement(&distance, &angle);
	if(errorState)
	{
		ROS_ERROR("getDisplacement failed in errorState.");
		return;
	}
		
	cur_theta += angle;
	cur_x += distance * cos(cur_theta);
	cur_y += distance * sin(cur_theta); 
	//ROS_DEBUG("PREVIOUS x = [%f], y = [%f], theta = [%f]\n", prevPose.x, prevPose.y, prevPose.theta);
	//ROS_DEBUG("CURRENT x = [%f], y = [%f], theta = [%f]\n", cur_x, cur_y, cur_theta);

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();

	// convert rotation about z into quaternion
	geometry_msgs::Quaternion odom_quat;
	odom_quat.z = sin(cur_theta/2.0);
	odom_quat.w = cos(cur_theta/2.0);

	//next, we'll publish the odometry message over ROS
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	//set the position (with respect to header.frame)
	odom.pose.pose.position.x = cur_x;
	odom.pose.pose.position.y = cur_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	odom.pose.covariance[0] = cov_x;
	odom.pose.covariance[7] = cov_y;
	odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = 1e9;
	odom.pose.covariance[35] = cov_th;

	//set the velocity (with respect to child_frame)
	odom.twist.twist.linear.x = distance/dt;
	odom.twist.twist.angular.z = angle/dt;
	odom.twist.covariance[0] = cov_vx;
	odom.twist.covariance[7] = cov_vy;
	odom.twist.covariance[14] = odom.twist.covariance[21] = odom.twist.covariance[28] = 1e9;
	odom.twist.covariance[35] = cov_vth;

	boost::mutex::scoped_lock lock(publish_mutex_);
	//publish the message
	odom_pub.publish(odom);	
	last_time = current_time;
} //void BotNode::publish_odom()
*/

void BotNode::watchdog()
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	if( ros::Time::now() > odom.header.stamp + ros::Duration(1.0/rate) )
	{
		odom.header.stamp = ros::Time::now();
		odom_pub.publish(odom);
	}
}

void BotNode::modify_cmdvel() {
	double tdist,dx,dy,fan;
	ros::NodeHandle nn;

	if (checkDist) {	
		dx = gx - px;
		dy = gy - py;
		tdist = sqrt((dx * dx) + (dy * dy));
		nn.getParam("ProfileMoveObsDist",obsdist);
		nn.getParam("ProfileMoveObsDDist",obsddist);
		nn.getParam("LeftLaserObsDist",leftlaserobsdist);
		nn.getParam("RightLaserObsDist",rightlaserobsdist);
		nn.getParam("MapObsDist",mapobsdist);
	
		if ((tdist < slowDist) ) {	
			if ( (tdist < 0.5) ) {
				if (linear_ > 0.22) {
					linear_ = 0.22;
				}
				if (tdist < 0.1) {
					linear_ = 0.0;
					angular_ = 0.0;					
				}
			}
			if ((linear_ == 0.0) && (tdist < 0.22) && !reach && (navstatus == 7) ) {
				fan = calcAn();
				if ((fan > 0.0)) {								
					reachTurn = true;
				} else {
					reachTurn = false;
				}
				reach = true;
				waitflag = false;							
			}						
			if (reach) {							
				double an = fabs(calcAn());
				linear_ = 0.0;
				//ROS_INFO(" AN : %.3f. Angular : %.3f",an, angular_);							
				if (reachTurn) {
					if (an > 1.8) {
						angular_ = 0.75;  // 0.45
					} else {
						if (an > 1.5) {
							angular_ = 0.7; 
						} else {
							if (an > 1.0) {
								angular_ = 0.6; 
							} else {
								if (an > 0.8) {
									angular_ = 0.45; 
								} else {
									if (an > 0.4) {
										angular_ = 0.35; 
									} else {
										angular_ = 0.12; 
									}
								}
							}
						}
					}
				} else {
					if (an > 1.8) {
						angular_ = -0.75;  // 0.45
					} else {
						if (an > 1.5) {
							angular_ = -0.7; 
						} else {
							if (an > 1.0) {
								angular_ = -0.6; 
							} else {
								if (an > 0.8) {
									angular_ = -0.45; 
								} else {
									if (an > 0.4) {
										angular_ = -0.35; 
									} else {
										angular_ = -0.12; 
									}
								}
							}
						}
					}
				}
				if (an < 0.03) {  // 0.06
					angular_ = 0.0;
					linear_ = 0.0;
					if (!waitflag) {
						waitTime = ros::Time::now();
						waitflag = true;
						cancel_goal();									
					}
					if (ros::Time::now()-waitTime > ros::Duration(2.5)) {
						reach = false;
					}
					//usleep(250000);
				}
							
			}						
		} else {						
			if (estop_pressed) {
				if (!estop_pressed_flag) { 								
					errorState = !robot.moveVelocity(0.0,0.0);						
					estop_pressed_flag = true;
				}
			}
		}
	}
}

void BotNode::mainLoop()
{
	ros::Rate r(rate);
	//ros::Rate r(0.2);
	ros::NodeHandle nn;
	double tdist,dx,dy;
	double distance,angle; //obsdist,obsddist;
	pcount = 0;	 
	count = 0;
	char buf[100];
	string s1;
	int lookforObs;
	double NPDist,NPAngle;
	double vLDist,vRDist;

	//stop = false;
	while(ros::ok())
	{
		//ROS_INFO("loop..");
		
		if (estop_pressed_flag && !estop_pressed) {
			restartRobot();
			estop_pressed_flag = false;
		}
		if (estop_pressed  && !estop_pressed_flag && (navstatus != 7)) {
			estop_pressed_flag = true;
		}
		
		// Stop moving if high level control loses link to driver node for more than 1 sec
		if (navstatus == 7) {
			if(ros::Time::now()-last_cmd_time > ros::Duration(1)) {
				//ROS_INFO("BTNode Error vel");
				linear_ = angular_ = 0;
				cmd_estopped = true;
			} else {
				cmd_estopped = false;
			}  
		} else {
			cmd_estopped = true;
		}

		nn.getParam("lookforObs",lookforObs);
		if (lookforObs == 1) {
			nn.getParam("ProfileMoveObsDist",obsdist);
			nn.getParam("ProfileMoveObsDDist",obsddist);
			//ROS_INFO("Dist : %.3f. DDist : %.3f",obsdist,obsddist);
		}
		
		switch (pstate) {
			case 0:	
				if (startPMove) {
					if (!docking) {						
						errorState = !robot.moveAngle(pangular);//pangular
						if (errorState) {
							//publish_debug("MoveAngle Error");
							//ROS_INFO("MA Err");
						} else {
							//publish_debug("MoveAngle OK");
							//ROS_INFO("MA OK");
						}
						delay_time = ros::Time::now();
						//publish_debug("angle move complete");
																
						startPMove = false;
						pstate = 2;// 1
					} else {
						pstate = 2;// 
					}
				}
				break;
			case 1:
				if (robot.checkMoveComplete()) {
					pstate = 2;// 2
					pcount = 0;
					delay_time = ros::Time::now();
					//ROS_INFO("MA Done");
					//publish_debug("angle move complete");
				}
				break;
			case 11:				
				estopped = true;			
				pstate = 15;	
				//publish_debug("BNode : Moving to Docking Station");
				//publish_status("BNode : Moving to Docking Station");
				errorState = !robot.moveDistance(dockdist);
				delay_time = ros::Time::now();
				break;			
			case 12:
        //if (robot.quickstop) {
				if( (robot.quickstop) || (current < 0.0) || (ros::Time::now() > (delay_time + ros::Duration(10.0))) ) {//7
					//ROS_INFO("pstate 12");
					pstate = 123; //121; //121; //0;
					//robot.setVelMode(); 
					errorState = !robot.moveVelocity(0.0,0.0); 
					//startPMove = false;
					//publish_move_status(1); // *
					delay_time = ros::Time::now();
					//errorState = !robot.moveVelocity(-0.01,0.0);
					//estopped = false;	//*
				} 
				break;
			case 121:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					delay_time = ros::Time::now();
					errorState = !robot.moveVelocity(-0.013,0.0);
					pstate = 122; //0;
				}
				break;
      case 122:
				if(ros::Time::now()-delay_time > ros::Duration(0.65)) {
					errorState = !robot.moveVelocity(0.0,0.0);
					//startPMove = false; //*
					//publish_move_status(1); //*
					pstate = 123; //0
					delay_time = ros::Time::now();
					//estopped = false; //*
				}
				break;
			case 123:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					ROS_INFO("BtMode current = %.3f",current);
					if (current < -0.5) {
						startPMove = false;
						publish_move_status(1);
						pstate = 0;
						estopped = false;
					} else {
						errorState = !robot.moveVelocity(-0.015,0.0);
						pstate = 124; //0
						delay_time = ros::Time::now();
					}
				}
				break;
			case 124:
				if(ros::Time::now()-delay_time > ros::Duration(0.6)) {
					errorState = !robot.moveVelocity(0.0,0.0);
					startPMove = false; 
					publish_move_status(1); 
					pstate = 0;
					estopped = false; 
				}
				break;
			case 13:  // move away from docking station
				pstate = 14;
				//publish_debug("Move Away from Docking Station");
				//publish_status("BtNode : Move Away from Docking Station");
				errorState = !robot.moveDistance(dockrdist);
				delay_time = ros::Time::now();
				break;
			case 14:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					errorState = !robot.moveAngle(dockrangle);
					pstate = 0;
					//publish_status("BtNode : Turning Away from Docking Station");
					publish_move_status(1);
				}
				break;
			case 15:		
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {			
					pstate = 12;	
					//publish_debug("BNode : Moving to Docking Station");
					//publish_status("BNode : Moving to Docking Station");
					//robot.setVelMode();
					errorState = !robot.moveVelocity(dockspeed,0.0);
					delay_time = ros::Time::now();
				}
				break;
			case 2:				
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {					
					if (pspeed > 0.0) {
						//pstate = 5;
						pstate = 21;
						startPMove = true;
						estopped = true;
						fspeedtime = (plinear * 5.3);
						//robot.setVelMode();
						errorState = !robot.moveVelocity(0.15,0.0);
						delay_time = ros::Time::now();
						//publish_debug("set low speed");
					} else {
						pstate = 4;
						//publish_debug("MoveDist");
						errorState = !robot.moveDistance(plinear);
						//publish_debug("dist move complete");
						delay_time = ros::Time::now();
					}
				} 
				break;
			case 21:
				if(ros::Time::now()-delay_time > ros::Duration(fspeedtime)) {
					errorState = !robot.moveVelocity(0.03,0.0);
					delay_time = ros::Time::now();
					pstate = 5;
				}
				break;
			case 22:				
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					pstate = 4;
					if (pspeed > 0.0) {
						robot.setSlowMove();
						//publish_debug("set low speed");
					}
					publish_debug("MoveDist");
					errorState = !robot.moveDistance(plinear);
					//publish_debug("dist move complete");
					delay_time = ros::Time::now();
					//ROS_INFO("MD Done");
				} 
				break;
			case 3:
				if (robot.checkMoveComplete()) {
					pstate = 4;
					delay_time = ros::Time::now();		
					//publish_debug("dist move complete");		
				}
				if (!startPMove) {  // stop
					robot.stopRobot();
					//publish_debug("stop robot");
					pstate = 4;
					delay_time = ros::Time::now();					
				}
				break;
			case 30:
				newMoveAngle();					
				if (stopAngleMove) {
					pstate = 0;
				}
				break;
			case 31:
				newMoveDist();					
				if (stopDistMove) {
					pstate = 0;
				}
				break;
			case 32:
				newMoveAngle();					
				if (stopAngleMove) {
					pstate = 321;
					delay_time = ros::Time::now();
				}
				break;
			case 321:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					newMoveDist();					
					if (stopDistMove) {						
						pstate = 322;
						stopAngleMove = false;
						delay_time = ros::Time::now();
					}
				}
				break;
			case 322:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					//tf::Quaternion qp(0.0,0.0,prz,prw);
					//yawp = tf::getYaw(qp);
					//yawStart = angles::shortest_angular_distance(yawp, yawg);
					yawStart = yawg;
					pstate = 323;
					stopAngleMove = false;
					delay_time = ros::Time::now();
				}
				break;
			case 323:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					newMoveAngle();			
					if (stopAngleMove) {
						pstate = 0;
					}
				}
				break;
			case 33:
				newMoveN();			
				if (stopMove) {
					pstate = 330;
					delay_time = ros::Time::now();
				}
				if (findnewpath) {
					pstate = 331;
					findNewPathForward();
					//delay_time = ros::Time::now();
				}
				break;
			case 330:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					alignRobot();		
					if (stopAlignRobot) {
						//startPMove = false; 
						publish_move_status(1);
						pstate = 0;
					}
				}
				break;
			case 331:
				nn.getParam("checkPlanDone",checkPlanDone);
				if (checkPlanDone) {
					nn.getParam("checkPlanSum",checkPlanSum);
					if (checkPlanSum < 100.0) {
						ROS_INFO("------ BtNode : CheckPlan Found. DistSum=%.3f. -----",checkPlanSum);
					} else {
						if (checkPlanSum == 100.0) {
							ROS_INFO("------ BtNode : CheckPlan Not Found . Empty -----");
						} else {
							if (checkPlanSum == 100.0) {
								ROS_INFO("------ BtNode : CheckPlan Failed to Call -----");
							}
						}
					}
					pstate = 0;
				}
				break;
			case 34:
				ptpAlignMove();
				if (stopAlignRobot) {
					pstate = 0;
				}
				break;
			case 4:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					pstate = 0;
					//robot.setVelMode();
					//robot.setFastMove();
					//last_cmd_time = ros::Time::now();
					nn.setParam("ReachPosition",1);
					startPMove = false;
					//publish_debug("complete all move");
					//ROS_INFO("ALl Done");
					publish_move_status(1);
				} 
				break;
			case 5:
				if ((!startPMove) || (ros::Time::now()-delay_time > ros::Duration(mstop_time))) {  // stop
					robot.stopRobot();
					//publish_debug("stop robot");
					pstate = 6;
					estopped = false;
					delay_time = ros::Time::now();					
				} 
				break;
			case 6:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					pstate = 0;
					//robot.setVelMode();
					//robot.setFastMove();
					//last_cmd_time = ros::Time::now();
					nn.setParam("ReachPosition",1);
					startPMove = false;					
					//publish_debug("complete all move");
					//ROS_INFO("ALl Done");
					publish_move_status(1);
				} 
				break;
			case 7:  // Rotate 
				startPMove = false;
				pstate = 70;
				errorState = !robot.moveAngle(rotate_angle);
				break;
			case 70:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					errorState = !robot.getDisplacement(&distance, &angle);
					//sprintf(buf,"Cal RLeft: px=%.3f.py=%.3f. prz=%.3f.prw=%.3f",px,py,prz,prw);
					//s1.assign(buf,strlen(buf));
					//publish_debug(s1);
					//robot.setVelMode();
					pstate = 0;
				}
				break;
			case 71:  // Rotate 
				startPMove = false;
				pstate = 72;
				errorState = !robot.moveAngle(-rotate_angle);
				break;
			case 72:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					//errorState = !robot.getDisplacement(&distance, &angle);
					//sprintf(buf,"Cal RRight: px=%.3f.py=%.3f. prz=%.3f.prw=%.3f",px,py,prz,prw);
					//s1.assign(buf,strlen(buf));
					//publish_debug(s1);
					//robot.setVelMode();
					pstate = 0;
				}
				break;
			case 8:  // Move Straight
				ROS_INFO("BT Move St : %.3f",move_distance);
				startPMove = false;
				pstate = 80;
				errorState = !robot.moveDistance(move_distance);
				if (errorState) {
					ROS_INFO("Error in St Move");
				}
				//ROS_INFO("Move St done");
				break;
			case 80:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					errorState = !robot.getDisplacement(&distance, &angle);
					//sprintf(buf,"Cal Forward: dist=%.3f. angle=%.3f",distance,angle);
					//sprintf(buf,"Cal Forward: px=%.3f.py=%.3f. prz=%.3f.prw=%.3f",px,py,prz,prw);
					//s1.assign(buf,strlen(buf));
					//publish_debug(s1);
					//robot.setVelMode();
					pstate = 0;
				}
				break;
			case 81:  // Move Straight
				startPMove = false;
				pstate = 82;
				errorState = !robot.moveDistance(-move_distance);
				break;
			case 82:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					errorState = !robot.getDisplacement(&distance, &angle);
					//sprintf(buf,"Cal Back: px=%.3f.py=%.3f. prz=%.3f.prw=%.3f",px,py,prz,prw);					
					//s1.assign(buf,strlen(buf));
					//publish_debug(s1);
					//robot.setVelMode();
					pstate = 0;
				}
				break;
			case 83:  // Pre/Post Move Dist				
				if ((plinear < ZDIST) && (!estop_pressed)) {
					//publish_debug("botnode : Pre/Post Move Dist");
					errorState = !robot.moveDistance(plinear);	
					if (estop_pressed) {
						errorState = true;
					}
				} 
				delay_time = ros::Time::now();	
				pstate = 84;
				break;
			case 84:  // Pre/Post Move Angle	
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {			
					pstate = 85;
					if ((pangular < ZANGLE) && (!estop_pressed)) {
						//publish_debug("botnode : Pre/Post Move Angle");
						errorState = !robot.moveAngle(pangular);	
						if (estop_pressed) {
							errorState = true;
						}					
					}
					delay_time = ros::Time::now();
					//publish_move_status(1);
					//publish_debug("Complete Pre/Post Move");
				}
				break;
			case 85:  // Pre/Post Move Angle
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					publish_move_status(1);
					//publish_debug("botnode : Complete Pre/Post Move");
					pstate = 0;
					//robot.setVelMode();
				}
				break;
			case 86:
				publish_debug("align final");
				startPMove = false;
				pstate = 0;
				//pstate = 91;				
				publish_move_status(1);
				break;
			case 9:
				//publish_debug("align final angle");
				startPMove = false;
				pstate = 0;
				//pstate = 91;
				errorState = !robot.moveAngle(alignAngle);
				if (errorState) {
					publish_debug("align error");
				} else {
					publish_debug("align done");
				}
				//robot.setVelMode();
				publish_move_status(1);
				if (estop_pressed) {
					errorState = true;
				}
				break;
			case 90:
				publish_move_status(1);
				pstate = 0;
				break;
			case 91:
				//publish_debug("align X angle");
				//startPMove = false;
				if (alignDX != 0.0) {
					pstate = 92;		
					delay_time = ros::Time::now(); 		
					errorState = !robot.moveAngle(alignX);
					if (errorState) {
						publish_debug("alignX error");
					} else {
						publish_debug("alignX angle done");
					}
				} else {
					pstate = 93;
				}
				//robot.setVelMode();
				break;
			case 100:
				pstate = 101;								
				errorState = !robot.moveAngle(alignX);
				if (errorState) {
					publish_debug("alignX error");
				} else {
					publish_debug("alignX angle done");
				}
				delay_time = ros::Time::now(); 
				//ROS_INFO("Move angle X");
				break;
			case 101:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {				
					//publish_debug("align dx");
					//startPMove = false;
					pstate = 102;
					errorState = !robot.moveDistance(alignDX);
					if (errorState) {
						publish_debug("align dx error");
					} else {
						publish_debug("align dx done");
					}
					delay_time = ros::Time::now(); 		
					//ROS_INFO("Move DX");
				}
				break;
			case 102:
				pstate = 103;								
				errorState = !robot.moveAngle(alignY);
				if (errorState) {
					publish_debug("alignY error");
				} else {
					publish_debug("alignY angle done");
				}
				delay_time = ros::Time::now(); 
				//ROS_INFO("Move angle Y");
				break;
			case 103:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {				
					//publish_debug("align dy");
					//startPMove = false;
					pstate = 9;
					errorState = !robot.moveDistance(alignDY);
					if (errorState) {
						publish_debug("align dy error");
					} else {
						publish_debug("align dy done");
					}
					//ROS_INFO("Move DY");
					//delay_time = ros::Time::now(); 		
				}
				break;
			case 110:
				pstate = 111;		
				//if ((alignX > 0.035) || (alignX < -0.035)) {			
				errorState = !robot.moveAngle(alignX);
				if (errorState) {
					publish_debug("alignX error");
				} else {
					publish_debug("alignX angle done");
				}
				//}
				delay_time = ros::Time::now(); 
				//ROS_INFO("Move angle X");
				if (estop_pressed) {
					errorState = true;
				}
				break;
			case 111:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {				
					//publish_debug("align dd");
					//startPMove = false;
					pstate = 112; //9;
					//if ((alignDX > 0.03) || (alignDX < -0.03)) {						
					errorState = !robot.moveDistance(alignDX);
					if (errorState) {
						publish_debug("align dd error");
					} else {
						publish_debug("align dd done");
					}
					//}
					delay_time = ros::Time::now(); 		
					//ROS_INFO("Move DX");
					if (estop_pressed) {
						errorState = true;
					}
				}
				break;
			case 112:
				if(ros::Time::now()-delay_time > ros::Duration(0.3)) {	
					//publish_debug("align final angle");
					startPMove = false;
					pstate = 0;
					//pstate = 91;
					errorState = !robot.moveAngle(alignAngle);
					if (errorState) {
						publish_debug("align error");
					} else {
						publish_debug("align done");
					}
					//robot.setVelMode();
					publish_move_status(1);
					if (estop_pressed) {
						errorState = true;
					}
				}
				break;
			case 92:
				//publish_debug("align dx");
				//startPMove = false;
				pstate = 93;
				errorState = !robot.moveDistance(alignDX);
				if (errorState) {
					publish_debug("align dx error");
				} else {
					publish_debug("align dx done");
				}
				//robot.setVelMode();
				break;
			case 93:
				//publish_debug("align Y angle");
				//startPMove = false;
				if (alignDY != 0.0) {
					pstate = 94;
					errorState = !robot.moveAngle(alignY);
					if (errorState) {
						publish_debug("align Y error");
					} else {
						publish_debug("align Y angle done");
					}					
				} else {
					pstate = 9;
				}
				break;
			case 94:
				//publish_debug("align dy");
				//startPMove = false;
				pstate = 9;
				errorState = !robot.moveDistance(alignDY);
				if (errorState) {
					publish_debug("align dy error");
				} else {
					publish_debug("align dy done");
				}
				//robot.setVelMode();
				break;

			case 95:
				//publish_debug("align T");
				pstate = 96;
				errorState = !robot.moveAngle(alignT);
				if (errorState) {
					publish_debug("align T error");
				} else {
					publish_debug("align T done");
				}				
				break;
			case 96:
				//publish_debug("align TD");
				pstate = 9;
				errorState = !robot.moveDistance(alignTD);
				if (errorState) {
					publish_debug("align TD error");
				} else {
					publish_debug("align TD done");
				}				
				break;
			case 97:
				delay_time = ros::Time::now();			
				pstate = 98;
				break;
			case 98:
				if(ros::Time::now()-delay_time > ros::Duration(15.0)) {
					pstate = 95;
				}
				break;
			case 1020:
				//robot.setVelMode();
				pstate = 1021;
				estopped = true;
				//errorState = !robot.moveAlign(0.0833,0.5236,3.0);
				//errorState = !robot.moveVelocity(0.5,0.2);
				ROS_INFO("Rotate Right 0.38");
				errorState = !robot.moveVelocity(0.0,0.4);	
				delay_time = ros::Time::now();	
				if (errorState) {
					publish_debug("new align 1 error");
				} else {
					publish_debug("new align 1 done");
				}		
				break;
			case 1021:
				if(ros::Time::now()-delay_time > ros::Duration(5.0)) {
					errorState = !robot.moveVelocity(0.0,-0.4);	
					ROS_INFO("Rotate Left 0.38");
					delay_time = ros::Time::now();
					pstate = 1022;
				}
				break;
			case 1022:
				if(ros::Time::now()-delay_time > ros::Duration(5.0)) {
					errorState = !robot.moveVelocity(0.0,0.4);	
					ROS_INFO("Rotate Right 0.38");
					delay_time = ros::Time::now();
					pstate = 1023;
				}
				break;
			case 1023:
				if(ros::Time::now()-delay_time > ros::Duration(5.0)) {
					errorState = !robot.moveVelocity(0.0,-0.4);	
					ROS_INFO("Stop Rotation");
					pstate = 1024;
					delay_time = ros::Time::now();
					//startPMove = false;
					//estopped = false;
				}
				break;
			case 1024:
				if(ros::Time::now()-delay_time > ros::Duration(5.0)) {
					errorState = !robot.moveVelocity(0.0,0.0);	
					ROS_INFO("Stop Rotation");
					pstate = 0;
					startPMove = false;
					estopped = false;
				}
				break;
			case 1025:
				pstate = 0;
				startPMove = false;
				errorState = !robot.moveAlign(0.0833,-0.5236,3.0);
				if (errorState) {
					publish_debug("new align 2 error");
				} else {
					publish_debug("new align 2 done");
				}		
				break;
		}
			
		switch(mcstate) {
			case 0:
				nn.getParam("navstatus",navstatus);
				if (navstatus == 7) {
					mcstate = 1;
					mc_time = ros::Time::now();
					robot.mcstate = 1; // moveVelocity
					break;
				}
				if (estop_pressed) {	
					robot.mcstate = 10; 
					mcstate = 11;
					break;
				}
				if ((joylinear_ != 0.0) || (joyangular_ != 0.0)) {
					robot.mlinear = joylinear_;
					robot.mangular = joyangular_;
					robot.mcstate = 5; // moveVelocity
					mcstate = 2;
				}
				break;
			case 1: // navigation : move_base movement
				if (estop_pressed) {	
					robot.mcstate = 10; 
					mcstate = 11;
				}
				if((ros::Time::now()-mc_time > ros::Duration(1.5)) && !checkDist) {
					//checkDist = true;
					mcstate = 10;
				}
				if(ros::Time::now()-last_cmd_time > ros::Duration(1)) {
					robot.mcstate = 10; 
					mcstate = 0;
				}
				//robot.MotorController();
				break;
			case 10:
				break;
			case 2: // joystick
				if(ros::Time::now()-last_joycmd_time > ros::Duration(1)) {
					robot.mcstate = 10; 
					mcstate = 0;
				} else {
					robot.mlinear = joylinear_;
					robot.mangular = joyangular_;
				}
				break;
			case 11: // estop activated
				if (!estop_pressed) {
					robot.mcstate = 4; 
					robot.completed = false;
					mcstate = 110;
				}				
				break;
			case 110:
				if (robot.completed) {
					mcstate = 0;
				}
				break;
		}
		robot.MotorController();
		// Take command from joystick if estopped
		if (!cmd_estopped) {
			//ROS_INFO("Move at vx [%f] & w [%f]", linear_, angular_);
			if (!estopped) {	
				if (checkDist) {	
					dx = gx - px;
					dy = gy - py;
					tdist = sqrt((dx * dx) + (dy * dy));
					nn.getParam("ProfileMoveObsDist",obsdist);
					nn.getParam("ProfileMoveObsDDist",obsddist);
					nn.getParam("LeftLaserObsDist",leftlaserobsdist);
					nn.getParam("RightLaserObsDist",rightlaserobsdist);
					nn.getParam("MapObsDist",mapobsdist);
					//ROS_INFO(" px=%.3f. py=%.3f. gx=%.3f. gy=%.3f. tdist : %.3f", px,py,gx,gy,tdist);
					/*
					if (tdist < 1.0) {
						nn.getParam("cancelDist",cancelDist);
						if ((tdist < cancelDist) && (cancelDist < 1.0) && (cancelDist > 0.0) && !navDone) {
							cancel_goal();
							ROS_INFO("*********** BTNode : Hit cancelDist = %.3f  *****************",cancelDist);
							publish_move_status(1);
							//nn.setParam("cancelMove",1);
							navDone = true;
						}
						if ((tdist < (cancelDist)) && ((cancelDist > 0.0) && (cancelDist < 1.0)) ) {	
							linear_ = linear_ * 1.2;
							if (linear_ > 0.25) {
								linear_ = 0.25;
							}
						}
					}
					*/
					//if ((tdist < slowDist) && ((cancelDist == 0.0) || (cancelDist >= 1.0)) ) {		
					if ((tdist < slowDist) ) {	
						//ROS_INFO("***** slowdist : %.3f *******", tdist);
						if ( (tdist < 0.5) ) {
							if (linear_ > 0.22) {
								linear_ = 0.22;
							}
							if (tdist < 0.1) {
								linear_ = 0.0;
								angular_ = 0.0;			
								//publish_sound(23,0,60);						
							}
							//if (tdist < 0.3) {
							//	nearLP = true;
							//	nn.setParam("nearLP",true);
							//} 

						}
						
						//ptpmoveN();
						if ((linear_ == 0.0) && (tdist < 0.22) && !reach && (navstatus == 7) ) {
						//if ((linear_ < 0.05) && (linear_ > -0.05) ) {
							//double an = fabs(calcAn());
							double fan = calcAn();
							//ROS_INFO(" AN = %.3f. Angular : %.3f. AAN : %.3f",an,angular_,fan);
							
							if ((fan > 0.0)) {								
								reachTurn = true;
							} else {
								reachTurn = false;
							}
							reach = true;
							waitflag = false;							
						}
						
						if (reach) {							
							double an = fabs(calcAn());
							linear_ = 0.0;
							//ROS_INFO(" AN : %.3f. Angular : %.3f",an, angular_);							
							if (reachTurn) {
								if (an > 1.8) {
									angular_ = 0.75;  // 0.45
								} else {
									if (an > 1.5) {
										angular_ = 0.7; 
									} else {
										if (an > 1.0) {
											angular_ = 0.6; 
										} else {
											if (an > 0.8) {
												angular_ = 0.45; 
											} else {
												if (an > 0.4) {
													angular_ = 0.35; 
												} else {
													angular_ = 0.12; 
												}
											}
										}
									}
								}
							} else {
								if (an > 1.8) {
										angular_ = -0.75;  // 0.45
									} else {
										if (an > 1.5) {
											angular_ = -0.7; 
										} else {
											if (an > 1.0) {
												angular_ = -0.6; 
											} else {
												if (an > 0.8) {
													angular_ = -0.45; 
												} else {
													if (an > 0.4) {
														angular_ = -0.35; 
													} else {
														angular_ = -0.12; 
													}
												}
											}
										}
									}
							}
							if (an < 0.03) {  // 0.06
								angular_ = 0.0;
								linear_ = 0.0;
								if (!waitflag) {
									waitTime = ros::Time::now();
									waitflag = true;
									cancel_goal();									
								}
								if (ros::Time::now()-waitTime > ros::Duration(2.5)) {
									reach = false;
								}
								//usleep(250000);
							}
							
						}
						
					} else {
						
						if (estop_pressed) {
							linear_ = linear_ * 0.0;
							angular_ = angular_ * 0.0 ;
							if (!estop_pressed_flag) { 								
								errorState = !robot.moveVelocity(linear_,angular_);	
								//ROS_INFO("BTNode estop : lin:%.3f. ang:%.3f",linear_,angular_);
								//botNode.robot.freeMotor();
								estop_pressed_flag = true;
							}
						}
					}
					
					if ( linear_ < -0.008 ){
						//if ( (obsdist < 0.35) || (mapobsdist < 0.35) || (leftlaserobsdist < 0.45) || (rightlaserobsdist < 0.45) ) {
						if ( (obsdist < 0.3) || (mapobsdist < 0.3) || (leftlaserobsdist < 0.4) || (rightlaserobsdist < 0.4) ) {
							linear_ = 0.0;
							angular_ = 0.0;							
							if ((!escape)) {	
								escape_cnt++;		
								if (escape_cnt >= 3) {
									escapeTime = ros::Time::now();
									escape = true;
									escapeNext = false;
									findpathflag = false;
									publish_sound(99,0,0);
									//ROS_INFO("Escape triggered. obsdist=%.3f. linear_=%.3f. tdist=%.3f",obsdist,linear_,tdist);
								}													
							} 
						} else {
							linear_ = 0.08; //0.03;
							//angular_ = 0.0;
						}
					}
					if (!escape && (linear_ > 0.0) && (tdist > slowDist) ) {
						if (linear_ < 0.06) {
							if ((obsdist > 0.9) && (mapobsdist > 1.0) && (leftlaserobsdist > 0.9) && (rightlaserobsdist > 0.9)){
								linear_ = 0.15; //0.15
							}
						}
					}
					if (escape) {
						linear_ = 0.0;
						//ROS_INFO("******* Escape activated ******");						
						//if (obsdist >= -0.02) { // 0.025
						if (linear_ == 0.0) {
							if (ros::Time::now()-escapeTime > ros::Duration(1.5)) {
								if (!escapeNext) {
									
									//ROS_INFO("Before  Angular_ = %.3f",angular_);
									if (!findpathflag) {
										if (findPathtoEscape()) {
											//escapeNext = true;
											//escape = false;
											//escapeTurn = false;
											//escapeNextTime = ros::Time::now();
											//usleep(300000);
											//ROS_INFO("findpath done");
											findpathflag = true;
										}
									}
									
									if (findpathflag) {
										if (checkfront()) {
											escapeNext = true;
											//escape = false;
											//escapeTurn = false;
											escapeNextTime = ros::Time::now();
											//usleep(300000);
											//ROS_INFO("checkfront done");
											findpathflag = false;
										}
									}
									
									//ROS_INFO(" After Angular_ = %.3f",angular_);
								} else {
									if (ros::Time::now()-escapeNextTime < ros::Duration(1.0)) {
										//ROS_INFO("** waiting 2.0s to exiting escape **. MapObsDist=%.3f. Obsdist=%.3f",mapobsdist,obsdist);
										if ((obsdist > 0.35) && (mapobsdist > 0.35) && (leftlaserobsdist > 0.45) && (rightlaserobsdist > 0.45)) {
											angular_ = 0.0;																					
											linear_ = 0.09;	//0.08		
										} else {											
										}
									} else {
										//ROS_INFO("******* exit escape ******");
										escape = false;
										escapeNext = false;
										linear_ = 0.15;	
										publish_sound(19,0,0);
									}
								}
							}		
							
						} else {
							angular_ = 0.0;
							if (!playsound) {
								publish_sound(14,0,0);
								playsound = true;
								playTime = ros::Time::now();
							}
							if ( ros::Time::now() > (playTime + ros::Duration(2.5)) ) {
								playsound = false;
							}
							//sleep(2);+
						}
					}
				}	
				if (!estop_pressed) { 
					errorState = !robot.moveVelocity(linear_,angular_);		
				}
				//z = sqrt(((x1-x) * (x1-x)) + ((y1-y) * (y1-y)) );
				//errorState = !robot.moveVelocity(linear_,angular_);		
			}			
		} else { 
			if (!estopped) {
				if ((joylinear_ != 0.0) || (joyangular_ != 0.0)) {
					errorState = !robot.moveVelocity(joylinear_,joyangular_);
				}
			}
		}
		if(errorState)
		{
			ROS_INFO("moveVelocity failed in errorState.");
			linear_ = angular_ = 0;
			joylinear_ = joyangular_ = 0;
			continue;
		}
		if (navstatus == 0) {
			hstate = 0;
			reach = false;
			navDone = false;
			linear_ = 0.0;
			angular_ = 0.0;
		}
		publish_speed(linear_, angular_);
		ros::spinOnce();
		r.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_base");
	ros::NodeHandle n;
	BotNode botNode;
	int readyflag;
	bool res;
	char buf [100];
	double x,y,x1,y1,z;
	int cnt;
	
	readyflag = 0;
	n.setParam("RobotReady",readyflag);	
	
	botNode.robot.start();
	
	//botNode.robot.changebaudrate(); // change if changebaudrate > 0
	while(ros::ok() && !botNode.robot.reset())
	{
		ROS_WARN("EPOS not ready. Trying again in 1 second....");
		//sleep(1);
		usleep(500000);
	}
	
	
	readyflag = 77;
	n.setParam("nearLP",false);
	n.setParam("RobotReady",readyflag);	// to let user know robot is ready
	n.getParam("RobotReady",readyflag);
	ROS_INFO("Botnode : RobotReady = %d",readyflag);
	botNode.initMove();
	botNode.robot.moveVelocity(0.0,0.0);
	//ROS_INFO("Bot Node : readyflag : %d",readyflag);
	//botNode.publish_toggleButton(ROBOTREADY_UNO);
	//ros::Timer timer = n.createTimer(ros::Duration(1.0/botNode.rate), boost::bind(&BotNode::watchdog, &botNode));
	//ros::Timer timer = n.createTimer(ros::Duration(0.2), boost::bind(&BotNode::watchdog, &botNode));
	boost::thread main_thread(boost::bind(&BotNode::mainLoop, &botNode));
	main_thread.interrupt() ;
	main_thread.join() ;
	//sprintf(buf,"firefox -new-window \"http://192.1.32.32/robot.html\"");
	//system(buf);	
	return 0;
}
