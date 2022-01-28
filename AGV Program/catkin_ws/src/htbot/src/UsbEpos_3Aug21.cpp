/*
 * Node that listens to cmd_vel and cmd_pos msgs
 * and publishes the robot odometry
 * last update : 26.4.21 3.00pm
                 3.5.21 : 1pm
								 4.5.21 : 4.45pm
								 5.5.21 : 1.20pm
								 10.5.21 : 12.20am
								 10.5.21 : 4pm
								 20.5.21 : 1.30pm
								 21.5.21 : 9.20am
								 24.5.21 : 10.45am
								 10.6.21 : 10.45am
								 14.6.21 : 13.50pm
								 18.6.21 : 4pm
								 22.6.21 : 3pm
	25.6.21 : 10.45am
		1. added 2nd left leg alignment
	6.7.21 : 11am
		1. change moveing into trolley with edge detection
	7.7.21 : 4.15pm
		1. add another moveAGVintoTrolleyVision 
	8.7.21 : 10am
		1. create 2 move into trolley. moveAGVintoTrolleyFront and moveAGVintoTrolleyLeft
	12.7.21 : 11.15am
		1. add adjustAGVVision to do fine adjustment using vision
	16.7.21
		1. changed the slowing down to reach destination (3.15pm)
		2. introduce new variable flinear and fangular
	19.7.21 
		1. 10am : adjusted agv vision algorithm with offset
		2. 10.50am : set max angle to turn to path at 30.0

 * Changes :
    19.5.21 : 2pm 
      1. reduce reaching destination speed
      2. trolley alignment limited to 20 attempts
		20.5.21 : 1.30pm
      1. try to move agv into trolley using moveDistance
			2. added alignment to left side of trolley
		21.5.21 : 
		 9.20am : 1.change turntopath only for angle greater than 45.0
              2.change trolleylen and width
		24.5.21 : 10.45am
			1. re-align again after agv move into trolley edge
		10.6.21 : 10.45am
      1. change and include slowRation to slow down AGV reaching destination
		14.6.21 : 13.50pm
			1. added maxVelocity to control max velocity for motor
		18.6.21 : 4pm
			1. added moveDistanceSpeed
		22.6.21 : 3pm
			1. adjusted the move-in distance to pick trolley
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

//#include "htbot/bot2wheel_driver.h"
#include "htbot/sound.h"
#include "htbot/status.h"
#include "htbot/clear.h"
#include "htbot/move_status.h"
#include "htbot/debug.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "htbot/move.h"
#include "htbot/odom.h"
#include "htbot/angle.h"
#include "htbot/goal.h"
#include "htbot/stat_speed.h"
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include "htbot/PLAYSOUND.h"
#include "htbot/MaxonEPOS.h"

using namespace std;
//using namespace boost::algorithm;

#define ROBOTREADY_UNO 70
#define ZDIST 2.0
#define ZANGLE 190.0
#define PI 3.141593
#define PI2 6.28319
#define PI_2 1.5707965
#define MAXLP 20
#define MAXPP 50
#define AGVLENHALF 0.384
#define FRONTLASERDIST 0.4 //0.384
#define CAMERAOFFSET 0.015
#define MAXANGLETURN 30.0

typedef int BOOL;

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

class BotNode
{
public:
	BotNode();
	//Bot2Wheel robot;
	double cur_x,cur_y,cur_theta;
	double linear_, angular_, joylinear_, joyangular_,cmdvel_offset;
	double maxjoylinear_,maxjoyangular_;
	double flinear,fangular;
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
	void publish_event(string s);
	void publish_clear(void);
	void publish_move_status(int stat);
	void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
	void moveCallback(const htbot::move::ConstPtr& msg);
	void goalCallback(const htbot::goal::ConstPtr& msg);
	void goalLPCallback(const htbot::move::ConstPtr& msg);
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
	void checkpath(void);
	void turnToPath(double tx, double ty);
	void turnToPathN(double tx, double ty);
	void pid_angle();
	void test_pid_angle(int slp, int elp, int pp);
	void pid_dist();
	void calc_dist_error();
	void pid_angle_dist();
	double pid_checkangle_NLP();
	void readPathfromFile();
	void turnToOrientation();
	void racplanMovealignRobot();
	void moveAGVintoTrolley(double trolleydist,double trolley);
	void oldmoveAGVintoTrolley(double trolleydist,double trolley);
	void computeTrollleyAlignment(int legcnt);
	void moveAGVintoTrolleyFront(double trolleydist,double trolley);
	void moveAGVintoTrolleyLeft(double trolleydist,double trolley);
	void adjustAGVVision(void);
	void ptpmoveAlign();

	bool playsound,alignDistReached,alignAngleReached;
	bool negativecurrent;
	
	// USB-Canopen
	void  SetDefaultParameters();
	int 	OpenDevice(unsigned int* p_pErrorCode);
	void 	MotorSetup();
	void 	setParams();	
	void  setVelocityMode();
	void 	getPosition(void);
	void 	moveVelocity(double linear,double angular);
	void  moveVel(double left,double right);
	void 	moveAngle(double angle);
	bool 	checkTragetReached();
	void 	quickStopMotor(void);
	void 	setProfilePositionMove(void);
	void 	moveDistance(double dist);
	void 	moveDistanceSpeed(double dist,int velocity);
	void 	freeMotor(void);
	void 	getDisplacement(double* distance, double* angle);

	//ntuc
	void publish_fButton(unsigned short cmd);
		
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
	ros::Subscriber goalLP_sub;
	ros::Publisher status_pub;
	ros::Publisher chkplan_pub;
	ros::Publisher fButton_pub;
	ros::Publisher vision_pub;

	void publish_toggleButton(unsigned short cmd);
	int navstatus;
	int hstate,pstate;
	bool stop,estop_pressed,estop_pressed_flag;
	int count,pcount,ocount;
	bool startPMove,escape,escapeNext,escapeTurn,escapePos;;
	double fspeedtime;
	double rotate_angle,move_distance;
	double gx,gy,gz,grx,gry,grz,grw,px,py,pz,prx,pry,prz,prw,targetDist,slowDist,slowRatio,alignAngle,alignDist,frontObsslowRatio;
	double alignX,alignY,alignDX,alignDY;
	double alignAX,alignAY,alignT,alignTD;
	double dockdist,dockspeed,docksspeed,dockrdist,frontobsdist,dockrangle;
	bool checkDist;
	double oddist, odangle;  // odom
	//int profile_move_flag;
	bool estop,reach,reachTurn,checkfrontflag,findpathflag,waitflag,findnewpath;
	int escape_cnt,amclcnt;
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
	double pathInfo[MAXLP][MAXLP][MAXPP][7];
	int pathNoPoint[MAXLP][MAXLP];		
	int cmovePtr,emovePtr;
	bool multiptMove,rotateFirst,stopFlag,checkPlanDone,pauseRobotMove;
	int pointInfo;
	double checkPlanSum;
	ros::Time stopTime;
	ros::ServiceClient makeplan;
	double turnTopathX,turnTopathY;
	double Angle_Err_Value,i_Temp,d_Temp;
	double P_Term,I_Term,D_Term,Kd,Kp,Ki,AKp,AKd;
	double Dist_Err_Value,i_DTemp,d_DTemp;
	double P_DTerm,I_DTerm,D_DTerm,DKd,DKp,DKi;
	int qtrum,nextPoint,delay_count;
	bool Cancel_PID,lastPoint,alignflag,multiLP;
	double angle_rot;
	std::string pathfile;
	int pathmaxpoint,startLP,endLP;
	double maxautolinear,maxautoangular;
	int manualvelflag; // 0=stop. 1=forward. 2=reverse. 3=rotate_left. 4=rotate_right
	bool racplanMove,slowzone;
	double slowzonelinear;
	double racplanMoveX,racplanMoveY;
	double trolleywidth,trolley,maxVelocity;
	
	
	// USB-Canopen
	void* g_pKey;
	unsigned short RMOTOR;  // gateway node. g_pNode
	unsigned short LMOTOR;  // sub node. g_sNode
	string g_deviceName;
	string g_subdeviceName;
	string g_protocolStackName;
	string g_subprotocolStackName;
	string g_interfaceName;
	string g_portName;
	int g_baudrate;
	unsigned int eposErrorCode;
	bool quickstop;
	double dist_factor;
	double mm_per_count;
	double rpm_factor;
	double radpm_factor;
	double angle_factor;
	double displacement_factor;
	int AXLE_LEN; // in mm
	int WHEEL_DIA; // in mm
	int GEAR_RATIO;
	int STEPS_PER_REV; // in quad-counts
	double wheel_circum_correction;
	double wheel_base_correction;
	double odom_angular_scale_correction,dDist;
	int CurrentRegulatorPGain;
	int CurrentRegulatorIGain;
	int VelocityPGain;
	int VelocityIGain;
	int PositionPGain;
	int PositionIGain;
	int PositionDGain;
	int ProfileAcceleration;
	int ProfileDeceleration;
	int ProfileVelocity;
	int MaxProfileVelocity;
	int MaxProfileVelocityMD;
	int MaxFollowError;
	int PositionWindow;
	int PositionWindowTime;
	int MotorMaxContinuousCurrent;
	int ThermalTimeConstantWinding;
	int LeftMotorActualPosition,RightMotorActualPosition;
	int prev_Lpos, prev_Rpos;

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
	ros::Publisher event_pub;
	ros::Publisher toggleButton_pub;
	ros::Subscriber button_sub;
	ros::Publisher velstat_pub;
	ros::Subscriber curr_sub;
	ros::Publisher cancel_pub;

	ros::ServiceServer reset_odom_srv;
	ros::Time current_time, last_time, delay_time;
	boost::mutex publish_mutex_;
};

BotNode::BotNode():
	cur_x(0.0),cur_y(0.0),cur_theta(0.0),cmdvel_offset(1.0),startPMove(false),docking(false),estop_pressed_flag(false),
	linear_(0.0),angular_(0.0), joylinear_(0.0),joyangular_(0.0),navstatus(0),stop(false),estop_pressed(false),
	errorState(false), cmd_estopped(false), estopped(false), resetting(false),hstate(0),pstate(0),reach(false),
	alignAngle(0.0),checkDist(false),escape(false),escapeNext(false),escapeTurn(false),ocount(0), ph("~"),
	sprz(9.0),sprw(9.0),checkfrontflag(false),findpathflag(false),playsound(false),cancelDist(0.0),ptpFlag(false),
	leftlaserobsdist(2.5),rightlaserobsdist(2.5),mapobsdist(2.5),nearLP(false),navDone(false),findnewpath(false),
	negativecurrent(false),amclcnt(0),i_Temp(0.0),d_Temp(0.0),Angle_Err_Value(0.0),lastPoint(false),
  alignflag(false),multiLP(false),maxautolinear(0.5),maxautoangular(0.5),maxjoyangular_(0.6),maxjoylinear_(0.6),
	pauseRobotMove(false),manualvelflag(0),maxVelocity(0.6)
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
	event_pub = nh.advertise<htbot::debug>("event",100);
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
	goalLP_sub = nh.subscribe<htbot::move>("goalLP", 1, &BotNode::goalLPCallback,this);  
	makeplan = nh.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
	fButton_pub = nh.advertise<std_msgs::UInt16>("fbutton",1);
	vision_pub = nh.advertise<std_msgs::Int16>("vision",1);
	
	nh.param("laptop_min_charge",laptop_min_charge, 10);
	nh.param("odom_pub_rate", rate, 15.0);
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
	nh.param("maxVelocity",maxVelocity,0.65);

	//ph.param("cov_x",cov_x, 1e-3);
	//ph.param("cov_y",cov_y, 1e-3);
	//ph.param("cov_th",cov_th, 1e-3);
	//ph.param("cov_vx",cov_vx, 1e-3);
	//ph.param("cov_vy",cov_vy, 1e-3);
	//ph.param("cov_vth",cov_vth, 1e-3);
	ph.param("cov_x",cov_x,0.22);
	ph.param("cov_y",cov_y,0.22);
	ph.param("cov_th",cov_th,0.22);
	ph.param("cov_vx",cov_vx,0.22);
	ph.param("cov_vy",cov_vy,0.22);
	ph.param("cov_vth",cov_vth,0.22);

	ph.param("AXLE_LEN",AXLE_LEN,0);
	ph.param("WHEEL_DIA",WHEEL_DIA,0);
	ph.param("GEAR_RATIO",GEAR_RATIO,0);
	ph.param("STEPS_PER_REV",STEPS_PER_REV,0);
	ph.param("wheel_circum_correction",wheel_circum_correction,(double)1.0);
	ph.param("wheel_base_correction",wheel_base_correction,(double)1.0);
	ph.param("odom_angular_scale_correction",odom_angular_scale_correction,(double)1.0);

	ph.param("CurrentRegulatorPGain",CurrentRegulatorPGain,0);
	ph.param("CurrentRegulatorIGain",CurrentRegulatorIGain,0);
	ph.param("VelocityPGain",VelocityPGain,0);
	ph.param("VelocityIGain",VelocityIGain,0);
	ph.param("PositionPGain",PositionPGain,0);
	ph.param("PositionIGain",PositionIGain,0);
	ph.param("PositionDGain",PositionDGain,0);

	ph.param("PositionProfileAcceleration",ProfileAcceleration,0);
	ph.param("PositionProfileDeceleration",ProfileDeceleration,0);
	ph.param("PositionProfileVelocity",ProfileVelocity,0);
	ph.param("MaxProfileVelocity",MaxProfileVelocity,0);
	ph.param("MaxProfileVelocityMD",MaxProfileVelocityMD,0);
	ph.param("MaxFollowError",MaxFollowError,0);
	ph.param("PositionWindow",PositionWindow,0);
	ph.param("PositionWindowTime",PositionWindowTime,0);

	ph.param("MotorMaxContinuousCurrent",MotorMaxContinuousCurrent,0);
	ph.param("ThermalTimeConstantWinding",ThermalTimeConstantWinding,0);

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

	nh.getParam("path_file",pathfile);

	current_time = last_time = last_cmd_time = last_joycmd_time = ros::Time::now();
	//ROS_INFO("cov_vth : %.4f",cov_vth);
} // BotNode constructor


void BotNode::adjustAGVVision(void) {
	ros::NodeHandle n;
	double vdx,vdy,vda;
	int vision_valid;
	std_msgs::Int16 msg;
	
	//return;
	// trigger camera scan 
	vision_valid = 0;
	n.setParam("vision_valid",0);
	usleep(100000);
	msg.data = 1;
	vision_pub.publish(msg);
	usleep(100000);

	for(int i=0;i<20;i++) {
		n.getParam("vision_valid",vision_valid);
		if (vision_valid == 1) {
			ROS_INFO("--------- adjustAGVVision : Detected Tag ------------");
			break;
		}
	}
	if ((vision_valid == -1) || (vision_valid == 0)) {
		ROS_INFO("--------- adjustAGVVision : Failed to Detect Tag ------------");
		return;
	}
	n.getParam("vision_angle",vda);
	n.getParam("vision_dx",vdx);
	n.getParam("vision_dy",vdy);
	ROS_INFO("--------- Vision : vda=%.3f. vdx=%.3f. vdy=%.3f. ---------",vda,vdx,vdy);
	if (fabs(vda) > 0.3) {
		ROS_INFO("--------- adjustAGVVision : Align Angle ------------");
		moveAngle(vda);
	}

	// trigger camera scan 
	vision_valid = 0;
	n.setParam("vision_valid",0);
	usleep(100000);
	msg.data = 1;
	vision_pub.publish(msg);
	usleep(100000);

	for(int i=0;i<20;i++) {
		n.getParam("vision_valid",vision_valid);
		if (vision_valid == 1) {
			ROS_INFO("--------- adjustAGVVision : 2nd :Detected Tag ------------");
			break;
		}
	}
	if ((vision_valid == -1) || (vision_valid == 0)) {
		ROS_INFO("--------- adjustAGVVision : 2nd :Fail to Detect Tag ------------");
		return;
	}
	n.getParam("vision_angle",vda);
	n.getParam("vision_dx",vdx);
	n.getParam("vision_dy",vdy);
	ROS_INFO("--------- Vision : 2nd : vda=%.3f. vdx=%.3f. vdy=%.3f. ---------",vda,vdx,vdy);
	vdx = vdx + CAMERAOFFSET;
	usleep(100000);
	if (fabs(vdx) > 0.004) {
		//moveDistance(vdx+0.015);
		moveDistance(vdx);
		ROS_INFO("--------- adjustAGVVision : Align Dist ------------");
		usleep(100000);
	}
	ROS_INFO("--------- adjustAGVVision : Completed ------------");	
}


void BotNode::oldmoveAGVintoTrolley(double trolleywidth,double trolley) {
	ros::NodeHandle n;
	int type,cnt,legcnt;
	double BOXW,BOXL,TROLLEYLEN,TROLLEYWIDTH;
	double vl,vr,vd,vb,ant,rra,hh,dxx,xx,dra,dan,turnangle;
	bool RightLeg,LeftLeg,RightZone,LeftZone,TrolleyAlign;
	int RightLegIdx,LeftLegIdx;
	double RightLegdist,RightLegAngle,LeftLegdist,LeftLegAngle;
	double agvx,agvy,dxxx;

	TrolleyAlign = false;
	ROS_INFO("-------------- moveAGVintoTrolley -------------------------");
	//return;
	type = (int) trolley;
	switch (type) {
		case 1: // type 1
			BOXW = 0.165;
			BOXL = 0.9;
			TROLLEYLEN = 1.07;
			TROLLEYWIDTH = 0.67;
			break;
		case 2:
			BOXW = 0.165;
			BOXL = 0.9;
			TROLLEYLEN = 1.07;
			TROLLEYWIDTH = 0.67;
			break;
		case 3:
			BOXW = 0.165;
			BOXL = 0.9;
			break;
	}
	
	n.setParam("TrolleyAlign",TrolleyAlign);
	//n.setParam("RESTARTDETECTLEGFRONT",true);
	n.setParam("STARTDETECTLEGFRONT",true); // initiate left and right laser detection of the legs
	usleep(100000);
	ROS_INFO("------- moveAGVintoTrolley : Start Front Laser Alignment -----------");
	cnt = 0;
	legcnt = 0;
	while (true) {
		n.getParam("TrolleyAlign",TrolleyAlign);
		if (TrolleyAlign) {			
			ROS_INFO("----------- TrolleyAlign : Done --------------- ");
			break;
		}
		cnt++;
		if (cnt > 50) {
			n.setParam("STARTDETECTLEGFRONT",false);
			ROS_INFO("------------ Epos : moveAGVintoTrolley : Trolley Alignment Failed. --------------------");
			return;
		}
		usleep(200000);
	} 
	n.setParam("STARTDETECTLEGFRONT",false);		
	n.getParam("TrolleyAlignCount",legcnt);
	computeTrollleyAlignment(legcnt);

	n.getParam("RightLegdist",RightLegdist);
	n.getParam("RightLegAngle",RightLegAngle);
	n.getParam("LeftLegdist",LeftLegdist);
	n.getParam("LeftLegAngle",LeftLegAngle);
	
	// calculate alginment : dx
	ant = RightLegAngle + LeftLegAngle;
	rra = asin((LeftLegdist * sin(ant)) / TROLLEYLEN);
	hh = ((LeftLegdist * sin(ant)) / TROLLEYLEN) * RightLegdist;
	xx = hh / tan(rra);
	dxx = (TROLLEYLEN / 2.0) - xx;
	dra = atan(xx / hh);
	dan = RightLegAngle - dra;
	agvx = FRONTLASERDIST * sin(fabs(dan));
	agvy = FRONTLASERDIST * cos(fabs(dan));
	turnangle = 90.0 + (dan * 57.2958);
	if (dan > 0.0) {
		dxxx = dxx + agvx;
	} else {
		dxxx = dxx - agvx;
	}
	ROS_INFO("----- ant=%3f. rra=%.3f. hh=%.3f. xx=%.3f. dxx=%.3f -----------",ant,rra,hh,xx,dxx);
	ROS_INFO("----- dra=%3f. dan=%.3f. -----------",dra,dan);  
	ROS_INFO("----- Turn=%3f. dxxx=%.3f. dan=%.3f. agvx=%.3f. agvy=%.3f -----------",turnangle,dxxx,dan,agvx,agvy); 

	if ((fabs(dxxx) > 0.015)) {		
		moveAngle(-turnangle);
		usleep(600000);
		//moveDistance(fabs(dxxx));
		moveDistance(-dxxx);
		usleep(600000);
		moveAngle(90.0);
		//ROS_INFO("--------- moveAGVintoTrolley : Adjust. Turn=%3f. Move=%.3f -----------",turnangle,fabs(dxxx)); 
	} else {
		if (fabs(dan) > 0.015) {
			turnangle = - (dan * 57.2958);
			moveAngle(turnangle);
			ROS_INFO("--------- moveAGVintoTrolley : Adjust. Turn=%3f. -----------",turnangle); 
		} else {
			ROS_INFO("--------- moveAGVintoTrolley : No Adjustment needed. dxxx=%.3f. dan=%.3f -----------",dxxx,dan);
		}
	}	
	ROS_INFO("------- moveAGVintoTrolley : End  Front Laser Alignment -----------");
	// move into trolley using moveDistance
	n.setParam("STARTDETECTLEG",false);
	usleep(500000);
	//moveDistance(hh+agvy+(TROLLEYWIDTH / 2.0));		
	moveDistanceSpeed(hh+agvy-0.18,750);		//-0.03
	//usleep(500000);
	usleep(500000);

	// ******* 2nd align again with front trolley legs ************

	TrolleyAlign = false;
	n.setParam("TrolleyAlign",TrolleyAlign);
	n.setParam("STARTDETECTLEGFRONT",true); // initiate left and right laser detection of the legs
	usleep(100000);
	ROS_INFO("------- moveAGVintoTrolley : Start Trolley 2nd front leg Laser Alignment -----------");
	cnt = 0;
	while (true) {
		n.getParam("TrolleyAlign",TrolleyAlign);
		if (TrolleyAlign) {
			ROS_INFO("----------- TrolleyAlign : Done --------------- ");
			break;
		}
		cnt++;
		if (cnt > 50) {
			n.setParam("STARTDETECTLEGFRONT",false);
			ROS_INFO("------------ Epos : moveAGVintoTrolley : 2nd Trolley Alignment Failed. --------------------");
			return;
		}
		usleep(200000);
	}
	n.setParam("STARTDETECTLEGFRONT",false);
	n.getParam("TrolleyAlignCount",legcnt);
	computeTrollleyAlignment(legcnt);
	n.getParam("RightLegdist",RightLegdist);
	n.getParam("RightLegAngle",RightLegAngle);
	n.getParam("LeftLegdist",LeftLegdist);
	n.getParam("LeftLegAngle",LeftLegAngle);

	// calculate alginment : dx
	ant = RightLegAngle + LeftLegAngle;
	rra = asin((LeftLegdist * sin(ant)) / TROLLEYLEN);
	hh = ((LeftLegdist * sin(ant)) / TROLLEYLEN) * RightLegdist;
	xx = hh / tan(rra);
	dxx = (TROLLEYLEN / 2.0) - xx;
	dra = atan(xx / hh);
	dan = RightLegAngle - dra;
	agvx = FRONTLASERDIST * sin(fabs(dan));
	agvy = FRONTLASERDIST * cos(fabs(dan));
	turnangle = 90.0 + (dan * 57.2958);
	if (dan > 0.0) {
		dxxx = dxx + agvx;
	} else {
		dxxx = dxx - agvx;
	}
	ROS_INFO("-----2nd frontalign : ant=%3f. rra=%.3f. hh=%.3f. xx=%.3f. dxx=%.3f -----------",ant,rra,hh,xx,dxx);
	ROS_INFO("-----2nd frontalign : dra=%3f. dan=%.3f. -----------",dra,dan);  
	ROS_INFO("-----2nd frontalign : Turn=%3f. dxxx=%.3f. dan=%.3f. agvx=%.3f -----------",turnangle,dxxx,dan,agvx); 
	if (fabs(dan) > 0.015) {
		turnangle = - (dan * 57.2958);
		moveAngle(turnangle);
		ROS_INFO("--------- 2nd frontalign : : Adjust. Turn=%3f. -----------",turnangle); 
	} else {
		ROS_INFO("--------- 2nd frontalign : : No Adjustment needed. dxxx=%.3f. dan=%.3f -----------",dxxx,dan);
	}
	dxxx = (hh+agvy+0.018) - (TROLLEYWIDTH / 2.0); //0.015
	if (fabs(dxxx) > 0.015) {
		ROS_INFO("--------- 2nd frontalign  : Adjust. Move=%3f. -----------",dxxx);
		moveDistance(dxxx);		
	} else {
		ROS_INFO("--------- 2nd frontalign  : No Adjust. Move=%3f. -----------",dxxx);
	}
	sleep(1);
	moveAngle(90.0);

	// ******* align to left trolley legs ************
	TrolleyAlign = false;
	n.setParam("TrolleyAlign",TrolleyAlign);
	n.setParam("STARTDETECTLEGFRONT",true); // initiate left and right laser detection of the legs
	usleep(100000);
	ROS_INFO("------- moveAGVintoTrolley : Start Trolley left Side Laser Alignment -----------");
	cnt = 0;
	while (true) {
		n.getParam("TrolleyAlign",TrolleyAlign);
		if (TrolleyAlign) {
			ROS_INFO("----------- TrolleyAlign : Done --------------- ");
			break;
		}
		cnt++;
		if (cnt > 30) {
			n.setParam("STARTDETECTLEGFRONT",false);
			ROS_INFO("------------ Epos : moveAGVintoTrolley : Trolley Alignment Failed. --------------------");
			return;
		}
		usleep(200000);
	}
	n.setParam("STARTDETECTLEGFRONT",false);
	n.getParam("TrolleyAlignCount",legcnt);
	computeTrollleyAlignment(legcnt);
	n.getParam("RightLegdist",RightLegdist);
	n.getParam("RightLegAngle",RightLegAngle);
	n.getParam("LeftLegdist",LeftLegdist);
	n.getParam("LeftLegAngle",LeftLegAngle);

	// calculate alginment : dx
	ant = RightLegAngle + LeftLegAngle;
	rra = asin((LeftLegdist * sin(ant)) / TROLLEYWIDTH);
	hh = ((LeftLegdist * sin(ant)) / TROLLEYWIDTH) * RightLegdist;
	xx = hh / tan(rra);
	dxx = (TROLLEYWIDTH / 2.0) - xx;
	dra = atan(xx / hh);
	dan = RightLegAngle - dra;
	agvx = FRONTLASERDIST * sin(fabs(dan));
	agvy = FRONTLASERDIST * cos(fabs(dan));
	turnangle = 90.0 + (dan * 57.2958);
	if (dan > 0.0) {
		dxxx = dxx + agvx;
	} else {
		dxxx = dxx - agvx;
	}
	ROS_INFO("----- ant=%3f. rra=%.3f. hh=%.3f. xx=%.3f. dxx=%.3f -----------",ant,rra,hh,xx,dxx);
	ROS_INFO("----- dra=%3f. dan=%.3f. -----------",dra,dan);  
	ROS_INFO("----- Turn=%3f. dxxx=%.3f. dan=%.3f. agvx=%.3f -----------",turnangle,dxxx,dan,agvx); 
	if (fabs(dan) > 0.015) {
		turnangle = - (dan * 57.2958);
		moveAngle(turnangle);
		ROS_INFO("--------- LeftSide : Adjust. Turn=%3f. -----------",turnangle); 
	} else {
		ROS_INFO("--------- LeftSide : No Adjustment needed. dxxx=%.3f. dan=%.3f -----------",dxxx,dan);
	}
	// adjust center distance
	dxxx = (hh+agvy+0.02) - (TROLLEYLEN / 2.0);
	if (fabs(dxxx) > 0.015) {
		ROS_INFO("--------- LeftSide : Adjust. Move=%3f. -----------",dxxx);
		moveDistance(dxxx);		
	} else {
		ROS_INFO("--------- LeftSide : No Adjust. Move=%3f. -----------",dxxx);
	}
	
	ROS_INFO("------------ UsbEpos : Finish moveAGVintoTrolley ------------------");	

}

void BotNode::computeTrollleyAlignment(int legcnt) {
	FILE *pfp;
	ros::NodeHandle n;
	double angle,dist;
	double objdataRange[10];
	double objdataAngle[10];
	double x[10];
	double y[10];
	double d1,a1,d2,a2,d3,a3;
	int k;
	bool ok;
	double vl,vr,vd,vb,ant,rra,hh,dxx,xx,dra,dan,turnangle;
	double RightLegdist,RightLegAngle,LeftLegdist,LeftLegAngle;
	double agvx,agvy,dxxx;
	
	pfp = fopen("/home/rac/catkin_ws/src/htbot/data/trolley.txt", "r");
  if (pfp == NULL) {
  	ROS_INFO("----- computeTrollleyAlignment : I couldn't open trolley.txt for reading. ---------\n");    		
  	return;
  }
	k = 0;
	for (int i=0;i<legcnt;i++) {
		if (fscanf(pfp,"%lf %lf\n",&angle,&dist) == EOF) {
			ROS_INFO("---------- computeTrollleyAlignment : End of Trolley.txt ------------");
			break;
		}
		objdataRange[k] = dist;
		objdataAngle[k] = angle;
		x[k] = dist * cos(angle);
		y[k] = dist * sin(angle);
		k++;
	}
	ok = false;
	if (k >= 2) {
		d1 = objdataRange[0] + 0.028;
		a1 = objdataAngle[0];
		for (int n=1;n<k;n++) {
			d2 = objdataRange[n] + 0.028;
			a2 = objdataAngle[n];
			if (fabs(d1 - d2) < 0.12) {
				ok = true;
				break;
			}
		}
	}
	if (!ok) {
		ROS_INFO("------------ computeTrollleyAlignment : Fail to align ------------");
		return;
	}
	RightLegdist = d1;
	RightLegAngle = PI_2 - a1;
	LeftLegdist = d2;
	LeftLegAngle = a2 - PI_2;

	n.setParam("RightLegdist",RightLegdist);
	n.setParam("RightLegAngle",RightLegAngle);
	n.setParam("LeftLegdist",LeftLegdist);
	n.setParam("LeftLegAngle",LeftLegAngle);

	ROS_INFO("------- computeTrollleyAlignment : End   -----------");
}


void BotNode::moveAGVintoTrolleyFront(double trolleywidth,double trolley) {
	ros::NodeHandle n;
	int type,cnt;
	double BOXW,BOXL,TROLLEYLEN,TROLLEYWIDTH;
	double vl,vr,vd,vb,ant,rra,hh,dxx,xx,dra,dan,turnangle;
	bool RightLeg,LeftLeg,RightZone,LeftZone,TrolleyAlign;
	int RightLegIdx,LeftLegIdx;
	double RightLegdist,RightLegAngle,LeftLegdist,LeftLegAngle;
	double agvx,agvy,dxxx;

	RightLeg = false;
	LeftLeg = false;
	RightZone = false;
	LeftZone = false;
	TrolleyAlign = false;
	ROS_INFO("-------------- moveAGVintoTrolleyFront -------------------------");
	//return;
	type = (int) trolley;
	switch (type) {
		case 1: // type 1
			TROLLEYLEN = 1.07;
			TROLLEYWIDTH = 0.67;
			break;
		case 2:
			TROLLEYLEN = 1.07;
			TROLLEYWIDTH = 0.67;
			break;
		case 3:
			BOXW = 0.165;
			BOXL = 0.9;
			break;
	}
	
	n.setParam("TrolleyAlign",TrolleyAlign);
	n.setParam("RESTARTDETECTLEGFRONT",true);
	n.setParam("LEFTLEGALIGN",false);
	n.setParam("STARTDETECTLEGFRONT",true); // initiate left and right laser detection of the legs
	usleep(100000);
	ROS_INFO("------- moveAGVintoTrolleyFront : Start Front Laser Alignment -----------");
	cnt = 0;
	while (true) {
		n.getParam("TrolleyAlign",TrolleyAlign);
		if (TrolleyAlign) {
			n.getParam("RightLegIdx",RightLegIdx);
			n.getParam("RightLegdist",RightLegdist);
			n.getParam("RightLegAngle",RightLegAngle);
			n.getParam("LeftLegIdx",LeftLegIdx);
			n.getParam("LeftLegdist",LeftLegdist);
			n.getParam("LeftLegAngle",LeftLegAngle);
			ROS_INFO("--- TrolleyAlign : rlidx=%d. rldist=%.3f. rlAn=%.3f. llidx=%d. lldist=%.3f. llAn=%.3f. ---",RightLegIdx,\
									RightLegdist,RightLegAngle,LeftLegIdx,LeftLegdist,LeftLegAngle);
			break;
		}
		cnt++;
		if (cnt > 50) {
			n.setParam("STARTDETECTLEGFRONT",false);
			ROS_INFO("------------ Epos : moveAGVintoTrolleyFront : Trolley Alignment Failed. --------------------");
			return;
		}
		usleep(200000);
	} 
	n.setParam("STARTDETECTLEGFRONT",false);
	
	// calculate alginment : dx
	ant = RightLegAngle + LeftLegAngle;
	rra = asin((LeftLegdist * sin(ant)) / TROLLEYLEN);
	hh = ((LeftLegdist * sin(ant)) / TROLLEYLEN) * RightLegdist;
	xx = hh / tan(rra);
	dxx = (TROLLEYLEN / 2.0) - xx;
	dra = atan(xx / hh);
	dan = RightLegAngle - dra;
	agvx = FRONTLASERDIST * sin(fabs(dan));
	agvy = FRONTLASERDIST * cos(fabs(dan));
	turnangle = 90.0 + (dan * 57.2958);
	if (dan > 0.0) {
		dxxx = dxx + agvx;
	} else {
		dxxx = dxx - agvx;
	}
	ROS_INFO("----- ant=%3f. rra=%.3f. hh=%.3f. xx=%.3f. dxx=%.3f -----------",ant,rra,hh,xx,dxx);
	ROS_INFO("----- dra=%3f. dan=%.3f. -----------",dra,dan);  
	ROS_INFO("----- Turn=%3f. dxxx=%.3f. dan=%.3f. agvx=%.3f. agvy=%.3f -----------",turnangle,dxxx,dan,agvx,agvy); 

	if ((fabs(dxxx) > 0.02)) {		
		moveAngle(-turnangle);
		usleep(600000);
		//moveDistance(fabs(dxxx));
		moveDistance(-dxxx);
		usleep(600000);
		moveAngle(90.0);
		//ROS_INFO("--------- moveAGVintoTrolleyVision : Adjust. Turn=%3f. Move=%.3f -----------",turnangle,fabs(dxxx)); 
	} else {
		if (fabs(dan) > 0.02) {
			turnangle = - (dan * 57.2958);
			moveAngle(turnangle);
			ROS_INFO("--------- moveAGVintoTrolleyFront : Adjust. Turn=%3f. -----------",turnangle); 
		} else {
			ROS_INFO("--------- moveAGVintoTrolleyFront : No Adjustment needed. dxxx=%.3f. dan=%.3f -----------",dxxx,dan);
		}
	}	
	ROS_INFO("------- moveAGVintoTrolleyFront : End  Front Laser Alignment -----------");
	// move into trolley using moveDistance
	n.setParam("STARTDETECTLEG",false);
	usleep(500000);
	moveDistance(hh+agvy+(TROLLEYWIDTH / 2.0));		
	//moveDistanceSpeed(hh+agvy-0.18,750);		//-0.03

	usleep(200000);
	ROS_INFO("------------ UsbEpos : Finish moveAGVintoTrolleyFront ------------------");	

}

void BotNode::moveAGVintoTrolleyLeft(double trolleywidth,double trolley) {
	ros::NodeHandle n;
	int type,cnt;
	double BOXW,BOXL,TROLLEYLEN,TROLLEYWIDTH;
	double vl,vr,vd,vb,ant,rra,hh,dxx,xx,dra,dan,turnangle;
	bool RightLeg,LeftLeg,RightZone,LeftZone,TrolleyAlign;
	int RightLegIdx,LeftLegIdx;
	double RightLegdist,RightLegAngle,LeftLegdist,LeftLegAngle;
	double agvx,agvy,dxxx;

	TrolleyAlign = false;
	ROS_INFO("-------------- moveAGVintoTrolleyLeft -------------------------");
	//return;
	type = (int) trolley;
	switch (type) {
		case 1: // type 1
			TROLLEYLEN = 1.07;
			TROLLEYWIDTH = 0.67;
			break;
		case 2:
			TROLLEYLEN = 1.07;
			TROLLEYWIDTH = 0.67;
			break;
		case 3:
			BOXW = 0.165;
			BOXL = 0.9;
			break;
	}
	
	// ******* align to left trolley legs ************
	RightLeg = false;
	LeftLeg = false;
	TrolleyAlign = false;
	n.setParam("RightLeg",RightLeg);
	n.setParam("LeftLeg",LeftLeg);
	n.setParam("TrolleyAlign",TrolleyAlign);
	n.setParam("RESTARTDETECTLEGFRONT",true);
	n.setParam("LEFTLEGALIGN",true);
	n.setParam("STARTDETECTLEGFRONT",true); // initiate left and right laser detection of the legs
	usleep(100000);
	ROS_INFO("------- moveAGVintoTrolleyLeft : Start Trolley left Side Laser Alignment -----------");
	cnt = 0;
	while (true) {
		n.getParam("TrolleyAlign",TrolleyAlign);
		if (TrolleyAlign) {
			n.getParam("RightLegIdx",RightLegIdx);
			n.getParam("RightLegdist",RightLegdist);
			n.getParam("RightLegAngle",RightLegAngle);
			n.getParam("LeftLegIdx",LeftLegIdx);
			n.getParam("LeftLegdist",LeftLegdist);
			n.getParam("LeftLegAngle",LeftLegAngle);
			ROS_INFO("--- LeftSide TrolleyAlign : rlidx=%d. rldist=%.3f. rlAn=%.3f. llidx=%d. lldist=%.3f. llAn=%.3f. ---",RightLegIdx,\
									RightLegdist,RightLegAngle,LeftLegIdx,LeftLegdist,LeftLegAngle);
			break;
		}
		cnt++;
		if (cnt > 30) {
			n.setParam("STARTDETECTLEGFRONT",false);
			ROS_INFO("------------ Epos : moveAGVintoTrolleyLeft : Trolley Alignment Failed. --------------------");
			return;
		}
		usleep(200000);
	}
	n.setParam("STARTDETECTLEGFRONT",false);
	// calculate alginment : dx
	ant = RightLegAngle + LeftLegAngle;
	rra = asin((LeftLegdist * sin(ant)) / TROLLEYWIDTH);
	hh = ((LeftLegdist * sin(ant)) / TROLLEYWIDTH) * RightLegdist;
	xx = hh / tan(rra);
	dxx = (TROLLEYWIDTH / 2.0) - xx;
	dra = atan(xx / hh);
	dan = RightLegAngle - dra;
	agvx = FRONTLASERDIST * sin(fabs(dan));
	agvy = FRONTLASERDIST * cos(fabs(dan));
	turnangle = 90.0 + (dan * 57.2958);
	if (dan > 0.0) {
		dxxx = dxx + agvx;
	} else {
		dxxx = dxx - agvx;
	}
	ROS_INFO("----- ant=%3f. rra=%.3f. hh=%.3f. xx=%.3f. dxx=%.3f -----------",ant,rra,hh,xx,dxx);
	ROS_INFO("----- dra=%3f. dan=%.3f. -----------",dra,dan);  
	ROS_INFO("----- Turn=%3f. dxxx=%.3f. dan=%.3f. agvx=%.3f -----------",turnangle,dxxx,dan,agvx); 
	if (fabs(dan) > 0.015) {
		turnangle = - (dan * 57.2958);
		moveAngle(turnangle);
		ROS_INFO("--------- LeftSide : Adjust. Turn=%3f. -----------",turnangle); 
	} else {
		ROS_INFO("--------- LeftSide : No Adjustment needed. dxxx=%.3f. dan=%.3f -----------",dxxx,dan);
	}
	// adjust center distance
	dxxx = (hh+agvy) - (TROLLEYLEN / 2.0);// 0.02
	if (fabs(dxxx) > 0.02) {
		ROS_INFO("--------- LeftSide : Adjust. Move=%3f. -----------",dxxx);
		moveDistance(dxxx);		
	} else {
		ROS_INFO("--------- LeftSide : No Adjust. Move=%3f. -----------",dxxx);
	}

	usleep(200000);
	ROS_INFO("------------ UsbEpos : Finish moveAGVintoTrolleyLeft ------------------");	

}


void BotNode::moveAGVintoTrolley(double trolleywidth,double trolley) {
	ros::NodeHandle n;
	int type,cnt;
	double BOXW,BOXL,TROLLEYLEN,TROLLEYWIDTH;
	double vl,vr,vd,vb,ant,rra,hh,dxx,xx,dra,dan,turnangle;
	bool RightLeg,LeftLeg,RightZone,LeftZone,TrolleyAlign;
	int RightLegIdx,LeftLegIdx;
	double RightLegdist,RightLegAngle,LeftLegdist,LeftLegAngle;
	double agvx,agvy,dxxx;

	RightLeg = false;
	LeftLeg = false;
	RightZone = false;
	LeftZone = false;
	TrolleyAlign = false;
	ROS_INFO("-------------- moveAGVintoTrolley -------------------------");
	//return;
	type = (int) trolley;
	switch (type) {
		case 1: // type 1
			BOXW = 0.165;
			BOXL = 0.9;
			TROLLEYLEN = 1.07;
			TROLLEYWIDTH = 0.67;
			break;
		case 2:
			BOXW = 0.165;
			BOXL = 0.9;
			TROLLEYLEN = 1.07;
			TROLLEYWIDTH = 0.67;
			break;
		case 3:
			BOXW = 0.165;
			BOXL = 0.9;
			break;
	}
	
	n.setParam("BOXW",BOXW);
	n.setParam("BOXL",BOXL);
	n.setParam("RightLeg",RightLeg);
	n.setParam("LeftLeg",LeftLeg);
	n.setParam("RightZone",RightZone);
	n.setParam("LeftZone",LeftZone);
	n.setParam("TrolleyAlign",TrolleyAlign);
	n.setParam("RESTARTDETECTLEGFRONT",true);
	n.setParam("LEFTLEGALIGN",false);
	n.setParam("STARTDETECTLEGFRONT",true); // initiate left and right laser detection of the legs
	usleep(100000);
	ROS_INFO("------- moveAGVintoTrolley : Start Front Laser Alignment -----------");
	cnt = 0;
	while (true) {
		n.getParam("TrolleyAlign",TrolleyAlign);
		if (TrolleyAlign) {
			n.getParam("RightLegIdx",RightLegIdx);
			n.getParam("RightLegdist",RightLegdist);
			n.getParam("RightLegAngle",RightLegAngle);
			n.getParam("LeftLegIdx",LeftLegIdx);
			n.getParam("LeftLegdist",LeftLegdist);
			n.getParam("LeftLegAngle",LeftLegAngle);
			ROS_INFO("--- TrolleyAlign : rlidx=%d. rldist=%.3f. rlAn=%.3f. llidx=%d. lldist=%.3f. llAn=%.3f. ---",RightLegIdx,\
									RightLegdist,RightLegAngle,LeftLegIdx,LeftLegdist,LeftLegAngle);
			break;
		}
		cnt++;
		if (cnt > 50) {
			n.setParam("STARTDETECTLEGFRONT",false);
			ROS_INFO("------------ Epos : moveAGVintoTrolley : Trolley Alignment Failed. --------------------");
			return;
		}
		usleep(200000);
	} 
	n.setParam("STARTDETECTLEGFRONT",false);
	
	// calculate alginment : dx
	ant = RightLegAngle + LeftLegAngle;
	rra = asin((LeftLegdist * sin(ant)) / TROLLEYLEN);
	hh = ((LeftLegdist * sin(ant)) / TROLLEYLEN) * RightLegdist;
	xx = hh / tan(rra);
	dxx = (TROLLEYLEN / 2.0) - xx;
	dra = atan(xx / hh);
	dan = RightLegAngle - dra;
	agvx = FRONTLASERDIST * sin(fabs(dan));
	agvy = FRONTLASERDIST * cos(fabs(dan));
	turnangle = 90.0 + (dan * 57.2958);
	if (dan > 0.0) {
		dxxx = dxx + agvx;
	} else {
		dxxx = dxx - agvx;
	}
	ROS_INFO("----- ant=%3f. rra=%.3f. hh=%.3f. xx=%.3f. dxx=%.3f -----------",ant,rra,hh,xx,dxx);
	ROS_INFO("----- dra=%3f. dan=%.3f. -----------",dra,dan);  
	ROS_INFO("----- Turn=%3f. dxxx=%.3f. dan=%.3f. agvx=%.3f. agvy=%.3f -----------",turnangle,dxxx,dan,agvx,agvy); 

	if ((fabs(dxxx) > 0.015)) {		
		moveAngle(-turnangle);
		usleep(600000);
		//moveDistance(fabs(dxxx));
		moveDistance(-dxxx);
		usleep(600000);
		moveAngle(90.0);
		//ROS_INFO("--------- moveAGVintoTrolley : Adjust. Turn=%3f. Move=%.3f -----------",turnangle,fabs(dxxx)); 
	} else {
		if (fabs(dan) > 0.015) {
			turnangle = - (dan * 57.2958);
			moveAngle(turnangle);
			ROS_INFO("--------- moveAGVintoTrolley : Adjust. Turn=%3f. -----------",turnangle); 
		} else {
			ROS_INFO("--------- moveAGVintoTrolley : No Adjustment needed. dxxx=%.3f. dan=%.3f -----------",dxxx,dan);
		}
	}	
	ROS_INFO("------- moveAGVintoTrolley : End  Front Laser Alignment -----------");
	// move into trolley using moveDistance
	n.setParam("STARTDETECTLEG",false);
	usleep(500000);
	moveDistance(hh+agvy+(TROLLEYWIDTH / 2.0));		
	//moveDistanceSpeed(hh+agvy-0.18,750);		//-0.03
	//usleep(500000);
	usleep(500000);

	// ******* 2nd align again with front trolley legs ************
	/*
	RightLeg = false;
	LeftLeg = false;
	TrolleyAlign = false;
	n.setParam("RightLeg",RightLeg);
	n.setParam("LeftLeg",LeftLeg);
	n.setParam("TrolleyAlign",TrolleyAlign);
	n.setParam("RESTARTDETECTLEGFRONT",true);
	n.setParam("STARTDETECTLEGFRONT",true); // initiate left and right laser detection of the legs
	usleep(100000);
	ROS_INFO("------- moveAGVintoTrolley : Start Trolley 2nd front leg Laser Alignment -----------");
	cnt = 0;
	while (true) {
		n.getParam("TrolleyAlign",TrolleyAlign);
		if (TrolleyAlign) {
			n.getParam("RightLegIdx",RightLegIdx);
			n.getParam("RightLegdist",RightLegdist);
			n.getParam("RightLegAngle",RightLegAngle);
			n.getParam("LeftLegIdx",LeftLegIdx);
			n.getParam("LeftLegdist",LeftLegdist);
			n.getParam("LeftLegAngle",LeftLegAngle);
			ROS_INFO("--- 2nd FrontTrolleyAlign : rlidx=%d. rldist=%.3f. rlAn=%.3f. llidx=%d. lldist=%.3f. llAn=%.3f. ---",RightLegIdx,\
									RightLegdist,RightLegAngle,LeftLegIdx,LeftLegdist,LeftLegAngle);
			break;
		}
		cnt++;
		if (cnt > 50) {
			n.setParam("STARTDETECTLEGFRONT",false);
			ROS_INFO("------------ Epos : moveAGVintoTrolley : 2nd Trolley Alignment Failed. --------------------");
			return;
		}
		usleep(200000);
	}
	n.setParam("STARTDETECTLEGFRONT",false);
	// calculate alginment : dx
	ant = RightLegAngle + LeftLegAngle;
	rra = asin((LeftLegdist * sin(ant)) / TROLLEYLEN);
	hh = ((LeftLegdist * sin(ant)) / TROLLEYLEN) * RightLegdist;
	xx = hh / tan(rra);
	dxx = (TROLLEYLEN / 2.0) - xx;
	dra = atan(xx / hh);
	dan = RightLegAngle - dra;
	agvx = FRONTLASERDIST * sin(fabs(dan));
	agvy = FRONTLASERDIST * cos(fabs(dan));
	turnangle = 90.0 + (dan * 57.2958);
	if (dan > 0.0) {
		dxxx = dxx + agvx;
	} else {
		dxxx = dxx - agvx;
	}
	ROS_INFO("-----2nd frontalign : ant=%3f. rra=%.3f. hh=%.3f. xx=%.3f. dxx=%.3f -----------",ant,rra,hh,xx,dxx);
	ROS_INFO("-----2nd frontalign : dra=%3f. dan=%.3f. -----------",dra,dan);  
	ROS_INFO("-----2nd frontalign : Turn=%3f. dxxx=%.3f. dan=%.3f. agvx=%.3f -----------",turnangle,dxxx,dan,agvx); 
	if (fabs(dan) > 0.015) {
		turnangle = - (dan * 57.2958);
		moveAngle(turnangle);
		ROS_INFO("--------- 2nd frontalign : : Adjust. Turn=%3f. -----------",turnangle); 
	} else {
		ROS_INFO("--------- 2nd frontalign : : No Adjustment needed. dxxx=%.3f. dan=%.3f -----------",dxxx,dan);
	}
	dxxx = (hh+agvy+0.018) - (TROLLEYWIDTH / 2.0); //0.015
	if (fabs(dxxx) > 0.015) {
		ROS_INFO("--------- 2nd frontalign  : Adjust. Move=%3f. -----------",dxxx);
		moveDistance(dxxx);		
	} else {
		ROS_INFO("--------- 2nd frontalign  : No Adjust. Move=%3f. -----------",dxxx);
	}
	*/
	//usleep(500000);
	sleep(1);
	moveAngle(90.0);

	// ******* align to left trolley legs ************
	RightLeg = false;
	LeftLeg = false;
	TrolleyAlign = false;
	n.setParam("RightLeg",RightLeg);
	n.setParam("LeftLeg",LeftLeg);
	n.setParam("TrolleyAlign",TrolleyAlign);
	n.setParam("RESTARTDETECTLEGFRONT",true);
	n.setParam("LEFTLEGALIGN",true);
	n.setParam("STARTDETECTLEGFRONT",true); // initiate left and right laser detection of the legs
	usleep(100000);
	ROS_INFO("------- moveAGVintoTrolley : Start Trolley left Side Laser Alignment -----------");
	cnt = 0;
	while (true) {
		n.getParam("TrolleyAlign",TrolleyAlign);
		if (TrolleyAlign) {
			n.getParam("RightLegIdx",RightLegIdx);
			n.getParam("RightLegdist",RightLegdist);
			n.getParam("RightLegAngle",RightLegAngle);
			n.getParam("LeftLegIdx",LeftLegIdx);
			n.getParam("LeftLegdist",LeftLegdist);
			n.getParam("LeftLegAngle",LeftLegAngle);
			ROS_INFO("--- LeftSide TrolleyAlign : rlidx=%d. rldist=%.3f. rlAn=%.3f. llidx=%d. lldist=%.3f. llAn=%.3f. ---",RightLegIdx,\
									RightLegdist,RightLegAngle,LeftLegIdx,LeftLegdist,LeftLegAngle);
			break;
		}
		cnt++;
		if (cnt > 30) {
			n.setParam("STARTDETECTLEGFRONT",false);
			ROS_INFO("------------ Epos : moveAGVintoTrolley : Trolley Alignment Failed. --------------------");
			return;
		}
		usleep(200000);
	}
	n.setParam("STARTDETECTLEGFRONT",false);
	// calculate alginment : dx
	ant = RightLegAngle + LeftLegAngle;
	rra = asin((LeftLegdist * sin(ant)) / TROLLEYWIDTH);
	hh = ((LeftLegdist * sin(ant)) / TROLLEYWIDTH) * RightLegdist;
	xx = hh / tan(rra);
	dxx = (TROLLEYWIDTH / 2.0) - xx;
	dra = atan(xx / hh);
	dan = RightLegAngle - dra;
	agvx = FRONTLASERDIST * sin(fabs(dan));
	agvy = FRONTLASERDIST * cos(fabs(dan));
	turnangle = 90.0 + (dan * 57.2958);
	if (dan > 0.0) {
		dxxx = dxx + agvx;
	} else {
		dxxx = dxx - agvx;
	}
	ROS_INFO("----- ant=%3f. rra=%.3f. hh=%.3f. xx=%.3f. dxx=%.3f -----------",ant,rra,hh,xx,dxx);
	ROS_INFO("----- dra=%3f. dan=%.3f. -----------",dra,dan);  
	ROS_INFO("----- Turn=%3f. dxxx=%.3f. dan=%.3f. agvx=%.3f -----------",turnangle,dxxx,dan,agvx); 
	if (fabs(dan) > 0.015) {
		turnangle = - (dan * 57.2958);
		moveAngle(turnangle);
		ROS_INFO("--------- LeftSide : Adjust. Turn=%3f. -----------",turnangle); 
	} else {
		ROS_INFO("--------- LeftSide : No Adjustment needed. dxxx=%.3f. dan=%.3f -----------",dxxx,dan);
	}
	// adjust center distance
	dxxx = (hh+agvy+0.02) - (TROLLEYLEN / 2.0);
	if (fabs(dxxx) > 0.015) {
		ROS_INFO("--------- LeftSide : Adjust. Move=%3f. -----------",dxxx);
		moveDistance(dxxx);		
	} else {
		ROS_INFO("--------- LeftSide : No Adjust. Move=%3f. -----------",dxxx);
	}
	/*
	// ******* align to left trolley legs 2nd time ************
	RightLeg = false;
	LeftLeg = false;
	TrolleyAlign = false;
	n.setParam("RightLeg",RightLeg);
	n.setParam("LeftLeg",LeftLeg);
	n.setParam("TrolleyAlign",TrolleyAlign);
	n.setParam("RESTARTDETECTLEGFRONT",true);
	n.setParam("STARTDETECTLEGFRONT",true); // initiate left and right laser detection of the legs
	usleep(100000);
	ROS_INFO("------- moveAGVintoTrolley : Start Trolley left Side Laser Alignment -----------");
	cnt = 0;
	while (true) {
		n.getParam("TrolleyAlign",TrolleyAlign);
		if (TrolleyAlign) {
			n.getParam("RightLegIdx",RightLegIdx);
			n.getParam("RightLegdist",RightLegdist);
			n.getParam("RightLegAngle",RightLegAngle);
			n.getParam("LeftLegIdx",LeftLegIdx);
			n.getParam("LeftLegdist",LeftLegdist);
			n.getParam("LeftLegAngle",LeftLegAngle);
			ROS_INFO("--- LeftSide TrolleyAlign : rlidx=%d. rldist=%.3f. rlAn=%.3f. llidx=%d. lldist=%.3f. llAn=%.3f. ---",RightLegIdx,\
									RightLegdist,RightLegAngle,LeftLegIdx,LeftLegdist,LeftLegAngle);
			break;
		}
		cnt++;
		if (cnt > 30) {
			n.setParam("STARTDETECTLEGFRONT",false);
			ROS_INFO("------------ Epos : moveAGVintoTrolley : Trolley Alignment Failed. --------------------");
			return;
		}
		usleep(200000);
	}
	n.setParam("STARTDETECTLEGFRONT",false);
	// calculate alginment : dx
	ant = RightLegAngle + LeftLegAngle;
	rra = asin((LeftLegdist * sin(ant)) / TROLLEYWIDTH);
	hh = ((LeftLegdist * sin(ant)) / TROLLEYWIDTH) * RightLegdist;
	xx = hh / tan(rra);
	dxx = (TROLLEYWIDTH / 2.0) - xx;
	dra = atan(xx / hh);
	dan = RightLegAngle - dra;
	agvx = FRONTLASERDIST * sin(fabs(dan));
	agvy = FRONTLASERDIST * cos(fabs(dan));
	turnangle = 90.0 + (dan * 57.2958);
	if (dan > 0.0) {
		dxxx = dxx + agvx;
	} else {
		dxxx = dxx - agvx;
	}
	ROS_INFO("----- ant=%3f. rra=%.3f. hh=%.3f. xx=%.3f. dxx=%.3f -----------",ant,rra,hh,xx,dxx);
	ROS_INFO("----- dra=%3f. dan=%.3f. -----------",dra,dan);  
	ROS_INFO("----- Turn=%3f. dxxx=%.3f. dan=%.3f. agvx=%.3f -----------",turnangle,dxxx,dan,agvx); 

	if ((fabs(dxxx) > 0.015)) {		
		moveAngle(-turnangle);
		usleep(600000);
		//moveDistance(fabs(dxxx));
		moveDistance(-dxxx);
		usleep(600000);
		moveAngle(90.0);
		//ROS_INFO("--------- moveAGVintoTrolley : Adjust. Turn=%3f. Move=%.3f -----------",turnangle,fabs(dxxx)); 
	} else {
		if (fabs(dan) > 0.02) {
			turnangle = - (dan * 57.2958);
			moveAngle(turnangle);
			ROS_INFO("--------- moveAGVintoTrolley : Adjust. Turn=%3f. -----------",turnangle); 
		} else {
			ROS_INFO("--------- moveAGVintoTrolley : No Adjustment needed. dxxx=%.3f. dan=%.3f -----------",dxxx,dan);
		}
	}	
	ROS_INFO("------- moveAGVintoTrolley : End  2nd Left Leg Laser Alignment -----------");
	*/
	usleep(200000);
	ROS_INFO("------------ UsbEpos : Finish moveAGVintoTrolley ------------------");	

}

void BotNode::publish_fButton(unsigned short cmd)
{
	std_msgs::UInt16 msg;
	msg.data = cmd;
	fButton_pub.publish(msg);
	return;
}

void BotNode::publish_event(string s)
{
  time_t rawtime;
	char buf [200];
	string sm,st;
	struct tm dashStartTime;
	int yy,mm,dd,hr,min,ss;

  time( &rawtime );
  dashStartTime = *localtime( &rawtime );
	yy = dashStartTime.tm_year;
	mm = dashStartTime.tm_mon;
	dd = dashStartTime.tm_mday;
	hr = dashStartTime.tm_hour;
	min = dashStartTime.tm_min;
	ss = dashStartTime.tm_sec;
	switch(mm) {
		case 1:
			sm = "Jan";
			break;
		case 2:
			sm = "Feb";
			break;
		case 3:
			sm = "Mar";
			break;
		case 4:
			sm = "Apr";
			break;
		case 5:
			sm = "May";
			break;
		case 6:
			sm = "Jun";
			break;
		case 7:
			sm = "Jul";
			break;
		case 8:
			sm ="Aug";
			break;
		case 9:
			sm = "Sep";
			break;
		case 10:
			sm = "Oct";
			break;
		case 11:
			sm = "Nov";
			break;
		case 12:
			sm = "Dec";
			break;
	}
	sprintf(buf,"%d%s%d_%d%d%d",dd,sm.c_str(),yy+1900,hr,min,ss);
	st.assign(buf,strlen(buf));
	htbot::debug status;
	status.msg = st + " : " + s;
	event_pub.publish(status);
	return;
}

void BotNode::getDisplacement(double* distance, double* angle) {
	unsigned short objectindx = 0x6064;
	unsigned char subIndx = 0x00;
	unsigned int BytesToRead;
	unsigned int BytesRead;
	unsigned int BytesToWrite;
	unsigned int BytesWritten;
	unsigned char bparam[4];
	int cur_Lpos=0, cur_Rpos=0;
	double left_mm=0, right_mm=0;
	
	prev_Lpos = LeftMotorActualPosition;
	prev_Rpos = RightMotorActualPosition;
	
	getPosition();
	cur_Lpos = LeftMotorActualPosition;
	cur_Rpos = RightMotorActualPosition;
	left_mm = (double)(cur_Lpos-prev_Lpos) * mm_per_count;
	right_mm = (double)(cur_Rpos-prev_Rpos) * mm_per_count;
	//*distance = (( left_mm - right_mm )/2000.0);
	*distance = (( left_mm + right_mm )/2000.0);
	//*angle = -(left_mm + right_mm) * displacement_factor;
	*angle = (right_mm - left_mm) * displacement_factor;
}

void BotNode::freeMotor(void) {
	unsigned short objectindx = 0x6040;
	unsigned char subIndx = 0x00;
	unsigned int BytesToRead;
	unsigned int BytesRead;
	unsigned int BytesToWrite;
	unsigned int BytesWritten;
	unsigned char bparam[4];
	ROS_INFO("---------- UsbEpos : Disengage Motor --------------------");
	quickStopMotor();
	BytesToRead = 2;
	if (VCS_GetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,BytesToRead,&BytesRead,&eposErrorCode) == 0) {
		//printf("\n -------- Get Controlword_1 Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : Get Controlword_1 Error -------------");
		ROS_INFO("--------BotNode : Get Controlword_1 Error -------------");
	}else {
		bparam[0] = bparam[0] & 0xfd;
		BytesToWrite = 2;
		if (VCS_SetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,BytesToWrite,&BytesWritten,&eposErrorCode) == 0) {
			//printf("\n -------- Set Motor Disable_1. Error=%x -------------\n",eposErrorCode);
			//publish_event("--------BotNode : Set Motor Disable_1 Error -------------");
			ROS_INFO("--------BotNode : Set Motor Disable_1 Error -------------");
		} 
	}
	if (VCS_GetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,BytesToRead,&BytesRead,&eposErrorCode) == 0) {
		//printf("\n -------- Get Controlword_2 Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : Get Controlword_2 Error -------------");
		ROS_INFO("--------BotNode : Get Controlword_2 Error -------------");
	}else {
		bparam[0] = bparam[0] & 0xfd;
		BytesToWrite = 2;
		if (VCS_SetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,BytesToWrite,&BytesWritten,&eposErrorCode) == 0) {
			//printf("\n -------- Set Motor Disable_2. Error=%x -------------\n",eposErrorCode);
			//publish_event("--------BotNode : Set Motor Disable_2 Error -------------");
			ROS_INFO("--------BotNode : Set Motor Disable_2 Error -------------");
		} 
	}
}

void BotNode::moveDistanceSpeed(double dist,int velocity) {
	ros::NodeHandle nn;
	double obsdist;
	long dleft,dright,d;
	BOOL Absolute = 0;
	BOOL Immediately = 1;
	BOOL pTargetReached = 0;
	
	//printf("\n ---- Move Dist = %.3f -------\n",dist);
	if (dist > 0.0) {
		while(true) {				
			publish_odom();
			nn.getParam("ProfileMoveObsDist",obsdist);
			//obsdist = 4.0;
			if (obsdist > dist) {				
				break;
			}			
			publish_sound(NEEDSPACE,0,5);  // robot need space					
		}
	}
	d=(long)( dist * dist_factor);
	quickstop = false;
	setProfilePositionMove();
	dleft = d;
	dright = d;

	if (VCS_SetPositionProfile(g_pKey,RMOTOR,velocity,ProfileAcceleration,ProfileDeceleration,&eposErrorCode) == 0) {
		//printf("\n -------- SetPositionProfile_RIGHT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : SetPositionProfile_RIGHT Error -------------");
		ROS_INFO("--------BotNode : SetPositionProfile_RIGHT Error -------------");
	}	
	if (VCS_SetPositionProfile(g_pKey,LMOTOR,velocity,ProfileAcceleration,ProfileDeceleration,&eposErrorCode) == 0) {
		//printf("\n -------- SetPositionProfile_LEFT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : SetPositionProfile_LEFT Error -------------");
		ROS_INFO("--------BotNode : SetPositionProfile_LEFT Error -------------");
	}	

	//printf("\n ------- dist count = %ld. --------\n",d);
	if(VCS_MoveToPosition(g_pKey,RMOTOR,dright,Absolute,Immediately,&eposErrorCode) == 0)
	{
		//printf("\n -------- MoveToPosition_RIGHT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : MoveToPosition_RIGHT Error -------------");
		ROS_INFO("--------BotNode : MoveToPosition_RIGHT Error -------------");
	}
	if(VCS_MoveToPosition(g_pKey,LMOTOR,dleft,Absolute,Immediately,&eposErrorCode) == 0)
	{
		//printf("\n -------- MoveToPosition_LEFT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : MoveToPosition_LEFT Error -------------");
		ROS_INFO("--------BotNode : MoveToPosition_LEFT Error -------------");
	}
	//printf("\n ------ start dist move ------\n");
	while(true)
	{
		usleep(20000);		
		//readKey();
		publish_odom();
		if (quickstop) {			
			quickStopMotor();
			//ROS_INFO(" ------- quick stop ------");
			//haltRobot();
			break;
		}
		if (checkTragetReached()) {
			//ROS_INFO(" ------- Target Reached ------");
			break;
		}
	}
	if (VCS_SetPositionProfile(g_pKey,RMOTOR,ProfileVelocity,ProfileAcceleration,ProfileDeceleration,&eposErrorCode) == 0) {
		//printf("\n -------- SetPositionProfile_RIGHT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : SetPositionProfile_RIGHT Error -------------");
		ROS_INFO("--------BotNode : SetPositionProfile_RIGHT Error -------------");
	}	
	if (VCS_SetPositionProfile(g_pKey,LMOTOR,ProfileVelocity,ProfileAcceleration,ProfileDeceleration,&eposErrorCode) == 0) {
		//printf("\n -------- SetPositionProfile_LEFT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : SetPositionProfile_LEFT Error -------------");
		ROS_INFO("--------BotNode : SetPositionProfile_LEFT Error -------------");
	}	
	setVelocityMode();
}

void BotNode::moveDistance(double dist) {
	ros::NodeHandle nn;
	double obsdist;
	long dleft,dright,d;
	BOOL Absolute = 0;
	BOOL Immediately = 1;
	BOOL pTargetReached = 0;
	
	//printf("\n ---- Move Dist = %.3f -------\n",dist);
	if (dist > 0.0) {
		while(true) {				
			publish_odom();
			nn.getParam("ProfileMoveObsDist",obsdist);
			//obsdist = 4.0;
			if (obsdist > dist) {				
				break;
			}			
			publish_sound(NEEDSPACE,0,5);  // robot need space					
		}
	}
	d=(long)( dist * dist_factor);
	quickstop = false;
	setProfilePositionMove();
	dleft = d;
	dright = d;
	//printf("\n ------- dist count = %ld. --------\n",d);
	if(VCS_MoveToPosition(g_pKey,RMOTOR,dright,Absolute,Immediately,&eposErrorCode) == 0)
	{
		//printf("\n -------- MoveToPosition_RIGHT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : MoveToPosition_RIGHT Error -------------");
		ROS_INFO("--------BotNode : MoveToPosition_RIGHT Error -------------");
	}
	if(VCS_MoveToPosition(g_pKey,LMOTOR,dleft,Absolute,Immediately,&eposErrorCode) == 0)
	{
		//printf("\n -------- MoveToPosition_LEFT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : MoveToPosition_LEFT Error -------------");
		ROS_INFO("--------BotNode : MoveToPosition_LEFT Error -------------");
	}
	//printf("\n ------ start dist move ------\n");
	while(true)
	{
		usleep(20000);		
		//readKey();
		publish_odom();
		if (quickstop) {			
			quickStopMotor();
			//ROS_INFO(" ------- quick stop ------");
			//haltRobot();
			break;
		}
		if (checkTragetReached()) {
			//ROS_INFO(" ------- Target Reached ------");
			break;
		}
	}
	setVelocityMode();
}

void BotNode::setProfilePositionMove(void) {
	if(VCS_ActivateProfilePositionMode(g_pKey,RMOTOR,&eposErrorCode) == 0)
	{
		//printf("\n -------- ActivateProfilePositionMode_RIGHT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : ActivateProfilePositionMode_RIGHT Error -------------");
		ROS_INFO("--------BotNode : ActivateProfilePositionMode_RIGHT Error -------------");
		return;
	}
	if(VCS_ActivateProfilePositionMode(g_pKey,LMOTOR,&eposErrorCode) == 0)
	{
		//printf("\n -------- ActivateProfilePositionMode_LEFT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : ActivateProfilePositionMode_LEFT Error -------------");
		ROS_INFO("--------BotNode : ActivateProfilePositionMode_LEFT Error -------------");
		return;
	}
	//printf("\n ------ Set Profile Position Move ------\n");
}

void BotNode::quickStopMotor(void) {
	unsigned short objectindx = 0x6040;
	unsigned char subIndx = 0x00;
	unsigned int BytesToRead;
	unsigned int BytesRead;
	unsigned int BytesToWrite;
	unsigned int BytesWritten;
	unsigned char bparam[4];
	unsigned char cw1,cw2;
	
	//printf("\n ------- quick Stop --------\n");
	BytesToRead = 2;
	if (VCS_GetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,BytesToRead,&BytesRead,&eposErrorCode) == 0) {
		//printf("\n -------- Get Controlword_RIGHT Error=%x -------------\n",eposErrorCode);
		publish_event("--------BotNode : Get Controlword_RIGHT Error -------------");
	}else {
		bparam[0] = bparam[0] & 0x0b;
		BytesToWrite = 2;
		if (VCS_SetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,BytesToWrite,&BytesWritten,&eposErrorCode) == 0) {
			//printf("\n -------- Set Controlword_RIGHT. Error=%x -------------\n",eposErrorCode);
			publish_event("--------BotNode : Set Controlword_RIGHT Error -------------");
		} else {
			bparam[0] = bparam[0] | 0x04;
			if (VCS_SetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,BytesToWrite,&BytesWritten,&eposErrorCode) == 0) {
				//printf("\n -------- Set Controlword_RIGHT. Error=%x -------------\n",eposErrorCode);
				publish_event("--------BotNode : Set Controlword_RIGHT Error -------------");
			}
		}
	}
	if (VCS_GetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,BytesToRead,&BytesRead,&eposErrorCode) == 0) {
		//printf("\n -------- Get Controlword_LEFT Error=%x -------------\n",eposErrorCode);
		publish_event("--------BotNode : Get Controlword_LEFT Error -------------");
	}else {
		bparam[0] = bparam[0] & 0x0b;
		BytesToWrite = 2;
		if (VCS_SetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,BytesToWrite,&BytesWritten,&eposErrorCode) == 0) {
			//printf("\n -------- Set Controlword_LEFT. Error=%x -------------\n",eposErrorCode);
			publish_event("--------BotNode : Set Controlword_LEFT Error -------------");
		} else {
			bparam[0] = bparam[0] | 0x04;
			if (VCS_SetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,BytesToWrite,&BytesWritten,&eposErrorCode) == 0) {
				//printf("\n -------- Set Controlword_LEFT. Error=%x -------------\n",eposErrorCode);
				publish_event("--------BotNode : Set Controlword_LEFT Error -------------");
			}
		}
	}
	//printf("\n ----- quickStop Motor --------\n");
}

bool BotNode::checkTragetReached() {
	BOOL pTargetReached = 0;
	bool m1,m2;
	
	m1 = false;
	m2 = false;
	pTargetReached = 0;
	if (VCS_GetMovementState(g_pKey,RMOTOR,&pTargetReached,&eposErrorCode) != 0) {
		if (pTargetReached) {
			m1 = true;
			//ROS_INFO(" -------Right Target Reached ------");
		}
	}
	pTargetReached = 0;
	if (VCS_GetMovementState(g_pKey,LMOTOR,&pTargetReached,&eposErrorCode) != 0) {
		if (pTargetReached) {
			m2 = true;
			//ROS_INFO(" ------- Left  Target Reached  ------");
		}
	}
	if (m1 || m2) {
		//ROS_INFO(" ------- Target Reached  ------");
		return true;
	} else {
		return false;
	}
}

void BotNode::moveAngle(double angle) {
	long th,dleft,dright;
	BOOL Absolute = 0;
	BOOL Immediately = 1;
	BOOL pTargetReached = 0;
	
	th=(long)( angle * angle_factor );
	quickstop = false;
	setProfilePositionMove();
	dleft = -th;
	dright = th;
	
	if(VCS_MoveToPosition(g_pKey,RMOTOR,dright,Absolute,Immediately,&eposErrorCode) == 0)
	{
		//printf("\n -------- MoveToPosition_RIGHT Error=%x -------------\n",eposErrorCode);
		publish_event("--------BotNode : MoveToPosition_RIGHT Error -------------");
		errorState = true;
	}
	if(VCS_MoveToPosition(g_pKey,LMOTOR,dleft,Absolute,Immediately,&eposErrorCode) == 0)
	{
		//printf("\n -------- MoveToPosition_LEFT Error=%x -------------\n",eposErrorCode);
		publish_event("--------BotNode : MoveToPosition_LEFT Error -------------");
		errorState = true;
	}
	while(true)
	{
		usleep(20000);		
		publish_odom();
		if (quickstop) {			
			quickStopMotor();
			//haltRobot();
			break;
		}
		if (checkTragetReached()) {
			//ROS_INFO(" ------- Target Reached  ------");
			break;
		}
	}
	setVelocityMode();
}

void BotNode::moveVelocity(double linear,double angular) {
	long v,w;
	long targetvelocity;
	if (linear > 1.15) {
		linear = 1.15;
	} else {
		if (linear < -1.15) {
			linear = -1.15;
		}
	}
	targetvelocity = 0;
	//printf("\n ---- Move Vel. lin=%.3f. an=%.3f -------\n",linear,angular);
	//v=(int)( (linear*1000)/(M_PI*WHEEL_DIA*wheel_circum_correction) * 60 * GEAR_RATIO ) * REFLECT;
	v = (long) (linear * rpm_factor);
	//w=(int)( angular*0.5*AXLE_LEN*wheel_base_correction/(M_PI*WHEEL_DIA*wheel_circum_correction) * 60 * GEAR_RATIO ) * REFLECT;
	w = (long) (angular * radpm_factor);
	
	//targetvelocity = (long)((-v-w)); // right
	//targetvelocity = (long)(v-w);
	targetvelocity = (long)(v+w);
	if(VCS_MoveWithVelocity(g_pKey,RMOTOR,targetvelocity,&eposErrorCode) == 0) {
		//printf("\n -------- MoveWithVelocity_RIGHT Error=%x. lin=%ld -------------\n",eposErrorCode,targetvelocity);
		errorState = true;
	}
	targetvelocity = (long)(v-w); // left
	//targetvelocity = (long)((v+w));
	if(VCS_MoveWithVelocity(g_pKey,LMOTOR,targetvelocity,&eposErrorCode) == 0) {
		//printf("\n -------- MoveWithVelocity_LEFT Error=%x. lin=%ld -------------\n",eposErrorCode,targetvelocity);
		errorState = true;
	}
}

void BotNode::moveVel(double left,double right) {
	long vl,vr;
	long targetvelocity;
	
	targetvelocity = 0;
	
	vr = (long) (right * rpm_factor);
	vl = (long) (left * rpm_factor);		

	targetvelocity = (long)(vr);
	if(VCS_MoveWithVelocity(g_pKey,RMOTOR,targetvelocity,&eposErrorCode) == 0) {
		//printf("\n -------- MoveWithVelocity_RIGHT Error=%x. lin=%ld -------------\n",eposErrorCode,targetvelocity);
		errorState = true;
	}
	targetvelocity = (long)(vl); // left
	//targetvelocity = (long)((v+w));
	if(VCS_MoveWithVelocity(g_pKey,LMOTOR,targetvelocity,&eposErrorCode) == 0) {
		//printf("\n -------- MoveWithVelocity_LEFT Error=%x. lin=%ld -------------\n",eposErrorCode,targetvelocity);
		errorState = true;
	}
}

void BotNode::getPosition(void) {
	unsigned short objectindx = 0x6064;
	unsigned char subIndx = 0x00;
	unsigned int BytesToRead;
	unsigned int BytesRead;
	unsigned int BytesToWrite;
	unsigned int BytesWritten;
	unsigned char bparam[4];
	
	BytesToRead = 4;
	if (VCS_GetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,BytesToRead,&BytesRead,&eposErrorCode) == 0) {
		//printf("\n -------- Get Actual Position_RIGHT Error=%x -------------\n",eposErrorCode);
		publish_event("--------BotNode : Get Actual Position_RIGHT Error -------------");
	} else {
		RightMotorActualPosition = (int)(bparam[0] + bparam[1] * 256 + bparam[2] * 65536 + bparam[3] * 16777216);
	}
	if (VCS_GetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,BytesToRead,&BytesRead,&eposErrorCode) == 0) {
		//printf("\n -------- Get Actual Position_LEFT Error=%x -------------\n",eposErrorCode);
		publish_event("--------BotNode : Get Actual Position_LEFT Error -------------");
	} else {
		LeftMotorActualPosition = (int)(bparam[0] + bparam[1] * 256 + bparam[2] * 65536 + bparam[3] * 16777216);
	}
}

void BotNode::setParams()
{
	int lResult = MMC_FAILED;
	unsigned int* p_pErrorCode;
	unsigned char bparam[4];
	unsigned int pNbOfBytesWritten;
	unsigned short objectindx = 0x6402;
	unsigned char subIndx = 0x00;
	unsigned int noByteToWrite;
	unsigned int BytesWritten;
	BOOL oIsFault = 0;

	//printf("\n -------- Point setparams -------------\n");

	
	//if (VCS_SetMaxFollowingError(g_pKey,RMOTOR,MaxFollowError,&eposErrorCode) == 0) {
	//	//printf("\n -------- SetMaxFollowingError_RIGHT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetMaxFollowingError_RIGHT Error -------------");
	//}
	
	//if (VCS_SetMaxFollowingError(g_pKey,LMOTOR,MaxFollowError,&eposErrorCode) == 0) {
	//	//printf("\n -------- SetMaxFollowingError_LEFT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetMaxFollowingError_LEFT Error -------------");
	//}
	
	//if (VCS_SetMaxProfileVelocity(g_pKey,RMOTOR,MaxProfileVelocity,&eposErrorCode) == 0) {		
	//	//printf("\n -------- SetMaxProfileVelocity_RIGHT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetMaxProfileVelocity_RIGHT Error -------------");
	//}
	//if (VCS_SetMaxProfileVelocity(g_pKey,LMOTOR,MaxProfileVelocity,&eposErrorCode) == 0) {		
	//	//printf("\n -------- SetMaxProfileVelocity_LEFT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetMaxProfileVelocity_LEFT Error -------------");
	//}	
	//if (VCS_SetPositionRegulatorGain(g_pKey,RMOTOR,PositionPGain,PositionIGain,PositionDGain,&eposErrorCode) == 0) {
	//	//printf("\n -------- SetPositionRegulatorGain_RIGHT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetPositionRegulatorGain_RIGHT Error -------------");
	//}	
	//if (VCS_SetPositionRegulatorGain(g_pKey,LMOTOR,PositionPGain,PositionIGain,PositionDGain,&eposErrorCode) == 0) {
	//	//printf("\n -------- SetPositionRegulatorGain_LEFT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetPositionRegulatorGain_LEFT Error -------------");
	//}	
	//if (VCS_SetVelocityRegulatorGain(g_pKey,RMOTOR,VelocityPGain,VelocityIGain,&eposErrorCode) == 0) {
	//	//printf("\n -------- SetVelocityRegulatorGain_RIGHT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetVelocityRegulatorGain_RIGHT Error -------------");
	//}
	//if (VCS_SetVelocityRegulatorGain(g_pKey,LMOTOR,VelocityPGain,VelocityIGain,&eposErrorCode) == 0) {
	//	//printf("\n -------- SetVelocityRegulatorGain_LEFT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetVelocityRegulatorGain_LEFT Error -------------");
	//}	
	//if (VCS_SetCurrentRegulatorGain(g_pKey,RMOTOR,CurrentRegulatorPGain,CurrentRegulatorIGain,&eposErrorCode) == 0) {
	//	//printf("\n -------- SetCurrentRegulatorGain_RIGHT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetCurrentRegulatorGain_RIGHT Error -------------");
	//}	
	//if (VCS_SetCurrentRegulatorGain(g_pKey,LMOTOR,CurrentRegulatorPGain,CurrentRegulatorIGain,&eposErrorCode) == 0) {
	//	//printf("\n -------- SetCurrentRegulatorGain_LEFT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetCurrentRegulatorGain_LEFT Error -------------");
	//}	
	//if (VCS_EnablePositionWindow(g_pKey,RMOTOR,PositionWindow,PositionWindowTime,&eposErrorCode) == 0) {
	//	//printf("\n -------- EnablePositionWindow_RIGHT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : EnablePositionWindow_RIGHT Error -------------");
	//}
	//if (VCS_EnablePositionWindow(g_pKey,LMOTOR,PositionWindow,PositionWindowTime,&eposErrorCode) == 0) {
	//	//printf("\n -------- EnablePositionWindow_LEFT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : EnablePositionWindow_LEFT Error -------------");
	//}
	if (VCS_SetVelocityProfile(g_pKey,RMOTOR,ProfileAcceleration,ProfileDeceleration,&eposErrorCode) == 0) {
		//printf("\n -------- SetVelocityProfile_RIGHT Error=%x -------------\n",eposErrorCode);
		publish_event("--------BotNode : SetVelocityProfile_RIGHT Error -------------");
	}
	if (VCS_SetVelocityProfile(g_pKey,LMOTOR,ProfileAcceleration,ProfileDeceleration,&eposErrorCode) == 0) {
		//printf("\n -------- SetVelocityProfile_LEFT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : SetVelocityProfile_LEFT Error -------------");
		ROS_INFO("--------BotNode : SetVelocityProfile_LEFT Error -------------");
	}	

	objectindx = 0x6410;
	subIndx = 0x01;
	noByteToWrite = 2;
	bparam[0] = (unsigned char)(0x00ff & MotorMaxContinuousCurrent);
	bparam[1] = (unsigned char)((0xff00 & MotorMaxContinuousCurrent) >> 8);
	//if (VCS_SetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,noByteToWrite,&pNbOfBytesWritten,&eposErrorCode) == 0)
	//{
	//	//printf("\n -------- SetMaxContCurrent_RIGHT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetMaxContCurrent_RIGHT Error -------------");
	//}
	//if (VCS_SetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,noByteToWrite,&pNbOfBytesWritten,&eposErrorCode) == 0)
	//{
	//	//printf("\n -------- SetMaxContCurrent_LEFT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetMaxContCurrent_LEFT Error -------------");
	//}
	objectindx = 0x6410;
	subIndx = 0x02;
	noByteToWrite = 2;
	bparam[0] = (unsigned char)(0x00ff & (MotorMaxContinuousCurrent*2));
	bparam[1] = (unsigned char)((0xff00 & (MotorMaxContinuousCurrent*2)) >> 8);
	//if (VCS_SetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,noByteToWrite,&pNbOfBytesWritten,&eposErrorCode) == 0)
	//{
	//	//printf("\n -------- SetMaxPeakCurrent_RIGHT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetMaxPeakCurrent_RIGHT Error -------------");
	//}
	//if (VCS_SetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,noByteToWrite,&pNbOfBytesWritten,&eposErrorCode) == 0)
	//{
	//	//printf("\n -------- SetMaxPeakCurrent_LEFT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : SetMaxPeakCurrent_LEFT Error -------------");
	//}
	objectindx = 0x6410;
	subIndx = 0x05;
	noByteToWrite = 2;
	bparam[0] = (unsigned char)(0x00ff & (ThermalTimeConstantWinding));
	bparam[1] = (unsigned char)((0xff00 & (ThermalTimeConstantWinding)) >> 8);
	//if (VCS_SetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,noByteToWrite,&pNbOfBytesWritten,&eposErrorCode) == 0)
	//{
	//	//printf("\n -------- ThermalTimeConstantWinding_RIGHT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : ThermalTimeConstantWinding_RIGHT Error -------------");
	//}
	//if (VCS_SetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,noByteToWrite,&pNbOfBytesWritten,&eposErrorCode) == 0)
	//{
	//	//printf("\n -------- ThermalTimeConstantWinding_LEFT Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : ThermalTimeConstantWinding_LEFT Error -------------");
	//}
	if (VCS_SetPositionProfile(g_pKey,RMOTOR,ProfileVelocity,ProfileAcceleration,ProfileDeceleration,&eposErrorCode) == 0) {
		//printf("\n -------- SetPositionProfile_RIGHT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : SetPositionProfile_RIGHT Error -------------");
		ROS_INFO("--------BotNode : SetPositionProfile_RIGHT Error -------------");
	}	
	if (VCS_SetPositionProfile(g_pKey,LMOTOR,ProfileVelocity,ProfileAcceleration,ProfileDeceleration,&eposErrorCode) == 0) {
		//printf("\n -------- SetPositionProfile_LEFT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : SetPositionProfile_LEFT Error -------------");
		ROS_INFO("--------BotNode : SetPositionProfile_LEFT Error -------------");
	}	
	objectindx = 0x6086;
	subIndx = 0x00;
	bparam[0] = 0;  // TRAPEZOIDAL
	noByteToWrite = 1;
	if (VCS_SetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,noByteToWrite,&BytesWritten,&eposErrorCode) == 0)
	{
		//printf("\n -------- Set MotionProfileType_RIGHT. Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : Set MotionProfileType_RIGHT Error -------------");
		ROS_INFO("--------BotNode : Set MotionProfileType_RIGHT Error -------------");
	}
	if (VCS_SetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,noByteToWrite,&BytesWritten,&eposErrorCode) == 0)
	{
		//printf("\n -------- Set MotionProfileType_LEFT. Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : Set MotionProfileType_LEFT Error -------------");
		ROS_INFO("--------BotNode : Set MotionProfileType_LEFT Error -------------");
	}
	objectindx = 0x607F; // set max profile velocity
	subIndx = 0x00;
	bparam[0] = (unsigned char)(0x000000ff & MaxProfileVelocity);
	bparam[1] = (unsigned char)((0x0000ff00 & MaxProfileVelocity) >> 8);
	bparam[2] = (unsigned char)((0x00ff0000 & MaxProfileVelocity) >> 16);
	bparam[3] = (unsigned char)((0xff000000 & MaxProfileVelocity) >> 24);
	noByteToWrite = 4;
	if (VCS_SetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,noByteToWrite,&BytesWritten,&eposErrorCode) == 0)
	{
		//printf("\n -------- Set MaxProfileVelocity_RIGHT. Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : Set MaxProfileVelocity_RIGHT Error -------------");
		ROS_INFO("--------BotNode : Set MaxProfileVelocity_RIGHT Error -------------");
	}
	if (VCS_SetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,noByteToWrite,&BytesWritten,&eposErrorCode) == 0)
	{
		//printf("\n -------- Set MaxProfileVelocity_LEFT. Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : Set MaxProfileVelocity_LEFT Error -------------");
		ROS_INFO("--------BotNode : Set MaxProfileVelocity_LEFT Error -------------");
	}
	objectindx = 0x605C;  // Disable Operation Option Code
	subIndx = 0x00;
	bparam[0] = 1;  //Decelerate with slowdown ramp; disabling of the drive function
	//noByteToWrite = 1;
	//if (VCS_SetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,noByteToWrite,&BytesWritten,&eposErrorCode) == 0)
	//{
	//	//printf("\n -------- Set DisableOperationOptionCode_RIGHT. Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : Set DisableOperationOptionCode_RIGHT Error -------------");
	//}
	//if (VCS_SetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,noByteToWrite,&BytesWritten,&eposErrorCode) == 0)
	//{
	//	//printf("\n -------- Set DisableOperationOptionCode_LEFT. Error=%x -------------\n",eposErrorCode);
	//	publish_event("--------BotNode : Set DisableOperationOptionCode_LEFT Error -------------");
	//}
	
	getPosition();
}

void BotNode::setVelocityMode()
{
	bool rs;
	unsigned char bparam[10];
	unsigned short objectindx = 0x6060;
	unsigned char subIndx = 0x00;
	unsigned int BytesWritten;
	unsigned int NoByteToWrite;
	
	bparam[0] = 3;  // profile velocity
	bparam[1] = 0x00;
	bparam[2] = 0x00;
	bparam[3] = 0x00;
	NoByteToWrite = 1;
	if (VCS_SetObject(g_pKey,RMOTOR,objectindx,subIndx,bparam,NoByteToWrite,&BytesWritten,&eposErrorCode) == 0)
	{
		//printf("\n -------- Set VelocityProfile_Right. Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : Set VelocityProfile_Right Error -------------");
		ROS_INFO("--------BotNode : Set VelocityProfile_Right Error -------------");
	}
	if (VCS_SetObject(g_pKey,LMOTOR,objectindx,subIndx,bparam,NoByteToWrite,&BytesWritten,&eposErrorCode) == 0)
	{
		//printf("\n -------- Set VelocityProfile_Left. Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : Set VelocityProfile_Left Error -------------");
		ROS_INFO("--------BotNode : Set VelocityProfile_Left Error -------------");
	}
}

void BotNode::MotorSetup() 
{
	BOOL oIsEnabled = 0;
	
	mm_per_count = M_PI*WHEEL_DIA*wheel_circum_correction/(STEPS_PER_REV*GEAR_RATIO);
	rpm_factor = ( 1000.0/(M_PI*WHEEL_DIA*wheel_circum_correction) * 60 * GEAR_RATIO ) ;
	radpm_factor = (0.5*AXLE_LEN*wheel_base_correction/(M_PI*WHEEL_DIA*wheel_circum_correction) * 60 * GEAR_RATIO );
	dist_factor=(1000.0/(M_PI*WHEEL_DIA*wheel_circum_correction) * STEPS_PER_REV * GEAR_RATIO );
	angle_factor=( 1.0/180.0 * 0.5*AXLE_LEN*wheel_base_correction / (WHEEL_DIA*wheel_circum_correction) * STEPS_PER_REV * GEAR_RATIO );
	displacement_factor = odom_angular_scale_correction / AXLE_LEN ;
	
	SetDefaultParameters();
	if(OpenDevice(&eposErrorCode)!=MMC_SUCCESS)
	{
		//printf("\n -------- OpenDevice Error=%x -------------\n",eposErrorCode);
		//publish_event("--------BotNode : OpenDevice Error -------------");
		ROS_INFO("--------  USBEpos : OpenDevice Error -------------");
	} else {
		//printf("\n-------- OpenDevice OK -------------\n");
		ROS_INFO("--------  USBEpos : OpenDevice OK -------------");
	}
	
	if (VCS_SetDisableState(g_pKey, RMOTOR,&eposErrorCode) == 0) {
		//printf("\n -------- SetDisableState_RIGHT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------USBEpos : SetDisableState_RIGHT Error -------------");
		ROS_INFO("--------  USBEpos : SetDisableState_RIGHT Error -------------");
	}
	if (VCS_SetDisableState(g_pKey, LMOTOR,&eposErrorCode) == 0) {
		//printf("\n -------- SetDisableState_LEFT Error=%x -------------\n",eposErrorCode);
		//publish_event("--------USBEpos : SetDisableState_LEFT Error -------------");
		ROS_INFO("--------USBEpos : SetDisableState_LEFT Error -------------");
	}
	
	//if (VCS_SetMotorType(g_pKey,RMOTOR,1,&eposErrorCode) == 0)
	//{
	//	//printf("---- SetMotorType_RIGHT Failed : Error=%d ---------\n",eposErrorCode);
	//	publish_event("--------USBEpos : SetMotorType_RIGHT Failed Error -------------");
	//} 
	
	//if (VCS_SetMotorType(g_pKey,LMOTOR,1, &eposErrorCode) == 0)
	//{
	//	//printf("---- SetMotorType_2 Failed : Error=%d ---------\n",eposErrorCode);
	//	publish_event("--------USBEpos : SetMotorType_2 Failed Error -------------");
	//} 
	if(VCS_ClearFault(g_pKey, RMOTOR,&eposErrorCode) == 0) {
		//printf("---- ClearFault_1 Failed : Error=%d ---------\n",eposErrorCode);
		//publish_event("--------USBEpos : ClearFault_1 Failed Error -------------");
		ROS_INFO("--------USBEpos : ClearFault_1 Failed Error -------------");
	}
	if(VCS_ClearFault(g_pKey, LMOTOR,&eposErrorCode) == 0) {
		//printf("---- ClearFault_2 Failed : Error=%d ---------\n",eposErrorCode);
		//publish_event("--------USBEpos : ClearFault_2 Failed Error -------------");
		ROS_INFO("--------USBEpos : ClearFault_2 Failed Error -------------");
	}
	if(VCS_SetEnableState(g_pKey, RMOTOR,&eposErrorCode) == 0) {
		//printf("---- SetEnableState_1 Failed : Error=%d ---------\n",eposErrorCode);
		//publish_event("--------USBEpos : SetEnableState_1 Failed Error -------------");
		ROS_INFO("--------USBEpos : SetEnableState_1 Failed Error -------------");
	} 
	if(VCS_SetEnableState(g_pKey, LMOTOR,&eposErrorCode) == 0) {
		//printf("---- SetEnableState_2 Failed : Error=%d ---------\n",eposErrorCode);
		//publish_event("--------USBEpos : SetEnableState_2 Failed Error -------------");
		ROS_INFO("--------USBEpos : SetEnableState_2 Failed Error -------------");
	} 
	setParams();	
	setVelocityMode();
}

void BotNode::SetDefaultParameters()
{
	RMOTOR = 1;
	g_deviceName = "EPOS2"; 
	g_protocolStackName = "MAXON SERIAL V2";
	g_interfaceName = "USB"; 
	g_portName = "USB0"; 
	g_baudrate = 1000000; 

	LMOTOR = 2;
	g_subdeviceName = "EPOS2"; 
	g_subprotocolStackName = "CANopen";  
}

int BotNode::OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	g_pKey = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName,&eposErrorCode);

	if(g_pKey!=0 && eposErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKey, &lBaudrate, &lTimeout, &eposErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKey, g_baudrate, lTimeout, &eposErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKey, &lBaudrate, &lTimeout, &eposErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKey = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

void BotNode::readPathfromFile() 
{		
	char buf[50];
	int slp,elp,stn,d;
	std::string s1;
	FILE *pfp;
	double tx,ty,tz,rx,ry,rz,rw;
	double tx1,ty1,tz1,rx1,ry1,rz1,rw1;

	pfp = fopen(pathfile.c_str(), "r");
  if (pfp == NULL) {
  	ROS_INFO("----- USBEpos : I couldn't open pathdata.dat for reading. ---------\n");    
		//publish_event("--------BotNode : SI couldn't open pathdata.dat for reading -------------");
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

		tx1 = pathInfo[slp][elp][d][0];
		ty1 = pathInfo[slp][elp][d][1];
		tz1 = pathInfo[slp][elp][d][2];
		rx1 = pathInfo[slp][elp][d][3];
		ry1 = pathInfo[slp][elp][d][4];
		rz1 = pathInfo[slp][elp][d][5];
		rw1 = pathInfo[slp][elp][d][6];		
		//ROS_INFO("---- BotNode : Path %d to %d. pp=%d : x=%.3f.y=%.3f.z=%.3f.rx=%.3f.ry=%.3f.rz=%.3f.rw=%.3f---",slp,elp,d,tx1,ty1,tz1,rx1,ry1,rz1,rw1);
		d++;
		pathNoPoint[slp][elp] = d;
	}
  fclose(pfp);
}

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
  	 	//ROS_INFO("------ CheckPlan : Empty Plan ------");
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
  	ROS_INFO("------ USBEpos : Failed to call make plan service ----------");
		//publish_event("--------BotNode : CheckPlan : Failed to call make plan service -------------");
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
	//ROS_INFO(" --------- testcheckPlan A -----------");
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
	//ROS_INFO(" --------- testcheckPlan B -----------");
	if (makeplan.call(srv)) {
		if (srv.response.plan.poses.empty()) {
			//publish_debug("Move_Node : Got empty plan" );
  	 	//ROS_INFO("BtNode_Node : Got empty plan");
			ret = false;
    } else {
			//ROS_INFO("BtNode_Node : Found Plan" );
			ret = true;
		}
 	} else {
		//publish_debug("Move_Node : Failed to call make plan service");
  	//ROS_INFO("BtNode_Node : Failed to call make plan service");
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

void BotNode::goalLPCallback(const htbot::move::ConstPtr& msg)
{

	{ 
		boost::mutex::scoped_lock lock(publish_mutex_);
		gx = msg->x;
		gy = msg->y;
		grz = msg->rz;
		grw = msg->rw;
	}
	
	return;
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

void BotNode::turnToPath(double tx, double ty)
{
	double an,yawp,diff;
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	if ( (tx >= px) && (ty >= py)) {
		// Q1
		an = atan((ty-py)/(tx-px));
	} else {
		if ( (px >= tx) && (ty >= py) ) {
			// Q2
			an = atan((ty-py)/(px-tx));
			an = PI - an;
		} else {
			if ( (px >= tx) && (py >= ty) ) {
				// Q3
				an = atan((py-ty)/(px-tx));
				an = -(PI - an);
			} else {
				// Q4
				an = -(atan((py-ty)/(tx-px)));
			}
		}
	}
	diff = angles::shortest_angular_distance(yawp, an);
	an = 57.2958 * diff;
	if (fabs(an) > MAXANGLETURN) {
		moveAngle(an);
	}
	return;
}

void BotNode::turnToPathN(double tx, double ty)
{
	double an,yawp,diff;
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	if ( (tx >= px) && (ty >= py)) {
		// Q1
		an = atan((ty-py)/(tx-px));
	} else {
		if ( (px >= tx) && (ty >= py) ) {
			// Q2
			an = atan((ty-py)/(px-tx));
			an = PI - an;
		} else {
			if ( (px >= tx) && (py >= ty) ) {
				// Q3
				an = atan((py-ty)/(px-tx));
				an = -(PI - an);
			} else {
				// Q4
				an = -(atan((py-ty)/(tx-px)));
			}
		}
	}
	diff = angles::shortest_angular_distance(yawp, an);
	an = 57.2958 * diff;
	moveAngle(an);
	return;
}

void BotNode::turnToOrientation()
{
	double yawg,yawp,diff;
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	tf::Quaternion qg(0.0,0.0,grz,grw);
	yawg = tf::getYaw(qg);
	diff = angles::shortest_angular_distance(yawp, yawg);
	moveAngle(57.2958 * diff);
	return;
}

void BotNode::test_pid_angle(int slp, int elp, int pp)
{
	ros::NodeHandle nn;

	Kp = 0.45;
	AKp = 0.7;
	Ki = 0.00;
	Kd = 0.0;
	AKd = 0.01;

	DKp = 1.3;
	DKi = 0.000;
	DKd = 0.01;
	
	nn.getParam("PID_AKp",AKp);
	nn.getParam("PID_Kp",Kp);
	nn.getParam("PID_AKd",AKd);
	nn.getParam("PID_Kd",Kd);
	nn.getParam("PID_DKp",DKp);
	nn.getParam("PID_DKd",DKd);
	//ROS_INFO("---- PID Kp=%.3f. Kd=%.3f. DKp=%.3f. DKd=%.3f -----",Kp,Kd,DKp,DKd);
/*
	px = 20.8749;
	py = 11.5080;
	pz = 0.0000;
	prx = 0.0000;
	pry = 0.0000;
	prz = -0.6911;
	prw = 0.7228;
*/
	alignflag = true;
	angle_rot = 0.55;
	
	if (pp == 0) {
		pathmaxpoint = pathNoPoint[slp][elp];
	}
	gx = pathInfo[slp][elp][pp][0];
	gy = pathInfo[slp][elp][pp][1];
	gz = pathInfo[slp][elp][pp][2];
	grx = pathInfo[slp][elp][pp][3];
	gry = pathInfo[slp][elp][pp][4];
	grz = pathInfo[slp][elp][pp][5];
	grw = pathInfo[slp][elp][pp][6];

	d_Temp = 0.0;
	i_Temp = 0.0;
	d_DTemp = 0.0;
	i_DTemp = 0.0;
	Angle_Err_Value = 0.0;
	delay_count = 0;
	
}

double BotNode::pid_checkangle_NLP()
{
	double angle_to_goal,tyawp;
	tf::Quaternion qp(0.0,0.0,prz,prw);
	tyawp = tf::getYaw(qp);	
	angle_to_goal = ptpAngle(); // find angle to goal
	Angle_Err_Value = angles::shortest_angular_distance(tyawp, angle_to_goal);
	return fabs(Angle_Err_Value);
}


void BotNode::pid_angle_dist()
{
	double angle_to_goal,tyawp,adrive;
	double dx,dy,ddrive;
	ros::NodeHandle nn;

	tf::Quaternion qp(0.0,0.0,prz,prw);
	tyawp = tf::getYaw(qp);	
	angle_to_goal = ptpAngle(); // find angle to goal
	Angle_Err_Value = angles::shortest_angular_distance(tyawp, angle_to_goal);
	P_Term = Kp * Angle_Err_Value;
	//i_Temp += Angle_Err_Value;
	//if (i_Temp > 1.0) {
	//	i_Temp = 1.0;
	//} else {
	//	if (i_Temp < -1.0) {
	//		i_Temp = -1.0;
	//	}
	//}
	//I_Term = Ki * i_Temp;
	D_Term = Kd * (d_Temp - Angle_Err_Value);
	d_Temp = Angle_Err_Value;
	//adrive = P_Term + I_Term + D_Term;
	adrive = P_Term + D_Term;
	/*
	if (Angle_Err_Value > 0.0) {
		if (adrive > 0.5) {
			adrive = 0.5;
		}
	} else {
		if (adrive < -0.5) {
			adrive = -0.5;
		}
	}
	*/
	//if (fabs(Angle_Err_Value) < 0.08) {
	//	adrive = adrive * 0.0;
	//}
	// dist	
	dx = gx - px;
	dy = gy - py;
	Dist_Err_Value = sqrt((dx * dx) + (dy * dy));
	P_DTerm = DKp * Dist_Err_Value;
	//i_DTemp += Dist_Err_Value;
	//if (i_DTemp > 1.0) {
	//	i_DTemp = 1.0;
	//} else {
	//	if (i_DTemp < -1.0) {
	//		i_DTemp = -1.0;
	//	}
	//}
	//I_DTerm = DKi * i_DTemp;
	D_DTerm = DKd * (d_DTemp - Dist_Err_Value);
	d_DTemp = Dist_Err_Value;
	//ddrive = P_DTerm + I_DTerm + D_DTerm;
	ddrive = P_DTerm + D_DTerm;
	if (ddrive > 0.5) {
		ddrive = 0.5;
	}
	//if (Dist_Err_Value < 0.35) {
	//	ddrive = 0.06;
	//}
	/*
	if (Dist_Err_Value < 0.7) {
		if (ddrive > 0.2) {
			ddrive = 0.2;
		}
		if (Dist_Err_Value < 0.4) {
			if (ddrive > 0.13) {
				ddrive = 0.13;
			}
			if (Dist_Err_Value < 0.2) {
				if (ddrive > 0.05) {
					ddrive = 0.05;
				}
			}
		}
		if (lastPoint) {
			if (Dist_Err_Value < 0.23) {
				ddrive = 0.0;
				adrive = 0.0;
				Cancel_PID = true;
				nn.setParam("Cancel_PID",true);
			}
		} else {
			if (Dist_Err_Value < 0.45) {
				//ddrive = 0.0;
				//adrive = 0.0;
				Cancel_PID = true;
				nn.setParam("Cancel_PID",true);
				ROS_INFO("---PID Exit AErr=%.3f. DErr=%.3f. adrive=%.3f. ddrive=%.3f. ---",Angle_Err_Value,Dist_Err_Value,adrive,ddrive);
			}
		}
	}
	*/
	if (lastPoint) {
		if (fabs(Dist_Err_Value) < 0.5) {
			if (ddrive > 0.04) {
				ddrive = 0.04;
			}
			if (fabs(Dist_Err_Value) < 0.06) {
				ddrive = 0.0;
				adrive = 0.0;
				Cancel_PID = true;
				nn.setParam("Cancel_PID",true);				
			}
			//ROS_INFO("---PID LastP AErr=%.3f. DErr=%.3f. adrive=%.3f. ddrive=%.3f. ---",Angle_Err_Value,Dist_Err_Value,adrive,ddrive);
		}
	} else {
		if (fabs(Dist_Err_Value) < 0.2) {
			//ddrive = 0.0;
			//adrive = 0.0;
			Cancel_PID = true;
			nn.setParam("Cancel_PID",true);
			//ROS_INFO("---PID NextP AErr=%.3f. DErr=%.3f. adrive=%.3f. ddrive=%.3f. ---",Angle_Err_Value,Dist_Err_Value,adrive,ddrive);
		}
	}
	/*
	if (!alignflag) {
		if (fabs(Angle_Err_Value) > 0.08) {
			ddrive = ddrive * 0.0;
			if (fabs(Angle_Err_Value) <= 0.08) {
				adrive = adrive * 0.0;//0.0;
				//Kp = 0.4;
				//Kd = 0.03;
				alignflag = true;
				delay_count = 0;
			}
		} else {
			//adrive = 0.0;
			adrive = adrive * 0.0;
			//Kp = 0.4;
			//Kd = 0.03;
			alignflag = true;	
			delay_count = 0;		
		}
		moveVelocity(ddrive,adrive);
	}
	*/
	ddrive = ddrive * fabs(1.0 - fabs(Angle_Err_Value/0.9));
	//ROS_INFO("---AErr=%.3f. DErr=%.3f. adrive=%.3f. ddrive=%.3f. ---",Angle_Err_Value,Dist_Err_Value,adrive,ddrive);
	//if (delay_count++ > 8) {
	//	moveVelocity(ddrive,adrive);
	//}
	moveVelocity(ddrive,adrive);
}

void BotNode::pid_angle()
{
	double angle_to_goal,tyawp,adrive;
	ros::NodeHandle nn;
	tf::Quaternion qp(0.0,0.0,prz,prw);
	tyawp = tf::getYaw(qp);	
	angle_to_goal = ptpAngle(); // find angle to goal
	Angle_Err_Value = angles::shortest_angular_distance(tyawp, angle_to_goal);
	
	P_Term = AKp * Angle_Err_Value;
	//i_Temp += Angle_Err_Value;
	//if (i_Temp > 1.0) {
	//	i_Temp = 1.0;
	//} else {
	//	if (i_Temp < -1.0) {
	//		i_Temp = -1.0;
	//	}
	//}
	//I_Term = Ki * i_Temp;
	D_Term = AKd * (d_Temp - Angle_Err_Value);
	d_Temp = Angle_Err_Value;
	//adrive = P_Term + I_Term + D_Term;
	adrive = P_Term + D_Term;
	//ROS_INFO("--------------------------------------------------------");
	//ROS_INFO("---Angle PID : agoal=%.3f. Err=%.3f. i_Temp=%.3f. d_Temp=%.3f---",angle_to_goal,Angle_Err_Value,i_Temp,d_Temp);
	//ROS_INFO("---PT=%.3f. IT=%.3f. DT=%.3f. TT=%.3f. ---",P_Term,I_Term,D_Term,adrive);
	//ROS_INFO("--------------------------------------------------------");

	//ROS_INFO("---PID : agoal=%.3f. Err=%.3f. PT=%.3f.---",angle_to_goal,Angle_Err_Value,P_Term);
	//ROS_INFO("---IT=%.3f. DT=%.3f. TT=%.3f. iTemp=%.3f. d_Temp=%.3f---",I_Term,D_Term,adrive,i_Temp,d_Temp);
	//ROS_INFO("---PID : agoal=%.3f. Err=%.3f. PT=%.3f.---",angle_to_goal,Angle_Err_Value,P_Term);
	
	if (fabs(Angle_Err_Value) < 0.06) {
		adrive = 0.0;
		Cancel_PID = true;
		nn.setParam("Cancel_PID",true);
		//return;
		//ROS_INFO("---PID Angle Exit : Err=%.3f. adrive=%.3f.---",Angle_Err_Value,adrive);
	}
	moveVelocity(0.0,adrive);
	
	//ROS_INFO("---PID Angle : Err=%.3f. adrive=%.3f.---",Angle_Err_Value,adrive);
}

void BotNode::calc_dist_error() {
	double dx,dy;
	dx = gx - px;
	dy = gy - py;
	Dist_Err_Value = sqrt((dx * dx) + (dy * dy));
	switch(qtrum) {
		case 1:
			if ((py>=gy) && (px>=gx)) {
				Dist_Err_Value = Dist_Err_Value ;
			} else {
				Dist_Err_Value = -Dist_Err_Value ;
			}
			break;
		case 2:
			if ((py>=gy) && (gx>px)) {
				Dist_Err_Value = Dist_Err_Value ;
			} else {
				Dist_Err_Value = -Dist_Err_Value ;
			}
			break;
		case 3:
			if ((gy>py) && (gx>px)) {
				Dist_Err_Value = Dist_Err_Value ;
			} else {
				Dist_Err_Value = -Dist_Err_Value ;
			}
			break;
		case 4:
			if ((gy>py) && (px>gx)) {
				Dist_Err_Value = Dist_Err_Value ;
			} else {
				Dist_Err_Value = -Dist_Err_Value ;
			}
			break;
	}
}

void BotNode::pid_dist()
{
	double dx,dy,ddrive;
	ros::NodeHandle nn;
	dx = gx - px;
	dy = gy - py;
	Dist_Err_Value = sqrt((dx * dx) + (dy * dy));
	//calc_dist_error();

	P_DTerm = DKp * Dist_Err_Value;
	i_DTemp += Dist_Err_Value;
	if (i_DTemp > 1.0) {
		i_DTemp = 1.0;ros::NodeHandle nn;
	} else {
		if (i_DTemp < -1.0) {
			i_DTemp = -1.0;
		}
	}
	I_DTerm = DKi * i_DTemp;
	D_DTerm = DKd * (d_DTemp - Dist_Err_Value);
	d_DTemp = Dist_Err_Value;
	ddrive = P_DTerm + I_DTerm + D_DTerm;
	//ROS_INFO("--------------------------------------------------------");
	//ROS_INFO("---PID Dist : Err=%.3f. i_DTemp=%.3f. d_DTemp=%.3f---",Dist_Err_Value,i_DTemp,d_DTemp);
	//ROS_INFO("---DPT=%.3f. DIT=%.3f. DDT=%.3f. DTT=%.3f. ---",P_DTerm,I_DTerm,D_DTerm,ddrive);
	//ROS_INFO("--------------------------------------------------------");
	if (ddrive > 0.5) {
		ddrive = 0.5;
	}
	if (Dist_Err_Value < 0.7) {
		if (ddrive > 0.2) {
			ddrive = 0.2;
		}
		if (Dist_Err_Value < 0.5) {
			if (ddrive > 0.1) {
				ddrive = 0.1;
			}
			if (Dist_Err_Value < 0.3) {
				if (ddrive > 0.04) {
					ddrive = 0.04;
				}
			}
		}
		if (Dist_Err_Value < 0.23) {
			ddrive = 0.0;
			Cancel_PID = true;
			nn.setParam("Cancel_PID",true);
			//ROS_INFO("---PID Dist Exit : Err=%.3f. ddrive=%.3f.---",Dist_Err_Value,ddrive);
		}
	}
	moveVelocity(ddrive,0.0);
	//ROS_INFO("---PID Dist : Err=%.3f. ddrive=%.3f.---",Dist_Err_Value,ddrive);
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
  // calculate angle to goal(gx,gy) from robot positioros::NodeHandle nn;n(px,py)
	an = ptpAngle();
	diff = angles::shortest_angular_distance(yawp, an);
	//angle = 57.2958 * diff;
	dd = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
	
	if (dd < 0.07) {
		linear_ = 0.0;
		angular_ = 0.0;
		moveVelocity(linear_,angular_);
		ptpFlag = false;		
		diff = angles::shortest_angular_distance(yawp, yawg);
		angle = 57.2958 * diff;
		moveAngle(angle);	
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
	moveVelocity(linear_,angular_);
	
}

void BotNode::ptpmoveAlign()
{
	double av,lv,dd,an,diff,angle,idist;
	
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);
	tf::Quaternion qg(0.0,0.0,grz,grw);
	yawg = tf::getYaw(qg);
	an = ptpAngle();
	diff = angles::shortest_angular_distance(yawp, an);
	angle = 57.2958 * diff;
	dd = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
	moveAngle(angle);	
	usleep(200000);
	moveDistanceSpeed(dd,750);
	//sleep(1);
	//turnToOrientation();	
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
	//moveVelocity(linear_,angular_);
	
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
	//ROS_INFO("ptpmove : ptpAngle = %.3f. Robot Angle = %.3f. diff = %.3f. ptpDist = %.2f",an,yawp,diff,ptpDist);
	moveAngle(angle);	
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
	//nn.setParam("navstatus",0);
	nn.setParam("Cancel_Nav",false);
	nn.setParam("Change_Nav",true);
	ROS_INFO("----------- USBEpos : Cancel Goal -------------");
	return;
}

void BotNode::currCallback(const std_msgs::Float32::ConstPtr& msg)
{

	{ 
		boost::mutex::scoped_lock lock(publish_mutex_);
		current = msg->data;	
		if (current < 0.0) {
			negativecurrent = true;
		}	
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
	//nn.getParam("LeftLaserObsDist",leftlaserobsdist);
	//nn.getParam("RightLaserObsDist",rightlaserobsdist);
	//ROS_INFO("******* ObsDist:%.3f. LeftLaserObsDist:%.3f. RightLaserObsDist:%.3f. MapObsDist:%.3f. ************",obsdist,leftlaserobsdist,rightlaserobsdist,mapobsdist);
	
	//ROS_INFO("Start find Path : Angular=%.3f. obsdist=%.3f. obsddist=%.3f",angular_,obsdist,obsddist);
	//if (obsdist <= 0.02) {
	//	 	angular_ = 0.0;
	//		ROS_INFO(" @@@@@@@@@@@@@@@@  obsdist < 0.03  @@@@@@@@@@@@@@@@@@@@@@@@");
	//		return true;
	//}
	
	if (sprz == 9.0) {
		nn.getParam("MapDirectionToSearch",mapdirection);
		nn.getParam("DirectionToSearch",moreopen);		
		//ROS_INFO("************** mapdirection:%d. moreopen:%d. ****************",mapdirection,moreopen);
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
		//ROS_INFO(" Align2 : dist=%.3f. yawp=%.3f. start=%.3f",distStart,yawp,yawStart);
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
		//ROS_INFO(" Rotate First ");
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
		//ROS_INFO(" Intermediate LP ");
	}
	//ptpDist = sqrt(((gx-px) * (gx-px)) + ((gy-py) * (gy-py)));
	
	//ROS_INFO(" getNextPtN :  gx=%.3f. gy=%.3f. AngleToGoal=%.3f. ptpDist=%.3f.",gx,gy,angleToGoal,ptpDist);
	//ROS_INFO(" ------------------------------------------------------------");
}

void BotNode::findNewPathForward(void) {
	ros::NodeHandle nm;
	bool ret;
	htbot::goal msg;

	//ROS_INFO("----- Find New Path. cptr=%d. eptr=%d ------- ",cmovePtr,emovePtr);
	if (cmovePtr <= emovePtr) {
		msg.x = nposeInfo[cmovePtr][0];
		msg.y = nposeInfo[cmovePtr][1];
		//ROS_INFO("--- findNewPathForward Next Location : x=%.3f. y=%.3f -----",msg.x,msg.y);
		chkplan_pub.publish(msg);
		nm.setParam("checkPlanDone",false);
	}
}

void BotNode::checkpath(void) {

	htbot::goal msg;
	msg.x = gx;
	msg.y = gy;
	chkplan_pub.publish(msg);
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
	moveVelocity(linear_,angular_);
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
	moveVelocity(linear_,angular_);
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
			publish_sound(NEEDSPACE,0,3);
			if (!stopFlag) {
				stopFlag = true;
				stopTime = ros::Time::now();
			} else {
				if ( ros::Time::now() > (stopTime + ros::Duration(rePlanTime)) ) {
					findnewpath = true;
					publish_sound(FINDPATH,0,3);
					linear_ = 0.0;
					angular_ = 0.0;
					//stopMove = true;
				}
			}
			moveVelocity(linear_,angular_);
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
			//ROS_INFO("********** Reached Dist Limit. diff=%.3f. dd=%.3f. linear=%.3f. angular=%.3f *******",diff,dd,linear_,angular_);
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
	moveVelocity(linear_,angular_);
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
			moveVelocity(linear_,angular_);
			//ROS_INFO("********** Reached and Aligned to ptpAlign Destination  *******");
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
		publish_sound(NEEDSPACE,0,3);
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
		//ROS_INFO("********** Reached Dist Limit. dd=%.3f.  *******",dd);
		linear_ = 0.0;		
		//angular_ = 0.0;
		alignDistReached = true;
		//publish_sound(NEEDSPACE,0,3);
	}
	moveVelocity(linear_,angular_);
}

void BotNode::racplanMovealignRobot()
{
	double an,av,diff;
	
	tf::Quaternion qp(0.0,0.0,prz,prw);
	yawp = tf::getYaw(qp);

	if ( (racplanMoveX >= px) && (racplanMoveY >= py)) {
		// Q1
		an = atan((racplanMoveY-py)/(racplanMoveX-px));
	} else {
		if ( (px >= racplanMoveX) && (racplanMoveY >= py) ) {
			// Q2
			an = atan((racplanMoveY-py)/(px-racplanMoveX));
			an = PI - an;
		} else {
			if ( (px >= racplanMoveX) && (py >= racplanMoveY) ) {
				// Q3
				an = atan((py-racplanMoveY)/(px-racplanMoveX));
				an = -(PI - an);
			} else {
				// Q4
				an = -(atan((py-racplanMoveY)/(racplanMoveX-px)));
			}
		}
	}
  
	diff = angles::shortest_angular_distance(yawp, an);
	av = maxAngleVel * fabs(diff/0.8);
	//if (av > 0.2) {
	//	av = 0.2;
	//}
	//if (av < 0.08) {
	//	av = 0.08;
	//}
	//av = fabs(diff / 0.7);
	if (diff > 0.0) {
		angular_ = angular_ + av;
	} else {
		angular_ = angular_ - av;
	}
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
		//ROS_INFO("Alignment Move Stopped. diff=%.3f",diff);
	}
	moveVelocity(linear_,angular_);
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
			quickstop = true;
			sprintf(buf,"BotNode : Stop Robot"); 
			s.assign(buf,strlen(buf));
			publish_event(s);
			publish_status("Docking Activated");
			break;
		case 15:
			//estop_pressed = true;
			//ROS_INFO("estop pressed");
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
/*
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
*/
void BotNode::publish_sound(int id,int sd, int rsd)
{
	ros::NodeHandle n; 
	ROS_INFO("-------- movNode Publish Sound : %d. ----------",id);	
	n.setParam("SoundCMD",id);	
	usleep(50000);
	n.setParam("SoundON",true);
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
	//publish_debug("bot_Node : Publish Clear Map" );
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
	moveVelocity(0.0,0.0);
	MotorSetup();
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
	angular_=  msg->angular.z;
	linear_ =  msg->linear.x;
	//ROS_INFO("Vel : Move at vx [%f] & w [%f] & offset [%.6f]", linear_, angular_,cmdvel_offset);
	last_cmd_time = ros::Time::now();
	/*
	if (linear_ < 0.0) {
		if (!escape) {
			escapeTime = ros::Time::now();
			escape = true;
		}
	} else {
		if (linear_ > 0.0) {
			escape = false;
		}
	}
	*/
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
	//publish_debug(s1);
}

void BotNode::posCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	
	//bool st;
	double dd,trz,dx,dy,pa,ga,xa,ya;
	double aa,aan,da,dan,dat,dant;
	char buf[100];
	string s1;
	bool front;
	ros::NodeHandle nn;

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
		//sprintf(buf,"Align35: outx=%.3f. outy=%.3f. outa=%.3f.",msg->linear.x,msg->linear.y,msg->angular.z);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
		//sprintf(buf,"Align35: alignX=%.3f. alignDX=%.3f. alignAngle=%.3f",alignX,alignDX,alignAngle);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
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
		//sprintf(buf,"Align30: outx=%.3f. outy=%.3f. outa=%.3f.",msg->linear.x,msg->linear.y,msg->angular.z);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
		//sprintf(buf,"Align30A: dx=%.3f. alignX=%.3f. dy=%.3f. alignY=%.3f. alignAngle=%.3f",alignDX,alignX,alignDY,alignY,alignAngle);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
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
		//sprintf(buf,"Align30: outx=%.3f. outy=%.3f. outa=%.3f.",msg->linear.x,msg->linear.y,msg->angular.z);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
		//sprintf(buf,"Align30: dx=%.3f. alignX=%.3f. dy=%.3f. alignY=%.3f. alignAngle=%.3f",alignDX,alignX,alignDY,alignY,alignAngle);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
		startPMove = true;
		//pstate = 9;
		pstate = 100;
		return;
	}

	if (msg->linear.z == 0.0) {
		// normal state m/c move
		startPMove = true;
		docking = true;
		//sprintf(buf,"posCall-startPM = true : Linear : %0.6f. Angle : %0.6f. Speed : %0.6f.",plinear,pangular,pspeed);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
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
		quickstop = true;
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
		quickstop = true;
		//sprintf(buf,"BotNode : Stop Robot"); 
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
		estopped = true;
		return;
	}
	if (msg->linear.z == 7.6) {
		// cancel operation		
		freeMotor();
		//publish_debug("Motors Released");
		//publish_sound(22,0,0);  // motors released
		return;
	}
	if (msg->linear.z == 7.7) {	
		MotorSetup();
		//publish_debug("Motors Re-Engaged");
		//publish_sound(22,0,0);  // motors re-engaged
		return;
	}
	if (msg->linear.z == 7.8) { // estop activated. reduce start velocity
		// slow down re-start speed
		//publish_sound(22,0,0);
		moveVelocity(0.0,0.0);
		//usleep(800000);
		//sleep(1.5);
		estop_pressed = true;
		freeMotor();
		//moveVelocity(0.0,0.0);
		//usleep(100000);
		//ROS_INFO("BotNode :estop pressed");
		//if (navstatus == 7) {
		//	publish_sound(15,0,0);
		//}
		//publish_sound(22,0,0);
		return;
	}
	if (msg->linear.z == 7.9) { // estop release. 
		MotorSetup();
		moveVelocity(0.0,0.0);
		usleep(100000);
		estop_pressed = false;
		//publish_sound(18,0,0);
		//ROS_INFO("BotNode estop released");	
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
		//ROS_INFO("-------- BotNode : Rotate Left ----------");
		return;
	}
	if (msg->linear.z == 9.1) {
		boost::mutex::scoped_lock lock(publish_mutex_);
		startPMove = true;
		pstate = 71;
		//ROS_INFO("-------- BotNode : Rotate Right ----------");
		//publish_debug("Rotate Clockwise");
		return;
	}
	if (msg->linear.z == 9.3) {  // special
		boost::mutex::scoped_lock lock(publish_mutex_);
		startPMove = true;
		pstate = 8;
		//ROS_INFO("-------- BotNode : Move Forward 9.3 ----------");
		//publish_debug("Move Straight Forward");
		return;
	}
	if (msg->linear.z == 9.5) {
		boost::mutex::scoped_lock lock(publish_mutex_);
		startPMove = true;
		pstate = 8;
		//ROS_INFO("-------- BotNode : Move Forward ----------");
		//publish_debug("Move Straight Forward");
		return;
	}
	if (msg->linear.z == 9.6) {
		boost::mutex::scoped_lock lock(publish_mutex_);
		startPMove = true;
		pstate = 81;
		//ROS_INFO("-------- BotNode : Move Reverse ----------");
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
		//sprintf(buf,"Align10: trz=%.3f. prz=%.3f. ga=%.3f. pa=%.3f. align=%.3f. Dist=%.3f",trz,prz,ga,pa,alignAngle,alignDist);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
		//sprintf(buf,"AlignXY10: dx=%.3f. alignX=%.3f. dy=%.3f. alignY=%.3f",alignDX,alignX,alignDY,alignY);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
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
			//sprintf(buf,"Align11O: dx=%.3f. dy=%.3f. dd=%.3f. prz=%.3f. ga=%.3f. pa=%.3f. alignAngle=%.3f",dx,dy,dd,prz,ga,pa,alignAngle);
			//s1.assign(buf,strlen(buf));
			//publish_debug(s1);
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
		//sprintf(buf,"Align11: dx=%.2f. dy=%.2f. dd=%.2f. aa=%.2f. aan=%.2f.",dx,dy,dd,aa,aan);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
		//sprintf(buf,"Align11a: aT=%.2f. aTD=%.2f. ga=%.2f. pa=%.2f. alignAngle=%.2f",alignT,alignTD,ga,pa,alignAngle);
		//s1.assign(buf,strlen(buf));
		//publish_debug(s1);
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

	if (msg->linear.z == 21.0) {
		// turn to path prior to start move		
		turnTopathX = msg->linear.x;
		turnTopathY = msg->angular.z;		
		pstate = 130;
		return;
	}

	if (msg->linear.z == 21.5) {
		// turn to path prior to start move	0.2m	
		turnTopathX = msg->linear.x;
		turnTopathY = msg->angular.z;		
		pstate = 133;
		return;
	}

	if (msg->linear.z == 22.0) {
		// re-localise		
		pstate = 136;
		return;
	}

	if (msg->linear.z == 23.0) {
		// turn to path prior to start move		
		moveVelocity(0.0,0.0); 
		turnTopathX = msg->linear.x;
		turnTopathY = msg->angular.z;		
		pstate = 150;
		delay_time = ros::Time::now();
		return;
	}

	if (msg->linear.z == 23.5) {
		// move to next point	
		moveVelocity(0.0,0.0); 
		turnTopathX = msg->linear.x;
		//turnTopathY = msg->angular.z;		
		pstate = 152;
		delay_time = ros::Time::now();
		return;
	}

	if (msg->linear.z == 45.0) {  // point to point move		
		ptpmoveS();	
		return;
	}
	if (msg->linear.z == 41.3) {
		pstate = 140;
		return;
	}
	if (msg->linear.z == 41.2) {
		pstate = 141;
		return;
	}
	if (msg->linear.z == 41.0) {
		pstate = 142;
		return;
	}
	if (msg->linear.z == 41.7) {
		pstate = 143;
		return;
	}
	if (msg->linear.z == 41.77) {
		pstate = 144;
		return;
	}
	if (msg->linear.z == 41.5) {
		pstate = 145;
		nextPoint = 0;
		lastPoint = false;
		nn.getParam("startLP",startLP);
		nn.getParam("endLP",endLP);
		//ROS_INFO("---------BotNode : ptp. startLP=%d. endLP=%d --------",startLP,endLP);
		return;
	} 
	if (msg->linear.z == 42.0) {  // AGV turn around inside lift
		alignAngle = msg->angular.z;
		pstate = 16;
		return;
	}
	if (msg->linear.z == 42.1) {  // AGV special move to pickup trolley
		trolleywidth = msg->angular.z;
		trolley = msg->linear.x;
		ROS_INFO("---------- UsbEp : move into trolley. type=%.2f. width=%.2f. ------------",trolley,trolleywidth);
		pstate = 17;
		return;
	}
	if (msg->linear.z == 42.2) {  // AGV reverse after pickup trolley
		alignDist = msg->angular.z;
		//ROS_INFO("---------- UsbEp : Reverse with trolley. dist=%.2f. ------------",alignDist);
		pstate = 18;
		return;
	}
	if (msg->linear.z == 42.3) {  // AGV special move to pickup trolley align to front legs
		trolleywidth = msg->angular.z;
		trolley = msg->linear.x;
		ROS_INFO("---------- UsbEp : move into trolley front. type=%.2f. width=%.2f. ------------",trolley,trolleywidth);
		pstate = 19;
		return;
	}
	if (msg->linear.z == 42.4) {  // AGV special move to pickup trolley align to left legs
		trolleywidth = msg->angular.z;
		trolley = msg->linear.x;
		ROS_INFO("---------- UsbEp : move into trolley left. type=%.2f. width=%.2f. ------------",trolley,trolleywidth);
		pstate = 191;
		return;
	}
	if (msg->linear.z == 42.5) {  // Vision Alignment
		ROS_INFO("---------- UsbEp : Vision Alignment. ------------");
		pstate = 193;
		return;
	}
	if (msg->linear.z == 42.6) {  // 
		ROS_INFO("---------- UsbEp : new alignment with scan matching ------------");
		pstate = 195;
		return;
	}
	if (msg->linear.z == 90.1) {		
		if (msg->linear.x != 99.0) {
			maxjoylinear_ = msg->linear.x;	
		}
		if (msg->angular.z != 99.0) {
			maxjoyangular_ = msg->angular.z;	
		}
		switch(manualvelflag) {
			case 0:
				//moveVelocity(0.0,0.0);
				break;
			case 1:
				//moveVelocity(maxjoylinear_,0.0);
				joyangular_ = 0.0;
				joylinear_ = maxjoylinear_;
				break;
			case 2:
				//moveVelocity(-maxjoylinear_,0.0);
				joyangular_ = 0.0;
				joylinear_ = maxjoylinear_;
				break;
			case 3:
				//moveVelocity(0.0,maxjoyangular_);
				joyangular_ = maxjoyangular_;
				joylinear_ = 0.0;
				break;
			case 4:
				//moveVelocity(0.0,-maxjoyangular_);
				joyangular_ = -maxjoyangular_;
				joylinear_ = 0.0;
				break;				
		}
		//ROS_INFO(" ------- Bot_Node : maxjoylinear = %.2f. maxjoyangular = %.2f	 --------",maxjoylinear_,maxjoyangular_);
		return;
	}
	if (msg->linear.z == 90.2) {		
		if (msg->linear.x != 99.0) {
			maxautolinear = msg->linear.x;	
		}
		if (msg->angular.z != 99.0) {
			maxautoangular = msg->angular.z;	
		}
		//ROS_INFO(" ------- Bot_Node : max auto linear = %.2f. max auto angular = %.2f	 --------",maxautolinear,maxautoangular);
		return;
	}
	if (msg->linear.z == 91.0) {
		// manual forward 
		estopped = false;
		cmd_estopped = true;
		joyangular_ = 0.0;
		joylinear_ = maxjoylinear_;
		//moveVelocity(maxjoylinear_,0.0);
		manualvelflag = 1;
		return;
	}
	if (msg->linear.z == 91.1) {
		// manual reverse 
		estopped = false;
		cmd_estopped = true;
		joyangular_ = 0.0;
		joylinear_ = -maxjoylinear_;
		//moveVelocity(-maxjoylinear_,0.0);
		manualvelflag = 2;
		return;
	}
	if (msg->linear.z == 91.2) {
		// manual rotate left 
		estopped = false;
		cmd_estopped = true;
		joyangular_ = maxjoyangular_;
		joylinear_ = 0.0;
		//moveVelocity(0.0,maxjoyangular_);
		manualvelflag = 3;
		return;
	}
	if (msg->linear.z == 91.3) {
		// manual rotate right 
		estopped = false;
		cmd_estopped = true;
		joyangular_ = -maxjoyangular_;
		joylinear_ = 0.0;
		//moveVelocity(0.0,-maxjoyangular_);
		manualvelflag = 4;
		return;
	}
	if (msg->linear.z == 91.4) {
		// manual rotate right 		
		joyangular_ = 0.0;
		joylinear_ = 0.0;
		moveVelocity(joylinear_,-maxjoyangular_);
		estopped = false;
		cmd_estopped = false;
		manualvelflag = 0;
		return;
	}
	if (msg->linear.z == 100.0) {
		pauseRobotMove = true;
		return;
	}
	if (msg->linear.z == 100.1) {
		pauseRobotMove = false;
		return;
	}
}
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
	
	getDisplacement(&distance, &angle);
	publish_podom(distance,angle);
} //void BotNode::publish_odom()
*/

void BotNode::publish_odom()
{
	if(errorState) return;
	ros::NodeHandle nn;
	nn.getParam("navstatus",navstatus);
	prevPose.x = cur_x;
	prevPose.y = cur_y;
	prevPose.theta = cur_theta;

	// calculate the new pose
	double distance, angle;
	
	getDisplacement(&distance, &angle);
		
	cur_theta += angle;
	cur_x += distance * cos(cur_theta);
	cur_y += distance * sin(cur_theta); 
	

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
} 


void BotNode::watchdog()
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	if( ros::Time::now() > odom.header.stamp + ros::Duration(1.0/rate) )
	{
		odom.header.stamp = ros::Time::now();
		odom_pub.publish(odom);
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
	int curcnt;
	bool Cancel_Nav;
	double pid_angle_temp;
	int ret;

	//stop = false;
	while(ros::ok())
	{
		//ROS_INFO("loop..");
		//test();
		
		publish_odom();
		if(errorState)
		{
			nn.setParam("MotorStatus",1);
			if (navstatus != 7) {
				restartRobot();
				continue;
			}
		}
		if (estop_pressed_flag && !estop_pressed) {
			restartRobot();
			estop_pressed_flag = false;
		}
		if (estop_pressed  && !estop_pressed_flag && (navstatus != 7)) {
			estop_pressed_flag = true;
		}
		
		// Stop moving if high level control loses link to driver node for more than 1 sec
		if (navstatus == 7) {
			//nn.setParam("checkPlanSum",0.0); 
			//checkpath();
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
						moveAngle(pangular);//pangular						
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
				if (checkTragetReached()) {
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
				moveDistance(dockdist);
				delay_time = ros::Time::now();
				negativecurrent = false;
				break;			
			case 12:
        //if (quickstop) {
				if( (quickstop) || (current < 0.0) || (negativecurrent) || (ros::Time::now() > (delay_time + ros::Duration(12.0))) ) {//7
					ROS_INFO("---------- UsbEpos 12 : detected  charging current");
					pstate = 123; //121; //121; //0;
					moveVelocity(0.0,0.0); 
					//startPMove = false;
					//publish_move_status(1); // *
					delay_time = ros::Time::now();
					//moveVelocity(-0.01,0.0);
					//estopped = false;	//*
					curcnt = 0;
				} 
				break;
			case 121:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					delay_time = ros::Time::now();
					moveVelocity(-0.013,0.0);
					pstate = 122; //0;
				}
				break;
      case 122:
				if(ros::Time::now()-delay_time > ros::Duration(0.65)) {
					moveVelocity(0.0,0.0);
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
					if ((current < 0.0) || (negativecurrent)) {
						startPMove = false;
						publish_move_status(1);
						ROS_INFO("------------- Epos : 123 : negative current = %.3f --------------",current);
						publish_sound(DETECTCURRENT,0,0);
						pstate = 0;
						estopped = false;
						negativecurrent = false;
					} else {
						moveVelocity(-0.01,0.0);
						pstate = 1230; //124; //0
						ROS_INFO("----------- Epos 123 : No charging current.Goto 1230 --------------------");
						delay_time = ros::Time::now();
					}
				}
				break;
			case 1230:
				if(ros::Time::now()-delay_time > ros::Duration(0.3)) {
					if ((current < 0.0) || (negativecurrent)) {
						startPMove = false;
						publish_move_status(1);
						publish_sound(DETECTCURRENT,0,0);
						pstate = 0;
						estopped = false;
						negativecurrent = false;
						ROS_INFO(" ============== Epso 1230 : Negative BtMode current = %.3f =============",current);
					} else {
						curcnt++;
						if (curcnt > 3) {
							pstate = 124; //0							
						}
						delay_time = ros::Time::now();
					}
				}
				break;
			case 124:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					moveVelocity(0.0,0.0);
					startPMove = false; 
					publish_move_status(1); 
					pstate = 0;
					estopped = false; 
					ROS_INFO("------------- Epos : 124 :  --------------");
					if ((current < 0.0) || (negativecurrent)) {
						publish_sound(DETECTCURRENT,0,0);
					}			
					negativecurrent = false;	
				}
				break;
			case 13:  // move away from docking station
				pstate = 14;
				//publish_debug("Move Away from Docking Station");
				//publish_status("BtNode : Move Away from Docking Station");
				moveDistance(dockrdist);
				delay_time = ros::Time::now();
				break;
			case 130:
				//estopped = true;			
				pstate = 132;	
				turnToPath(turnTopathX,turnTopathY);
				delay_time = ros::Time::now();
				break;
			case 131:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					pstate = 132;
					//moveDistance(0.2);
					delay_time = ros::Time::now();
				}
				break;
			case 132:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					pstate = 0;
					publish_move_status(1);
					//ROS_INFO(" ======== Bot_Node 131 =========");
				}
				break;
			case 133:
				//estopped = true;			
				pstate = 134;	
				turnToPath(turnTopathX,turnTopathY);
				//moveAngle(180);
				delay_time = ros::Time::now();
				break;
			case 134:
				if(ros::Time::now()-delay_time > ros::Duration(0.05)) {
					pstate = 135;
					//moveDistance(0.1);
					//moveVelocity(0.25,0.0);
					//ROS_INFO(" =================== Bot_Node 134 =================================");
					delay_time = ros::Time::now();
				}
				break;
			case 135:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					pstate = 0;
					publish_move_status(1);
					//moveVelocity(0.0,0.0);
					//ROS_INFO(" ======== Bot_Node 135 =========");
				}
				break;
			case 136:
				pstate = 137;	
				//moveVelocity(0.0,0.2);
				ret = system("rosservice call /request_nomotion_update");
				amclcnt = 0;
				nn.getParam("Cancel_Nav",Cancel_Nav);
				if (Cancel_Nav) {
					cancel_goal();
				}
				delay_time = ros::Time::now();
				break;
			case 137:
				if(ros::Time::now()-delay_time > ros::Duration(0.3)) {
					//pstate = 138;
					//moveVelocity(0.0,-0.2);
					ret = system("rosservice call /request_nomotion_update");
					amclcnt++;
					if (amclcnt > 6) {
						pstate = 138;
					}
					delay_time = ros::Time::now();
				}
				break;
			case 138:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					pstate = 0;
					amclcnt = 0;
					//moveVelocity(0.0,0.0);
					//system("rosservice call /request_nomotion_update");
					publish_move_status(1);
				}
				break;
			case 14:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					moveAngle(dockrangle);
					pstate = 0;
					//publish_status("BtNode : Turning Away from Docking Station");
					publish_move_status(1);
				}
				break;
			case 140:
				estopped = true;
				//test_pid_angle(3);
				delay_time = ros::Time::now();
				Cancel_PID = false;
				//ROS_INFO("----------------- Start PID 3------------");
				pstate = 149;
				break;
			case 141:
				estopped = true;
				//test_pid_angle(2);
				delay_time = ros::Time::now();
				Cancel_PID = false;
				//ROS_INFO("----------------- Start PID 2------------");
				pstate = 149;
				break;
			case 142:
				estopped = true;
				//test_pid_angle(0);
				delay_time = ros::Time::now();
				Cancel_PID = false;
				//ROS_INFO("----------------- Start PID 0 ------------");
				pstate = 149;
				break;
			case 143:
				estopped = true;
				//test_pid_angle(7);
				delay_time = ros::Time::now();
				Cancel_PID = false;
				//ROS_INFO("----------------- Start Dist PID 7 ------------");
				pstate = 148;
				break;
			case 144:
				estopped = true;
				//test_pid_angle(7);
				delay_time = ros::Time::now();
				Cancel_PID = false;
				//ROS_INFO("----------------- Start Dist PID 7 ------------");
				pstate = 149;
				break;
			case 145:
				estopped = true;
				test_pid_angle(startLP,endLP,nextPoint);
				delay_time = ros::Time::now();
				Cancel_PID = false;				
				pid_angle_temp = pid_checkangle_NLP();
				//ROS_INFO("----------------- 145 :PID Angle to Point %.3f ------------",pid_angle_temp);
				if (pid_angle_temp > 0.11) {
					pstate = 147;
					//ROS_INFO("----------------- 147 :Start PID Angle to Point %d ------------",nextPoint);
				} else {
					pstate = 1471;
					//ROS_INFO("----------------- 1471 : Start PID Angle_Dist to Point %d ------------",nextPoint);
				}
				//lastPoint = false;
				break;
			case 147:
				if(ros::Time::now()-delay_time > ros::Duration(0.05)) {	
					pid_angle();
					nn.getParam("Cancel_PID",Cancel_PID);
					if (Cancel_PID) {
						//moveVelocity(0.0,0.0);
						pstate = 1471;
						//estopped = false;
						Cancel_PID = false;
						nn.setParam("Cancel_PID",false);
						//ROS_INFO("----------------- Stop Angle and Start Dist PID Point %d------------" ,nextPoint);
					}
					delay_time = ros::Time::now();
				}
				break;
			case 1470:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {	
					pid_dist();
					nn.getParam("Cancel_PID",Cancel_PID);
					if (Cancel_PID) {
						moveVelocity(0.0,0.0);
						pstate = 145;
						if (nextPoint == 11) {
							nextPoint = 2;
							//ROS_INFO("----------------- PID to Last Point : %d ------------",nextPoint);
						} else {
							if (nextPoint == 2) {
								pstate = 0;
								estopped = false;
								nn.setParam("Cancel_PID",false);
								//ROS_INFO("----------------- End Multi Points PID ------------");
							} else {
								nextPoint++;
								//ROS_INFO("----------------- PID to Next Point : %d  ------------",nextPoint);
							}
						}
						Cancel_PID = false;
						nn.setParam("Cancel_PID",false);
						//ROS_INFO("----------------- Stop Dist PID  ------------");
					}
					delay_time = ros::Time::now();
				}
				break;
			case 1471:
				if(ros::Time::now()-delay_time > ros::Duration(0.05)) {	
					pid_angle_dist();
					nn.getParam("Cancel_PID",Cancel_PID);
					if (Cancel_PID) {
						moveVelocity(0.0,0.0);
						pstate = 145;
						nextPoint++;
						if (nextPoint == pathmaxpoint-1) {
							//nextPoint = 2;
							lastPoint = true;
							//ROS_INFO("----------------- PID to Last Point : %d ------------",nextPoint);
						} else {
							if (nextPoint == pathmaxpoint) {
								pstate = 1472;
								moveVelocity(0.0,0.0);
								//estopped = false;
								//nn.setParam("Cancel_PID",false);
								//ROS_INFO("----------------- End Multi Points PID ------------");
							}
						}						
						//estopped = false;
						Cancel_PID = false;
						nn.setParam("Cancel_PID",false);
						//ROS_INFO("----------------- PID to Next Point  ------------");
					}
					delay_time = ros::Time::now();
				}
				break;
			case 1472:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {	
					turnToOrientation();
					pstate = 0;
					estopped = false;
					//ROS_INFO("----------------- End Multi Points PID ------------");
				}
				break;
			case 1473:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {	
					pid_angle_dist();
					nn.getParam("Cancel_PID",Cancel_PID);
					if (Cancel_PID) {
						moveVelocity(0.0,0.0);
						pstate = 0; //1474;
						estopped = false;
						//test_pid_angle(11);
						nn.setParam("Cancel_PID",false);
						//ROS_INFO("----------------- PID to Point 11 ------------");
					}
					delay_time = ros::Time::now();
				}
				break;
			case 1474:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {	
					pid_angle_dist();
					nn.getParam("Cancel_PID",Cancel_PID);
					if (Cancel_PID) {
						moveVelocity(0.0,0.0);
						pstate = 1475;
						//estopped = false;
						//test_pid_angle(2);
						nn.setParam("Cancel_PID",false);
						//ROS_INFO("----------------- PID to Point 2 ------------");
					}
					delay_time = ros::Time::now();
				}
				break;
			case 1475:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {	
					pid_angle_dist();
					nn.getParam("Cancel_PID",Cancel_PID);
					if (Cancel_PID) {
						moveVelocity(0.0,0.0);
						pstate = 0;
						estopped = false;
						//test_pid_angle(9);
						nn.setParam("Cancel_PID",false);
						//ROS_INFO("----------------- Stop PID Path  ------------");
					}
					delay_time = ros::Time::now();
				}
				break;
			case 148:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {	
					pid_dist();
					nn.getParam("Cancel_PID",Cancel_PID);
					if (Cancel_PID) {
						moveVelocity(0.0,0.0);
						pstate = 0;
						estopped = false;
						nn.setParam("Cancel_PID",false);
						//ROS_INFO("----------------- Stop PID ------------");
					}
					delay_time = ros::Time::now();
				}
				break;
			case 149:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {	
					pid_angle();
					nn.getParam("Cancel_PID",Cancel_PID);
					if (Cancel_PID) {
						moveVelocity(0.0,0.0);
						pstate = 0;
						estopped = false;
						nn.setParam("Cancel_PID",false);
						//ROS_INFO("----------------- Stop PID ------------");
					}
					delay_time = ros::Time::now();
				}
				break;
			case 15:		
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {			
					pstate = 12;	
					//publish_debug("BNode : Moving to Docking Station");
					//publish_status("BNode : Moving to Docking Station");
					moveVelocity(dockspeed,0.0);
					delay_time = ros::Time::now();
				}
				break;
			case 150:
				//estopped = true;		
				if(ros::Time::now()-delay_time > ros::Duration(0.3)) {	
					pstate = 151;	
					turnToPath(turnTopathX,turnTopathY);
					delay_time = ros::Time::now();
				}
				break;
			case 151:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					pstate = 0;
					publish_move_status(100);
					//ROS_INFO(" ======== Bot_Node 131 =========");
				}
				break;
			case 152:
				//estopped = true;		
				if(ros::Time::now()-delay_time > ros::Duration(0.3)) {	
					pstate = 153;	
					moveDistance(turnTopathX);
					//turnToPath(turnTopathX,turnTopathY);
					delay_time = ros::Time::now();
				}
				break;
			case 153:
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					pstate = 0;
					publish_move_status(100);
					//ROS_INFO(" ======== Bot_Node 131 =========");
				}
				break;
			case 16:  // AGV turn around inside lift
				moveAngle(alignAngle);
				pstate = 160;
				break;
			case 160:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					pstate = 0;
					publish_move_status(1);
				}
				break;			
			case 17:
				moveAngle(-90.0);
				pstate = 170;
				delay_time = ros::Time::now();
				break;
			case 170:
				if(ros::Time::now()-delay_time > ros::Duration(0.5)) {
					moveAGVintoTrolleyFront(trolleywidth,trolley);
					pstate = 171;
					delay_time = ros::Time::now();
				}
				break;
			case 171:  // vision alignment
				if(ros::Time::now()-delay_time > ros::Duration(0.5)) {
					adjustAGVVision();
					pstate = 172;
					delay_time = ros::Time::now();
				}
				break;
			case 172:
				if(ros::Time::now()-delay_time > ros::Duration(0.5)) {
					moveAngle(90.0);
					pstate = 173;
					delay_time = ros::Time::now();
				}
				break;
			case 173:  // special move into trolley
				if(ros::Time::now()-delay_time > ros::Duration(0.5)) {
					moveAGVintoTrolleyLeft(trolleywidth,trolley);
					pstate = 174;
					delay_time = ros::Time::now();
				}
				break;
			case 174:  // vision alignment
				if(ros::Time::now()-delay_time > ros::Duration(0.5)) {
					adjustAGVVision();
					pstate = 175;
					delay_time = ros::Time::now();
				}
				break;
			case 175:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					pstate = 0;
					publish_move_status(1);
				}
				break;
			case 18:  
				moveDistanceSpeed(alignDist,800);
				delay_time = ros::Time::now();
				pstate = 180;
				break;
			case 180:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					pstate = 0;
					publish_move_status(1);
				}
				break;
			case 19:  // special move into trolley
				moveAGVintoTrolleyFront(trolleywidth,trolley);
				pstate = 190;
				break;
			case 190:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					pstate = 0;
					publish_move_status(1);
				}
				break;
			case 191:  // special move into trolley
				moveAGVintoTrolleyLeft(trolleywidth,trolley);
				pstate = 192;
				break;
			case 192:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					pstate = 0;
					publish_move_status(1);
				}
				break;
			case 193:  // vision alignment
				adjustAGVVision();
				pstate = 194;
				break;
			case 194:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					pstate = 0;
					publish_move_status(1);
				}
				break;
			case 195:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					ptpmoveAlign();
					pstate = 1950;
					delay_time = ros::Time::now();
				}
				break;
			case 1950:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					turnToOrientation();					
					pstate = 1951;
					delay_time = ros::Time::now();
				}
				break;
			case 1951:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {
					publish_move_status(1);
					pstate = 0;
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
						moveVelocity(0.15,0.0);
						delay_time = ros::Time::now();
						//publish_debug("set low speed");
					} else {
						pstate = 4;
						//publish_debug("MoveDist");
						moveDistance(plinear);
						//publish_debug("dist move complete");
						delay_time = ros::Time::now();
					}
				} 
				break;
			case 21:
				if(ros::Time::now()-delay_time > ros::Duration(fspeedtime)) {
					moveVelocity(0.03,0.0);
					delay_time = ros::Time::now();
					pstate = 5;
				}
				break;
			case 22:				
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					pstate = 4;
					//publish_debug("MoveDist");
					moveDistance(plinear);
					//publish_debug("dist move complete");
					delay_time = ros::Time::now();
					//ROS_INFO("MD Done");
				} 
				break;
			case 3:
				if (checkTragetReached()) {
					pstate = 4;
					delay_time = ros::Time::now();		
					//publish_debug("dist move complete");		
				}
				if (!startPMove) {  // stop
					quickStopMotor();
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
						//ROS_INFO("------ BtNode : CheckPlan Found. DistSum=%.3f. -----",checkPlanSum);
					} else {
						if (checkPlanSum == 100.0) {
							//ROS_INFO("------ BtNode : CheckPlan Not Found . Empty -----");
						} else {
							if (checkPlanSum == 100.0) {
								//ROS_INFO("------ BtNode : CheckPlan Failed to Call -----");
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
					nn.setParam("ReachPosition",1);
					startPMove = false;
					//publish_debug("complete all move");
					//ROS_INFO("ALl Done");
					publish_move_status(1);
				} 
				break;
			case 5:
				if ((!startPMove) || (ros::Time::now()-delay_time > ros::Duration(mstop_time))) {  // stop
					quickStopMotor();
					//publish_debug("stop robot");
					pstate = 6;
					estopped = false;
					delay_time = ros::Time::now();					
				} 
				break;
			case 6:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					pstate = 0;
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
				moveAngle(rotate_angle);
				break;
			case 70:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					pstate = 0;
				}
				break;
			case 71:  // Rotate 
				startPMove = false;
				pstate = 72;
				moveAngle(-rotate_angle);
				break;
			case 72:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					pstate = 0;
				}
				break;			
			case 8:  // Move Straight
				ROS_INFO("BT Move St : %.3f",move_distance);
				startPMove = false;
				pstate = 80;
				moveDistance(move_distance);
				//ROS_INFO("Move St done");
				break;
			case 80:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					pstate = 0;
				}
				break;
			case 81:  // Move Straight
				startPMove = false;
				pstate = 82;
				moveDistance(-move_distance);
				break;
			case 82:
				if(ros::Time::now()-delay_time > ros::Duration(1.0)) {
					pstate = 0;
				}
				break;
			case 83:  // Pre/Post Move Dist				
				if ((plinear < ZDIST) && (!estop_pressed)) {
					//publish_debug("botnode : Pre/Post Move Dist");
					moveDistance(plinear);	
					if (estop_pressed) {
						errorState = true;
					}
				} 
				delay_time = ros::Time::now();	
				pstate = 84;
				break;
			case 84:  // Pre/Post Move Angle	
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {			
					pstate = 85;
					if ((pangular < ZANGLE) && (!estop_pressed)) {
						//publish_debug("botnode : Pre/Post Move Angle");
						moveAngle(pangular);	
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
				if(ros::Time::now()-delay_time > ros::Duration(0.1)) {
					publish_move_status(1);
					//publish_debug("botnode : Complete Pre/Post Move");
					pstate = 0;
				}
				break;
			case 86:
				//publish_debug("align final");
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
				moveAngle(alignAngle);
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
					moveAngle(alignX);					
				} else {
					pstate = 93;
				}
				break;
			case 100:
				pstate = 101;								
				moveAngle(alignX);				
				delay_time = ros::Time::now(); 
				//ROS_INFO("Move angle X");
				break;
			case 101:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {				
					//publish_debug("align dx");
					//startPMove = false;
					pstate = 102;
					moveDistance(alignDX);
					delay_time = ros::Time::now(); 		
					//ROS_INFO("Move DX");
				}
				break;
			case 102:
				pstate = 103;								
				moveAngle(alignY);				
				delay_time = ros::Time::now(); 
				//ROS_INFO("Move angle Y");
				break;
			case 103:
				if(ros::Time::now()-delay_time > ros::Duration(0.2)) {				
					//publish_debug("align dy");
					//startPMove = false;
					pstate = 9;
					moveDistance(alignDY);					
					//ROS_INFO("Move DY");
					//delay_time = ros::Time::now(); 		
				}
				break;
			case 110:
				pstate = 111;		
				//if ((alignX > 0.035) || (alignX < -0.035)) {			
				moveAngle(alignX);				
				//}
				delay_time = ros::Time::now(); 
				//ROS_INFO("Move angle X");
				if (estop_pressed) {
					errorState = true;
				}
				break;
			case 111:
				if(ros::Time::now()-delay_time > ros::Duration(0.5)) {				
					//publish_debug("align dd");
					//startPMove = false;
					pstate = 112; //9;
					//if ((alignDX > 0.03) || (alignDX < -0.03)) {						
					moveDistance(alignDX);
					
					//}
					delay_time = ros::Time::now(); 		
					//ROS_INFO("Move DX");
					if (estop_pressed) {
						errorState = true;
					}
				}
				break;
			case 112:
				if(ros::Time::now()-delay_time > ros::Duration(0.5)) {	
					//publish_debug("align final angle");
					startPMove = false;
					pstate = 0;
					//pstate = 91;
					moveAngle(alignAngle);					
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
				moveDistance(alignDX);
				break;
			case 93:
				//publish_debug("align Y angle");
				//startPMove = false;
				if (alignDY != 0.0) {
					pstate = 94;
					moveAngle(alignY);										
				} else {
					pstate = 9;
				}
				break;
			case 94:
				//publish_debug("align dy");
				//startPMove = false;
				pstate = 9;
				moveDistance(alignDY);
				break;

			case 95:
				//publish_debug("align T");
				pstate = 96;
				moveAngle(alignT);								
				break;
			case 96:
				//publish_debug("align TD");
				pstate = 9;
				moveDistance(alignTD);
							
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
				pstate = 1021;
				estopped = true;
				//ROS_INFO("Rotate Right 0.38");
				moveVelocity(0.0,0.4);	
				delay_time = ros::Time::now();							
				break;
			case 1021:
				if(ros::Time::now()-delay_time > ros::Duration(5.0)) {
					moveVelocity(0.0,-0.4);	
					//ROS_INFO("Rotate Left 0.38");
					delay_time = ros::Time::now();
					pstate = 1022;
				}
				break;
			case 1022:
				if(ros::Time::now()-delay_time > ros::Duration(5.0)) {
					moveVelocity(0.0,0.4);	
					//ROS_INFO("Rotate Right 0.38");
					delay_time = ros::Time::now();
					pstate = 1023;
				}
				break;
			case 1023:
				if(ros::Time::now()-delay_time > ros::Duration(5.0)) {
					moveVelocity(0.0,-0.4);	
					//ROS_INFO("Stop Rotation");
					pstate = 1024;
					delay_time = ros::Time::now();
					//startPMove = false;
					//estopped = false;
				}
				break;
			case 1024:
				if(ros::Time::now()-delay_time > ros::Duration(5.0)) {
					moveVelocity(0.0,0.0);	
					//ROS_INFO("Stop Rotation");
					pstate = 0;
					startPMove = false;
					estopped = false;
				}
				break;
		}
		
		switch (hstate) {
			case 0:
				if (navstatus == 7) {
					//nn.setParam("lookforObs",1);	
					hstate = 1;  // 1
					honk_time = ros::Time::now();
					last_cmd_time = ros::Time::now();
					//publish_debug("hstate 5");
					//clearMap_time = ros::Time::now();
				}
				checkDist = false;
				break;
			case 1:
				if (ros::Time::now()-honk_time > ros::Duration(1.5)) {
					hstate = 5;
					honk_time = ros::Time::now();
					checkDist = true;
					escape_cnt = 0;
					//publish_debug("hstate 2");
					//last_cmd_time = ros::Time::now();
				}
				break;
			case 2:
				if (ros::Time::now()-last_cmd_time > ros::Duration(8)) {				
					//publish_debug("Stuck..cmdveloffset=0.2");					
					//publish_clear();
					cmdvel_offset = 0.2; // to slow down robot giving time for clearcostmap
					honk_time = ros::Time::now();
					hstate = 6;
				}
				if (ros::Time::now()-honk_time > ros::Duration(6)) {
					hstate = 5;				
					count = 0;
					honk_time = ros::Time::now();	
					last_cmd_time = ros::Time::now();
					//publish_debug("hstate 5");
				}
				if (navstatus == 0) {
					//publish_debug("hstate 0");
					hstate = 0;
					checkDist = false;
				}
				break;
			case 5:
				/*
				if (ros::Time::now()-last_cmd_time > ros::Duration(3.5)) {			
					if (count > 3) {
						//last_cmd_time = ros::Time::now();
						hstate = 5;
						//count = 0;
					} else {
						honk_time  = ros::Time::now();					
						hstate = 3;
					}
				} else {
					count = 0;
				}
				*/
				if (navstatus == 0) {
					//publish_debug("hstate 0");
					hstate = 0;
				}
				break;
			case 3:
				if (ros::Time::now()-honk_time > ros::Duration(3.5)) {
					hstate = 5;					
					//last_cmd_time = ros::Time::now();
					//publish_debug("hstate 5");
				}
				if (navstatus == 0) {
					//publish_debug("hstate 0");
					hstate = 0;
				}
				break;
			case 4:
				if (ros::Time::now()-last_cmd_time > ros::Duration(4)) {
					hstate = 5;
				}
				if (navstatus == 0) {
					hstate = 0;
				}
				break;
			case 6:
				if (ros::Time::now()-honk_time > ros::Duration(10)) {
					//publish_debug("hstate 3. cmdvel_offset=1.0");
					cmdvel_offset = 1.0;
					honk_time = ros::Time::now();
					last_cmd_time = ros::Time::now();
					hstate = 5;
					count = 0;
				}
				break;
		}		

		// Take command from joystick if estopped
		if (!cmd_estopped) {
			//ROS_INFO("----------------- Move at vx [%.2f] & w [%.2f] --------------------", linear_, angular_);
			if (!estopped) {	
				if (checkDist) {	
					dx = gx - px;
					dy = gy - py;
					tdist = sqrt((dx * dx) + (dy * dy));
					nn.getParam("ProfileMoveObsDist",obsdist);
					//nn.getParam("ProfileMoveObsDDist",obsddist);
					//nn.getParam("LeftLaserObsDist",leftlaserobsdist);
					//nn.getParam("RightLaserObsDist",rightlaserobsdist);
					nn.getParam("MapObsDist",mapobsdist);
					//nn.getParam("multiLP",multiLP);
					//nn.getParam("racplanMove",racplanMove);
					//if ((racplanMove) && (tdist > slowDist)) {
					//	nn.getParam("racplanMoveX",racplanMoveX);
					//	nn.getParam("racplanMoveY",racplanMoveY);
					//	ROS_INFO("--------------- BotNode : nx=%.2f. ny=%.2f. --------------",racplanMoveX,racplanMoveY);
						//racplanMovealignRobot();
					//}
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
					if (linear_ > maxVelocity) {
						linear_ = maxVelocity;
					}
					//if ((tdist < slowDist) && ((cancelDist == 0.0) || (cancelDist >= 1.0)) ) {		
					if ((tdist < slowDist) ) {	
						//if (!slowzone) {
						//	slowzonelinear = linear_;
						//	slowzone = true;
						//}
						//ROS_INFO("***** slowdist : %.3f *******", tdist);
						//if ( (tdist < 0.25) && !multiLP  ) {
						//slowRatio = tdist / slowDist;
						//linear_ = linear_ * slowRatio;
						//if (!slowzone) {
						//	slowzonelinear = linear_;
						//	slowzone = true;
						//}
						if ( (tdist < 0.5)) {
							//ROS_INFO("==============  BotNode : Reaching....    ===============");
							nn.setParam("ReachingStatus","tdist < 0.5");
							if (linear_ > 0.15) {  // 0.22
								linear_ = 0.15;  // 0.22
							}
							if (tdist < 0.4) {
								nn.setParam("ReachingStatus","tdist < 0.4");
								if (linear_ > 0.1) {
									linear_ = 0.1;
								}
								if (tdist < 0.11) {
									nn.setParam("ReachingStatus","tdist < 0.11");
									linear_ = 0.0;
									angular_ = 0.0;	
								}
								//angular_ = 0.0;			
								//ROS_INFO("==============  BotNode : Reached.    ===============");
								//publish_sound(23,0,60);						
							}
							//if (tdist < 0.3) {
							//	nearLP = true;
							//	nn.setParam("nearLP",true);
							//} 
						} else {							
							//linear_ = slowzonelinear;							
							if (linear_ > 0.25) {
								linear_ = 0.25;
							}
						}			
						if (fabs(linear_) > maxautolinear) {
							//linear_ = linear_ >= 0.0 ? maxautolinear : -maxautolinear;
						}			
						if (fabs(angular_) > maxautoangular) {
							//angular_ = angular_ >= 0.0 ? maxautoangular : -maxautoangular;
						}	
						
						if ((linear_ == 0.0) && (angular_ == 0.0) && (tdist < 0.27) && !reach && (navstatus == 7) ) {
						//if ((linear_ < 0.05) && (linear_ > -0.05) ) {
							//double an = fabs(calcAn());
							double fan = calcAn();
							//ROS_INFO(" Angular : %.3f. fan : %.3f",angular_,fan);
							
							if ((fan > 0.0)) {								
								reachTurn = true;
							} else {
								reachTurn = false;
							}
							reach = true;
							ROS_INFO("---------------- USBEPOS : Reached ----------------------");
							waitflag = false;							
						}
						
						if (reach) {							
							double an = fabs(calcAn());
							linear_ = 0.0;
							//ROS_INFO(" AN : %.3f. Angular : %.3f",an, angular_);							
							if (reachTurn) {
								if (an > 1.8) {
									angular_ = 0.5;  // 0.45
								} else {
									if (an > 1.5) {
										angular_ = 0.45; 
									} else {
										if (an > 1.0) {
											angular_ = 0.4; 
										} else {
											if (an > 0.8) {
												angular_ = 0.35; 
											} else {
												if (an > 0.55) {
													angular_ = 0.3; 
												} else {
													angular_ = 0.2; 
												}
											}
										}
									}
								}
							} else {
								if (an > 1.8) {
										angular_ = -0.5;  // 0.45
									} else {
										if (an > 1.5) {
											angular_ = -0.45; 
										} else {
											if (an > 1.0) {
												angular_ = -0.4; 
											} else {
												if (an > 0.8) {
													angular_ = -0.35; 
												} else {
													if (an > 0.55) {
														angular_ = -0.3; 
													} else {
														angular_ = -0.2; 
													}
												}
											}
										}
									}
							}
							if (an < 0.04) {  // 0.06
								angular_ = 0.0;
								linear_ = 0.0;
								if (!waitflag) {
									waitTime = ros::Time::now();
									waitflag = true;
									//reach = false;
									//cancel_goal();			
									ROS_INFO("==============  BotNode : Rotated. Stop A   ===============");						
								}
								if (ros::Time::now()-waitTime > ros::Duration(3.0)) {
									reach = false;
									nn.setParam("robotReached",true);
									ROS_INFO("==============  BotNode : Rotated. Stop B   ===============");
								}
								//usleep(250000);
							}
							
						}
						
					} else {
						slowzone = false;
						reach = false;
						if (estop_pressed) {
							linear_ = linear_ * 0.0;
							angular_ = angular_ * 0.0 ;
							ROS_INFO("---------- UsbEpos : estop pressed. ---------");
							if (!estop_pressed_flag) { 								
								//moveVelocity(linear_,angular_);	
								//ROS_INFO("BTNode estop : lin:%.3f. ang:%.3f",linear_,angular_);
								//freeMotor();
								estop_pressed_flag = true;
							}
						}
					}
					
				}	
				if (!estop_pressed) { 
					
					if (pauseRobotMove) {
						linear_ = 0.0;
						angular_ = 0.0;
						nn.setParam("ReachingStatusB","pauseRobot");
					}
					//ROS_INFO("---------- UsbEpos pauseRobot : linear_ = %.2f. angular_ = %.2f. --------------",linear_,angular_);
					//moveVelocity(linear_,angular_);		
					flinear = linear_;
					fangular = angular_;
					moveVelocity(flinear,fangular);	
				}
				
			}			
		} else { 
			if (!estopped) {
				if (fabs(joylinear_) > maxjoylinear_) {
					joylinear_ = joylinear_ >= 0.0 ? maxjoylinear_ : -maxjoylinear_;
				}			
				if (fabs(joyangular_) > maxjoyangular_) {
					joyangular_ = joyangular_ >= 0.0 ? maxjoyangular_ : -maxjoyangular_;
				}	
				//ROS_INFO("---------- BotNode : joylinear_ = %.3f. joyangular_ = %.3f. --------------",joylinear_,joyangular_);
				//moveVelocity(joylinear_,joyangular_);				
				flinear = joylinear_;
				fangular = joyangular_;
				moveVelocity(flinear,fangular);	
			}
		}
		if(errorState)
		{
			//ROS_INFO("moveVelocity failed in errorState.");
			//publish_event("--------BotNode : moveVelocity failed in errorState -------------");
			ROS_INFO("--------UsbEpos : moveVelocity failed in errorState -------------");
			//linear_ = angular_ = 0;
			//joylinear_ = joyangular_ = 0;
			continue;
		}
		if (navstatus == 0) {
			hstate = 0;
			reach = false;
			navDone = false;
			//linear_ = 0.0;
			//angular_ = 0.0;
			nn.setParam("ReachingStatusA","navstatus = 0");
			//ROS_INFO("------- UsbEpos : navstatus = 0. -----");
		}
		publish_speed(flinear, fangular);
		//moveVelocity(flinear,fangular);	
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
	
	botNode.MotorSetup();
	
	sleep(1);
	readyflag = 77;
	n.setParam("nearLP",false);
	n.setParam("RobotReady",readyflag);	// to let user know robot is ready
	n.getParam("RobotReady",readyflag);
	ROS_INFO("------------------- Botnode : RobotReady = %d ------------------",readyflag);
	botNode.moveVelocity(0.0,0.0);
	sleep(1);
	//ROS_INFO("---------------------------------- USBEpos : moveangle  Anti-Clockwise  ---------------------------");
	//botNode.moveVelocity(0.0,0.5);
	//botNode.moveAngle(360.0);
	//botNode.moveDistance(0.628319);
	//sleep(5);
	//ROS_INFO("---------------------------------- USBEpos : moveangle -360 Clockwise---------------------------");
	//botNode.moveVelocity(0.0,-0.5);
	//botNode.moveAngle(-360.0);
	//botNode.moveDistance(-0.628319);
	//sleep(5);
	//ROS_INFO(" ----------------------------- Stop -------------------------------------------");
	//botNode.moveDistance(0.62832);
	//sleep(5);
	//botNode.moveDistance(-0.62832);
	//sleep(2);
	//ROS_INFO("----------- Rotate 360 left ---------");
	//botNode.moveAngle(360.0);
	//sleep(5);
	//ROS_INFO("----------- Rotate 360 right ---------");
	//botNode.moveAngle(-360.0);
	//sleep(5);
	//botNode.moveDistance(-0.5);
	//sleep(2);
	//botNode.moveAngle(-360.0);
	//sleep(2);
	//botNode.moveVelocity(0.0,0.0);
	//botNode.readPathfromFile();
	//ROS_INFO("------------- RACLUM BotNode Node Ready : readyflag : %d -------------------",readyflag);
	//botNode.publish_toggleButton(ROBOTREADY_UNO);
	//ros::Timer timer = n.createTimer(ros::Duration(1.0/botNode.rate), boost::bind(&BotNode::watchdog, &botNode));
	//ros::Timer timer = n.createTimer(ros::Duration(0.2), boost::bind(&BotNode::watchdog, &botNode));
	boost::thread main_thread(boost::bind(&BotNode::mainLoop, &botNode));
	main_thread.interrupt() ;
	main_thread.join() ;
		
	return 0;
}
