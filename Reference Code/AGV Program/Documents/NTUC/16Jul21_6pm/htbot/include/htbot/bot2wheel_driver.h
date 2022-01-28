#include "htbot/EPOS2.h"
#include "htbot/odom.h"
#include "htbot/sound.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#ifndef BOT_DRIVER
#define BOT_DRIVER

class Bot2Wheel
{
	
	int leftmotor_fd,rightmotor_fd;
	int prev_Lpos, prev_Rpos;
	double mm_per_count;
	double rpm_factor;
	double radpm_factor;
	double dist_factor;
	double angle_factor;
	double displacement_factor;
public:
	Bot2Wheel ();
	~Bot2Wheel ();

	EPOS leftMotor, rightMotor;
	std::string LEFT_MOTOR;
	std::string RIGHT_MOTOR;
	int pbaudL,pbaudR,ebaudL,ebaudR,cbaudrateL,cbaudrateR;  // port baudrate and epos baudrate. cbaudrate=1 to change rate
	int REFLECT; //  if robot moves anticlockwise when both motors are given +ve vel command
	int AXLE_LEN; // in mm
	int WHEEL_DIA; // in mm
	int GEAR_RATIO;
	int STEPS_PER_REV; // in quad-counts
	double wheel_circum_correction;
	double wheel_base_correction;
	double odom_angular_scale_correction,dDist;
	ros::NodeHandle nn;
	ros::Publisher odom_pub,play_pub;
	ros::Subscriber pose_sub;
	ros::Time delay_time;
	double timelimit;
	double px,py,pz,prx,pry,prz,prw;

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

	double leftMotorOffset,rightMotorOffset;	
	bool quickstop;


	

	void start();
	bool reset();
	void shutDown();
	void setParams(EPOS* m);
	bool moveVelocityO(double vx,double w); // [m/s,rad/s]
	bool moveVelocity(double vx,double w); // [m/s,rad/s]
	bool moveDistanceO(double x); // [m]
	bool moveDistance(double x); // [m]
	bool moveD(double dist,double time);
	int moveDLoop(void);
	bool moveAngle(double theta); // [degrees]
	bool moveA(double angle,double time);
	int moveALoop(void);
	bool getDisplacement(double* distance, double* angle); // [m, rad]
	//void changebaudrate();
	void setVelMode();
	void motorVelMode(EPOS* motor);
	bool checkMoveComplete(void);
	bool stopRobot(void);
	void setSlowMove(void);
	void setFastMove(void);
	void publish_odom();
	void pub_odom(void);
	void publish_sound(int id,int sd, int rsd);
	bool FaultReset();
	void freeMotor();
	bool engageMotor();
	bool moveAlign(double linear, double angular,double timelimit);
	void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);

}; //class Bot2Wheel

#endif
