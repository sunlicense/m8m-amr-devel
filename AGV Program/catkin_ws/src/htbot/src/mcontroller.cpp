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
#include "htbot/motorcmd.h"
#include "htbot/move_status.h"

#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"


#include "htbot/PLAYSOUND.h"


using namespace std;

#define ZDIST 2.0
#define ZANGLE 190.0
#define PI 3.141593
#define PI2 6.28319
#define PI_2 1.5707965
#define MAXLP 50

class MControllerNode
{
public:
	MControllerNode();
	Bot2Wheel robot;
	
	ros::Time last_cmd_time, last_joycmd_time;
	nav_msgs::Odometry odom;
	double cur_x,cur_y,cur_theta;
	double mlinear,mangular,omlinear,omangular;
	bool mc_errorState,resetting;
	ros::Publisher play_pub;
	ros::Publisher odom_pub;
	ros::Subscriber velcmd_sub;
	ros::Subscriber motormove_sub;
	ros::Publisher status_mmov;
	
	double timelimit,pdist,pangle,obsdist;
	int navstatus;
	int mdstate,motormode,RobotQuickStop;
	bool completed,admove,vmove;
	double vlinear,vangular,vdist,vangle;
	
	void publish_sound(int id,int startdelay,int restartdelay);
	void mainLoop();
	void publish_odom(void);
	void adjVelCmdCallback(const htbot::motorcmd::ConstPtr& msg);
	void motormoveCallback(const htbot::motorcmd::ConstPtr& msg);
	void publish_mmove_status(int stat);
	void restartRobot();

private:
	ros::NodeHandle nh,ph;
	double cov_x, cov_y, cov_th;
	double cov_vx, cov_vy, cov_vth;
	ros::Time current_time, last_time, delay_time;
	boost::mutex publish_mutex_;
};

MControllerNode::MControllerNode():
	navstatus(0),pdist(0.0),mdstate(0),motormode(99),omangular(0.0),mangular(0.0),omlinear(0.0),mlinear(0.0),
  mc_errorState(false),admove(false),vmove(false),resetting(false),RobotQuickStop(false),
	cur_x(0.0),cur_y(0.0),cur_theta(0.0),ph("~")
{		

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

	play_pub = nh.advertise<htbot::sound>("sound", 1);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
	velcmd_sub = nh.subscribe<htbot::motorcmd>("/adj_velcmd",100, &MControllerNode::adjVelCmdCallback,this);
	motormove_sub = nh.subscribe<htbot::motorcmd>("/motormove",100,&MControllerNode::motormoveCallback,this);
	status_mmov = nh.advertise<htbot::move_status>("/mcmove_status",1);
	

	current_time = last_time = last_cmd_time = ros::Time::now();
	
} // MControlNode constructor

void MControllerNode::adjVelCmdCallback(const htbot::motorcmd::ConstPtr& msg)
{
	{
		boost::mutex::scoped_lock lock(publish_mutex_);
		vangular =  msg->angular;
		vlinear =  msg->linear;	
		vmove = true;
	}
	//ROS_INFO("------- MControllerNode : Velocity Move callBack Linear=%.3f. Angular=%.3f--------",vlinear,vangular);
}

void MControllerNode::motormoveCallback(const htbot::motorcmd::ConstPtr& msg)
{
	{
		boost::mutex::scoped_lock lock(publish_mutex_);
		motormode = msg->type;
		pdist =  msg->dist;
		pangle =  msg->angle;	
		vangular =  msg->angular;
		vlinear =  msg->linear;	
		admove = true;
	}
	//ROS_INFO("------- MControllerNode : Motor Dist/Angle Move callBack Mode=%d--------",motormode);
}

void MControllerNode::publish_sound(int id,int startdelay,int restartdelay)
{
	htbot::sound cmd;
	cmd.id = id;
	cmd.startdelay = startdelay;
	cmd.restartdelay = restartdelay;
	play_pub.publish(cmd);
	//ROS_INFO("Publish Sound AA");
	return;
}

void MControllerNode::publish_mmove_status(int stat)
{
	htbot::move_status status;
	status.stat = stat;
	status_mmov.publish(status);
	//ROS_INFO("----------MController : mmove_status=%d. ------------",stat);
	return;
}

void MControllerNode::restartRobot()
{
	ros::NodeHandle nh;
	int tries = 0;
	ROS_INFO("******* MotorController : ReStart Motor.  *******");
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
	resetting = mc_errorState = false;
	nh.setParam("MotorError",0);
	//nh.setParam("mc_errorState",mc_errorState);
}


void MControllerNode::publish_odom(void)
{
	double distance, angle;
	
	if(mc_errorState) return;
	mc_errorState = !robot.getDisplacement(&distance, &angle);
	if(mc_errorState)
	{
		ROS_ERROR("***** MController : getDisplacement failed in mc_errorState. *******");
		return;
	}

	cur_theta += angle;
	cur_x += distance * cos(cur_theta);
	cur_y += distance * sin(cur_theta); 

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();

	//ROS_INFO("Odom. Dist : %.3f. Angle : %.3f. dt : %.3f",distance,angle,dt);
	//ROS_INFO("cov_x=%.4f. cov_y=%.4f. cov_th=%.4f",cov_x,cov_y,cov_th);
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

	//publish the message
	odom_pub.publish(odom);
	//last_time = current_time;
  return ;
}

void MControllerNode::mainLoop()
{
	ros::Rate r(15);
	//ros::Rate r(0.2);
	ros::NodeHandle nn;
	double tdist,dx,dy;
	double distance,angle; //obsdist,obsddist;
	int ret;

	//ROS_INFO(" ******* MController *****");
	//stop = false;
	while(ros::ok())
	{
		//ROS_INFO(" ******* Odom Loop *****");
		publish_odom();	 
		//nh.getParam("navstatus",navstatus);		
		if (mc_errorState) {
			mdstate = 40;
			ROS_INFO("******* MotorController : mc_errorState *******");
		} else {
			if (vmove) {
				mdstate = 20;
				vmove = false;
			} else {
				if (admove) {
					mdstate = 1;
					admove = false;
				} 
			}
		}
		switch (mdstate) {
			case 0:
				break;
			case 1:				
				switch(motormode) {
					case 1: // move dist mode
						mdstate = 10;						
						break;
					case 2: // move angle mode
						mdstate = 30;		
						//ROS_INFO("------------------MController : mdstate = 30. --------------");				
						break;
					case 3: // joystick move
						mdstate = 20;						
						break;
					case 4: // robot reset
						mdstate = 50;						
						break;
					case 5: // robot start
						mdstate = 60;						
						break;
					case 6: // stop robot
						mdstate = 70;
						break;
					case 7: // free robot
						mdstate = 80;
						break;
					case 8: // check move complete 
						mdstate = 90;
						break;
					case 9: // set slow move
						mdstate = 100;
						break;
				}
				break;
			case 10: // dist move
				ROS_INFO("******* MotorController : Dist Move. Dist=%.3f *******",pdist);
				timelimit = fabs(pdist / 0.05);  //0.12
				if (timelimit < 2.0) {
					timelimit = 2.0;
				}
				if (pdist > 0.0) {
					mdstate = 11;
				} else {
					mdstate = 12;
				}			
				break;
			case 11:	
				nh.getParam("ProfileMoveObsDist",obsdist);
				if (obsdist > pdist) {
					mdstate = 12;
				} else {
					publish_sound(NEEDSPACE,0,6);		
				}				
			case 12:
				if (!robot.moveD(pdist,timelimit)) {
					mdstate = 0;
					publish_mmove_status(1); // aborted
				} else {
					mdstate = 13;
				}
				break;	
			case 13:
				nh.getParam("RobotQuickStop",RobotQuickStop);
				if (RobotQuickStop) {
					robot.quickstop = true;
					RobotQuickStop = false;
					nh.setParam("RobotQuickStop",RobotQuickStop);
				}
				ret = robot.moveDLoop();
				if ((ret == 0)) {
					// ok
					mdstate = 0;
					motormode = 99;					
					publish_mmove_status(1);// completed
				} else {
					if ((ret == -1)) {
						mdstate = 0;						
						publish_mmove_status(1); // aborted
					}
				}
				break;
			case 20: // velocity move
				robot.moveVelocity(vlinear,vangular);
				ROS_INFO("******* MotorController : moveVelocity. lin=%.3f. Ang=%.3f  *****",vlinear,vangular);				
				mdstate = 0;		
				break;
			case 30: // angle move
				//ROS_INFO("******* MotorController : Angle Move=%.3f *******",pangle);
				timelimit = fabs(pangle / 0.05);  //0.12
				if (timelimit < 2.0) {
					timelimit = 2.0;
				}	
				if (!robot.moveA(pangle,timelimit)) {
					mdstate = 0;
					publish_mmove_status(1); // aborted
				} else {
					mdstate = 31;
				}				
				break;
			case 31:
				ret = robot.moveALoop();
				if ((ret == 0)) {
					// ok
					mdstate = 0;
					motormode = 99;
					publish_mmove_status(1);// completed
				} else {
					if ((ret == -1)) {
						mdstate = 0;
						publish_mmove_status(1); // aborted
					}
				}
				break;
			case 40: // mc_errorState. Restart				
				restartRobot();
				mdstate = 0;
				motormode = 99;
				break;
			case 50:
				if (robot.reset()) {
					// ok
					mdstate = 0;
					motormode = 99;
					publish_mmove_status(1);// completed
					ROS_INFO("******* MotorController : Motor Rest OK *******");
				} else {
					mdstate = 0;
					motormode = 99;
					publish_mmove_status(1);// aborted
					ROS_INFO("******* MotorController : Motor Rest Failed *******");
				}
				break;
			case 60:
				robot.start();
				mdstate = 0;
				motormode = 99;
				publish_mmove_status(1);// completed
				ROS_INFO("******* MotorController : Motor Start *******");
				break;
			case 70:
				robot.stopRobot();
				mdstate = 0;
				motormode = 99;
				publish_mmove_status(1);// completed
				ROS_INFO("******* MotorController : Stop  Motor.  *******");
				break;
			case 80:
				robot.freeMotor();
				mdstate = 0;
				motormode = 99;
				publish_mmove_status(1);// completed
				ROS_INFO("******* MotorController : Free Motor.  *******");
				break;
			case 90:
				robot.checkMoveComplete();
				mdstate = 0;
				motormode = 99;
				publish_mmove_status(1);// completed
				break;
			case 100:
				robot.setSlowMove();
				mdstate = 0;
				motormode = 99;
				publish_mmove_status(1);// completed
				break;
		}
		ros::spinOnce();
		r.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor controller");
	ros::NodeHandle n;
	MControllerNode MCNode;
	int readyflag;
	bool res;
	char buf [100];
	double x,y,x1,y1,z;
	int cnt;
	
	readyflag = 0;
	n.setParam("RobotReady",readyflag);	
	MCNode.robot.start();

	while(ros::ok() && !MCNode.robot.reset())
	{
		ROS_WARN("---- MCNode : EPOS not ready. Trying again in 0.5 second....");
		//sleep(1);
		usleep(500000);
	}
	sleep(1);
	readyflag = 55;
	n.setParam("RobotReady",readyflag);	// to let user know robot is ready
	
	ROS_INFO("******** MControllerNode Ready to Run *********");

	boost::thread main_thread(boost::bind(&MControllerNode::mainLoop, &MCNode));
	main_thread.interrupt() ;
	main_thread.join() ;

	return 0;
}
