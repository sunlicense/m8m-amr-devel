/*
 * This file was modified from the original turtlebot teleop file
 * to do tele-operation using keyboard control
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

#define KEYCODE_R 0x72 //0x43 
#define KEYCODE_L 0x6c //0x44
#define KEYCODE_U 0x75 //0x41  
#define KEYCODE_D 0x64 //0x42
#define KEYCODE_Q 0x71
#define KEYCODE_S 0x73

class BotTeleop
{
public:
	BotTeleop();
	void keyLoop();
	void watchdog();
private:
	ros::NodeHandle nh_,ph_;
	double linear_, angular_;
	double olinear_, oangular_;
	double l_scale_, a_scale_;
	double ol_scale_, oa_scale_;
	ros::Time first_publish_;
	ros::Time last_publish_;
	ros::Publisher vel_pub_;
	void publish(double, double);
	boost::mutex publish_mutex_;
};

BotTeleop::BotTeleop():
	ph_("~"),
	linear_(0),	angular_(0),
	l_scale_(1.0), a_scale_(1.0)
{
	ph_.param("scale_linear", l_scale_, l_scale_);
	ph_.param("scale_angular", a_scale_, a_scale_);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_keyboard");
	BotTeleop bot_teleop;
	ros::NodeHandle n;

	signal(SIGINT,quit);
	boost::thread my_thread(boost::bind(&BotTeleop::keyLoop, &bot_teleop));
	//ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&BotTeleop::watchdog, &bot_teleop));

	ros::spin();
	my_thread.interrupt() ;
	my_thread.join() ;

	return(0);
}

void BotTeleop::watchdog()
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
			(ros::Time::now() > first_publish_ + ros::Duration(0.50)))
		publish(0, 0);
}

void BotTeleop::keyLoop()
{
	char c;
	bool arrow1,arrow2;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the robot");
	puts("+/- to increase/decrease speed");
	puts("s to stop");
	puts("q or Ctrl-C to quit");

	olinear_ = 0.0;
	oangular_ = 0.0;
	linear_ = 0.0;
	angular_ = 0.0;
	l_scale_ = 0.1;
	ol_scale_ = 0.1;
	a_scale_ = 0.1;
	oa_scale_ = 0.1;
	
	arrow1 = false;
	arrow2 = false;
	while (ros::ok())
	{
		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		//linear_ = angular_ = 0;
		//ROS_INFO("value: 0x%02X\n", c);

		switch(c)
		{
			case KEYCODE_L:
				ROS_INFO("LEFT");
				angular_ += 1.0;
				arrow1 = false;
				arrow2 = false;
				break;
			case KEYCODE_R:
				ROS_INFO("RIGHT");
				angular_ -= 1.0;
				arrow1 = false;
				arrow2 = false;
				break;
			case KEYCODE_U:
				ROS_INFO("UP");
				linear_ += 1.0;
				arrow1 = false;
				arrow2 = false;
				break;
			case KEYCODE_D:
				ROS_INFO("DOWN");
				linear_ -= 1.0;
				arrow1 = false;
				arrow2 = false;
				break;
			case KEYCODE_Q:
				ROS_INFO("QUIT");
				linear_ = 0.0;
				angular_ = 0.0;
				arrow1 = false;
				arrow2 = false;
				break;
			case '+':
				l_scale_ += 0.1;
				a_scale_ += 0.1;
				ROS_INFO("Increase speed to %f", l_scale_);
				arrow1 = false;
				arrow2 = false;
				break;
			case '-':
				l_scale_ -= 0.1;
				a_scale_ -= 0.1;
				ROS_INFO("Decrease speed to %f", l_scale_);
				arrow1 = false;
				arrow2 = false;
				break;
			case KEYCODE_S:
				//l_scale_ = 0.0;
				//a_scale_ = 0.0;
				linear_ = 0.0;
				angular_ = 0.0;				
				arrow1 = false;
				arrow2 = false;
				break;
			case 0x1b:
				arrow1 = true;
				break;
			case 0x5b:
				if (arrow1) {
					arrow2 = true;
				} else {
					arrow1 = false;
				}
				break;
			case 0x41:
				if (arrow2) {
					ROS_INFO("UP");
					linear_ += 1.0;
					arrow1 = false;
					arrow2 = false;
				}
				break;
			case 0x42:
				if (arrow2) {
					ROS_INFO("Down");
					linear_ -= 1.0;
					arrow1 = false;
					arrow2 = false;
				}
				break;
			case 0x43:
				if (arrow2) {
					ROS_INFO("Right");
					angular_ -= 1.0;
					arrow1 = false;
					arrow2 = false;
				}
				break;
			case 0x44:
				if (arrow2) {
					ROS_INFO("Left");
					angular_ += 1.0;
					arrow1 = false;
					arrow2 = false;
				}
				break;
		}

		boost::mutex::scoped_lock lock(publish_mutex_);
		if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) { 
			first_publish_ = ros::Time::now();
		}
		last_publish_ = ros::Time::now();
		if ( (angular_ != oangular_) || (linear_ != olinear_) || (l_scale_ != ol_scale_) || (a_scale_ != oa_scale_) ) {
			publish(angular_, linear_);
			olinear_ = linear_;
			oangular_ = angular_;
			ol_scale_ = l_scale_;
			oa_scale_ = a_scale_;
			//ROS_INFO(" lin = %f, ang = %f", linear_, angular_);
		}

		if(c==KEYCODE_Q) raise(SIGINT);
	} //while (ros::ok())

	return;
} // void BotTeleop::keyLoop()

void BotTeleop::publish(double angular, double linear)
{
	geometry_msgs::Twist vel;
	vel.angular.z = a_scale_ * angular;
	vel.linear.x = l_scale_ * linear;
	//ROS_INFO("publish : %f", vel.linear.x);
	vel_pub_.publish(vel);
	return;
}



