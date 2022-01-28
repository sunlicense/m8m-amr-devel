#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include "std_msgs/UInt16.h"

class HTBOTJoy
{
public:
  HTBOTJoy();	

	ros::Publisher button_pub;
	ros::Publisher fButton_pub;

private:
  ros::NodeHandle nh;

  int linear, angular, linear_inc, linear_dec,angular_inc,angular_dec,shutbutton,jackbutton;
  double l_scale, a_scale;
  ros::Publisher vel_pub;
  ros::Subscriber joy_sub;
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) ;
	void publish_Button(unsigned short cmd);
	void publish_fButton(unsigned short cmd);
	int trigger;
	bool jack,jackup;
	int shutdown;
	ros::Time shutdown_start_time;

};

HTBOTJoy::HTBOTJoy():
  linear(0),
  angular(1),
	linear_inc(0),
  angular_inc(1),
	linear_dec(0),
  angular_dec(1),
  l_scale(0.3),
  a_scale(0.3),shutdown(0),shutbutton(5),jackbutton(4),  
	trigger(0),jack(false),jackup(false)
{
	button_pub = nh.advertise<std_msgs::UInt16>("button",10);
	fButton_pub = nh.advertise<std_msgs::UInt16>("fbutton",1);
  nh.param("/joystick/axis_linear", linear, linear);
  nh.param("/joystick/axis_angular", angular, angular);
  nh.param("/joystick/scale_angular", a_scale, a_scale);
  nh.param("/joystick/scale_linear", l_scale, l_scale);
	nh.param("/joystick/linear_inc", linear_inc, linear_inc);
	nh.param("/joystick/angular_inc", angular_inc, angular_inc);
	nh.param("/joystick/linear_dec", linear_dec, linear_dec);
	nh.param("/joystick/angular_dec", angular_dec, angular_dec); 
	nh.param("/joystick/trigger", trigger, trigger);
	nh.param("/joystick/jack", jack, false);
	nh.param("/joystick/shutbutton", shutbutton, shutbutton);
	nh.param("/joystick/jackbutton", jackbutton, jackbutton);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &HTBOTJoy::joyCallback, this);
	//ROS_INFO("angular scale : %.3f. Linear scale  : %.3f",a_scale,l_scale);
	//ROS_INFO("angular  : %d. Linear : %d",angular,linear);

}

void HTBOTJoy::publish_Button(unsigned short cmd)
{
	std_msgs::UInt16 msg;
	msg.data = cmd;
	button_pub.publish(msg);
	//ROS_INFO("-------- Jnode : Button %d .---------------",cmd);
	return;
}

void HTBOTJoy::publish_fButton(unsigned short cmd)
{
	std_msgs::UInt16 msg;
	msg.data = cmd;
	fButton_pub.publish(msg);
	return;
}

void HTBOTJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) 
{ 
	int x,tt;
  geometry_msgs::Twist vel;
	ros::NodeHandle nh;
	//ROS_INFO("linear : %angular : %d",angular);
	//ROS_INFO("linear : %d",linear);
	double vx,vz;
  //ROS_INFO("   --------------------   ");
	//ROS_INFO("Button : 0:%d. 1:%d. 2:%d. 3:%d. 4:%d. 5:%d. 6:%d. 7:%d",joy->buttons[0],joy->buttons[1],joy->buttons[2],joy->buttons[3],joy->buttons[4],joy->buttons[5],joy->buttons[6],joy->buttons[7]);
	//ROS_INFO("Axes : 0:%.2f. 1:%.2f. 2:%.2f. 3:%.2f. 4:%.2f. 5:%.2f. 6:%.2f. 7:%.2f",joy->axes[0],joy->axes[1],joy->axes[2],joy->axes[3],joy->axes[4],joy->axes[5],joy->axes[6],joy->axes[7]);
	x =  joy->buttons[linear_inc];
	//tt =  joy->buttons[trigger];
	if (x > 0) {
		l_scale = l_scale + 0.01;		
		shutdown = 0;
	}
	x =  joy->buttons[linear_dec];
	if (x > 0) {
		l_scale = l_scale - 0.01;
		if (l_scale < 0.0) {
			l_scale = 0.0;
		}
	}
	x =  joy->buttons[angular_inc];
	if (x > 0) {
		a_scale = a_scale + 0.01;		
	}
	x =  joy->buttons[angular_dec];
	if (x > 0) {
		a_scale = a_scale - 0.01;
		if (a_scale < 0.0) {
			a_scale = 0.0;
		}		
	}
	if (jack) {
		x =  joy->buttons[jackbutton];
		//x = (int) joy->axes[jackbutton];
		//ROS_INFO("-------- Jnode : x = %d. ----------",x);
		if (x > 0) {
			if (jackup) {
				jackup = false;
				//publish_fButton(603);
				publish_Button(200);
				ROS_INFO("---------- JoyNode : Jack Down -----------");
			} else {
				jackup = true;
				//publish_fButton(602);
				publish_Button(201);
				ROS_INFO("---------- JoyNode : Jack Up -----------");
			}
		}
	}
	x =  joy->buttons[shutbutton];
	if (x > 0) {		
		shutdown++;
		ROS_INFO("--------- JoyNode : Shutdown cnt=%d. ------------",shutdown);
		if (shutdown >= 5) {
			nh.setParam("shutdownRAC",true);
		}
		// check if still pressed
	} 
		
	
	vel.angular.z = a_scale*joy->axes[angular];
  vel.linear.x = l_scale*joy->axes[linear];
	//ROS_INFO("Before : angular : %.3f. Linear : %.3f",joy->axes[angular],joy->axes[linear]);
	//ROS_INFO("Before : angular : %.3f. Linear : %.3f",vel.angular.z,vel.linear.x);
	vz = vel.angular.z;
	vx = vel.linear.x;
	if (vz < 0.0) {
		vz = -vz;
	}
	if (vx < 0.0) {
		vx = -vx;
	}
	if (vz < vx) {
		vel.angular.z = 0.0;
	} else {
		if (vz > vx) {
			vel.linear.x = 0.0;
		} 
	}
  vel_pub.publish(vel);
	//if (tt == 1) {
	//	publish_Button(36);
	//}
	//ROS_INFO("After : angular : %.3f. Linear : %.3f",vel.angular.z,vel.linear.x);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "htbot_joy");
  HTBOTJoy htbot_joy;
	
  ros::spin();
}
