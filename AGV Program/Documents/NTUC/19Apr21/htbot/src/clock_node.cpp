/*
 * This node is to publish clock
 */
/* 	History
*		Date Modified : 23.12.2020
*		Changes :
*/

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <rosgraph_msgs/Clock.h>

using namespace std;

ros::Publisher clock_pub;
ros::Time zeroTime;
rosgraph_msgs::Clock clkmsg;
ros::Duration dt;

// prototype


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Sound Server");
  ros::NodeHandle n;
	ros::Rate loop_rate(15.0);
	
	clock_pub = n.advertise<rosgraph_msgs::Clock>("clock",100);
	zeroTime = ros::Time::now();

	while (true) {  
		//dt = ros::Time::now()-zeroTime;
		clkmsg.clock = ros::Time::now();
		clock_pub.publish(clkmsg);		
		ros::spinOnce();	
  	loop_rate.sleep();
  }
  
  return 0;
}



