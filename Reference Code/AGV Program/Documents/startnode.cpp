/*
 * This node is start nodes on rac1
 */

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "start node");
  ros::NodeHandle n;
	int ret,count;
	bool startRAC1;

	startRAC1 = false;
	count = 0;
	/*
	n.setParam("startRAC1",startRAC1);	
	while(true) {
		n.getParam("startRAC1",startRAC1);
		if(startRAC1) {
			break;
		}
		count++;
		if (count > 5) {
			break;
		}
		sleep(1);
	}
	*/
	
	sleep(5);
	system("~/honk.sh &");
	ret = system("~/startRAC1.sh &");	
	system("~/honk.sh &");
	printf("\n ----------------------- Started RAC1 --------------------------\n");
  return 0;
}
