/*
 * This node is to store event messages 
 */
/* 	History
*		Date Modified : 2.10.2018
*		Changes :
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include "htbot/move_status.h"
#include "htbot/status.h"
#include "boost/algorithm/string.hpp"
#include <math.h>
#include "htbot/debug.h"


using namespace std;


ros::Subscriber event;
std::string filename;


bool eventStore(const htbot::debug::ConstPtr& msg)
{
	FILE *fp;
	string estr;
  time_t rawtime;
	char buf [300];
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

	estr = msg->msg;
	st = st + " : " + estr;
	fp = fopen(filename.c_str(), "a");
	fprintf(fp,"%s\n",st.c_str())	;
	fclose(fp);	
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Event Node");
  ros::NodeHandle n;
	
	//event = n.subscribe<htbot::debug>("event", 100, eventStore);  //                                                                                                                 
	n.getParam("/event/filename",filename);
	
	
  ros::spin();

  return 0;
}



