#include <iostream>
#include <sqlite3.h>
#include <stdio.h>
#include <ros/ros.h>
#include "htbot/debug.h"
using namespace std;

// global variable
std::string calendarDB;
int year,month,day,hour,minute;
int updatecnt;
int updateidx[10];

ros::Publisher event_pub;

// prototype
static int callback(void *NotUsed, int argc, char **argv, char **szColName);
void getCurrentDateTime();
void checkSchedule();
void publish_event(string s);
void updateSchedule(int id);

// This is the callback function to display the select data in the table
static int callback(void *NotUsed, int argc, char **argv, char **szColName)
{
	char *szErrMsg = 0;
	char buf [300];

  for(int i = 0; i < argc; i++)
  {
    //std::cout << szColName[i] << " = " << argv[i] << std::endl;
		printf("i=%d. The data in column \"%s\" is: %s\n", i,szColName[i], argv[i]);				
  }
	updateidx[updatecnt++] = atoi(argv[0]);
  return 0;
}

void getCurrentDateTime() {
	time_t rawtime;
	char buf [300];
	string sm,st;
	struct tm currTime;

	time( &rawtime );
  currTime = *localtime( &rawtime );
	year = currTime.tm_year + 1900;
	month = currTime.tm_mon + 1;
	day = currTime.tm_mday;
	hour = currTime.tm_hour;
	minute = currTime.tm_min;
	ROS_INFO("-------------- SQL : currTime = yr=%d. month=%d. day=%d. hour=%d. min=%d ------------------",year,month,day,hour,minute);
}

void publish_event(string s)
{
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
	htbot::debug status;
	status.msg = st + " : " + s;
	event_pub.publish(status);
	return;
}

void checkSchedule() {
	sqlite3 *db;
	int rc;
	char *szErrMsg = 0;
	char buf [300];
	string s1;

	ROS_INFO("--------------- calendarDB = %s ------------------",calendarDB.c_str());
	//rc = sqlite3_open(calendarDB.c_str(), &db);
	rc = sqlite3_open("/home/rac/catkin_ws/src/htbot/data/calendar.db", &db);
	if(rc)
  {
     publish_event("\n--------------- Can't open calendar database ------------ \n");
  } 
	getCurrentDateTime();
	sprintf(buf,"SELECT id, action FROM schedule WHERE year=%d AND month=%d AND day=%d AND hour=%d AND min=%d AND done=0",year,month,day,hour,minute);
	s1.assign(buf,strlen(buf));
	ROS_INFO("--------- SQL : %s ---------------------------",s1.c_str());
  // execute sql
	rc = sqlite3_exec(db,buf, callback, 0, &szErrMsg);
  if(rc != SQLITE_OK) {
		sprintf(buf,"----- SQL Error: %s ------------",szErrMsg);
		s1.assign(buf,strlen(buf));
		publish_event(s1);
    sqlite3_free(szErrMsg);
  } else {
		ROS_INFO("----------------- executed sql command ---------------------");
	}

  // close database
  if(db)
  {
    sqlite3_close(db);
  }

}

void updateSchedule(int id) {
	sqlite3 *db;
	int rc;
	char *szErrMsg = 0;
	char buf [300];
	string s1;

	ROS_INFO("----------- Index id to Update is =%d. -------------",id);
	rc = sqlite3_open("/home/rac/catkin_ws/src/htbot/data/calendar.db", &db);
	if(rc)
  {
     publish_event("\n--------------- Can't open calendar database to Update ------------ \n");
  } 
	sprintf(buf,"UPDATE schedule SET done=1 WHERE id=%d",id);
	s1.assign(buf,strlen(buf));
	ROS_INFO("--------- SQL Update  : %s ---------------------------",s1.c_str());
	rc = sqlite3_exec(db,buf, callback, 0, &szErrMsg);
  if(rc != SQLITE_OK) {
		sprintf(buf,"----- SQL Update Error: %s ------------",szErrMsg);
		s1.assign(buf,strlen(buf));
		publish_event(s1);
    sqlite3_free(szErrMsg);
  }
  // close database
  if(db)
  {
    sqlite3_close(db);
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "web_node");  
  ros::NodeHandle n;
	double schrate;
	int count;
  
  
	schrate = 0.0333;

	//movstat_sub = n.subscribe<htbot::move_status>("move_status", 10, movStatusCallback);

	n.getParam("SCHEDULE_RATE",schrate);
	n.getParam("SCHEDULE_DB",calendarDB);

	ros::Rate loop_rate(schrate);
	updatecnt = 0;
	sleep(1);
	ROS_INFO("\n---------------------- Starting Scheduling Monitoring Node -------------------------\n");
	count = 0;
	while (true) {  
		ROS_INFO("-------------- SQL : schedule monitor count = %d ------------------",count++);
		checkSchedule();		
		if (updatecnt > 0) {
			ROS_INFO("-------------- SQL : updatecnt = %d ------------------",updatecnt);
			int j = updatecnt;
			for (int i=0;i<j;i++) {
				updateSchedule(updateidx[i]);
				updatecnt--;
			}
			ROS_INFO("-------------- SQL : updatecnt = %d ------------------",updatecnt);
		}
		ros::spinOnce();	
  	loop_rate.sleep();
	}

  // prepare our sql statements


  return 0;
}

