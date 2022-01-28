/*
 * This node is the dashboard of the transporter
 * Information to be provided
 * 1. Total runtime : total time transporter is running from start of recording time
 * 2. Total Downtime : Total time transporter is shutdown for charging etc.
 * 3. Total Idletime : Total runtime - total worktime
 * 4. Total Worktime : Total time transporter is working.
 * 5. Average voltage 
 * 6. Avberage current
 * 7. Average power (= average voltage * average current )
 * 8. Total Energy consumed ( sume of voltage * current every sec)
 * 9. Number of estop
 * 10. Number of cancelled job
 * 11. Laser Scanners status
 * 12. Motor Status
 * 13. Transporter Stuck
 * 14. Average trip time
 */
/* 	History
*		Date Modified :30.5.2018
*		
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include "htbot/status.h"
#include "htbot/srvcmd.h"
#include "htbot/move.h"
#include "htbot/queue.h"
#include "boost/algorithm/string.hpp"
#include "htbot/sound.h"
#include "htbot/debug.h"
#include <geometry_msgs/Pose.h>
#include "htbot/stat_speed.h"
#include "htbot/stat.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Path.h>
#include "htbot/angle.h"
#include <time.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_layer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#define MAXVP 100
#define UPTIMEFACTOR 0.08  // 100% = 8hrs. 1% = 8/100

using namespace std;
using namespace boost::algorithm;

class DashNode
{
public:
	DashNode(ros::NodeHandle rn);	
	void readParamfromFile();
	void savePose();
	void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
	void saveParamToFile();
	void publish_debug(string s);	
	void publish_sound(int id);
	void speedCallback(const htbot::stat_speed::ConstPtr& msg);
	void publish_robot_stat();
	void buttonCallback(const std_msgs::UInt16::ConstPtr& msg);
	void voltCallback(const std_msgs::Float32::ConstPtr& msg);
	void currCallback(const std_msgs::Float32::ConstPtr& msg);
	void readVoltDatafromFile();
	double findBatteryEnergy(double volt);
	void diffinTime();
	void readTimeDatafromFile();
	void saveTimeData();
	void publish_alarm_state(string s);
	void initialiseTimeData();
	void compute_time();
	void resetTimeData();
	//void testCostmap();
	void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);


	ros::ServiceServer cmd_service;
	ros::Publisher debug_pub;
	ros::Publisher queue_pub;
	ros::Subscriber pose_sub;
	ros::Subscriber button_sub;
	ros::Subscriber volt_sub;
	ros::Subscriber curr_sub;
	ros::Subscriber amcl_sub;
	ros::Publisher stat_pub;

	double looprate;
	double voltage,current,avgvolt,avgcur,amphr,maxcurr,mincurr,energy;
	ros::Publisher play_pub;
	double maxlinear_,minlinear_,curlinear_;
	int startnav;
	bool estop, navflag;
	ros::Time tripStartTime,waitStartTime,initialstartTime;
	ros::Duration tripTime,waitTime,totalTime;
	double trip1Duration,trip2Duration,totalruntime,totaltriptime;
	double trip1DurationAvg,trip2DurationAvg,tripavgspeed;
	double runningtriptime;
	double tripdist;
	double px,py,prz,prw,opx,opy;
	double pathx,pathy;
	double pathx1,pathy1;
	double pathx2,pathy2;
	int count;
	int numVP;
	int tnumtrip;
	FILE *vfp, *tfp;  
	std::string voltfile,timefile,alarmlogstring;
	double perInfo[MAXVP],voltInfo[MAXVP];
	int tm_hour,tm_min,tm_year,tm_mon,tm_mday;
	double sysruntime,sysidletime,sysdowntime,sysrecordtime,systriptime;
	int CentralLaserConnected,LeftLaserConnected,RightLaserConnected,MainLaserConnected;
	int startTime;
	struct tm dashStartTime;
  bool canStopROS;
	double amclx,amcly;
	int amclc;
	int heartbeat;

private:
	ros::NodeHandle nh;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::TransformStamped robotpose;
	boost::mutex mut;
};

DashNode::DashNode(ros::NodeHandle rn):
	maxlinear_(0.0),minlinear_(9.0),startnav(0),estop(false),avgvolt(0.0),avgcur(0.0),
	looprate(1.0),amphr(0.0),maxcurr(0.0),mincurr(0.0),count(0),energy(100.0),tnumtrip(0),
	navflag(false),trip1DurationAvg(0.0),trip2DurationAvg(0.0),tripavgspeed(0.0),
	totalruntime(0.0),sysruntime(0.0),sysidletime(0.0),sysdowntime(0.0),systriptime(0.0),
	totaltriptime(0.0),canStopROS(false),amclx(10.0),amcly(10.0),amclc(0),heartbeat(0)
{
	ros::NodeHandle sn(rn);
	debug_pub = sn.advertise<htbot::debug>("event",100);
	play_pub = sn.advertise<htbot::sound>("sound", 100);
	pose_sub = sn.subscribe<geometry_msgs::Pose>("/robot_pose", 1, &DashNode::poseCallback,this);
	//amcl_sub = sn.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &DashNode::amclCallback,this);
	stat_pub = sn.advertise<htbot::stat>("stat",10);
	button_sub = sn.subscribe<std_msgs::UInt16>("button", 100, &DashNode::buttonCallback,this);
	volt_sub = nh.subscribe<std_msgs::Float32>("voltage", 100, &DashNode::voltCallback,this);
	curr_sub = nh.subscribe<std_msgs::Float32>("current", 100, &DashNode::currCallback,this);
	nh.getParam("volt_file",voltfile);	
	nh.getParam("time_file",timefile);
	nh.getParam("tm_hour",tm_hour);
	nh.getParam("tm_min",tm_min);
	nh.getParam("tm_year",tm_year);
	nh.getParam("tm_mon",tm_mon);
	nh.getParam("tm_mday",tm_mday);
	ROS_INFO("Start Time : hr:%d. min:%d. yr:%d. mon:%d. mday:%d",tm_hour,tm_min,tm_year,tm_mon,tm_mday);
	ROS_INFO("Time file=%s",timefile.c_str());

	nh.getParam("CentralLaserConnected",CentralLaserConnected);
	nh.getParam("LeftLaserConnected",LeftLaserConnected);
	nh.getParam("RightLaserConnected",RightLaserConnected);
	nh.getParam("MainLaserConnected",MainLaserConnected);
}

void DashNode::diffinTime()
{
  time_t timer;	
  struct tm y2k = {0};

	y2k.tm_hour = tm_hour;
	y2k.tm_min = tm_min;
	y2k.tm_sec = 0;
	y2k.tm_year = tm_year;
	y2k.tm_mon = tm_mon;
	y2k.tm_mday = tm_mday;
	time(&timer);
	sysrecordtime = difftime(timer,mktime(&y2k));
	sysrecordtime = sysrecordtime / 60.0;  // mins
	//ROS_INFO("sysrecordtime:%.3f",sysrecordtime);
}

double DashNode::findBatteryEnergy(double volt) 
{
	double per;
	int i;

	if (volt < voltInfo[0]) {
		return 0.0;
	} else {
		if (volt > voltInfo[numVP-1]) {
			return 100.0;
		}
	}
	for (i=0;i<numVP;i++) {
		if (volt <= voltInfo[i]) {
			per = perInfo[i];
			//ROS_INFO(" Volt=%.3f. per=%.3f",volt,per);
			return (per * 100.0);
		} 
	}	
}

void DashNode::initialiseTimeData() 
{		
	
	time_t rawtime;

  time( &rawtime );
  dashStartTime = *localtime( &rawtime );
	if (startTime != 1) {
		dashStartTime.tm_year = tm_year;
		dashStartTime.tm_mon = tm_mon;
		dashStartTime.tm_mday = tm_mday;
		dashStartTime.tm_hour = tm_hour;
		dashStartTime.tm_min = tm_min;
		dashStartTime.tm_sec = 0;
	} else {
		// start new time
		tm_year = dashStartTime.tm_year;
		tm_mon = dashStartTime.tm_mon;
		tm_mday = dashStartTime.tm_mday;
		tm_hour = dashStartTime.tm_hour;
		tm_min = dashStartTime.tm_min;
		sysruntime = 0.0;
		sysidletime = 0.0;
		sysdowntime = 0.0;
		systriptime = 0.0;
		startTime = 0;
		saveTimeData();
	}
	ROS_INFO("DashBoard Start Time : year=%d. month=%d. day=%d. hr=%d. min=%d",tm_year,tm_mon,tm_mday,tm_hour,tm_min);
	
}

void DashNode::readTimeDatafromFile() 
{		
	tfp = fopen(timefile.c_str(), "r");
  if (tfp == NULL) {
  	ROS_INFO("I couldn't open time.dat for reading.\n");    
  	return;
  }

	fscanf(tfp,"%d\n",&startTime);
	fscanf(tfp,"%lf\n",&sysruntime);
	fscanf(tfp,"%lf\n",&sysidletime);
	fscanf(tfp,"%lf\n",&sysdowntime);
	fscanf(tfp,"%lf\n",&systriptime);
	fscanf(tfp,"%d\n",&tm_year);
	fscanf(tfp,"%d\n",&tm_mon);
	fscanf(tfp,"%d\n",&tm_mday);
	fscanf(tfp,"%d\n",&tm_hour);
	fscanf(tfp,"%d\n",&tm_min);
	fscanf(tfp,"%d\n",&tnumtrip);
	totaltriptime = systriptime;

	//ROS_INFO("System Time : runtime=%.2f. idletime=%.2f. downtime=%.2f",sysruntime,sysidletime,sysdowntime);
	
  fclose(tfp);	
}

void DashNode::saveTimeData() {
	tfp = fopen(timefile.c_str(), "r+");  // r+ : open existing file to r/w
  if (tfp == NULL) {
  	ROS_INFO("I couldn't open time.dat to write.\n");    
  	return;
  }
	fprintf(tfp,"%d\n",startTime);
	fprintf(tfp,"%.3f\n",(sysruntime + totalruntime));
	fprintf(tfp,"%.3f\n",sysidletime);
	fprintf(tfp,"%.3f\n",sysdowntime);
	fprintf(tfp,"%.3f\n",totaltriptime);
	fprintf(tfp,"%d\n",tm_year);
	fprintf(tfp,"%d\n",tm_mon);
	fprintf(tfp,"%d\n",tm_mday);
	fprintf(tfp,"%d\n",tm_hour);
	fprintf(tfp,"%d\n",tm_min);
	fprintf(tfp,"%d\n",tnumtrip);

	fclose(tfp);
}

void DashNode::resetTimeData() {

	totalruntime = 0.0;
	sysruntime = 0.0;
	sysidletime = 0.0;
	sysdowntime = 0.0;
	totaltriptime = 0.0;
	runningtriptime = 0.0;
	startTime = 1;
	tnumtrip = 0;
	initialstartTime = ros::Time::now();
	initialiseTimeData();
	startTime = 0;
	saveTimeData();

}

void DashNode::readVoltDatafromFile() 
{		
	double per,volt;
	int j;
	//ROS_INFO("start readVoltDatafromFile");
	vfp = fopen(voltfile.c_str(), "r");
  if (vfp == NULL) {
  	ROS_INFO("I couldn't open voltdata.dat for reading.\n");    
  	return;
  }
	j = 0;

	while(true) {
		if (fscanf(vfp,"%lf %lf\n",&per,&volt) == EOF) {
			break;
		}
		perInfo[j] = per;
		voltInfo[j] = volt;
		j++;
	}
	numVP = j;
	//for (int i=0;i<numVP;i++) {
	//	ROS_INFO("Bat : per=%.3f. volt=%.3f",perInfo[i],voltInfo[i]);
	//}
  fclose(vfp);	
}



void DashNode::voltCallback(const std_msgs::Float32::ConstPtr& msg)
{
	ros::NodeHandle xn; 
	{ 
		boost::mutex::scoped_lock lock(mut);
		voltage = msg->data;		
	}
	if (avgvolt < 15.0) {
		avgvolt = voltage;
	}
	avgvolt = avgvolt + voltage;
	avgvolt = avgvolt / 2.0;
	energy = findBatteryEnergy(avgvolt);
	//ROS_INFO(" AvgVolt=%.3f. Energy=%.3f",avgvolt,energy);
	return;
}

void DashNode::currCallback(const std_msgs::Float32::ConstPtr& msg)
{
	ros::NodeHandle xn; 
	char buf [100];
	string s;

	{ 
		boost::mutex::scoped_lock lock(mut);
		current = msg->data;		
	}
	//if (avgcur = 0.0) {
	//	avgcur = current;
	//}
	if ((mincurr == 0.0) && (current > 0.0)) {
		mincurr = current;
	}
	if (current > maxcurr) {
		maxcurr = current;
	}
	if ((current > 0.0) && (current < mincurr)) {
		mincurr = current;
	}
	avgcur = avgcur + current;
	avgcur = avgcur / 2.0;
	amphr = amphr + (avgcur / 3600.0);
	
	return;
}

void DashNode::buttonCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	//ros::NodeHandle rn; 
	int nGP, nLP, butNo;
	//bool ok;
	geometry_msgs::Twist pos;
	bool ok;
	char buf [100];
	string s;

	butNo = msg->data; 
	//ROS_INFO("Button_Node : Button Code Received = %d",butNo);

	switch (butNo) {

		case 15:
			// stop robot.
			if (!estop) {
				estop = true;
			}
			break;
		case 16:
			if (estop) {
				estop = false;
			}
			break;
	}

	return; 
}


void DashNode::publish_alarm_state(string s)
{
	ros::NodeHandle xn; 
	
  time_t t = time(NULL);
	char buf [100];
	string st,sf;	
  struct tm *tm = localtime(&t);
	strftime(buf, sizeof(buf), "%c", tm);	
	st.assign(buf,strlen(buf));
	
	alarmlogstring = s;
	alarmlogstring = alarmlogstring + " @ " + st;
	xn.setParam("alarmlog",alarmlogstring);
	return;
}

void DashNode::compute_time()
{
	diffinTime();
	totalTime = ros::Time::now() - initialstartTime;
	totalruntime = totalTime.toSec() / 60.0; // mins
	sysdowntime = sysrecordtime - (sysruntime + totalruntime);
	sysidletime = (sysruntime + totalruntime) - ((totaltriptime+runningtriptime)/60.0);
}

void DashNode::publish_robot_stat()
{
	htbot::stat stat;
	ros::NodeHandle nn;
	int target,numcanceljob;
	int hr,min,sec,shutdown;
	string currentstate,lasttripinfo,robotstate,alarmlog;
	char buf [100];
	int mainlaser,leftlaser,rightlaser,centrallaser,motorstatus;
	double avgenergy;
	motorstatus = 0;
	shutdown = 0;
	heartbeat++;
	if (heartbeat > 99) {
		heartbeat = 0;
	}
	nn.getParam("startnav",startnav);
	nn.getParam("navTarget",target);
	nn.getParam("NumCancelJob",numcanceljob);
	nn.getParam("currentstate",currentstate);
	nn.getParam("lasttripinfo",lasttripinfo);
	nn.getParam("robotstate",robotstate); 
	nn.getParam("shutdown",shutdown);
	nn.getParam("MotorStatus",motorstatus);
	nn.getParam("avgenergy",avgenergy);
	nn.getParam("avgvolt",avgvolt);
	compute_time();
	/*
	if (tripavgspeed > 1.0) {
		stat.avgspeed = 0.0;
	} else {
		stat.avgspeed = tripavgspeed;
	}

	nn.getParam("MainLaserStatus",mainlaser);
	if ( (MainLaserConnected > 0) ) {
		if (  (mainlaser > 0) ) {
			publish_alarm_state("Front Laser OFF ");
			stat.laserfront = 0;
		} else {
			stat.laserfront = 1;
		}
	} else {
		stat.laserfront = 2;
	}
	nn.getParam("LeftLaserStatus",leftlaser);
	if ( (LeftLaserConnected > 0)  ) {
		if ( (leftlaser > 0) ) {
			publish_alarm_state("Left Laser OFF ");
			stat.laserleft = 0;
		}	else {
			stat.laserleft = 1;
		}
	} else {
		stat.laserleft = 2;
	}
	nn.getParam("RightLaserStatus",rightlaser);
	if ( (RightLaserConnected > 0)) {
		if ( (RightLaserConnected > 0) && (rightlaser > 0) ) {
			publish_alarm_state("Right Laser OFF ");
			stat.laserright = 0;
		} else {
			stat.laserright = 1;
		}
	} else {
		stat.laserright = 2;
	}
	nn.getParam("CentralLaserStatus",centrallaser);
	if ( (CentralLaserConnected > 0) ) {
		if ( (CentralLaserConnected > 0) && (centrallaser > 0) ) {
			publish_alarm_state("Central Laser OFF ");
			stat.lasercenter = 0;
		} else {
			stat.lasercenter = 1;
		}
	} else {
		stat.lasercenter = 2;
	}
	nn.setParam("MainLaserStatus",1);
	nn.setParam("LeftLaserStatus",1);
	nn.setParam("RightLaserStatus",1);
	nn.setParam("CentralLaserStatus",1);
	*/
	stat.totalruntime = sysruntime + totalruntime;
	stat.totaltriptime = ((totaltriptime+runningtriptime)/60.0);
	//stat.totaltriptime1 = trip1DurationAvg;
	//stat.totaltriptime2 = trip2DurationAvg;
	stat.totaldowntime = sysdowntime;
	stat.batlevel = avgenergy;
	stat.uptime = UPTIMEFACTOR * avgenergy;  // hrs
	stat.tnumtrip = tnumtrip;
	//stat.canceljob = numcanceljob;
	//stat.currentstate = currentstate;
	//stat.lasttripinfo = lasttripinfo;
	//stat.robotmode = robotstate;
	//if (shutdown > 0) {
	//	stat.robotmode = "OFFLINE";
	//}	

	//stat.motorstatus = motorstatus;
	stat.alarmlog = alarmlogstring;
	stat.totalidletime = sysidletime;	
	stat.avgvolt = avgvolt;
	stat.avgcurr = avgcur;
	stat.amphr = amphr;
	stat.heartbeat = heartbeat;
	stat_pub.publish(stat);
	if ((shutdown > 0) && !canStopROS) {
		saveTimeData();
		canStopROS = true;
		ROS_INFO("-------Dash Node : shutdown triggered ---------");
	}
	return;
}

/*
void DashNode::testCostmap() 
{
	tf::TransformListener tf(ros::Duration(10));	
	costmap_2d::Costmap2DROS* controller_costmap_ros;
	ros::NodeHandle rn; 
	int x,y,sizex,sizey;
	double orgx,orgy;

	rn.getParam("costx",x);
	rn.getParam("costy",y);	
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros->getCostmap()->getMutex()));
	controller_costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", tf);
	//unsigned char cost = controller_costmap_ros->getCostmap()->getCost(x,y);
	//orgx = controller_costmap_ros->getCostmap()->getOriginX();
	//orgy = controller_costmap_ros->getCostmap()->getOriginY();
	//sizex = controller_costmap_ros->getCostmap()->getSizeInCellsX();
	//sizey = controller_costmap_ros->getCostmap()->getSizeInCellsY();
			
	//ROS_INFO("origin : X=%.3f. Y=%.3f",orgx,orgy);
	//ROS_INFO("cost at 10,10 = %d",(int)cost);
	ROS_INFO("Size : X=%d. Y=%d",sizex,sizey);
}
*/

void DashNode::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	double x,y,xx,yy;
	ros::NodeHandle rn; 
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		x = msg->pose.covariance[0];
		y = msg->pose.covariance[7];			
	}
	xx = sqrt(x);
	yy = sqrt(y);
	if (xx < amclx) {
		amclx = xx;
	}
	if (yy < amcly) {
		amcly = yy;
	}
	if ((fabs(xx) < 2.0) && (fabs(yy) < 2.0) ) {
		ROS_INFO("---- Dash Node : amcl localised. x=%.3f. xx=%.3f y=%.3f. yy=%.3f --------",x,xx,y,yy);
		rn.setParam("Localised",true);		
	}
	//amclc++;
	//if (amclc > 5) {
	//	amclc = 0;
	//	ROS_INFO("-------------- Dash Node : lowest xx=%.3f. yy=%.3f ------------",amclx,amcly);
	//}
	ROS_INFO("-------------- Dash Node :  xx=%.3f. yy=%.3f ------------",xx,yy);
}

void DashNode::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	ros::NodeHandle rn; 
	double dx,dy,td;
	int target;
	
	
	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		px = msg->position.x;
		py = msg->position.y;
		prz = msg->orientation.z;
		prw = msg->orientation.w;				
	}
	rn.getParam("startnav",startnav);
	rn.getParam("navTarget",target);
	//target = 1;  // temp
	if ((startnav == 1) && (navflag)) {
		dx = px - opx;
		dy = py - opy;
		td = sqrt((dx * dx) + (dy * dy));
		tripdist = tripdist + td;
		opx = px;
		opy = py;
		tripTime = ros::Time::now() - tripStartTime;
		runningtriptime = tripTime.toSec();
		//totaltriptime = totaltriptime + temptime;
	}
	if ((startnav == 0) && (navflag)) {
		// stop navigation
		runningtriptime = 0.0;
		tripTime = ros::Time::now() - tripStartTime;
		totaltriptime = totaltriptime + tripTime.toSec();
		if (target == 1) {
			trip1Duration = tripTime.toSec();
			if (trip1DurationAvg == 0.0) {
				trip1DurationAvg = trip1Duration;
			}			
			trip1DurationAvg = trip1DurationAvg + trip1Duration;
			//totaltriptime = totaltriptime + trip1Duration;
			trip1DurationAvg = trip1DurationAvg / 2.0;
			if (tripavgspeed == 0.0) {
				tripavgspeed = tripdist / trip1Duration;
			}
			tripavgspeed = tripavgspeed + (tripdist / trip1Duration) ;
			tripavgspeed = tripavgspeed / 2.0;
		}
		if (target == 2) {
			trip2Duration = tripTime.toSec();
			if (trip2DurationAvg == 0.0) {
				trip2DurationAvg = trip2Duration;
			}			
			trip2DurationAvg = trip2DurationAvg + trip2Duration;
			//totaltriptime = totaltriptime + trip2Duration;
			trip2DurationAvg = trip2DurationAvg / 2.0;
			if (tripavgspeed == 0.0) {
				tripavgspeed = tripdist / trip2Duration;
			}
			tripavgspeed = tripavgspeed + (tripdist / trip2Duration) ;
			tripavgspeed = tripavgspeed / 2.0;
		}
		//tripDuration = tripDuration / 60.0; // minutes.
		//publish_tripstat();
		navflag = false;
	}
	if ((startnav == 1) && (!navflag)) {
		// start navigation
		tripStartTime = ros::Time::now(); 
		tripdist = 0.0;
		if (target == 1) {
			trip1Duration = 0.0;
		}
		if (target == 2) {
			trip2Duration = 0.0;
		}
		minlinear_ = 0.0;
		maxlinear_ = 0.0;
		count = 0;
		opx = px;
		opy = py;
		//ROS_INFO("Nav Start Pos : x=%.2f. y=%.2f. ",opx,opy);
		navflag = true;
		tnumtrip++;
		return;
	}
	//ROS_INFO("RobotPose : x=%.2f. y=%.2f. ",px,py);
}


void DashNode::publish_sound(int id)
{
	htbot::sound cmd;
	cmd.id = id;
	play_pub.publish(cmd);
	return;
}


void DashNode::publish_debug(string s)
{
	htbot::debug deb;
	deb.msg = s;
	debug_pub.publish(deb);
	return;
}

 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "service node");  
  ros::NodeHandle rn;  		
	DashNode sNode(rn);
	int cc,dd,ccn,ccs,shutdn,resetTime;
	double ss;
	ros::Rate loop_rate(5.0);
	cc = 0;
	ccn = 0;
	ccs = 0;
	sleep(1);
	sNode.readVoltDatafromFile();
	sNode.readTimeDatafromFile();
	sNode.initialstartTime = ros::Time::now();
	sNode.initialiseTimeData();
	//sNode.testCostmap();
	rn.setParam("MotorStatus",0);
	while (true) {  	 
		cc++;
		ccs++;
		if (cc == 5) {	
			shutdn = 0; 	
			//rn.getParam("shutdown",shutdn);			
			sNode.publish_robot_stat();
			cc = 0;	
			rn.getParam("resetTime",resetTime);
			if (resetTime == 1) {
				sNode.resetTimeData();
				resetTime = 0;
				rn.setParam("resetTime",resetTime);
			}
			if (sNode.canStopROS) {
				ccn++;
				if (ccn == 5) {
					ROS_INFO("------ dash : starting shutdown sequence---------");
				}
				if (ccn == 8) {
					system("sudo chmod +x /sbin/shutdown");
					system("sudo shutdown -h now");
					//system("gnome-terminal -x ~/shut.sh");
				}				
			}
		}
		if (ccs == 150) {
			sNode.saveTimeData();
			ccs = 0;
		}
		
		ros::spinOnce();	
  	loop_rate.sleep();
  }
  
  return 0;
}


