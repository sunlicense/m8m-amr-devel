/*
 * This node is to web node 
 */
/* 	History
*		Date Modified :18.5.2017
									: 29.4.21 : 245pm
									: 4.5.21 4.45pm
*		Changes :
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include "htbot/status.h"
#include "htbot/srvcmd.h"
#include "htbot/move.h"
#include "htbot/queue.h"
#include "htbot/mqueue.h"
#include "boost/algorithm/string.hpp"
#include "htbot/sound.h"
#include "htbot/debug.h"
#include "htbot/scanCmd.h"
#include "htbot/goal.h"
#include <geometry_msgs/Pose.h>
#include "htbot/clear.h"
#include "std_msgs/UInt16.h"
#include <actionlib/client/simple_action_client.h>
#include "htbot/lumstatus.h"
#include "htbot/navstatus.h"
#include "htbot/PLAYSOUND.h"
#include "htbot/path.h"
#include "std_srvs/Empty.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "htbot/angle.h"
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include "htbot/cleanlist.h"
#include <dirent.h>
#include "htbot/DYNA.h"
#include "htbot/dyna.h"
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "htbot/task.h"
#include "htbot/ntuc_status.h"

#define MAXLP 150
#define POSENOTINUSE 999
#define PI 3.141593
#define MAXMAPFILE 50
#define MAXCLEANFILE 100

#define LPSHIFT 0

using namespace std;
using namespace boost::algorithm;

class WEBNode
{
public:
	WEBNode(ros::NodeHandle rn);	
	bool webcmdServiceCallback(htbot::mqueue::Request &req,htbot::mqueue::Response &res);	
	bool qServiceCallback(htbot::mqueue::Request &req,htbot::mqueue::Response &res);	
	void readPosefromFileS();
	bool getPose();
	void savePoseS();
	int storePoseS(string lps);
	void publish_debug(string s);	
	void initialisation();
	void publish_sound(int id,int sd,int rsd);
	void removePose(int GN, int LP);
	void removeAllPose();
	void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
	bool mqPush(int LP);
	int mqPop();
	void clearLPQueue();
	void publish_clear(void);
	void publish_status(string s);
	int searchLP(string lps);
	void buttonCallback(const std_msgs::UInt16::ConstPtr& msg);
	void cancel_goal(void);
	void publish_navstatus(void);
	void publish_queue(void);
	void test_escape(void);
	bool sendGoalpub(double x,double y, double z, double rx, double ry, double rz, double rw, double pd, double pa, int opt);
	bool sendGoalpubMP(int slp, int elp, int opt);
	bool cmdGoalpub(int cmd,int startidx, int lastidx, int numgoal, double x, double y, double rz, double rw, double pd, double pa, int opt);
	void checkLP(void);
	bool global_localise(void);
	void reLocalise(void);
	void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void initialpose();
	void test_pid_angle();
	void pid_angle();
	double ptpAngle(void);
	void publish_event(string s);
	void checkPathTest();
	void recordPathLP();
	void recordPathName();
	void publish_cleanPlan(void);
	bool checkPasswd(string s);
	void LoadCleanMapPlan(void);
	void readCleanPlanfiles(void);
	const char *get_filename_ext(const char *filename);
	char *remove_ext(char* myStr, char extSep, char pathSep);
	void dynaReconfig(int pm, int iV, double dV, string sV);
	void testdyna();
	void calcQuat(double sx, double sy,double ex, double ey, double *qx,double *qy, double *qz, double *qw);
	void testcalcq();
	void taskCallback(const htbot::task::ConstPtr& msg);
	void removeLPfromQQ(int LP);
	void testsound();
	//void depthCallback(const sensor_msgs::Image::ConstPtr& img);

	ros::ServiceServer webcmd_service;
	ros::ServiceServer qq_service;
	ros::Publisher debug_pub;
	ros::Publisher queue_pub;
	ros::Publisher updategroup_pub;
	ros::Publisher scanM_pub;
	ros::Publisher pos_pub;
	ros::Publisher clear_pub;
	ros::Subscriber pose_sub;
	ros::Publisher status_pub;
	ros::Publisher cancel_pub; 
	ros::Publisher chklp_pub;
	htbot::mqueue mq_srv;
	ros::ServiceClient mq_client;
	ros::Subscriber button_sub;
	ros::Publisher navstat_pub;
	ros::Publisher move_pub;
	ros::Publisher goal_pub;
	ros::Subscriber costmap_sub;
	ros::Subscriber costmapUpd_sub;
	ros::ServiceClient localise;
	ros::Subscriber amcl_sub;
	ros::Publisher ipose_pub;
	ros::Publisher event_pub;
	ros::Publisher cleanPlan_pub;
	ros::Subscriber depth_sub;
	ros::Publisher dyna_pub;
	ros::Subscriber task_sub;
	ros::Publisher ntucstatus_pub;

	string LandingPointString[MAXLP];
	int numLP;
	FILE *pfp;  // pose data file
	FILE *pathfp;  // path LP data file

	std::string posefile,pathfile,cleandir;
	std::string sdir,homedir;
	double tx,ty,tz;
	double rx,ry,rz,rw;
	double pre_dist,pre_angle,post_dist,post_angle;  // pre=before start. post=after reaching dest.
	double pre_dist1,pre_angle1,post_dist1,post_angle1,align,func;
	double trolley,trolleydist;
	double otx,oty,otz;
	double orx,ory,orz,orw;
	double poseInfo[MAXLP][20];	
	int poseStatus[MAXLP]; // 0=not called. 1~900 =prority. 999=not in use
	double looprate;
	ros::Publisher play_pub;
	//double px,py,pz,prx,pry,prz,prw;
	int numLPInQ,QNo;
	bool estop;
	char costbuf [2600];
	char ncostbuf [50][50];
	double amclx,amcly;
	int amclc,operation_mode;  //operation_mode=0 > base. =1 > navigation mode. =2 > mapping mode
	bool localised,image_enabled,correctPasswd,AutoLogin,motorONOFF;
	double Angle_Err_Value,i_Temp,d_Temp;
	double P_Term,I_Term,D_Term,Kd,Kp,Ki;
	double gx,gy,gz,grx,gry,grz,grw,px,py,pz,prx,pry,prz,prw;
	std::string mapfilearray[MAXMAPFILE];
	std::string cleanfilearray[MAXCLEANFILE];

private:
	ros::NodeHandle nh;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::TransformStamped robotpose;
	boost::mutex mut;
};

WEBNode::WEBNode(ros::NodeHandle rn):
	rx(0.0),ry(0.0),rz(0.0),rw(0.0),numLPInQ(0),QNo(1),estop(false),localised(false),
	tx(0.0),ty(0.0),tz(0.0),numLP(0),looprate(1.0),amclx(10.0),amcly(10.0),amclc(0),
	i_Temp(0.0),d_Temp(0.0),Angle_Err_Value(0.0),image_enabled(false),correctPasswd(false),
	AutoLogin(false),operation_mode(1),motorONOFF(false)
{
	ros::NodeHandle sn(rn);
	webcmd_service = nh.advertiseService("web_cmd",&WEBNode::webcmdServiceCallback,this); 
	qq_service = nh.advertiseService("mqueue",&WEBNode::qServiceCallback,this); 
	button_sub = nh.subscribe<std_msgs::UInt16>("button", 100, &WEBNode::buttonCallback,this);
	debug_pub = sn.advertise<htbot::debug>("debug",100);
	queue_pub = sn.advertise<htbot::queue>("queue",100);
	updategroup_pub = sn.advertise<htbot::status>("group",100);
	play_pub = sn.advertise<htbot::sound>("sound", 100);
	scanM_pub = sn.advertise<htbot::scanCmd>("scanCmd", 1);
	pose_sub = sn.subscribe<geometry_msgs::Pose>("/robot_pose", 1, &WEBNode::poseCallback,this);
	mq_client = nh.serviceClient<htbot::mqueue>("mqueue");
	pos_pub = nh.advertise<geometry_msgs::Twist>("cmd_pos", 1);
	clear_pub = nh.advertise<htbot::clear>("clearMap", 100);
	status_pub = nh.advertise<htbot::status>("feedback",100);
	cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
	navstat_pub = nh.advertise<htbot::navstatus>("navstatus",100);
	move_pub = nh.advertise<htbot::move>("move", 100);
	goal_pub = nh.advertise<htbot::goal>("goal", 100);
	chklp_pub = nh.advertise<htbot::path>("checkpath",1);
	localise = nh.serviceClient<std_srvs::Empty>("global_localization");
	ipose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
	event_pub = nh.advertise<htbot::debug>("event",100);
	cleanPlan_pub = nh.advertise<nav_msgs::Path>("/cleanPlan",10);
	dyna_pub = sn.advertise<htbot::dyna>("reconfigure",100);
	//depth_sub = sn.subscribe<sensor_msgs::Image>("/robot_pose", 1, &WEBNode::depthCallback,this);
	//amcl_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &WEBNode::amclCallback,this);
	task_sub = nh.subscribe<htbot::task>("fr_fleet", 100, &WEBNode::taskCallback,this);

	nh.getParam("pose_file",posefile);	
	nh.getParam("home_dir",homedir);
	nh.getParam("NUmberOfStation",numLP);
	nh.getParam("path_file",pathfile);	
	nh.getParam("clean_dir",cleandir);
	nh.getParam("AutoLogin",AutoLogin);
	if (AutoLogin) {
		correctPasswd = true;
	}
	ROS_INFO("------- WebNode : Clean Plan Dir = %s.-------",cleandir.c_str());
}

void WEBNode::calcQuat(double sx, double sy,double ex, double ey, double *qx,double *qy, double *qz, double *qw)
{
	double an,diff;
	tf2::Quaternion quat;
	geometry_msgs::Quaternion qq;
	double x,y,z,w;
	if ( (ex >= sx) && (ey >= sy)) {
		// Q1
		an = atan((ey-sy)/(ex-sx));
	} else {
		if ( (sx >= ex) && (ey >= sy) ) {
			// Q2
			an = atan((ey-sy)/(sx-ex));
			an = PI - an;
		} else {
			if ( (sx >= ex) && (sy >= ey) ) {
				// Q3
				an = atan((sy-ey)/(sx-ex));
				an = -(PI - an);
			} else {
				// Q4
				an = -(atan((sy-ey)/(ex-sx)));
			}
		}
	}
	quat.setRPY( 0.0, 0.0,an);
	quat.normalize();
	qq = tf2::toMsg(quat);
	*qx = qq.x;
	*qy = qq.y;
	*qz = qq.z;
	*qw = qq.w;
	return;
}

void WEBNode::publish_cleanPlan(void)
{
	nav_msgs::Path plan;
	plan.poses.resize(8);
	plan.header.frame_id = "/map";
	plan.poses[0].pose.position.x = 19.8705;
	plan.poses[0].pose.position.y = 10.0820;
	plan.poses[0].pose.position.z = 0.0000;
	plan.poses[0].pose.orientation.x = 0.0000;    
	plan.poses[0].pose.orientation.y = 0.0000; 
	plan.poses[0].pose.orientation.z = -0.7039; 
	plan.poses[0].pose.orientation.w = 0.7103;
	
	plan.poses[1].pose.position.x = 19.8914;
	plan.poses[1].pose.position.y = 8.5017;
	plan.poses[1].pose.position.z = 0.0000;
	plan.poses[1].pose.orientation.x = 0.0000;    
	plan.poses[1].pose.orientation.y = 0.0000; 
	plan.poses[1].pose.orientation.z = -0.5528; 
	plan.poses[1].pose.orientation.w = 0.8333;  

	plan.poses[2].pose.position.x = 20.6; //20.5113;
	plan.poses[2].pose.position.y = 6.58; //7.1299;
	plan.poses[2].pose.position.z = 0.0000;
	plan.poses[2].pose.orientation.x = 0.0000;    
	plan.poses[2].pose.orientation.y = 0.0000; 
	plan.poses[2].pose.orientation.z = -0.6773; 
	plan.poses[2].pose.orientation.w = 0.7357;

	plan.poses[3].pose.position.x = 20.7400;
	plan.poses[3].pose.position.y = 4.06; //4.3340;
	plan.poses[3].pose.position.z = 0.0000;
	plan.poses[3].pose.orientation.x = 0.0000;    
	plan.poses[3].pose.orientation.y = 0.0000; 
	plan.poses[3].pose.orientation.z = -0.0583; 
	plan.poses[3].pose.orientation.w = 0.9983;  

	plan.poses[4].pose.position.x = 28.5; //26.2632;
	plan.poses[4].pose.position.y = 4.18; //4.2943;
	plan.poses[4].pose.position.z = 0.0000;
	plan.poses[4].pose.orientation.x = 0.0000;    
	plan.poses[4].pose.orientation.y = 0.0000; 
	plan.poses[4].pose.orientation.z = 0.0037; 
	plan.poses[4].pose.orientation.w = 1.0000;
	
	plan.poses[5].pose.position.x = 43.8565;
	plan.poses[5].pose.position.y = 4.2472;
	plan.poses[5].pose.position.z = 0.0000;
	plan.poses[5].pose.orientation.x = 0.0000;    
	plan.poses[5].pose.orientation.y = 0.0000; 
	plan.poses[5].pose.orientation.z = 0.0013; 
	plan.poses[5].pose.orientation.w = 1.0000;

	plan.poses[6].pose.position.x = 51.2168;
	plan.poses[6].pose.position.y = 4.3480;
	plan.poses[6].pose.position.z = 0.0000;
	plan.poses[6].pose.orientation.x = 0.0000;    
	plan.poses[6].pose.orientation.y = 0.0000; 
	plan.poses[6].pose.orientation.z = 0.7066; 
	plan.poses[6].pose.orientation.w = 0.7076;

	plan.poses[7].pose.position.x = 51.0946;
	plan.poses[7].pose.position.y = 9.5421;
	plan.poses[7].pose.position.z = 0.0000;
	plan.poses[7].pose.orientation.x = 0.0000;    
	plan.poses[7].pose.orientation.y = 0.0000; 
	plan.poses[7].pose.orientation.z = 0.6854; 
	plan.poses[7].pose.orientation.w = 0.7282;
  
	cleanPlan_pub.publish(plan);
	
	return;
}

/*
void WEBNode::depthCallback(const sensor_msgs::Image::ConstPtr& img)
{
	double x,y,z,rz,rw;
	int wd,ht,step,size;

	if (!image_enabled) {
		return;
	}
	wd = img->width;
	ht = img->height;
	step = img->step;
	size = img->data.size();
	image_enabled = false;
	ROS_INFO("-----------WebNode : Depth Image wd=%d. ht=%d. step=%d. size=%d. -------------",wd,ht,step,size);
	return;
}
*/

void WEBNode::publish_event(string s)
{
	htbot::debug status;
	status.msg = s;
	event_pub.publish(status);
	return;
}

double WEBNode::ptpAngle(void)
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

void WEBNode::test_pid_angle()
{
	Kp = 0.01;
	Ki = 0.01;
	Kd = 0.001;
	
	px = 20.8749;
	py = 11.5080;
	pz = 0.0000;
	prx = 0.0000;
	pry = 0.0000;
	prz = -0.6911;
	prw = 0.7228;
	      
	gx = 11.9625;
	gy = 11.0667;
	gz = 0.0000;
	grx = 0.0000;
	gry = 0.0000;
	grz = -0.0100;
	grw = 1.0000;
	      
	Angle_Err_Value = 0.0;
}

void WEBNode::pid_angle()
{
	double angle_to_goal,tyawp,adrive;
	tf::Quaternion qp(0.0,0.0,prz,prw);
	tyawp = tf::getYaw(qp);	
	angle_to_goal = ptpAngle(); // find angle to goal
	Angle_Err_Value = angles::shortest_angular_distance(tyawp, angle_to_goal);
	P_Term = Kp * Angle_Err_Value;
	i_Temp += Angle_Err_Value;
	if (i_Temp > 3.14) {
		i_Temp = 3.14;
	} else {
		if (i_Temp < -3.14) {
			i_Temp = -3.14;
		}
	}
	I_Term = Ki * i_Temp;
	D_Term = Kd * (d_Temp - Angle_Err_Value);
	d_Temp = Angle_Err_Value;
	adrive = P_Term + I_Term + D_Term;
	ROS_INFO("---PID : agoal=%.3f. Err=%.3f. PT=%.3f.---",angle_to_goal,Angle_Err_Value,P_Term);
	ROS_INFO("---IT=%.3f. DT=%.3f. TT=%.3f. iTemp=%.3f. d_Temp=%.3f---",I_Term,D_Term,adrive,i_Temp,d_Temp);
	//robot.moveVelocity(0.0,adrive);
}

void WEBNode::initialpose() 
{
	geometry_msgs::PoseWithCovarianceStamped ipose;
	geometry_msgs::Quaternion quat;

	//ROS_INFO("initialpose..");
	ipose.header.frame_id = "map";
	ipose.header.stamp = ros::Time::now();
	ipose.pose.pose.position.x = 11.9705; //20.7651;
	ipose.pose.pose.position.y = 7.3807; //11.5259;
	ipose.pose.pose.position.z = 0.0000;
	ipose.pose.pose.orientation.x = 0.0000;
	ipose.pose.pose.orientation.y = 0.0000;
	ipose.pose.pose.orientation.z =0.0076;
	ipose.pose.pose.orientation.w = 1.0; //0.7145;
	ipose.pose.covariance[0] = 0.55;//1e-3;
	ipose.pose.covariance[7] = 0.55;//1e-3;
	ipose.pose.covariance[14] = ipose.pose.covariance[21] = ipose.pose.covariance[28] = 1e9;
	ipose.pose.covariance[35] = 0.55; //1e-3;
	ipose_pub.publish(ipose);
}

void WEBNode::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	double x,y,xx,yy;
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
	if ((fabs(xx) < 0.05) || (fabs(yy) < 0.05) ) {
		ROS_INFO("-------- WebNode Node : amcl localised. x=%.3f. xx=%.3f y=%.3f. yy=%.3f ------------",x,xx,y,yy);
		localised = true;
	}
	//amclc++;
	//if (amclc > 5) {
	//	amclc = 0;
	//	ROS_INFO("-------------- WebNode Node : AMCL_POSE xx=%.3f. yy=%.3f ------------",xx,yy);
	//}
	//ROS_INFO("-------------- WebNode Node : AMCL_POSE xx=%.3f. yy=%.3f ------------",xx,yy);
}

bool WEBNode::global_localise(void) {
	
	std_srvs::Empty srv;
	bool ret;
		
	ret = false;
	if (localise.call(srv)) {
		ROS_INFO("====== WebNode : Activated Global Localisation ==========");
		ret = true;		
	} else {
		ROS_INFO("====== WebNode : Global Localisation Failed ==========");
		ret = false;
	}
	return ret;
}

void WEBNode::recordPathLP() {
	std::string s1;
	pathfp = fopen(pathfile.c_str(), "a");  // w : open new file

  if (pathfp == NULL) {
  	ROS_INFO("I couldn't open pathdata.dat for writing.\n");    
  	return;
  }
	//ROS_INFO("------- WebNode : Save Path LP to File ----------");
	fprintf(pathfp,"%.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",px,py,pz,prx,pry,prz,prw);		
  fclose(pathfp);	
}

void WEBNode::recordPathName() {
	std::string s1;
	ros::NodeHandle rn;
	pathfp = fopen(pathfile.c_str(), "a");  // w : open new file

  if (pathfp == NULL) {
  	ROS_INFO("I couldn't open pathdata.dat for writing.\n");    
  	return;
  }
	rn.getParam("PathName",s1);
	//ROS_INFO("Save Path Name to File");
	fprintf(pathfp,"********* Path Name : %s ***************\n",s1.c_str());		
  fclose(pathfp);	
}

const char *WEBNode::get_filename_ext(const char *filename) {
    const char *dot = strrchr(filename, '.');
    if(!dot || dot == filename) return "";
    return dot + 1;
}

char *WEBNode::remove_ext(char* myStr, char extSep, char pathSep) {
    char *retStr, *lastExt, *lastPath;

    // Error checks and allocate string.
    if (myStr == NULL) return NULL;
    if ((retStr = (char *)malloc (strlen (myStr) + 1)) == NULL) return NULL;

    // Make a copy and find the relevant characters.
    strcpy (retStr, myStr);
    lastExt = strrchr (retStr, extSep);
    lastPath = (pathSep == 0) ? NULL : strrchr (retStr, pathSep);

    // If it has an extension separator.

    if (lastExt != NULL) {
        // and it's to the right of the path separator.

        if (lastPath != NULL) {
            if (lastPath < lastExt) {
                // then remove it.

                *lastExt = '\0';
            }
        } else {
            // Has extension separator with no path separator.

            *lastExt = '\0';
        }
    }
    // Return the modified string.
    return retStr;
}

void WEBNode::readCleanPlanfiles(void) 
{
	DIR* FD;
  struct dirent* in_file;
  FILE    *output_file;
  FILE    *entry_file;
  char    *s, *r;
	int ret,ret1;
	std::string ss;
	int mapidx,cleanidx;

	/* Scanning the in directory */
  if (NULL == (FD = opendir (cleandir.c_str()))) 
  {
  	ROS_INFO("----- Test : Failed to open cleanplan directory\n");
    return;
  }
	mapidx = 0;
	cleanidx = 0;
	while ((in_file = readdir(FD))) 
  {
  	// On linux/Unix we don't want current and parent directories
    // If you're on Windows machine remove this two lines   
    if (!strcmp (in_file->d_name, "."))
    	continue;
    if (!strcmp (in_file->d_name, ".."))    
      continue;
    //Open directory entry file for common operation     
    //entry_file = fopen(in_file->d_name, "a");
		//if (entry_file == NULL)
    //{
    //	ROS_INFO("---- Test : Failed to open cleanplan file : %s  -------",in_file->d_name);    
    //  continue;
    //}
		//ROS_INFO("------ Test : FIlename : %s --------",in_file->d_name);
		//ROS_INFO("----- Test : File Extension = %s -------", get_filename_ext(in_file->d_name));
		ret = strcmp (get_filename_ext(in_file->d_name), "jpeg");
		ret1 = strcmp (get_filename_ext(in_file->d_name), "jpg");
		if ((ret == 0) || (ret1 == 0)) {
			//ROS_INFO("----- Test : JPEG/Map File -------");
			// check for "map" or "clean"
			s = remove_ext(in_file->d_name,'.','/');
			r = strstr(s,"map");
			if (r != NULL) {
				//ROS_INFO("----- Test : JPEG/Map File = %s -------",s);
				ss.assign(s,strlen(s));
				mapfilearray[mapidx] = ss;
				mapidx++;
			} else {
				r = strstr(s,"clean");
				if (r != NULL) {
					//ROS_INFO("----- Test : JPEG/Clean File = %s -------",s);
					ss.assign(s,strlen(s));
					cleanfilearray[cleanidx] = ss;
					cleanidx++;
				}
			}
		}		
	}
	if (s != NULL) {
		free(s);
	}
	for (int i=0;i<mapidx;i++) {
		//ROS_INFO("------- WebNode : idx=%d. mapfile=%s ---------",i,mapfilearray[i].c_str());
	}
	for (int i=0;i<cleanidx;i++) {
		//ROS_INFO("------- WebNode : idx=%d. cleanfile=%s ---------",i,cleanfilearray[i].c_str());
	}
	return;
}

void WEBNode::checkPathTest() {
	htbot::path msg;
	ros::Time delay_time;
	ros::NodeHandle rn;
	string l1,l2;
	int i,j;
	double checkPlanSum;
	char buf [200];
	string s;

	rn.getParam("StartLP",i);
	rn.getParam("EndLP",j);  
	sprintf(buf,"-- checkPathTest : startLP==%d. EndLP=%d.--",i,j);
	s.assign(buf,strlen(buf));
	publish_event(s);

	msg.px = poseInfo[i][0];
	msg.py = poseInfo[i][1];
	msg.pz = poseInfo[i][2];
	msg.prx = poseInfo[i][3];
	msg.pry = poseInfo[i][4];
	msg.prz = poseInfo[i][5];
	msg.prw = poseInfo[i][6];
	msg.gx = poseInfo[j][0];
	msg.gy = poseInfo[j][1];
	msg.gz = poseInfo[j][2];
	msg.grx = poseInfo[j][3];
	msg.gry = poseInfo[j][4];
	msg.grz = poseInfo[j][5];
	msg.grw = poseInfo[j][6];
	msg.tol = 0.5;

	chklp_pub.publish(msg);
}

void WEBNode::checkLP() {
	htbot::path msg;
	ros::Time delay_time;
	ros::NodeHandle rn;
	string l1,l2;
	double checkPlanSum;

	for (int i=0;i<numLP;i++) {
		for (int j=0;j<numLP;j++) {
			if (i==j) {
				continue;
			}
			// check path from i to j
			msg.px = poseInfo[i][0];
			msg.py = poseInfo[i][1];
			msg.pz = poseInfo[i][2];
			msg.prx = poseInfo[i][3];
			msg.pry = poseInfo[i][4];
			msg.prz = poseInfo[i][5];
			msg.prw = poseInfo[i][6];
			msg.gx = poseInfo[j][0];
			msg.gy = poseInfo[j][1];
			msg.gz = poseInfo[j][2];
			msg.grx = poseInfo[j][3];
			msg.gry = poseInfo[j][4];
			msg.grz = poseInfo[j][5];
			msg.grw = poseInfo[j][6];
			msg.tol = 0.5;
			l1 = LandingPointString[i];
			l2 = LandingPointString[j];
			chklp_pub.publish(msg);
			delay_time = ros::Time::now();
			while(true) {
				if ( ros::Time::now() > (delay_time + ros::Duration(0.2)) ) {
					break;
				}
			}
			rn.getParam("checkPlanSum",checkPlanSum); 
			if (checkPlanSum == 200.0) {
				ROS_INFO(" ========== web_node checkLP : Path from %s to %s is Good ===========",l1.c_str(),l2.c_str());
			} else {
				ROS_INFO(" ========== web_node checkLP : Path from %s to %s is Bad ===========",l1.c_str(),l2.c_str());
			}
		}
	}
}

bool WEBNode::sendGoalpub(double x,double y, double z, double rx, double ry, double rz, double rw, double pd, double pa, int opt)
{

	htbot::move cmd;
	
	cmd.x = x;
	cmd.y = y;
	cmd.z = z;
	cmd.rx = rx;
	cmd.ry = ry;
	cmd.rz = rz;
	cmd.rw = rw;
	cmd.pd = pd;
	cmd.pa = pa;
	cmd.opt = opt;

	move_pub.publish(cmd);
	//ROS_INFO(" ------- WebNode : opt=%d -------------",opt);
	return true;
}

bool WEBNode::sendGoalpubMP(int slp, int elp, int opt)
{

	htbot::move cmd;
	
	cmd.slp = slp;
	cmd.elp = elp;
	cmd.opt = opt;

	move_pub.publish(cmd);
	//ROS_INFO("------- WebNode New SendGoal : slp=%d. elp=%d. opt=%d -------------",slp,elp,opt);
	return true;
}

bool WEBNode::cmdGoalpub(int cmd,int startidx, int lastidx, int numgoal, double x, double y, double rz, double rw, double pd, double pa, int opt)
{

	htbot::goal msg;
	msg.cmd = cmd;
	msg.startidx = startidx;
	msg.lastidx = lastidx;
	msg.numgoal = numgoal;
	msg.x = x;
	msg.y = y;
	msg.rz = rz;
	msg.rw = rw;
	msg.pd = pd;
	msg.pa = pa;
	msg.opt = opt;

	goal_pub.publish(msg);
	return true;
}

void WEBNode::test_escape(void) {
	ros::NodeHandle rn; 
	int moreopen,mapdirection;
	double leftlaserobsdist,rightlaserobsdist;

	rn.getParam("MapDirectionToSearch",mapdirection);
	rn.getParam("DirectionToSearch",moreopen);
	rn.getParam("LeftLaserObsDist",leftlaserobsdist);
	rn.getParam("RightLaserObsDist",rightlaserobsdist);

	//ROS_INFO("******* mapdirection:%d. moreopen:%d. RightLaserObsDist:%.3f. leftlaserobsdist:%.3f. ************",mapdirection,moreopen,rightlaserobsdist,leftlaserobsdist);
	if (leftlaserobsdist < 0.3) {
		moreopen--;
	}
	if (rightlaserobsdist < 0.3) {
		moreopen++;
	}
	moreopen = moreopen + mapdirection;
	if (moreopen < 0) {
		//angular_ = -0.3;  // rotate right
		ROS_INFO(" ******* Rotate Right : Angular = negative *********");
	} else {
		if (moreopen > 0) {
			//angular_ = 0.3;  // rotate left
			ROS_INFO(" ******* Rotate Left : Angular = positive *********");
		}
	}
}	

void WEBNode::publish_queue(void)
{
	int lp,lp1,lp2,lp3;
	int ps, psmin;
	htbot::queue qmsg;

	//ROS_INFO("Publish Queue");
	lp1 = 999;
	lp2 = 999;
	lp3 = 999;
	if (numLPInQ > 0) {
		psmin = POSENOTINUSE;
		for (int i=0;i<numLP;i++) {
			if ( (poseStatus[i] != POSENOTINUSE) && (poseStatus[i] > 0) ) {
				ps = poseStatus[i];
				if (ps < psmin) {
					psmin = ps;
					lp = i;
				}
			}
		}
		lp1 = lp;
		qmsg.fLP1 = LandingPointString[lp1];
		qmsg.noQ = 1;
		if (numLPInQ > 1) {
			psmin = POSENOTINUSE;
			for (int i=0;i<numLP;i++) {
				if ( (poseStatus[i] != POSENOTINUSE) && (poseStatus[i] > 0) ) {
					ps = poseStatus[i];
					if (ps < psmin) {
						if ((i != lp1)) {
							psmin = ps;
							lp = i;
						}
					}
				}
			}
			lp2 = lp;
			qmsg.fLP2 = LandingPointString[lp2];
			qmsg.noQ = 2;
		}		
		if (numLPInQ > 2) {
			psmin = POSENOTINUSE;
			for (int i=0;i<numLP;i++) {
				if ( (poseStatus[i] != POSENOTINUSE) && (poseStatus[i] > 0) ) {
					ps = poseStatus[i];
					if (ps < psmin) {
						if ((i != lp1) && (i != lp2)) {
							psmin = ps;
							lp = i;
						}
					}
				}
			}
			lp3 = lp;
			qmsg.fLP3 = LandingPointString[lp3];
			qmsg.noQ = 3;
		}
	} else {
		qmsg.noQ = 0;
		qmsg.fLP1 = "";
		qmsg.fLP2 = "";
		qmsg.fLP3 = "";
	}
	queue_pub.publish(qmsg);
	return;
}

void WEBNode::cancel_goal(void)
{
	ros::NodeHandle rn; 
	int navs;
	actionlib_msgs::GoalID msg;
	rn.getParam("navstatus",navs);
	if (navs == 7) {
		rn.setParam("Cancel_Nav",true);
	}
	msg.stamp = ros::Time::now();
	msg.id = "CancelGoal";
	cancel_pub.publish(msg);
	//ROS_INFO("\n ----------- Sent Cancel Goal : Web Node ------------\n ");
	return;
}

int WEBNode::searchLP(string s)
{
	std::string lps, s1;
	int rLP;
	std::size_t found;

	lps = s;
	rLP = -1;
	trim(lps);

	for (int i=0;i<numLP;i++) {
		s1 = LandingPointString[i];
		trim(s1);
		if (s1.compare(lps) == 0) {
			rLP = i;
			break;
		} 
	}
	return rLP;
}

bool WEBNode::checkPasswd(string s)
{
	std::string s1,pw;
	int rLP;
	std::size_t found;
	ros::NodeHandle rn; 	
	
	//rn.getParam("AutoLogin",AutoLogin);
	//if (AutoLogin) {
	//	correctPasswd = true;
	//	return correctPasswd;
	//}
	pw="Test12345";
	s1 = s;
	trim(s1);
	ROS_INFO("-------WebNode-checkPasswd : input passwd=%s. -----------",s1.c_str());
	if (s1.compare(pw) == 0) {
		correctPasswd = true;
		ROS_INFO("-------WebNode-checkPasswd : Password Correct -----------");
	} else {
		correctPasswd = false;
		ROS_INFO("-------WebNode-checkPasswd : Password Wrong -----------");
	}
	return correctPasswd;
}

void WEBNode::publish_status(string s)
{
	htbot::status status;
	status.msg = s;
	status_pub.publish(status);
	return;
}

void WEBNode::publish_clear(void)
{
	htbot::clear cmd;
	cmd.cmd = 1;
	clear_pub.publish(cmd);
	return;
}

bool WEBNode::qServiceCallback(htbot::mqueue::Request &req,htbot::mqueue::Response &res)
{
	//ROS_INFO("Queue Command Services"); 
	char buf [100];
	string s1;
	int cmd,LP;
	ros::NodeHandle rn;  
	cmd = req.cmd;
  //ROS_INFO("numLP : %d.",numLP);
	switch (cmd) {	
    case 1:  // push to queue			
			LP = req.LP;
			//ROS_INFO("Push : %d.",LP);
			if (mqPush(LP)) {
				res.status = 1; 
			} else {
				res.status = 0; 
			}      
      break;	
		case 2: // pop from queue
			LP = mqPop();
			//ROS_INFO("Pop : %d.",LP);
			if (LP >= 0) {
				res.tx = poseInfo[LP][0];
				res.ty = poseInfo[LP][1];
				res.tz = poseInfo[LP][2];
				res.rx = poseInfo[LP][3];
				res.ry = poseInfo[LP][4];
				res.rz = poseInfo[LP][5];
				res.rw = poseInfo[LP][6];
				res.prd = poseInfo[LP][7];
				res.pra = poseInfo[LP][8];
				res.psd = poseInfo[LP][9];
				res.psa = poseInfo[LP][10];
				res.prd1 = poseInfo[LP][11];
				res.pra1 = poseInfo[LP][12];
				res.psd1 = poseInfo[LP][13];
				res.psa1 = poseInfo[LP][14];
				res.align = poseInfo[LP][15];
				res.func = poseInfo[LP][16];
				res.trolley = poseInfo[LP][17];
				res.trolleydist = poseInfo[LP][18];
				res.LP = LP;
				res.lps = LandingPointString[LP];
				res.status = 1;
			} else {
				res.status = 0;
			}
			break;	
		case 3: // clear queue
      clearLPQueue();
      res.status = 1;
			break;	
	}  
	return true;
}

void WEBNode::taskCallback(const htbot::task::ConstPtr& msg)
{
	ros::NodeHandle rn; 
	int nfLP, ntLP, cmd,type;
	char buf [100];
	string s;

	cmd = msg->cmd; 
	ROS_INFO("------------------ WebNode : Task cmd : %d. ------------------",cmd);
	switch(cmd) {
		case 2: // assigned task
			nfLP = (int) msg->fromLP;
			ntLP = (int) msg->toLP;
			type = (int) msg->type;
			rn.setParam("NTUCNav",true);
			if ((nfLP >= 0) && (ntLP >= 0)) {
				mqPush(nfLP);
				poseInfo[nfLP][16] = 1.0; //1.0; // pickup LP
				poseInfo[nfLP][17] = (double)type;
				mqPush(ntLP+LPSHIFT);
				poseInfo[ntLP+LPSHIFT][16] = 2.0; //2.0; // dropoff LP
				poseInfo[ntLP+LPSHIFT][17] = (double)type;
				switch(type) {
					case 1: // LxB
						poseInfo[nfLP][18] = 0.35; // width = 1.0m
						break;
					case 2:
						poseInfo[nfLP][18] = 0.35; // width = 1.0m
						break;
					case 3:
						poseInfo[nfLP][18] = 0.35; // width = 1.0m
						break;
				}
				ROS_INFO("------------- WebNode : new Task : fromLP=%d. toLP=%d. trolley=%d-----------",nfLP,ntLP+LPSHIFT,type);
			}
			break;
		case 3: // cancel task
			nfLP = (int) msg->fromLP;
			ntLP = (int) msg->toLP;
			if ((nfLP >= 0) && (ntLP >= 0)) {
				removeLPfromQQ(nfLP);
				removeLPfromQQ(ntLP);
			}
			break;
		case 4: // assigned special one LP task
			ntLP = (int) msg->toLP;
			type = (int) msg->type;
			rn.setParam("NTUCNav",true);
			if ((ntLP >= 0)) {
				mqPush(ntLP+LPSHIFT);
				poseInfo[ntLP+LPSHIFT][16] = 0.0; //do nothing
				poseInfo[ntLP+LPSHIFT][17] = (double)type;
				ROS_INFO("------------- WebNode : new Special One LP Task : toLP=%d. trolley=%d-----------",ntLP+LPSHIFT,type);
			}
			break;
	}
}

void WEBNode::buttonCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	ros::NodeHandle rn; 
	int nGP, nLP, butNo;
	//bool ok;
	geometry_msgs::Twist pos;
	bool ok;
	char buf [100];
	string s;

	butNo = msg->data; 

	//ROS_INFO("Web_Node : Button Code Received = %d",butNo);

	switch (butNo) {

		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
			mqPush(butNo);
			break;
		case 13: // robot charging now. Stop robot
			// stop robot.
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.0;  // 7.0
			pos_pub.publish(pos);
			publish_status("Docking Activated");
			publish_debug("Move Node : Docking Achieved.Stop Robot");
			break;

		case 15:
			// stop robot.
			/*
			if (!estop) {
				pos.angular.z = 0.0;
				pos.linear.x = 0.0;
				pos.linear.z = 7.8;  // 7.0
				pos_pub.publish(pos);
				publish_status("Emergency Stop Activated");
				publish_debug("Move Node : Emergency Stop Activated");
				estop = true;
				//publish_sound(15);		
				//sleep(2);
			}
			*/
			break;
		case 16:
			/*
			if (estop) {
				pos.angular.z = 0.0;
				pos.linear.x = 0.0;
				pos.linear.z = 7.9;  // 7.0
				pos_pub.publish(pos);
				publish_status("Emergency Stop Released");
				publish_debug("Move Node : Emergency Released");
				estop = false;
			}
			*/
			break;
		case 21:
			rn.setParam("MoveON",true);
			break;
		case 32:
			ROS_INFO("Cancelled Move");
			//publish_sound(17,0,0);  // navigation aborted
			publish_sound(NAVABORT,0,0);	
			cancel_goal();
			break;
		case 35:
			rn.setParam("Wait15M",true);
			break;
		case 36:			
			recordPathLP();
			break;
	}

	return; 
}

void WEBNode::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	double x,y,z,rz,rw;
	
	px = msg->position.x;
	py = msg->position.y;
	pz = msg->position.z;
	prx = msg->orientation.x;
	pry = msg->orientation.y;
	prz = msg->orientation.z;
	prw = msg->orientation.w;
	//ROS_INFO("Pose : x=%.2f. y=%.2f. rz=%.2f",x,y,rz);
}

bool WEBNode::getPose() 
{
	char buf [100];	
	try
	{
		listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		tf::transformStampedTFToMsg(transform,robotpose);
		tx = robotpose.transform.translation.x;
		ty = robotpose.transform.translation.y;
		tz = robotpose.transform.translation.z;
		rx = robotpose.transform.rotation.x;
		ry = robotpose.transform.rotation.y;
		rz = robotpose.transform.rotation.z;
		rw = robotpose.transform.rotation.w;
		return true;
	}
	catch (tf::TransformException ex)
	{
		ROS_INFO("%s",ex.what());	
		return false;	
	}
}

void WEBNode::publish_sound(int id,int sd, int rsd)
{
	htbot::sound cmd;
	cmd.id = id;
	cmd.startdelay = sd;
	cmd.restartdelay = rsd;
	play_pub.publish(cmd);
	//ROS_INFO("Publish Sound AA");
	return;
}


int WEBNode::storePoseS(string lps) 
{		
	int lp;
	
	lp = searchLP(lps);
	if (lp < 0) {
		numLP++;
		poseInfo[numLP][0] = px;
		poseInfo[numLP][1] = py;
		poseInfo[numLP][2] = pz;
		poseInfo[numLP][3] = prx;
		poseInfo[numLP][4] = pry;
		poseInfo[numLP][5] = prz;
		poseInfo[numLP][6] = prw;	
		poseInfo[numLP][7] = 2.0;
		poseInfo[numLP][8] = 190.0;
		poseInfo[numLP][9] = 2.0;
		poseInfo[numLP][10] = 190.0;
		poseInfo[numLP][11] = 2.0;
		poseInfo[numLP][12] = 190.0;
		poseInfo[numLP][13] = 2.0;
		poseInfo[numLP][14] = 190.0;
		poseInfo[numLP][15] = 2.0;
		poseInfo[numLP][16] = 1.0;
		poseInfo[numLP][17] = 1.0;		
		poseInfo[numLP][18] = 1.0;				
		LandingPointString[numLP] = lps;
		poseStatus[numLP] = 0; // in use not activated.
		lp = numLP;
		//numLP++;
	} else {
		poseInfo[lp][0] = px;
		poseInfo[lp][1] = py;
		poseInfo[lp][2] = pz;
		poseInfo[lp][3] = prx;
		poseInfo[lp][4] = pry;
		poseInfo[lp][5] = prz;
		poseInfo[lp][6] = prw;	
		//poseInfo[lp][7] = pre_dist;
		//poseInfo[lp][8] = pre_angle;
		//poseInfo[lp][9] = post_dist;
		//poseInfo[lp][10] = post_angle;
		//poseInfo[lp][11] = pre_dist1;
		//poseInfo[lp][12] = pre_angle1;
		//poseInfo[lp][13] = post_dist1;
		//poseInfo[lp][14] = post_angle1;
		//poseInfo[lp][15] = align;
		//poseInfo[lp][16] = func;	
		//LandingPointString[lp] = lps;	
	}
	ROS_INFO("LP Name : %s. LP : %d. px=%.3f. py=%.3f",lps.c_str(),lp,px,py);
	return lp;
}

void WEBNode::removePose(int GN, int LP) 
{		
	poseInfo[LP][0] = 0.0;
	poseInfo[LP][1] = 0.0;
	poseInfo[LP][2] = 0.0;
	poseInfo[LP][3] = 0.0;
	poseInfo[LP][4] = 0.0;
	poseInfo[LP][5] = 0.0;
	poseInfo[LP][6] = 0.0;	
}

void WEBNode::removeAllPose() 
{	
	int tnLP;
	for (int j=0;j<tnLP;j++) {
		poseInfo[j][0] = 0.0;
		poseInfo[j][1] = 0.0;
		poseInfo[j][2] = 0.0;
		poseInfo[j][3] = 0.0;
		poseInfo[j][4] = 0.0;
		poseInfo[j][5] = 0.0;
		poseInfo[j][6] = 0.0;
	}
}

void WEBNode::initialisation() 
{	
	for (int j=0;j<MAXLP;j++) {
		poseInfo[j][0] = 0.0;
		poseInfo[j][1] = 0.0;
		poseInfo[j][2] = 0.0;
		poseInfo[j][3] = 0.0;
		poseInfo[j][4] = 0.0;
		poseInfo[j][5] = 0.0;
		poseInfo[j][6] = 0.0;
		poseInfo[j][7] = 2.0;
		poseInfo[j][8] = 190.0;
		poseInfo[j][9] = 2.0;
		poseInfo[j][10] = 190.0;
		poseInfo[j][11] = 2.0;
		poseInfo[j][12] = 190.0;
		poseInfo[j][13] = 2.0;
		poseInfo[j][14] = 190.0;
		poseInfo[j][15] = 2.0;
		poseInfo[j][16] = 1.0;
		poseInfo[j][17] = 1.0;		
		poseInfo[j][18] = 1.0;				
		LandingPointString[j] = "";
		poseStatus[j] = POSENOTINUSE;
	}
}

void WEBNode::clearLPQueue() 
{	
	for (int j=0;j<MAXLP;j++) {
		if (poseStatus[j] != POSENOTINUSE) {
			poseStatus[j] = 0;
		}
	}
	numLPInQ = 0;
}

void WEBNode::savePoseS() 
{		
	std::string s1;
	pfp = fopen(posefile.c_str(), "w");  // w : open new file

  if (pfp == NULL) {
  	ROS_INFO("I couldn't open posedata.dat for writing.\n");    
  	return;
  }
	ROS_INFO("------------ WebNode : Save Pose to File. numLP : %d. ----------------",numLP);
	//fprintf(pfp,"%d\n",numLP);
	for (int j=0;j<numLP;j++) {
		tx = poseInfo[j][0];
		ty = poseInfo[j][1];
		tz = poseInfo[j][2];
		rx = poseInfo[j][3];
		ry = poseInfo[j][4];
		rz = poseInfo[j][5];
		rw = poseInfo[j][6];
		pre_dist = poseInfo[j][7];
		pre_angle = poseInfo[j][8];
		post_dist = poseInfo[j][9];
		post_angle = poseInfo[j][10];
		pre_dist1 = poseInfo[j][11];
		pre_angle1 = poseInfo[j][12];
		post_dist1 = poseInfo[j][13];
		post_angle1 = poseInfo[j][14];
		align = poseInfo[j][15];
		func = poseInfo[j][16];
		trolley = poseInfo[j][17];
		trolleydist = poseInfo[j][18];
		s1 = LandingPointString[j];
		fprintf(pfp,"%d %s %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",j,s1.c_str(),\
								tx,ty,tz,rx,ry,rz,rw,pre_dist,pre_angle,pre_dist1,pre_angle1,post_dist,post_angle,post_dist1,post_angle1,\
								align,func,trolley,trolleydist);	
	}	
  fclose(pfp);	
}


void WEBNode::readPosefromFileS() 
{		
	char buf[50];
	int i,j,stn,ret;
	std::string s1;

	pfp = fopen(posefile.c_str(), "r");
  if (pfp == NULL) {
  	ROS_INFO("I couldn't open posedata.dat for reading.\n");    
  	return;
  }
	i = 0;
	j = 0;
	numLP = 0;
	//ret = fscanf(pfp, "%d\n", &numLP);	
	//ROS_INFO("\nNumber of Stations : %d",numLP);
	while(true) {
		if (fscanf(pfp,"%d %s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",\
					&j,buf,&tx,&ty,&tz,&rx,&ry,&rz,&rw,&pre_dist,&pre_angle,&pre_dist1,&pre_angle1,&post_dist,\
					&post_angle,&post_dist1,&post_angle1,&align,&func,&trolley,&trolleydist) == EOF) {
			break;
		}
		s1.assign(buf,strlen(buf));
		//ROS_INFO("\nPoseData : LPName : %s. tx : %.3f. func : %.3f",s1.c_str(),tx,func);
		poseInfo[j][0] = tx;
		poseInfo[j][1] = ty;
		poseInfo[j][2] = tz;
		poseInfo[j][3] = rx;
		poseInfo[j][4] = ry;
		poseInfo[j][5] = rz;
		poseInfo[j][6] = rw;
		poseInfo[j][7] = pre_dist;				
		poseInfo[j][8] = pre_angle;
		poseInfo[j][9] = post_dist;				
		poseInfo[j][10] = post_angle;
		poseInfo[j][11] = pre_dist1;				
		poseInfo[j][12] = pre_angle1;
		poseInfo[j][13] = post_dist1;				
		poseInfo[j][14] = post_angle1;
		poseInfo[j][15] = align;
		poseInfo[j][16] = func;
		poseInfo[j][17] = trolley;
		poseInfo[j][18] = trolleydist;
		LandingPointString[j] = s1;
		poseStatus[j] = 0;
		numLP++;
		//i++;
		//ROS_INFO("\nPoseData : StnNo:%d. LPName : %s. tx : %.3f. func : %.3f",j,s1.c_str(),tx,func);
		//if (i >= (numLP)) {
		//	break;
		//}
	}
	ROS_INFO("----------- WebNode : Number of LPs : %d. ---------------------",numLP);
  fclose(pfp);
}


void WEBNode::publish_debug(string s)
{
	htbot::debug deb;
	deb.msg = s;
	debug_pub.publish(deb);
	return;
}

bool WEBNode::mqPush(int LP) {
	bool ret;
	ros::NodeHandle xn;
  //ROS_INFO("DB Push . LP : %d",LP);

	if (poseStatus[LP] == POSENOTINUSE) {
		return false;
	}
	if (poseStatus[LP] > 0) {  // already activated
		return true;
	}
	if (numLPInQ == 0) {
		QNo = 1;
	}
	poseStatus[LP] = QNo;
	QNo++;
	numLPInQ++;
	xn.setParam("numLPInQ",numLPInQ);
	return true;
}

void WEBNode::removeLPfromQQ(int LP) {
	
	ros::NodeHandle xn;
  
	if (numLPInQ > 0) {
		if ( (poseStatus[LP] != POSENOTINUSE) && (poseStatus[LP] > 0) ) {
			poseStatus[LP] = 0;
			numLPInQ--;
			if (numLPInQ == 0) {
				QNo = 1;
			}
			xn.setParam("numLPInQ",numLPInQ);
		}
	} 
}

int WEBNode::mqPop() {
	bool ret;
	ros::NodeHandle xn;
	int ps, psmin,lp;
  //ROS_INFO("DB Pop");

	psmin = POSENOTINUSE;
	if (numLPInQ > 0) {
		for (int i=0;i<numLP;i++) {
			if ( (poseStatus[i] != POSENOTINUSE) && (poseStatus[i] > 0) ) {
				ps = poseStatus[i];
				if (ps < psmin) {
					psmin = ps;
					lp = i;
				}
			}
		}
		if (psmin != POSENOTINUSE) {
			poseStatus[lp] = 0;
			numLPInQ--;
			if (numLPInQ == 0) {
				QNo = 1;
			}
			xn.setParam("numLPInQ",numLPInQ);
			return lp;
		} else {
			return -1;
		}	
	} else {
		return -1;
	}
}

void WEBNode::publish_navstatus(void)
{
	htbot::navstatus status;
	ros::NodeHandle xn; 
	int ps;
	
	if (numLPInQ > 0) {
		for (int i=0;i<numLP;i++) {
			if ( (poseStatus[i] != POSENOTINUSE) && (poseStatus[i] > 0) ) {
				ps = poseStatus[i];
				switch(ps) {
					case 0:
						status.stn0 = 1;
						break;
					case 1:
						status.stn1 = 1;
						break;
					case 2:
						status.stn2 = 1;
						break;
					case 3:
						status.stn3 = 1;
						break;
					case 4:
						status.stn4 = 1;
						break;
					case 5:
						status.stn5 = 1;
						break;
					case 6:
						status.stn6 = 1;
						break;
					case 7:
						status.stn7 = 1;
						break;
					case 8:
						status.stn8 = 1;
						break;
					case 9:
						status.stn9 = 1;
						break;
				}
			}
		}
	}
	//ROS_INFO("*********************  Lumstatus **************************");
	navstat_pub.publish(status);
	return;
}

/*
bool WEBNode::mqPush(int LP) {
	bool ret;
  ROS_INFO("DB Push . LP : %d",LP);
	mq_srv.request.LP = LP;  		
	mq_srv.request.cmd = 1;
	mq_srv.request.tx = poseInfo[LP][0];	
	mq_srv.request.ty = poseInfo[LP][1];
	mq_srv.request.tz = poseInfo[LP][2];	
	mq_srv.request.rx = poseInfo[LP][3];	
	mq_srv.request.ry = poseInfo[LP][4];		
	mq_srv.request.rz = poseInfo[LP][5];	
	mq_srv.request.rw = poseInfo[LP][6];
	mq_srv.request.prd = poseInfo[LP][7];	
	mq_srv.request.pra = poseInfo[LP][8];	
	mq_srv.request.psd = poseInfo[LP][9];
	mq_srv.request.psa = poseInfo[LP][10];	
	mq_srv.request.prd1 = poseInfo[LP][11];
	mq_srv.request.pra1 = poseInfo[LP][12];	
	mq_srv.request.psd1 = poseInfo[LP][13];	
	mq_srv.request.psa1 = poseInfo[LP][14];
	mq_srv.request.align = poseInfo[LP][15];
	mq_srv.request.func = poseInfo[LP][16];
	mq_srv.request.lps = LandingPointString[LP];
	if (mq_client.call(mq_srv)) {
		if (mq_srv.response.status == 1) {	 
			ret = true;
			//ROS_INFO("DB ok push to move queue");	
		} else {
	    ROS_ERROR("Failed to push to move queue");	
			ret = false;
	 	}
	} else {
		ROS_INFO("DB mq_srv error");
	}
	return ret;
}
*/

void WEBNode::dynaReconfig(int pm, int iV, double dV, string sV)
{
	htbot::dyna dyna;
	dyna.paramid = pm;
	dyna.intValue = iV;
	dyna.doubleValue = dV;
	dyna.strValue = sV;
	dyna_pub.publish(dyna);
	//ROS_INFO("---------- dynaReconfig_int %d --------------",param);
}

void WEBNode::reLocalise(void)
{
	ros::NodeHandle xn;
	int ret;
	ROS_INFO(" ------------WebNode : Start global localisation ----------");	
	localised = false;
	xn.setParam("activate_localisation",true);		
	global_localise();
	while(true) {
		sleep(1);
		ret = system("rosservice call /request_nomotion_update");
		xn.getParam("Found_Localisation",localised);	
		if (localised) {
			break;
		}
	}
	// move to reference
	ROS_INFO("----------- WebNode : Localisation Achieved. Moving to Reference Point ------------");
	mqPush(0);
}

bool WEBNode::webcmdServiceCallback(htbot::mqueue::Request &req,htbot::mqueue::Response &res)
{
	//ROS_INFO("Dbase Command Services"); 
	int fi,ti;
	char buf [100];
	string s1;
	int readyflag;
	int cmd,lp,ret;
	htbot::scanCmd scmd;
	ros::NodeHandle xn;
	geometry_msgs::Twist pos;
	bool ZeroReference;

	cmd = req.cmd;
  //ROS_INFO("---------------WebNode : cmd : %d. --------------------------",cmd);
	switch (cmd) {		
		case 1:
			break;		
		case 10:
			recordPathName();
			break;
		case 100:
			recordPathLP();
			break;
		case 11: // 
			if (correctPasswd) {
				ROS_INFO("WebNode : Command 11 : lps : %s. LP : %d",req.lps.c_str(),req.LP);
				s1 = "Activating LP :  "+req.lps;
				publish_status(s1);
				lp = searchLP(req.lps);
				if (lp >=0) {
					mqPush(lp);
					res.status = 11;
				} else {
					ROS_INFO("--------- WebNode : Error in search for LPname -----------");
					res.status = 0;
				}
			} else {
				res.status = 0;
				ROS_INFO("WebNode : Access Denied. Not Login");
			}
			break;		
		case 110: // 
			ROS_INFO("---------- WebNode : 110 : fromLP : %d. toLP : %d -------------",req.LP,req.LPT);
			lp = req.LP;
			if (lp >=0) {				
				mqPush(lp);
				lp = req.LPT;
				if (lp >=0) {	
					mqPush(lp);
					res.status = 110;
				} else {
					ROS_INFO("---------- WebNode : 110 : Error toLP is invlaid. -------------");
					res.status = 0;
				}
			} else {
				ROS_INFO("---------- WebNode : 110 : Error fromLP is invlaid. -------------");
				res.status = 0;
			}
			break;
		case 12: // find LP using LP name
			lp = searchLP(req.lps);
			if (lp >= 0) {
				mqPush(lp);
				res.status = 12;	
			} else {
				ROS_INFO("Web : Error in search for LP");
				res.status = 0;	
			}
			break;
		case 13:
			lp = mqPop();
			break;
		case 14: 
			test_escape();
			break;
		case 15:
			sendGoalpub(4.5327,4.2259, 0.0000,0.0000,0.0000,-0.0106,0.9999,2.0,190,70);  // opt=70
			break;
		case 16:  // rotate right
			cmdGoalpub(0,0,0,0, 0.0,0.0,0.0,0.0,2.0,-90.0,0);  // opt=71
			break;
		case 17: // rotate left
			cmdGoalpub(0,0,0,0, 0.0,0.0,0.0,0.0,2.0,90.0,0);  // opt=71
			break;
		case 18:
			cmdGoalpub(1,0,0,0, 0.0,0.0,0.0,0.0,1.5,190.0,0);  // opt=71			
			break;
		case 19:
			cmdGoalpub(1,0,0,0, 0.0,0.0,0.0,0.0,-1.5,190.0,0);  // opt=71	
			break;		
		case 2: // Set/Reset Map
			ROS_INFO("@@@@@@@@@@@@   WebNode : Shutting Activated @@@@@@@@@@@@@@@@@  ");
			//sleep(2);
			//system("sudo shutdown -h now");
			//xn.setParam("shutdown",1);
			xn.setParam("shutdownRAC1",true);		
			//publish_sound(28,0,0); 
			publish_sound(SHUTDOWN,0,0);	
			break;
		case 20:
			//sendGoalpub(2.1472,6.0485, 0.0000,0.0000,0.0000,-0.6934,0.7205,2.0,190.0,70);  // opt=70
			cmdGoalpub(2,0,0,0,2.0,2.15,0.7115,0.7027,2.0,190,0);
			break;
		case 21:
			sdir = homedir+"remap.sh";
			ret = system(sdir.c_str());
			res.status = 21;
			break;
		case 22:
			//sendGoalpub(2.1472,6.0485, 0.0000,0.0000,0.0000,-0.6934,0.7205,2.0,190.0,73);  // to Stn2
			//cmdGoalpub(3,0,0,0, 0.0,0.0,0.0,0.0,-1.5,190.0,0);  // opt=71	
			cmdGoalpub(3,0,3,0,0.0,0.0,-0.6934,0.7205,2.0,190,0);
			break;
		case 23:
			sendGoalpub(2.1472,6.0485, 0.0000,0.0000,0.0000,-0.6934,0.7205,2.0,190.0,74);  // opt=74
			break;
		case 24:
			//sendGoalpub(2.1472,6.0485, 0.0000,0.0000,0.0000,-0.6934,0.7205,2.0,190.0,75);  // opt=75
			cmdGoalpub(3,4,5,0,0.0,0.0,-0.6934,0.7205,2.0,190,0);
			break;
		case 25:
			sendGoalpub(1.7284,1.9136, 0.0000,0.0000,0.0000,0.7115,0.7027,2.0,190.0,0);  // opt=75
			ROS_INFO(" Gp back to Power Station");
			break;
		case 26:
			//sendGoalpub(2.1472,6.0485, 0.0000,0.0000,0.0000,-0.6934,0.7205,2.0,190.0,70);  // opt=70
			cmdGoalpub(4,0,0,0,2.0,2.15,0.7115,0.7027,2.0,190,0);
			break;
		case 27:
			//sendGoalpub(2.1472,6.0485, 0.0000,0.0000,0.0000,-0.6934,0.7205,2.0,190.0,70);  // opt=70
			//cmdGoalpub(5,0,0,0,2.1239,6.8568,-0.7132,0.7009,2.0,190,0);
			//checkLP();
			checkPathTest();
			break;
		case 28:
			reLocalise();
			res.status = 28;	
			break;
		case 29:
			publish_sound(NAVABORT,0,0);	
			cancel_goal();
			publish_sound(STOPMOVESOUND,0,5);
			res.status = 29;
			break;
		case 3:
			publish_clear();	
			res.status = 3;	
			break;
		case 30:
			lp = searchLP(req.lps);
			if (lp >=0) {
				res.tx = poseInfo[lp][0];
				res.ty = poseInfo[lp][1];
				res.tz = poseInfo[lp][2];
				res.rx = poseInfo[lp][3];
				res.ry = poseInfo[lp][4];
				res.rz = poseInfo[lp][5];
				res.rw = poseInfo[lp][6];
				res.prd = poseInfo[lp][7];
				res.pra = poseInfo[lp][8];
				res.psd = poseInfo[lp][9];
				res.psa = poseInfo[lp][10];
				res.prd1 = poseInfo[lp][11];
				res.pra1 = poseInfo[lp][12];
				res.psd1 = poseInfo[lp][13];
				res.psa1 = poseInfo[lp][14];
				res.align = poseInfo[lp][15];
				res.func = poseInfo[lp][16];
				res.lps = LandingPointString[lp];
				res.LP = lp;
				res.status = 30;
			} else {
				ROS_INFO("Error in search for LPname");
				res.status = 0;
			}
			break;
		case 31: // save map and pose info.
			ROS_INFO("Save Map");				
			//savePoseS();
			//saveParamToFile();
			sdir = "rosrun map_server map_saver -f "+homedir+"catkin_ws/src/htbot/maps/docmap";
			//sprintf(buf,sdir.c_str());
			//sprintf(buf,"rosrun map_server map_saver -f /home/racnys/catkin_ws/src/htbot/maps/docmap");			
			//ROS_INFO("Save Map and Pose Info After");
			//system(buf);	
			ret = system(sdir.c_str());
			res.status = 31;
			break;		
		case 32: // request LP Pose Info
			ROS_INFO("DB Request for LP Pose Inof. GN : %d. LP : %d",req.GN,req.LP);	
			res.tx = poseInfo[req.LP][0];
			res.ty = poseInfo[req.LP][1];
			res.tz = poseInfo[req.LP][2];
			res.rx = poseInfo[req.LP][3];
			res.ry = poseInfo[req.LP][4];
			res.rz = poseInfo[req.LP][5];
			res.rw = poseInfo[req.LP][6];
			res.prd = poseInfo[req.LP][7];
			res.pra = poseInfo[req.LP][8];
			res.psd = poseInfo[req.LP][9];
			res.psa = poseInfo[req.LP][10];
			res.prd1 = poseInfo[req.LP][11];
			res.pra1 = poseInfo[req.LP][12];
			res.psd1 = poseInfo[req.LP][13];
			res.psa1 = poseInfo[req.LP][14];
			res.align = poseInfo[req.LP][15];
			res.func = poseInfo[req.LP][16];			
			res.lps = LandingPointString[req.LP];
			res.LP = req.LP;
			res.status = 32;
			break;	
		case 33: // request  Pose Info
			if (getPose()) {  // get robot pose
				res.rz = rz;
				res.rw = rw;				
			}
			res.status = 33;

			break;	
		case 34: // un mark landing point						
			removePose(req.GN,req.LP);			
			res.status = 34;
			break;
		case 35: // un mark all landing point						
			removeAllPose();			
			res.status = 35;
			break;
		case 36:
			xn.setParam("resetTime",1);
			res.status = 36;
			break;
		case 37: // mark landing point and save ref laser scan	
			ROS_INFO("Save Pose Info to Array : 37");	
			poseInfo[req.LP][0] = req.tx;
			poseInfo[req.LP][1] = req.ty;
			poseInfo[req.LP][2] = req.tz;
			poseInfo[req.LP][3] = req.rx;
			poseInfo[req.LP][4] = req.ry;
			poseInfo[req.LP][5] = req.rz;
			poseInfo[req.LP][6] = req.rw;
			poseInfo[req.LP][7] = req.prd;
			poseInfo[req.LP][8] = req.pra;
			poseInfo[req.LP][9] = req.psd;
			poseInfo[req.LP][10] = req.psa;
			poseInfo[req.LP][11] = req.prd1;
			poseInfo[req.LP][12] = req.pra1;
			poseInfo[req.LP][13] = req.psd1;
			poseInfo[req.LP][14] = req.psa1;
			poseInfo[req.LP][15] = req.align;				
			poseInfo[req.LP][16] = req.func;		
			//storePose(req.GN,req.LP);
			// save ref laser scan
			ROS_INFO("LPInfo : tx=%.3f,ty=%.3f,rz=%.3f,rw=%.3f",req.tx,req.ty,req.rz,req.rw);
			sdir = homedir+"catkin_ws/src/htbot/laser/RefScan%d.json";
			sprintf(buf,sdir.c_str(),req.LP);
			//sprintf(buf,"/home/racnys/catkin_ws/src/htbot/laser/RefScan%d.json",req.LP);
			s1.assign(buf,strlen(buf));
			scmd.cmd = 0;
			scmd.file = s1;				
			scanM_pub.publish(scmd);
			res.status = 37;			
			break;
		case 38: // save pose info.
			ROS_INFO("Save Pose Info to File : 38");				
			savePoseS();
			res.status = 38;
			break;
		case 39: // stop sendgoal.
			ROS_INFO("Stop Sendgoal : 39");				
			xn.setParam("/stop_move",1);
			res.status = 39;
			break;
		case 4:
			xn.setParam("WebLowPower",true);
			publish_status("Web Lower Power Activated");
			res.status = 4;
			break;
		case 40:
			// free robot.
			if (motorONOFF) {
				ROS_INFO("web : free robot");
				pos.angular.z = 0.0;
				pos.linear.x = 0.0;
				pos.linear.z = 7.6;  // 7.0
				pos_pub.publish(pos);
				publish_status("Free Robot");
				motorONOFF = false;
				publish_sound(MOTORRELEASED,0,0);
			}
			res.status = 40;
			break;
		case 41:
			xn.setParam("WebLowPower",false);
			publish_status("Web Lower Power De-Activated");
			res.status = 41;
			break;
		case 42: // mark landing point and save ref laser scan			
			ROS_INFO("Mark LP and Ref Scan : LPName:%s",req.lps.c_str());				
			lp = storePoseS(req.lps);	
			// save ref laser scan
			sdir = homedir+"catkin_ws/src/htbot/laser/RefScan%d.json";
			sprintf(buf,sdir.c_str(),lp);
			//sprintf(buf,"/home/racnys/catkin_ws/src/htbot/laser/RefScan%d.json",req.LP);
			s1.assign(buf,strlen(buf));
			scmd.cmd = 0;
			scmd.file = s1;				
			scanM_pub.publish(scmd);
			res.status = 42;
			break;
		case 43:
			ROS_INFO("----------- WebNode : Activated initialpose 0-----------------");
			initialpose();
			ROS_INFO("----------- WebNode : Activated initialpose 1-----------------");
			res.status = 43;	
			break;
		case 44:
			ret = system("export ROS_MASTER_URI=http://192.168.1.120:11311");
			ret = system("export ROS_HOSTNAME=192.168.1.120");
			ROS_INFO(" -------------- WebNode : set ros master -------------");
			break;
		case 45:
			// stop robot.
			ROS_INFO("web : stop robot");
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.0;  // 7.0
			pos_pub.publish(pos);
			publish_status("Stop Robot");
			res.status = 45;
			break;
		case 46: // mark landing point and save ref laser scan			
			ROS_INFO("Mark LP and Ref Scan : LPName : %s",req.lps.c_str());		
			//px = req.tx;
			//py = req.ty;
			//pz = req.tz;
			storePoseS(req.lps); // store pose into array if pose ok				
			// save ref laser scan
			sdir = homedir+"catkin_ws/src/htbot/laser/"+req.lps+".json";
			//sprintf(buf,sdir.c_str());
			//sprintf(buf,"/home/racnys/catkin_ws/src/htbot/laser/RefScan%d.json",req.LP);
			//s1.assign(buf,strlen(buf));
			scmd.cmd = 0;
			scmd.file = sdir;				
			scanM_pub.publish(scmd);
			res.status = 46;
			break;
		case 47: // mark landing point and save ref laser scan			
			ROS_INFO("Laser Scan : LPName : %s",req.lps.c_str());					
			// save ref laser scan
			sdir = homedir+"catkin_ws/src/htbot/laser/"+req.lps+".json";
			//sprintf(buf,sdir.c_str());
			//sprintf(buf,"/home/racnys/catkin_ws/src/htbot/laser/RefScan%d.json",req.LP);
			//s1.assign(buf,strlen(buf));
			scmd.cmd = 0;
			scmd.file = sdir;				
			scanM_pub.publish(scmd);
			res.status = 47;
			break;
		case 48:
			xn.setParam("restart_localisation",true);
			ROS_INFO("------------------ WebNode 48 : Re-Start Localization ------------");
			break;
		case 49:
			ROS_INFO("-----WebNode : 49 passwd=%s. ------------",req.lps.c_str());
			if (checkPasswd(req.lps)) {
				res.status = 1;
				ROS_INFO("-----WebNode : 49 Login Successful. ------------");
			} else {
				res.status = -1;
				ROS_INFO("-----WebNode : 49 Login Failed. ------------");
			}
			break;
		case 50: // get LP Info data			
			res.lps = LandingPointString[req.LP];  // get LP String
			//res.gps = GroupName[req.GN];
			res.LP = req.LP;
			//res.nLP = GroupNumLP[req.GN];
			
			//ROS_INFO("Request for Info. LP : %d. LP Name : %s. Gp Name : %s",req.LP,res.lps.c_str(),res.gps.c_str());
			break;
		case 51: // rotate  left
			pos.linear.z = 9.0; 
			pos_pub.publish(pos);
			break;
		case 52: // rotate right
			pos.linear.z = 9.1; 
			pos_pub.publish(pos);
			break;
		case 53:  // move forward
			ROS_INFO("------------------ WebNode : Move Forward. ------------------");
			pos.linear.z = 9.5;  
			pos_pub.publish(pos);
			break;
		case 54: // move back
			pos.linear.z = 9.6;  
			pos_pub.publish(pos);
			break;
		case 55:
			/*
			xn.getParam("AutoLogin",AutoLogin);
			if (AutoLogin) {
				switch(operation_mode) {
					case 0: // start
						res.status = 0;
						break;
					case 1: // navigation
						res.status = 1;
						break;
					case 2: // mapping
						res.status = 2;
						break;
				}
			} else {			
				if (correctPasswd) {
					switch(operation_mode) {
						case 0: // start
							res.status = 0;
							break;
						case 1: // navigation
							res.status = 1;
							break;
						case 2: // mapping
							res.status = 2;
							break;
					}
				} else {
					res.status = -1;
				}
			}
			*/
			if (motorONOFF) {
				res.status = 55;
			} else {
				res.status = 0;
			}
			break;
		case 56:  // test estop activation
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.8;  // 7.0
			pos_pub.publish(pos);
			publish_status("Emergency Stop Activated");
			publish_debug("Web Node : Emergency Stop Activated");
			nh.setParam("WebeStop",true);
			//estop = true;
			//publish_sound(15,0,0);		
			publish_sound(ESTOP,0,0);	
			res.status = 56;
			break;
		case 57:  // test estop release
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 7.9;  // 7.0
			pos_pub.publish(pos);
			publish_status("Emergency Stop Released");
			publish_debug("Web Node : Emergency Released");
			nh.setParam("WebeStop",false);
			publish_sound(ESTOPREL,0,0);	
			res.status = 57;
			break;
		case 58:
			//correctPasswd = false;
			//ROS_INFO("--------- WebNode : Logout Done-----------");
			ret = system("~/stopx.sh");
			break;
		case 59:
			// free robot.
			if (!motorONOFF) {
				ROS_INFO("web : engage robot");
				pos.angular.z = 0.0;
				pos.linear.x = 0.0;
				pos.linear.z = 7.7;  // 7.0
				pos_pub.publish(pos);
				publish_status("Engage Robot");
				motorONOFF = true;
				publish_sound(MOTORENGAGED,0,0);
			}
			res.status = 59;
			break;
		case 60:
			xn.setParam("WNMoveON",true);
			break;
		case 61:
			xn.setParam("WNWait15M",true);
			break;
		case 62:
			xn.setParam("REACH",true);
			break;
		case 63:
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 41.3;  // 7.0
			pos_pub.publish(pos);
			break;
		case 64:
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 41.2;  // 7.0
			pos_pub.publish(pos);
			break;
		case 65:
			//test_pid_angle();
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 41.0;  // 7.0
			pos_pub.publish(pos);
			break;
		case 66:
			xn.setParam("Cancel_PID",true);
			break;
		case 67:
			//test_pid_angle();
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 41.7;  // 7.0
			pos_pub.publish(pos);
			break;
		case 68:
			//test_pid_angle();
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 41.77;  // 7.0
			pos_pub.publish(pos);
			break;
		case 69:
			//test_pid_angle();
			pos.angular.z = 0.0;
			pos.linear.x = 0.0;
			pos.linear.z = 41.5;  // 7.0
			pos_pub.publish(pos);
			break;
		case 70:
			pos.linear.z = 42.0;  
			pos_pub.publish(pos);
			break;
		case 71:
			sendGoalpub(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,80);
			break;
		case 72:
			ROS_INFO("-------------- WebNode 72. Setup Path to Stn7  -------------");
			publish_cleanPlan();			
			break;
		case 73:
			lp = req.LP;
			if (lp >=0) {
				res.tx = poseInfo[lp][0];
				res.ty = poseInfo[lp][1];
				res.tz = poseInfo[lp][2];
				res.rx = poseInfo[lp][3];
				res.ry = poseInfo[lp][4];
				res.rz = poseInfo[lp][5];
				res.rw = poseInfo[lp][6];
				res.prd = poseInfo[lp][7];
				res.pra = poseInfo[lp][8];
				res.psd = poseInfo[lp][9];
				res.psa = poseInfo[lp][10];
				res.prd1 = poseInfo[lp][11];
				res.pra1 = poseInfo[lp][12];
				res.psd1 = poseInfo[lp][13];
				res.psa1 = poseInfo[lp][14];
				res.align = poseInfo[lp][15];
				res.func = poseInfo[lp][16];
				res.lps = LandingPointString[lp];
				res.LP = lp;
				res.status = 73;
			} else {
				ROS_INFO("Error in search for LPname");
				res.status = 0;
			}
			break;
		case 77:
			readyflag = 88;
			nh.setParam("RobotReady",readyflag);
			break;
		case 78:
			image_enabled = true;
			break;
		case 79: // switch to gmapping mode
			switch (operation_mode) {
				case 0: // base
					ret = system("roslaunch htbot gmapping.launch &");
					publish_sound(MAPPINGMODE,0,0);					
					operation_mode = 2;
					ROS_INFO("---------- WebNode 79 : Switching to Mapping Mode -------");
					break;
				case 1: // navigation mode					
					ret = system("rosnode kill amcl &");
					ret = system("rosnode kill amclpose &");
					ret = system("rosnode kill map_server &");					
					ret = system("roslaunch htbot gmapping.launch &");
					//ret = system("~/recordbag.sh &");
					publish_sound(MAPPINGMODE,0,0);
					operation_mode = 2;
					ROS_INFO("---------- WebNode 79 : Switching to Mapping Mode -------");
					break;
			}
			res.status = 79;	
			break;	
		case 80:
			ROS_INFO("-------------- WebNode 80 : LP=%d. GN=%d. -------------",req.LP,req.GN);
			sendGoalpubMP(req.LP,req.GN,80);
			break;
		case 81:  // switch to navigation mode
			switch (operation_mode) {
				case 0: // base
					//ret = system("roslaunch htbot amcl_htbot_sick.launch &");
					publish_sound(NAVIGATIONMODE,0,0);
					operation_mode = 1;
					break;
				case 2: // mapping mode
					ret = system("rosnode kill slam_gmapping &");	
					ret = system("roslaunch htbot amcl.launch &");					
					publish_sound(NAVIGATIONMODE,0,0);
					ROS_INFO("---------- WebNode 81 : Switching to Navigation  Mode -------");
					operation_mode = 1;
					//ret = system("rosnode kill /record &");
					break;
			}
			res.status = 81;
			break;
		case 82:
			ROS_INFO("------- WebNode :Zero Reference ------------");
			nh.getParam("ZeroReference",ZeroReference);
			if (!ZeroReference) {
				nh.setParam("ZeroReference",true);
			} else {
				ROS_INFO("------- WebNode :Zero Referencing on-going ------------");
			}
			res.status = 82;
			break;
		case 83: // save map
			ROS_INFO("------- WebNode : Save Map as : %s -----------",req.gps.c_str());			
			sdir = "rosrun map_server map_saver -f "+homedir+"catkin_ws/src/htbot/maps/"+req.gps+" &";			
			ret = system(sdir.c_str());
			ret = system("~/killrosbag.sh &");
			res.status = 83;
			break;
		case 84: // loading cleaning plans and maps
			ROS_INFO("------ WebNode : Loading Cleaning maps and plans -------------");
			LoadCleanMapPlan();
			res.status = 84;
			break;
		case 103:
			ROS_INFO("------ WebNode : Testing Cleaning Plan Move -------------");
			//dynaReconfig(global_planner,1,0.0,"rac_planner/RacPlanner");
			//testdyna();
			nh.setParam("cleanplanfile","globotix.txt");
			//nh.setParam("testglobalplanner",true);		
			nh.setParam("cleanplanmove",true);	
			testcalcq();
			break;
		case 104:
			nh.setParam("cleanplanfile","genericplan10.txt");
			nh.setParam("testglobalplanner",true);	
			break;
		case 105:
			ROS_INFO("------ WebNode : Testing Special Trolley Move to Pickup -------------");
			pos.linear.z = 42.1; //
			pos.linear.x = 1.0; //  trolley type
			pos.angular.z = 0.33; // trolley width
			pos_pub.publish(pos);
			break;
		case 9:
			ROS_INFO("------------------ WebNode : system Backup Activated. -------------------");
			ret = system("/home/rac/sysBackup.sh");
			break;
	}  
	return true;
}
 
void WEBNode::testcalcq()
{
	double qx,qy,qz,qw;
	calcQuat(16.4,5.08,16.4,5.6,&qx,&qy,&qz,&qw);
	ROS_INFO("----------------- WebNode : qz=%.3f. qw=%.3f. ------------------",qz,qw);
	calcQuat(16.4,5.6,16.8,5.6,&qx,&qy,&qz,&qw);
	ROS_INFO("----------------- WebNode : qz=%.3f. qw=%.3f. ------------------",qz,qw);
	calcQuat(23.6,5.6,23.6,6.1,&qx,&qy,&qz,&qw);
	ROS_INFO("----------------- WebNode : qz=%.3f. qw=%.3f. ------------------",qz,qw);
}

void WEBNode::testdyna()
{
	ROS_INFO("--------- Web Node : test dyna ---------------");
	//dynaReconfig(planner_frequency,0,10.0,"");
	//dynaReconfig(planner_patience,0,10.0,"");
	//dynaReconfig(controller_frequency,0,10.0,"");
	//dynaReconfig(controller_patience,0,10.0,"");
	//dynaReconfig(sim_time,0,4.0,"");
	//dynaReconfig(path_distance_bias,0,2.0,"");
	//dynaReconfig(goal_distance_bias,0,2.0,"");	
	
	dynaReconfig(max_vel_x,0,0.65,"");	
	/*
	dynaReconfig(min_vel_x,0,0.23,"");	
	dynaReconfig(acc_lim_x,0,0.3,"");	
	dynaReconfig(acc_lim_y,0,0.3,"");	
	dynaReconfig(acc_lim_theta,0,0.3,"");	
	dynaReconfig(max_vel_theta,0,0.3,"");	
	dynaReconfig(min_vel_theta,0,-0.3,"");	
	dynaReconfig(min_in_place_vel_theta,0,0.3,"");	
	dynaReconfig(vx_samples,11,0.0,"");	
	dynaReconfig(vtheta_samples,11,0.0,"");	
	dynaReconfig(heading_lookahead,0,0.2,"");	
	dynaReconfig(escape_vel,0,-0.2,"");	
	dynaReconfig(global_costmap_update_frequency,0,6.5,"");	
	dynaReconfig(global_costmap_publish_frequency,0,6.5,"");	
	*/
	//dynaReconfig(global_costmap_width,16,0.0,"");	
	//dynaReconfig(global_costmap_height,16,0.0,"");	
	//dynaReconfig(global_costmap_robot_radius,0,0.5,"");	
	//dynaReconfig(global_costmap_footprint_padding,0,0.05,"");	
	//dynaReconfig(local_costmap_update_frequency,0,6.5,"");	
	//dynaReconfig(local_costmap_publish_frequency,0,6.5,"");	
	
	//dynaReconfig(local_costmap_width,15,0.0,"");	
	//dynaReconfig(local_costmap_height,15,0.0,"");	
	//dynaReconfig(local_costmap_robot_radius,0,0.5,"");	
	//dynaReconfig(local_costmap_footprint_padding,0,0.05,"");	
	//dynaReconfig(amcl_max_particles,9000,0.0,"");	
	//dynaReconfig(amcl_min_particles,500,0.0,"");
	//dynaReconfig(global_footprint,0,0.0,"");
	//dynaReconfig(local_footprint,1,0.0,"");
	dynaReconfig(global_planner,1,0.0,"rac_planner/RacPlanner");
}

void WEBNode::LoadCleanMapPlan(void)
{
	htbot::cleanlist msg;
	
}

void WEBNode::testsound()
{
	sleep(3);
	publish_sound(SYSREADY,0,0);
	sleep(3);
	publish_sound(NEEDSPACE,0,0);
	sleep(3);
	publish_sound(NAVABORT,0,0);
	sleep(3);
	publish_sound(ESTOPREL,0,0);
	sleep(3);
	publish_sound(BATLOW,0,0);
	sleep(3);
	publish_sound(ESTOP,0,0);
	sleep(3);
	publish_sound(REACHED,0,0);
	sleep(3);
	publish_sound(FINDPATH,0,0);
	sleep(3);
	publish_sound(MOVEOUTDOCKSTATION,0,0);
	sleep(3);
	publish_sound(MOVETODOCK,0,0);
	sleep(3);
	publish_sound(NEXTJOB,0,0);
	sleep(3);
	publish_sound(RETURNTOCHARGE,0,0);
	sleep(3);
	publish_sound(SHUTDOWN,0,0);
	sleep(3);
	publish_sound(STOCKER,0,0);
	sleep(3);
	publish_sound(WAFESTART,0,0);
	sleep(3);
	publish_sound(SSMC,0,0);
	sleep(3);
	publish_sound(STOPMOVESOUND,0,0);
	sleep(3);
	publish_sound(DETECTCURRENT,0,0);
	sleep(3);
	publish_sound(PREMPTED,0,0);
	sleep(3);
	publish_sound(LOSTLOCALISE,0,0);
	sleep(3);
	publish_sound(RELOCALISE,0,0);
	sleep(3);
	publish_sound(LOCALISE,0,0);
	sleep(3);
	publish_sound(CHECKLOCALISE,0,0);
	sleep(3);
	publish_sound(LOCALISEREADY,0,0);
	sleep(3);
	publish_sound(MAPPINGMODE,0,0);
	sleep(3);
	publish_sound(NAVIGATIONMODE,0,0);
	sleep(3);
	publish_sound(SYSLOADED,0,0);
	sleep(3);
	publish_sound(ZEROING,0,0);
	sleep(3);
	publish_sound(RESTORELOCALISATION,0,0);
	sleep(3);
	publish_sound(SICKLS,0,0);
	sleep(3);
	publish_sound(RIGHTRS,0,0);
	sleep(3);
	publish_sound(NETWORK,0,0);
	sleep(3);
	publish_sound(MOTOROK,0,0);
	sleep(3);
	publish_sound(LEFTRS,0,0);
	sleep(3);
	publish_sound(LS3D,0,0);
	sleep(3);
	publish_sound(REARSICKON,0,0);
	sleep(3);
	publish_sound(REARSICKOFF,0,0);
	sleep(3);
	publish_sound(FRONTSICKOFF,0,0);
	sleep(3);
	publish_sound(FRONTSICKON,0,0);
	sleep(3);
	publish_sound(RIGHTRSOFF,0,0);
	sleep(3);
	publish_sound(REARHDON,0,0);
	sleep(3);
	publish_sound(REARHDOFF,0,0);
	sleep(3);
	publish_sound(MOTOROFF,0,0);
	sleep(3);
	publish_sound(MIDDLERSON,0,0);
	sleep(3);
	publish_sound(MIDDLERSOFF,0,0);
	sleep(3);
	publish_sound(LS3DOFF,0,0);
	sleep(3);
	publish_sound(LEFTRSOFF,0,0);
	sleep(3);
	publish_sound(FRONTHDON,0,0);
	sleep(3);
	publish_sound(FRONTHDOFF,0,0);
	sleep(3);
	publish_sound(BOTTOMRSON,0,0);
	sleep(3);
	publish_sound(BOTTOMRSOFF,0,0);
	sleep(3);
	publish_sound(ALLSYSTEMON,0,0);
	sleep(3);
	publish_sound(LOGOFF,0,0);
	sleep(3);
	publish_sound(ASKLIFT,0,0);
	sleep(3);
	publish_sound(LIFTOPENED,0,0);
	sleep(3);
	publish_sound(MOVEINTOLIFT,0,0);
	sleep(3);
	publish_sound(MOVEOUTOFLIFT,0,0);
	sleep(3);
	publish_sound(LOBBYDOORCMDERROR,0,0);
	sleep(3);
	publish_sound(LOBBYDOORCONTROLERROR,0,0);
	sleep(3);
	publish_sound(REQUESTLOBYDOOROPEN,0,0);
	sleep(3);
	publish_sound(LOBBYDOOROPENED,0,0);
	sleep(3);
	publish_sound(STOPCLEAN,0,0);
	sleep(3);
	publish_sound(RESUMECLEAN,0,0);
	sleep(3);
	publish_sound(PAUSECLEAN,0,0);
	sleep(3);
	publish_sound(MOTORRELEASED,0,0);
	sleep(3);
	publish_sound(MOTORENGAGED,0,0);
	sleep(3);
	publish_sound(CLEANING,0,0);
	sleep(3);
	publish_sound(CLEANREPORT,0,0);
	sleep(3);
	publish_sound(CLEANCOMPLETED,0,0);
	sleep(3);
	publish_sound(SYSTEMBACKUP,0,0);
	sleep(3);
	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "web node");  
  ros::NodeHandle rn;  	
	int cc,ret,cc1;	
	bool moveready,loadfire,ALLSENSORSOK,closeROSRAC,shutdownRAC,closeshutFlag;
	ros::Rate loop_rate(10.0);
	WEBNode cNode(rn);
	cNode.initialisation();
	sleep(3);
	cNode.readPosefromFileS();	
	//cNode.readCleanPlanfiles();
	cc = 0;
	cc1 = 0;
	moveready = false;
	loadfire = false;
	ALLSENSORSOK = false;
	closeshutFlag = false;
	closeROSRAC = false;
	shutdownRAC = false;
	rn.setParam("MOVEREADY",moveready);
	//rn.setParam("ALLSENSORSOK",ALLSENSORSOK);		
	while (true) {  	  	 	  
		cc++;
		if (cc == 10) {
			cNode.publish_navstatus();
			cNode.publish_queue();
			cc = 0;			
			if (!moveready) {
				rn.getParam("MOVEREADY",moveready);
				//rn.getParam("ALLSENSORSOK",ALLSENSORSOK);		
				if (moveready) {
					cNode.motorONOFF = true;
					cNode.publish_sound(SYSREADY,0,5);
					loadfire = true;
					cc1 = 0;
					//ret = system("firefox -new-window -reload http://127.0.0.1/index.html &");
				}
			}		
			rn.getParam("closeROSRAC",closeROSRAC);
			rn.getParam("shutdownRAC",shutdownRAC);		
			if (closeROSRAC && !closeshutFlag) {
				cNode.publish_sound(LOGOFF,0,0);
				ROS_INFO("-------------- WebNode : Logoff Activated --------------------");
				closeshutFlag = true;
				ret = system("killall checkIP.sh &");
				ret = system("~/stop5.sh");
			}
			if (shutdownRAC && !closeshutFlag) {
				ROS_INFO("-------------- WebNode : Shutdown Activated --------------------");
				cNode.publish_sound(SHUTDOWN,0,0);
				closeshutFlag = true;
				ret = system("~/shut.sh");
			}
			//ROS_INFO("------------WebNode : Loop--------------");
		}
		if (loadfire) {
			cc1++;
			//ROS_INFO("---------------- WebNode : Loading cc1: %d------------",cc1);
			if (cc1 > 25) {
				//ROS_INFO("---------------- WebNode : Loadfire ------------");
				ret = system("firefox -new-window http://127.0.0.1/index.html &");
				//ROS_INFO("---------------- WebNode : Loadfire A------------");
				loadfire = false;
				//cNode.testsound();
			}
		}
		ros::spinOnce();	
  	loop_rate.sleep();
  }
	
	//ros::MultiThreadedSpinner spinner(4);
  

  //spinner.spin();
  
  return 0;
}
