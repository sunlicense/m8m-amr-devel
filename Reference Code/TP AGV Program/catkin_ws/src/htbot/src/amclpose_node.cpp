/*
 * This node is the amcl pose node
 * The node will monitor the localisation of the robot and report lost of localisation
 * and achievement of localisation.
 */
/* 	History
*		Date Modified :14.10.2020
*		
*/

#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "boost/algorithm/string.hpp"
#include "htbot/sound.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "htbot/PLAYSOUND.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include "htbot/debug.h"
#include "htbot/robot.h"


using namespace std;
using namespace boost::algorithm;

class AmclPoseNode
{
public:
	AmclPoseNode(ros::NodeHandle rn);	
	void publish_sound(int id,int sd, int rsd);
	void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void cancel_goal(void);
	void stop_Nav(void);
	void publish_event(string s);
	void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
	void publish_robot_state(void);
	void nomotion_update(int *cnt, int *ms);
	

	ros::Subscriber amcl_sub;
	ros::Publisher fButton_pub;
	double looprate;
	ros::Publisher play_pub;
	ros::Publisher cancel_pub; 
	ros::Publisher event_pub;
	ros::Subscriber pose_sub;
	ros::Publisher robotstat_pub;

	double localise_x,localise_y,localise_a,lostlocalise_x,lostlocalise_y,lostlocalise_a;
	double border_x,border_y,border_a,relocalise_x,relocalise_y,relocalise_a;
	bool Lost_Localisation,borderline,amcl_monitor,printamclstatus;
	int num_request_nomotion_update,navstatus,num_lost_localization,numlostcnt;
	double max_x,max_y,max_a;
	double px,py,pz,prx,pry,prz,prw;
	double opx,opy;
	ros::Time poseTime;
	bool PoseReady;
	double amclx,amcly,amcla,RobotPose_Step,robot_pose_diff;
	std::string localstatus;
	bool restartLocalisation,StablisedAMCL;
	

private:
	ros::NodeHandle nh;
	boost::mutex mut;
};

AmclPoseNode::AmclPoseNode(ros::NodeHandle rn):
	localise_x(0.18),localise_y(0.18),localise_a(0.18),lostlocalise_x(2.5),lostlocalise_y(2.5),lostlocalise_a(1.2),
	navstatus(0),max_x(0.0),max_y(0.0),max_a(0.0),	Lost_Localisation(false),
  borderline(false),amcl_monitor(false),num_lost_localization(0),numlostcnt(0),printamclstatus(false),
  PoseReady(false),robot_pose_diff(0.0),StablisedAMCL(false),	restartLocalisation(false)
{
	ros::NodeHandle sn(rn);
	play_pub = sn.advertise<htbot::sound>("sound", 100);
	amcl_sub = sn.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &AmclPoseNode::amclCallback,this);
	cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
	fButton_pub = nh.advertise<std_msgs::UInt16>("button",1);
	event_pub = nh.advertise<htbot::debug>("event",100);
	pose_sub = nh.subscribe<geometry_msgs::Pose>("/robot_pose", 1, &AmclPoseNode::poseCallback,this);
	robotstat_pub = nh.advertise<htbot::robot>("/robotstatus",10);

	nh.param("localise_x",localise_x, 0.1);
	nh.param("localise_y",localise_y, 0.1);
	nh.param("localise_a",localise_a, 0.1);

	nh.param("border_x",border_x, 0.13);
	nh.param("border_y",border_y, 0.13);
	nh.param("border_a",border_a, 0.10);

	nh.param("relocalise_x",relocalise_x, 2.0);	
	nh.param("relocalise_y",relocalise_y, 2.0);	
	nh.param("relocalise_a",relocalise_a, 1.2);

	nh.param("lostlocalise_x",lostlocalise_x, 0.13);	
	nh.param("lostlocalise_y",lostlocalise_y, 0.13);	
	nh.param("lostlocalise_a",lostlocalise_a, 0.13);
	nh.param("RobotPose_Step",RobotPose_Step, 1.5);

	nh.param("num_lost_localization",num_lost_localization,3);
	nh.param("num_request_nomotion_update",num_request_nomotion_update,15);
}

void AmclPoseNode::publish_robot_state(void)
{
	ros::NodeHandle xn; 
	
	htbot::robot stat;
  time_t t = time(NULL);
	char buf [100];
	string st,sf;	
  struct tm *tm = localtime(&t);
	strftime(buf, sizeof(buf), "%c", tm);	
	st.assign(buf,strlen(buf));
	
	stat.amclx = amclx;
	stat.amcly = amcly;
	stat.amcla = amcla;
	stat.robotposediff = robot_pose_diff;
	stat.localisationstatus = st + " @ " + localstatus;
	robotstat_pub.publish(stat);
	return;
}

void AmclPoseNode::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	double dx,dy,dd;
	ros::NodeHandle rn; 
	char buf[200];
	string s1;
	int navstatus;

	if (!PoseReady) {
		rn.getParam("PoseReady",PoseReady);
		if (PoseReady) {
			px = msg->position.x;
			py = msg->position.y;
			opx = px;
			opy = py;
			sprintf(buf,"--- AMCLPose : First Time PoseReady. px=%.3f. py=%.3f. opx=%.3f. opy=%.3f. ---",px,py,opx,opy);
			s1.assign(buf,strlen(buf));
			publish_event(s1);
			poseTime = ros::Time::now();			
		}
		return;
	}
	pz = msg->position.z;
	prx = msg->orientation.x;
	pry = msg->orientation.y;
	prz = msg->orientation.z;
	prw = msg->orientation.w;
	if ( ros::Time::now() > (poseTime + ros::Duration(1.0)) ) {
		rn.getParam("Lost_Localisation",Lost_Localisation);	
		px = msg->position.x;
		py = msg->position.y;
		dx = px - opx;
		dy = py - opy;
		robot_pose_diff = sqrt((dx * dx) + (dy * dy));
		if ((robot_pose_diff > RobotPose_Step) && !Lost_Localisation && PoseReady && amcl_monitor) {  // assuming max speed of 1.5m/s for robot
			// localisation problem
			Lost_Localisation = true;
			rn.setParam("Lost_Localisation",true);
			sprintf(buf,"--- amclpose Node: Sudden Jump in Pose. dd=%.3f. ---",robot_pose_diff);
			s1.assign(buf,strlen(buf));
			publish_event(s1);
			rn.getParam("navstatus",navstatus);
			if (navstatus == 7) {
				cancel_goal();
			}
			localstatus = "Lost Localisation due to Sudden Jump in Pose";
		} else {
			opx = px;
			opy = py;
			if (Lost_Localisation) {
				PoseReady = false;
				rn.setParam("PoseReady",PoseReady);
			}
		}		
		poseTime = ros::Time::now();		
				
	}
}

void AmclPoseNode::publish_event(string s)
{

	htbot::debug status;
	status.msg = s;
	event_pub.publish(status);
	return;
}

void AmclPoseNode::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	double x,y,an,xx,yy,anx;
	ros::NodeHandle rn; 
	char buf[200];
	string s1;

	{  // lock scope
		boost::mutex::scoped_lock lock(mut);
		x = msg->pose.covariance[0];
		y = msg->pose.covariance[7];
		an = msg->pose.covariance[35];						
	}
	amclx = sqrt(x);
	amcly = sqrt(y);
	amcla = sqrt(an);
	
	if (printamclstatus) {
		//sprintf(buf,"--- amclpose Node : AMCL Status:  xx=%.3f. yy=%.3f. anx=%.3f ---",amclx,amcly,amcla);
		//s1.assign(buf,strlen(buf));
		//publish_event(s1);
		ROS_INFO("------- amclpose Node : AMCL Status:  xx=%.3f. yy=%.3f. anx=%.3f -------",amclx,amcly,amcla);
	}
	
}

void AmclPoseNode::stop_Nav(void)
{
	std_msgs::UInt16 msg;
	msg.data = 32;
	fButton_pub.publish(msg);
	return;
}

void AmclPoseNode::nomotion_update(int *cnt, int *ms)
{
	ros::NodeHandle rn; 
	int ret,rv;
	int status;
	status = 1;
	rn.getParam("localisation_status",status); // 0=localisation stopped. 1=localisation started. 2=localisation completed. 3=localisation paused
	switch(status) {
		case 0: // stop
		case 2: // completed
			*ms = 0;
			break;
		case 1: // continue
			ret = system("rosservice call /request_nomotion_update");
			ROS_INFO("-------------- AMCLPose : nomotion_update ------------");
			*cnt = *cnt + 1;
			break;		
		case 3: // do nothing
			break;			
	}
	return;
}

void AmclPoseNode::cancel_goal(void)
{
	ros::NodeHandle rn;  		
	actionlib_msgs::GoalID msg;
	msg.stamp = ros::Time::now();
	//rn.setParam("Cancel_Nav",true);
	msg.id = "CancelGoal";
	cancel_pub.publish(msg);
	//rn.setParam("Cancel_Nav",true);
	//ROS_INFO("------------ Sent Cancel Goal : amclpose Node ---------");
	publish_event("--- Amclpose Node : Sent Cancel Goal. ---");
	localstatus = "Cancel Goal due to Lost Localisation";
	//rn.setParam("Cancel_Nav",true);
	return;
}

void AmclPoseNode::publish_sound(int id,int sd, int rsd)
{
	htbot::sound cmd;
	cmd.id = id;
	cmd.startdelay = sd;
	cmd.restartdelay = rsd;
	play_pub.publish(cmd);
	return;
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "amclpose node");  
  ros::NodeHandle rn;  		
	AmclPoseNode sNode(rn);
	ros::Rate loop_rate(2.0);
	bool start_localise,init_localise;
	int cnt,cnt1,cnt2,cnt3,cnt4,ret;
	int mstate,readyflag,lstatus;
	char buf[200];
	string s1;
	ros::Time delayTime,delayTime1;
	bool moveready,pauseLocalisation,testmode;
	//sleep(1);
	start_localise = false;
	init_localise = false;
	rn.setParam("amcl_monitor",false);
	rn.setParam("StablisedAMCL",false);
	rn.setParam("restartLocalisation",false);
	while (true) {
		rn.getParam("MOVEREADY",moveready);
		if (moveready) {
			break;
		}
		sleep(1);
	}	
	ROS_INFO("---------------- AMCLPOSENode : Starting.... !!!!! --------------");
  cnt = 0;
	cnt1 = 0;
	cnt2 = 0;
	cnt3 = 0;
	cnt4 = 0;
	mstate = 0;
	delayTime = ros::Time::now();	
	sNode.poseTime = ros::Time::now();
	testmode = false;
	rn.getParam("testmode",testmode);
	while (true) {
		switch (mstate) {
			case 0:
				if (!testmode) {
					rn.getParam("localisation_status",lstatus);
					rn.getParam("amcl_monitor",sNode.amcl_monitor);
					rn.getParam("restartLocalisation",sNode.restartLocalisation);
					rn.getParam("StablisedAMCL",sNode.StablisedAMCL);
				} else {
					break;
				}
				if (lstatus == 1) { // start localisation
					mstate = 1;
					cnt4 = 0;
					delayTime = ros::Time::now();	
					ROS_INFO("------------- amclpose : Starting Localisation -------------------");
					//sNode.publish_event("--- AmclPoseNode : Initialed Localisation Test ---");
					break;
				} 
				if (sNode.amcl_monitor && !sNode.Lost_Localisation) {
					if (((fabs(sNode.amclx) > sNode.lostlocalise_x) && (fabs(sNode.amcly) > sNode.lostlocalise_y)) || (fabs(sNode.amcla) > sNode.lostlocalise_a) ) {
						sNode.numlostcnt++;
						if ((sNode.numlostcnt > sNode.num_lost_localization)) {
							rn.setParam("Lost_Localisation",true);
							sNode.Lost_Localisation = true;
							sNode.PoseReady = false;
							rn.setParam("PoseReady",sNode.PoseReady);
							sNode.publish_sound(LOSTLOCALISE,0,5);							
							sprintf(buf,"--- Amclpose Node : S1 : AMCL lost localisation. x=%.3f. y=%.3f. an=%.3f ---",sNode.amclx,sNode.amcly,sNode.amcla);
							s1.assign(buf,strlen(buf));
							sNode.publish_event(s1);
							sNode.localstatus = "Cancel Goal due to AMCL Lost Localisation";
							sNode.cancel_goal();
						}					
					}
					sNode.StablisedAMCL = false;
					rn.setParam("StablisedAMCL",false);
				} else {					
					sNode.numlostcnt = 0;
				}
				if (sNode.restartLocalisation) {
					mstate = 31;	
					ret = system("rosservice call /global_localization");	
					sNode.publish_event("--- Amclpose Node : Restarted global localisztion service ---");
					delayTime = ros::Time::now();
					cnt1 = 0;		
					sNode.localstatus = "Re-Started Global Localisation";	
					sNode.restartLocalisation = false;
					rn.setParam("restartLocalisation",false);
					sNode.numlostcnt = 0;
					sNode.publish_sound(RESTORELOCALISATION,0,5);
					break;
				}
				if (sNode.StablisedAMCL || (!sNode.amcl_monitor) ) {
					if (((fabs(sNode.amclx) > sNode.lostlocalise_x) || (fabs(sNode.amcly) > sNode.lostlocalise_y)) || (fabs(sNode.amcla) > sNode.lostlocalise_a) ) {
						if ( ros::Time::now() > (delayTime + ros::Duration(1.0)) ) {
							ret = system("rosservice call /request_nomotion_update");
							delayTime = ros::Time::now();		
							rn.setParam("AMCLPOSE_GOOD",false);	
						}
					} else {
						rn.setParam("AMCLPOSE_GOOD",true);	
					}
				}
				break;
			case 1: // initialised localisation checking
				if ( ros::Time::now() > (delayTime + ros::Duration(0.5)) ) {
					delayTime = ros::Time::now();	
					//ret = system("rosservice call /request_nomotion_update");
					//cnt4++;
					sNode.nomotion_update(&cnt4,&mstate);
					if ((cnt4 > sNode.num_request_nomotion_update-2) && !sNode.printamclstatus) {
						sNode.printamclstatus = true;
					}					
					if (cnt4 > sNode.num_request_nomotion_update) {
						ROS_INFO("----------- AMCLPose : start monitoring localisation status ------------");
						//sNode.publish_event("******* Amclpose Node : start monitoring localisation status ********");
						delayTime = ros::Time::now();		
						mstate = 2;
						break;
					}
					if (cnt4 > 15) {
						if ((fabs(sNode.amclx) < sNode.localise_x) && (fabs(sNode.amcly) < sNode.localise_y) && (fabs(sNode.amcla) < sNode.localise_a)) {
							rn.setParam("Localised_Ready",true);
							mstate = 0; 			
							rn.setParam("localisation_status",2);
							sNode.printamclstatus = false;			
							sNode.localstatus = "Robot Localized @ Initial Localisation Test";
							break;
						}
					}
				}
				break;
			case 2:
				if ( ros::Time::now() > (delayTime + ros::Duration(3.0)) ) {
					if ((fabs(sNode.amclx) < sNode.localise_x) && (fabs(sNode.amcly) < sNode.localise_y) && (fabs(sNode.amcla) < sNode.localise_a)) {
						//sNode.publish_sound(LOCALISE,0,5);	
						rn.setParam("Localised_Ready",true);
						mstate = 0; 			
						rn.setParam("localisation_status",2);
						sNode.printamclstatus = false;		
						sNode.localstatus = "Robot Localized @ Initial Localisation Test";							
					} else {
						mstate = 3;	
						ret = system("rosservice call /global_localization");	
						ROS_INFO("----------------- AMCLPose : started global localisztion service -----------");
						//sNode.publish_event("******* Amclpose Node : started global localisztion service ********");
						sNode.publish_sound(RESTORELOCALISATION,0,0);
						delayTime = ros::Time::now();
						cnt1 = 0;		
						sNode.localstatus = "Robot Not Localised @ Start. Initied Global Localisation";	
						//sNode.publish_sound(RESTORELOCALISATION,3,5);
					}						
					delayTime = ros::Time::now();
				}
				break;
			case 3:
				if ( ros::Time::now() > (delayTime + ros::Duration(0.5)) ) {
					//ret = system("rosservice call /request_nomotion_update");
					//cnt1++;						
					if (cnt1 >= 10) {
						mstate = 4;
						delayTime = ros::Time::now();						
						ROS_INFO("----------- AMCLPOSE : activated re-localisation ------");
						//sNode.publish_event("----------- AMCLPOSE : activated re-localisation ------");
						break;
					}	
					delayTime = ros::Time::now();
					delayTime1 = ros::Time::now();
					sNode.nomotion_update(&cnt1,&mstate);
				}			
				break;
			case 31:
				if ( ros::Time::now() > (delayTime + ros::Duration(1.0)) ) {
					ret = system("rosservice call /request_nomotion_update");
					cnt1++;	
					if (cnt1 >= 5) {
						mstate = 41;
						delayTime = ros::Time::now();
						ROS_INFO("----------- AMCLPOSE 31: activated re-localisation ------");
						sNode.publish_event("----------- AMCLPOSE 31: activated re-localisation ------");
					}	
					delayTime = ros::Time::now();
					delayTime1 = ros::Time::now();
				}			
				break;
			case 4:
				if ( ros::Time::now() > (delayTime + ros::Duration(0.5)) ) {
					//ret = system("rosservice call /request_nomotion_update");					
					if ((fabs(sNode.amclx) < sNode.relocalise_x) && (fabs(sNode.amcly) < sNode.relocalise_y) ) {			
						//sNode.publish_sound(RELOCALISE,0,5);
						sNode.Lost_Localisation = false;
						rn.setParam("Lost_Localisation",false);
						cnt = 0;
						ROS_INFO("----------- AMCLPOSE 4 : localisation Achieved ------");
						mstate = 0;
						rn.setParam("localisation_status",2);
						rn.setParam("Localised_Ready",true);
						sNode.localstatus = "Robot Localised @ Start";	
						break;
					} 
					delayTime = ros::Time::now();
					sNode.nomotion_update(&cnt1,&mstate);
				}
				if ( ros::Time::now() > (delayTime1 + ros::Duration(300.0)) ) {
					ROS_INFO("----------- AMCLPOSE : localisation is Not possible ------");
					//sNode.publish_event("----------- AMCLPOSE 4 : Re-localisation is Not possible ------");
					mstate = 16;
				}
				break;		
			case 41:
				if ( ros::Time::now() > (delayTime + ros::Duration(1.0)) ) {
					ret = system("rosservice call /request_nomotion_update");
					//if (sNode.Re_Localisation) {		
					if ((fabs(sNode.amclx) < sNode.relocalise_x) && (fabs(sNode.amcly) < sNode.relocalise_y) ) {			
						sNode.publish_sound(RELOCALISE,0,5);
						sNode.Lost_Localisation = false;
						rn.setParam("Lost_Localisation",false);
						cnt = 0;
						ROS_INFO("----------- AMCLPOSE 41 : Re-localisation is Achieved ------");
						mstate = 0;
						rn.setParam("localisation_status",2);
						rn.setParam("Localised_Ready",true);						
						sNode.localstatus = "Robot Re-Localised Again";	
						sNode.StablisedAMCL = true;
						rn.setParam("StablisedAMCL",true);
					} 
					delayTime = ros::Time::now();
				}
				if ( ros::Time::now() > (delayTime1 + ros::Duration(300.0)) ) {
					ROS_INFO("----------- AMCLPOSE 41 : Re-localisation is Not possible ------");
					//sNode.publish_event("----------- AMCLPOSE 4 : Re-localisation is Not possible ------");
					mstate = 16;
				}
				break;
			case 16:
				if ( ros::Time::now() > (delayTime + ros::Duration(1.0)) ) {
					delayTime = ros::Time::now();	
					ROS_INFO("---------------- AmclPoseNode 16 : Lost Localisation.  ------------");
					rn.getParam("localisation_status",lstatus);
					if (lstatus == 0) {
						mstate = 0;
					}
				}
				break;
		}
		sNode.publish_robot_state();
		ros::spinOnce();	
  	loop_rate.sleep();
	}
  return 0;
}

