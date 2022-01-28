/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Modified : 26.2.2019
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
#include "boost/algorithm/string.hpp"
#include <math.h>
#include "htbot/goal.h"
#include <nav_msgs/GetPlan.h>
#include "htbot/move.h"
#include "htbot/path.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "htbot/debug.h"
#include "htbot/angle.h"

#define PI 3.141593

using namespace std;


ros::Subscriber chk_cmd;
ros::Subscriber chkpath_cmd;
ros::ServiceClient makeplan;
ros::Subscriber pose_sub;
ros::Subscriber move_cmd;
ros::Publisher event_pub;
double px,py,pz,prx,pry,prz,prw;
double gx,gy,grz,grw;
int navstatus,PathIndex;
ros::Time last_cmd_time;
ros::Duration srvTime;
ros::Time srvStartTime;
double checkPlanRX,checkPlanRY,checkPlanRZ,checkPlanRW;
double angle_to_goal;

// prototype
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
void goalCallback(const htbot::goal::ConstPtr& msg);
void calcQuat(double sx, double sy,double ex, double ey);
void publish_event(string s);
double checkangle_NLP(double gx, double gy, double px, double py,double prz, double prw);
double checkangle_NLP1(double gx, double gy, double px, double py,double ano);

double checkangle_NLP1(double gx, double gy, double px, double py,double ano)
{
	double an;
	//tf::Quaternion qp(0.0,0.0,prz,prw);
	//tyawp = tf::getYaw(qp);	

	if ( (gx > px) && (gy >= py)) {
		// Q1
		an = atan((gy-py)/(gx-px));
	} else {
		if ( (px > gx) && (gy >= py) ) {
			// Q2
			an = atan((gy-py)/(px-gx));
			an = PI - an;
		} else {
			if ( (px > gx) && (py >= gy) ) {
				// Q3
				an = atan((py-gy)/(px-gx));
				an = -(PI - an);
			} else {
				// Q4
				if (gx != px) {
					an = -(atan((py-gy)/(gx-px)));
				} else {
					if (gy > py) {
						an = PI / 2.0;
					} else {
						an = - PI / 2.0;
					}
				}
			}
		}
	}
	angle_to_goal = an; // find angle to goal
	an = angles::shortest_angular_distance(ano, angle_to_goal);
	return an;
}

double checkangle_NLP(double gx, double gy, double px, double py,double prz, double prw)
{
	double angle_to_goal,tyawp,an;
	tf::Quaternion qp(0.0,0.0,prz,prw);
	tyawp = tf::getYaw(qp);	

	if ( (gx > px) && (gy >= py)) {
		// Q1
		an = atan((gy-py)/(gx-px));
	} else {
		if ( (px > gx) && (gy >= py) ) {
			// Q2
			an = atan((gy-py)/(px-gx));
			an = PI - an;
		} else {
			if ( (px > gx) && (py >= gy) ) {
				// Q3
				an = atan((py-gy)/(px-gx));
				an = -(PI - an);
			} else {
				// Q4
				if (gx != px) {
					an = -(atan((py-gy)/(gx-px)));
				} else {
					if (gy > py) {
						an = PI / 2.0;
					} else {
						an = - PI / 2.0;
					}
				}
			}
		}
	}
	angle_to_goal = an; // find angle to goal
	an = angles::shortest_angular_distance(tyawp, angle_to_goal);
	return an;
}

void publish_event(string s)
{
	string estr;
  time_t t = time(NULL);
	char buf [200];
	string st;	
  struct tm *tm = localtime(&t);
	strftime(buf, sizeof(buf), "%c", tm);	
	st.assign(buf,strlen(buf));
	htbot::debug status;
	status.msg = st + " @ " + s;
	event_pub.publish(status);
	return;
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	px = msg->position.x;
	py = msg->position.y;
	pz = msg->position.z;
	prx = msg->orientation.x;
	pry = msg->orientation.y;
	prz = msg->orientation.z;
	prw = msg->orientation.w;

}

void calcQuat(double sx, double sy,double ex, double ey)
{
	double an;
	tf2::Quaternion quat;
	geometry_msgs::Quaternion qq;
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
	checkPlanRX = qq.x;
	checkPlanRY = qq.y;
	checkPlanRZ = qq.z;
	checkPlanRW = qq.w;
	return;
}

void checkplanCallback(const htbot::goal::ConstPtr& msg)
{
	ros::NodeHandle nm;
	nav_msgs::GetPlan srv;
	int size,idx;
	double x,y,x1,y1,dd,dsum,x0,y0,x2,y2;
	double sx,sy,ex,ey;
	
	//if ( ros::Time::now() < (last_cmd_time + ros::Duration(2.5)) ) {
	//	return;
	//}
	//ROS_INFO(" --- checkPlan Point : x=%.3f. y=%.3f -----------",msg->x,msg->y);
	srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = px;
  srv.request.start.pose.position.y = py;
	srv.request.start.pose.position.z = 0.0;
	srv.request.start.pose.orientation.x = 0.0;
	srv.request.start.pose.orientation.y = 0.0;
	srv.request.start.pose.orientation.z = prz;
  srv.request.start.pose.orientation.w = prw;
	  
	srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = msg->x;
  srv.request.goal.pose.position.y = msg->y;
	srv.request.goal.pose.position.z = 0.0;
	srv.request.goal.pose.orientation.x = 0.0;
	srv.request.goal.pose.orientation.y = 0.0;
	srv.request.goal.pose.orientation.z = prz;
	srv.request.goal.pose.orientation.w = prw;
	srv.request.tolerance = 1.0;

	//ROS_INFO(" --------- checkPlan Node B -----------");
	if (makeplan.call(srv)) {
		if (srv.response.plan.poses.empty()) {
  	 	ROS_INFO("----- checkPlan_Node : Got empty plan -----");
			nm.setParam("checkPlanSum",100.0);			
    } else {
			size = srv.response.plan.poses.size();
			ROS_INFO("------ checkPlan_Node : Found Plan : size=%d",size);
			x = px;
			y = py;	
			if (PathIndex >= size) {
				PathIndex = size -1;		
				if (PathIndex < 0) {
					PathIndex = 0;
				}		
			}
			x0 = srv.response.plan.poses[0].pose.position.x;
			y0 = srv.response.plan.poses[0].pose.position.y;
			x1 = srv.response.plan.poses[PathIndex].pose.position.x;
			y1 = srv.response.plan.poses[PathIndex].pose.position.y;
			dd = sqrt(((x0-x) * (x0-x)) + ((y0-y) * (y0-y)));
			if (dd > 0.3) {
				if ((PathIndex+5) >= size) {						
					x2 = srv.response.plan.poses[size-1].pose.position.x;
					y2 = srv.response.plan.poses[size-1].pose.position.y;
				} else {
					x2 = srv.response.plan.poses[PathIndex+5].pose.position.x;
					y2 = srv.response.plan.poses[PathIndex+5].pose.position.y;
				}
				//calcQuat(x1,y1,x2,y2); // orginal
				calcQuat(x,y,x2,y2);
				nm.setParam("checkPlanRX",checkPlanRX);
				nm.setParam("checkPlanRY",checkPlanRY);
				nm.setParam("checkPlanRZ",checkPlanRZ);
				nm.setParam("checkPlanRW",checkPlanRW);
			} 			
			//ROS_INFO("---- checkPlan_Node : x0=%.3f. y0=%.3f. id=%d :: x1=%.3f. y1=%.3f. size=%d. Dist=%.3f --------",x2,y2,PathIndex,x1,y1,size,dd);
			nm.setParam("checkPlanSum",200.0);
			nm.setParam("checkPlanDD",dd);
			nm.setParam("checkPlanX",x1);
			nm.setParam("checkPlanY",y1);
			
			//ROS_INFO("---- checkPlan_Node : rx=%.3f. ry=%.3f. rz=%.3f rw=%.3f. --------",checkPlanRX,checkPlanRY,checkPlanRZ,checkPlanRW);
		}
 	} else {
  	ROS_INFO("----- checkPlan_Node : Failed to call make plan service ------");
		nm.setParam("checkPlanSum",300.0);
  }
	//nm.setParam("checkPlanDone",true);
	//last_cmd_time = ros::Time::now();
}

void checkPathCallback(const htbot::path::ConstPtr& msg)
{
	ros::NodeHandle nm;
	nav_msgs::GetPlan srv;
	int size,idx;
	double x,y,x1,y1,dd,dsum,x0,y0,x2,y2,an,ano,dan,rz,rw;
	double sx,sy,ex,ey;
	char buf [200];
	string s;
	
	//if ( ros::Time::now() < (last_cmd_time + ros::Duration(2.5)) ) {
	//	return;
	//}
	ROS_INFO(" --- checkPath Point : x=%.3f. y=%.3f -----------",msg->px,msg->py);
	srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = msg->px;
  srv.request.start.pose.position.y = msg->py;
	srv.request.start.pose.position.z = msg->pz;
	srv.request.start.pose.orientation.x = msg->prx;
	srv.request.start.pose.orientation.y = msg->prx;
	srv.request.start.pose.orientation.z = msg->prz;
  srv.request.start.pose.orientation.w = msg->prw;
	  
	srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = msg->gx;
  srv.request.goal.pose.position.y = msg->gy;
	srv.request.goal.pose.position.z = msg->gz;
	srv.request.goal.pose.orientation.x = msg->grx;
	srv.request.goal.pose.orientation.y = msg->gry;
	srv.request.goal.pose.orientation.z = msg->grz;
	srv.request.goal.pose.orientation.w = msg->grw;
	srv.request.tolerance = msg->tol;
	ano = 0.0;
	an = 0.0;
	//ROS_INFO(" --------- checkPlan Node B -----------");
	if (makeplan.call(srv)) {
		if (srv.response.plan.poses.empty()) {
  	 	ROS_INFO("----- checkPath : Got empty plan -----");
			nm.setParam("checkPlanSum",100.0);			
    } else {
			size = srv.response.plan.poses.size();
			ROS_INFO("------ checkPath : Found Plan : size=%d",size);
			// print the plans. publish_event(string s);
			x = msg->px; //px;
			y = msg->py; //py;
			rz = msg->prz;
			rw = msg->prw;
			tf::Quaternion qp(0.0,0.0,rz,rw);
			ano = tf::getYaw(qp);	
			publish_event(" ******************************************************** \n");
			sprintf(buf,"----- checkPath : px=%.3f. py=%.3f. ano=%.3f. Size=%d ------\n",x,y,ano,size);
			s.assign(buf,strlen(buf));
			publish_event(s);
			for (int i=0;i<size;i++) {
				x1 = srv.response.plan.poses[i].pose.position.x;
				y1 = srv.response.plan.poses[i].pose.position.y;
				dd = sqrt(((x1-x) * (x1-x)) + ((y1-y) * (y1-y)));
				an = checkangle_NLP1(x1,y1,x,y,ano);
				dan = fabs(an);
				if (dan < 0.10) {
					sprintf(buf,"------- checkPath Close : i=%d. x1==%.3f. y1=%.3f. dd=%.3f. dan=%.3f. ano=%.3f --------",i,x1,y1,dd,dan,ano);
					s.assign(buf,strlen(buf));
					publish_event(s);
					continue;
				}
				sprintf(buf,"*********** checkPath Far : i=%d. x1==%.3f. y1=%.3f. dd=%.3f. dan=%.3f. ano=%.3f ***********",i,x1,y1,dd,dan,ano);
				s.assign(buf,strlen(buf));
				publish_event(s);
				//ROS_INFO("---- checkPath : i=%d. x1==%.3f. y1=%.3f. dd=%.3f. angle=%.3f --------",i,x1,y1,dd,an);
				x = x1;
				y = y1;			
				//rz = srv.response.plan.poses[i].pose.orientation.z;
				//rw = srv.response.plan.poses[i].pose.orientation.w;
				ano = angle_to_goal;
			}
			publish_event("\n ******************************************************** ");
		}
 	} else {
  	ROS_INFO("----- checkPath : Failed to call make plan service ------");
		nm.setParam("checkPlanSum",300.0);
  }
	//nm.setParam("checkPlanDone",true);
	//last_cmd_time = ros::Time::now();
}

void checkLPCallback(const htbot::path::ConstPtr& msg)
{
	ros::NodeHandle nm;
	nav_msgs::GetPlan srv;
	int size;
	double x,y,x1,y1,dd,dsum,lapseTime;

	srvStartTime = ros::Time::now(); 
	
	//ROS_INFO(" --------- checkPlan Node  A -----------");
	srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = msg->px;
  srv.request.start.pose.position.y = msg->py;
	srv.request.start.pose.position.z = msg->pz;
	srv.request.start.pose.orientation.x = msg->prx;
	srv.request.start.pose.orientation.y = msg->pry;
	srv.request.start.pose.orientation.z = msg->prz;
  srv.request.start.pose.orientation.w = msg->prw;
	  
	srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = msg->gx;
  srv.request.goal.pose.position.y = msg->gy;
	srv.request.goal.pose.position.z = msg->gz;
	srv.request.goal.pose.orientation.x = msg->grx;
	srv.request.goal.pose.orientation.y = msg->gry;
	srv.request.goal.pose.orientation.z = msg->grz;
	srv.request.goal.pose.orientation.w = msg->grw;
	srv.request.tolerance = msg->tol;

	//ROS_INFO(" --------- checkPlan Node B -----------");
	if (makeplan.call(srv)) {
		if (srv.response.plan.poses.empty()) {
  	 	ROS_INFO("----- checkLP_Node : Got empty plan -----");
			nm.setParam("checkLPSum",100.0);			
    } else {
			size = srv.response.plan.poses.size();
			//ROS_INFO("------ checkPlan_Node : Found Plan : size=%d",size);
			ROS_INFO("---- checkLP_Node : size=%d. --------",size);
			nm.setParam("checkPlanSum",200.0);
		}
 	} else {
  	ROS_INFO("----- checkLP_Node : Failed to call make plan service ------");
		nm.setParam("checkLPSum",300.0);
  }
	srvTime = ros::Time::now() - srvStartTime;
	lapseTime = srvTime.toSec();
	ROS_INFO(" ====== checkLP : checkPath Time =%.2f =========",lapseTime);
	
}

void checkplanLoop()
{
	ros::NodeHandle nm;
	nav_msgs::GetPlan srv;
	int size;
	double x,y,x1,y1,dd,dsum;
	
	srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = px;
  srv.request.start.pose.position.y = py;
	srv.request.start.pose.position.z = 0.0;
	srv.request.start.pose.orientation.x = 0.0;
	srv.request.start.pose.orientation.y = 0.0;
	srv.request.start.pose.orientation.z = prz;
  srv.request.start.pose.orientation.w = prw;
	  
	srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = gx;
  srv.request.goal.pose.position.y = gy;
	srv.request.goal.pose.position.z = 0.0;
	srv.request.goal.pose.orientation.x = 0.0;
	srv.request.goal.pose.orientation.y = 0.0;
	srv.request.goal.pose.orientation.z = grz;
	srv.request.goal.pose.orientation.w = grw;
	srv.request.tolerance = 1.0;

	//ROS_INFO(" --------- checkPlan Node B -----------");
	if (makeplan.call(srv)) {
		if (srv.response.plan.poses.empty()) {
  	 	ROS_INFO("----- checkPlan_Node : Got empty plan -----");
			nm.setParam("checkPlanSum",100.0);			
    } else {
			size = srv.response.plan.poses.size();
			//ROS_INFO("------ checkPlan_Node : Found Plan : size=%d",size);
			dsum = 0.0;
			x = px;
			y = py;
			//for (int i=0;i<size;i++) {
			//	x1 = srv.response.plan.poses[i].pose.position.x;
			//	y1 = srv.response.plan.poses[i].pose.position.y;
			//	dd = sqrt(((x1-x) * (x1-x)) + ((y1-y) * (y1-y)));
			//	dsum = dsum + dd;
			//	//ROS_INFO("---- checkPlan_Node : i=%d. x1==%.3f. y1=%.3f. dd=%.3f. dsum=%.3f --------",i,x1,y1,dd,dsum);
			//	x = x1;
			//	y = y1;				
			//}
			nm.setParam("checkPlanSum",200.0);
			ROS_INFO("---- checkPlan_Node : size=%d. --------",size);
		}
 	} else {
  	ROS_INFO("----- checkPlan_Node : Failed to call make plan service ------");
		nm.setParam("checkPlanSum",300.0);
  }
	//nm.setParam("checkPlanDone",true);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Record Path Node");
  ros::NodeHandle n;
	ros::Rate loop_rate(1.0);

	chk_cmd = n.subscribe<htbot::goal>("checkplan", 1,checkplanCallback);
	chkpath_cmd = n.subscribe<htbot::path>("checkpath", 1,checkPathCallback);
	
	pose_sub = n.subscribe<geometry_msgs::Pose>("/robot_pose", 1,poseCallback);
	event_pub = n.advertise<htbot::debug>("event",100);
	last_cmd_time = ros::Time::now();
	
  ROS_INFO("------- Ready to Record Path. ---------");
  ros::spin();
	
  return 0;
}



