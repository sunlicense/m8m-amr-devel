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
ros::Publisher racplan_pub;
ros::Subscriber racplan_sub;

double px,py,pz,prx,pry,prz,prw;
double gx,gy,grz,grw;
int navstatus,PathIndex;
ros::Time last_cmd_time;
ros::Duration srvTime;
ros::Time srvStartTime;
double checkPlanRX,checkPlanRY,checkPlanRZ,checkPlanRW;
double angle_to_goal;
double pathX[500],pathY[500];
std::string racpathDir,racpath;

// prototype
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
void goalCallback(const htbot::goal::ConstPtr& msg);
void calcQuat(double sx, double sy,double ex, double ey);
void publish_event(string s);
double checkangle_NLP(double gx, double gy, double px, double py,double prz, double prw);
double checkangle_NLP1(double gx, double gy, double px, double py,double ano);
double checkangle(double gx, double gy, double px, double py);
void racplanCallback(const nav_msgs::Path::ConstPtr& msg);

double checkangle(double gx, double gy, double px, double py)
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
	return an;
}

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
	htbot::debug status;
	status.msg = s;
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

void racplanCallback(const nav_msgs::Path::ConstPtr& msg)
{
	nav_msgs::Path racgp;
	
	ROS_INFO("--- checkPlan Node : Received RAC Global Plan ---");
	publish_event("--- checkPlan Node : Recieved RAC Global Plan ---");
	racgp.header.frame_id = msg->header.frame_id;
	racgp.header.stamp = ros::Time::now();;
	racgp.poses.resize(msg->poses.size());
	for (int i = 0; i < msg->poses.size(); i++) {
  	racgp.poses[i] = msg->poses[i];
  }
  racplan_pub.publish(racgp);
	
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
	bool normalPath;
	FILE *fp;
	std::string fn;
	double xx,yy,zz,rrx,rry,rrz,rrw,st;
	
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
			nm.getParam("normalPath",normalPath);
			if (normalPath) {
				fn = racpathDir + racpath;
				ROS_INFO("----- CheckPlan : Save Plan FN = %s. --------",fn.c_str());
				fp = fopen(fn.c_str(), "w");
				for (int i=0;i<size;i++) {
					xx = srv.response.plan.poses[i].pose.position.x;
					yy = srv.response.plan.poses[i].pose.position.y;
					zz = srv.response.plan.poses[i].pose.position.z;
					rrx = srv.response.plan.poses[i].pose.orientation.x;
					rry = srv.response.plan.poses[i].pose.orientation.y;
					rrz = srv.response.plan.poses[i].pose.orientation.z;
					rrw = srv.response.plan.poses[i].pose.orientation.w;
					st = 0.0;
					fprintf(fp,"%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",xx,yy,zz,rrx,rry,rrz,rrw,st);
				}
				fclose(fp);
			}
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
	int j;
	bool close;
	
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
			j=0;
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
			x1 = srv.response.plan.poses[0].pose.position.x;
			y1 = srv.response.plan.poses[0].pose.position.y;
			dd = sqrt(((x1-x) * (x1-x)) + ((y1-y) * (y1-y)));
			ano = checkangle(x1,y1,x,y);
			sprintf(buf,"------- checkPath 1st LP : x1==%.3f. y1=%.3f. dd=%.3f. ano=%.3f --------",x1,y1,dd,ano);
			s.assign(buf,strlen(buf));
			publish_event(s);
			x2 = x1;
			y2 = y1;
			close = false;
			for (int i=1;i<size;i++) {
				x1 = srv.response.plan.poses[i].pose.position.x;
				y1 = srv.response.plan.poses[i].pose.position.y;
				dd = sqrt(((x1-x) * (x1-x)) + ((y1-y) * (y1-y)));
				an = checkangle(x1,y1,x,y);
				dan = fabs(an-ano);
				ROS_INFO("------- checkPath : i=%d. x1==%.3f. y1=%.3f. dd=%.3f. dan=%.3f. an=%.3f --------",i,x1,y1,dd,dan,an);
				if ((dan < 0.10) && (dd < 1.75)) {
					sprintf(buf,"------- checkPath Close : i=%d. x1==%.3f. y1=%.3f. dd=%.3f. dan=%.3f. an=%.3f --------",i,x1,y1,dd,dan,an);
					s.assign(buf,strlen(buf));
					publish_event(s);
					x2 = x1;
					y2 = y1;
					//close = true;
					continue;
				}				
				//ROS_INFO("---- checkPath : i=%d. x1==%.3f. y1=%.3f. dd=%.3f. angle=%.3f --------",i,x1,y1,dd,an);
				x = x2;
				y = y2;	
				x2 = x1;
				y2 = y1;
				ano = checkangle(x1,y1,x,y);;
				sprintf(buf,"*********** checkPath Far : i=%d. x=%.3f. y=%.3f. dd=%.3f. ano=%.3f. ***********",i-1,x,y,dd,ano);
				s.assign(buf,strlen(buf));
				publish_event(s);
				pathX[j] = x;
				pathY[j++] = y;
			}
			publish_event("\n ********************* Loop 2 Start *********************************** ");
			x = msg->px; //px;
			y = msg->py; //py;
			x1 = pathX[0];
			y1 = pathY[0];
			dd = sqrt(((x1-x) * (x1-x)) + ((y1-y) * (y1-y)));
			ano = checkangle(x1,y1,x,y);
			sprintf(buf,"------- checkPath2 1st LP : x1==%.3f. y1=%.3f. dd=%.3f. ano=%.3f --------",x1,y1,dd,ano);
			s.assign(buf,strlen(buf));
			publish_event(s);
			x2 = x1;
			y2 = y1;
			for (int i=1;i<j;i++) {
				x1 = pathX[i];
				y1 = pathY[i];
				dd = sqrt(((x1-x) * (x1-x)) + ((y1-y) * (y1-y)));
				an = checkangle(x1,y1,x,y);
				dan = fabs(an-ano);
				ROS_INFO("------- checkPath2 : i=%d. x1==%.3f. y1=%.3f. dd=%.3f. dan=%.3f. an=%.3f --------",i,x1,y1,dd,dan,an);
				if ((dan < 0.10) && (dd < 1.5)) {
					sprintf(buf,"------- checkPath2 Close : i=%d. x1==%.3f. y1=%.3f. dd=%.3f. dan=%.3f. ano=%.3f --------",i,x1,y1,dd,dan,ano);
					s.assign(buf,strlen(buf));
					publish_event(s);
					x2 = x1;
					y2 = y1;
					continue;
				}	
				x = x2;
				y = y2;			
				x2 = x1;
				y2 = y1;
				ano = checkangle(x1,y1,x,y);;
				sprintf(buf,"*********** checkPath2 Far : i=%d. x=%.3f. y=%.3f. dd=%.3f. ano=%.3f. ***********",i-1,x,y,dd,ano);
				s.assign(buf,strlen(buf));
				publish_event(s);
			}
			publish_event("\n ********************* Loop 2 End********************************** ");
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

void moveCallback(const htbot::move::ConstPtr& msg)
{

	if (msg->opt == 0) {
		gx = msg->x;
		gy = msg->y;
		grz = msg->rz;
		grw = msg->rw;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "check Plan Server");
  ros::NodeHandle n;
	ros::Rate loop_rate(1.0);

	chk_cmd = n.subscribe<htbot::goal>("checkplan", 1,checkplanCallback);
	chkpath_cmd = n.subscribe<htbot::path>("checkpath", 1,checkPathCallback);
	//makeplan = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
	makeplan = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
	pose_sub = n.subscribe<geometry_msgs::Pose>("/robot_pose", 1,poseCallback);
	move_cmd = n.subscribe<htbot::move>("move", 10,moveCallback); 
	event_pub = n.advertise<htbot::debug>("event",500);
	racplan_pub = n.advertise<nav_msgs::Path>("racgplan",10);
	racplan_sub = n.subscribe<nav_msgs::Path>("/racplan", 1,racplanCallback);
	last_cmd_time = ros::Time::now();
	n.param("PathIndex",PathIndex,5);
	n.getParam("racpathDir",racpathDir);
	n.getParam("racpath",racpath);
  ROS_INFO("------- Ready to check Plan.  ---------");
	ROS_INFO("------- checkPlan : racpathDir=%s. racpath=%s   ---------",racpathDir.c_str(),racpath.c_str());
  ros::spin();
	/*
	n.setParam("checkPlanSum",200.0);	
	navstatus = 0;
	while (true) { 
		n.getParam("navstatus",navstatus);
		if (navstatus == 7) {
			//checkplanLoop();
		}
		ros::spinOnce();	
  	loop_rate.sleep();
	}
	*/
  return 0;
}



