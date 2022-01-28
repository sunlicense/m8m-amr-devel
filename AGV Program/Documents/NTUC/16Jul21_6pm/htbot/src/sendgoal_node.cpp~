/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Modified : 3.12.2014
*		Changes :
*/

//#define ODROID
#define SSMC
//#define RAC

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "htbot/move.h"
#include "htbot/move_status.h"
#include "htbot/status.h"
#include "boost/algorithm/string.hpp"
#include <math.h>
//#include "htbot/dbcmd.h"
#include <sensor_msgs/LaserScan.h>
#include "htbot/clear.h"
#include "htbot/debug.h"

using namespace std;

#define LSIZE 918
#define	CIDX 460
#define RIDX 310 //375  //280
#define	LIDX 610 //545  //640
#define IDXGAP 150 
#define	MAXDIST	1.8  // metre
#define	MAXMAX	30.0  // metre
#define	DEGIDX	0.25
#define BOXWIDTH 0.6
#define BOXHALF 0.3
#define STEP 0.05   //0.15
#define STNSIZE 0.20 //0.18   // diameter of stn 0.18
#define DIFF 0.08
#define DIFFRAD 0.025
#define DEGPERIDX 0.235
#define DIA 0.2
#define PI 3.14159
#define RADIUS 0.09

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Subscriber move_cmd;
ros::Publisher status_pub;
ros::Publisher debug_pub;
ros::Publisher status_mov;
ros::Publisher vel_pub;
ros::Publisher cmdvel_pub;
ros::Publisher pos_pub;
//htbot::dbcmd db_srv;
//ros::ServiceClient db_client;
ros::Subscriber laser_sub;
ros::Subscriber movx_sub;
ros::Publisher clear_pub;

double rz,rw,angle;
bool laser_use;
int laser_size,cidx,lidx,ridx;
int xcount;
double lvalue[LSIZE+1];
int stop_move;
double fmove,ffmove,mdist,offset,toffset,speed;
double degperidx,stnsize;
bool bootup;
double align_method;

// prototype
void publish_status(string s);
void publish_debug(string s);
void publish_move_status(int stat);
void publish_clear(void);
void calAngle();
//void alignMoveToContact();
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void changeCallBack(const htbot::move_status::ConstPtr& msg);
void move_to_charge(double x, double an, double speed);
bool checksize(double dd,int wx);

void publish_status(string s)
{
	htbot::status status;
	status.msg = s;
	status_pub.publish(status);
	return;
}

void publish_debug(string s)
{
	htbot::debug status;
	status.msg = s;
	debug_pub.publish(status);
	return;
}

void publish_clear(void)
{
	htbot::clear cmd;
	cmd.cmd = 1;
	publish_debug("sendgoal : publish clearmap");
	//ROS_INFO("publise clear Map");
	clear_pub.publish(cmd);
	return;
}

void calAngle() {
	double dc,ds;

	dc = 2.0 * acos(rw);
	ds = 2.0 * asin(rz);
	if (ds >= 0.0) {
		angle = 360.0 - dc;
	} else {
		angle = dc;
	}	
}


void publish_move_status(int stat)
{
	htbot::move_status status;
	status.stat = stat;
	status_mov.publish(status);
	return;
}

//bool alignMoveToContact() {
///
//}

void changeCallBack(const htbot::move_status::ConstPtr& msg)
{
	publish_debug("Change Movement Triggered");	
	if (msg->stat == 7) {
		stop_move = true;
		publish_debug("Stop Moving to Charging Point");
	} else {
		stop_move = false;
	}
}

void move_to_charge(double x, double an, double speed) {
	geometry_msgs::Twist pos;
	char buf[100];
	string s1;

	pos.angular.z = an;
	pos.linear.x = x - toffset;
	pos.linear.y = speed;
	pos.linear.z = 0.0;
	pos_pub.publish(pos);	
	sprintf(buf,"Move to Charge : Linear : %0.6f. Angle : %0.6f. Speed : %0.6f.",pos.linear.x,pos.angular.z,pos.linear.y);
	s1.assign(buf,strlen(buf));
	publish_debug(s1);
}


bool move_test(const htbot::move::ConstPtr& msg)
{
	geometry_msgs::Twist pos;
	ros::NodeHandle nn;
	int reached;
	if (msg->opt == 1) {
		//pos.angular.z = 90.0;
		//pos.linear.x = 0.0;
		//pos_pub.publish(pos);
		publish_debug("sendgoal : move_test activated");
		mdist = fmove;
		reached = 0;
		nn.setParam("ReachPosition",reached);
		laser_use = true;
		publish_move_status(1); // movement completed and ok
	}
	if (msg->opt == 2) {
		//pos.angular.z = 90.0;
		//pos.linear.x = 0.0;
		//pos_pub.publish(pos);
		publish_debug("sendgoal : move_test activated");
		mdist = ffmove;
		reached = 0;
		nn.setParam("ReachPosition",reached);
		laser_use = true;
		publish_move_status(1); // movement completed and ok
	}
	return true;
}

bool checksize(double dc,double de, int wx) {
	double angle,size;
	char buf[100];
	string s1;
	ros::NodeHandle n;
	
	// check size of detected cyclinder if matches docking stn size of about 0.2m
	// wx is the angle in index
	// check if depth equal radius of station
	/*
	sprintf(buf,"CheckSize : dist edge : %.5f. Dist Center : %.5f. Index Width : %d",de,dc,wx);
	s1.assign(buf,strlen(buf));
	if (((de - dc)-RADIUS) > DIFFRAD) {
		publish_debug("Depth Wrong");
		return false;
	}
	*/

	angle = ((wx * degperidx) * PI)/180.0;  // radian
	size = dc * angle;
	if (bootup) {
		n.setParam("stationsize",size);
		stnsize = size;
		//bootup = false;
		sprintf(buf,"Bootup : Size of Stn : %.5f",stnsize);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		//publish_move_status(1); // movement completed and ok
	} else {
		sprintf(buf,"CheckSize : dist : %.5f. Angle : %.5f. Size of Stn : %.5f",dc,angle,size);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
	}
	if (bootup) {
		//publish_move_status(1); // movement completed and ok
		//bootup = false;
		return true;
	}
	if (stnsize == 0.0) {
		stnsize = STNSIZE;
	}
	if (stnsize > size) {
		if ((stnsize - size) < DIFF) {
			publish_debug("Match Size of Stn");
			return true;
		} else {
			publish_debug("Size of Stn Do no Match");
			return false;
		}
	} else {
		if ((size - stnsize) < DIFF) {
			publish_debug("Match Size of Stn");
			return true;
		} else {
			publish_debug("Size of Stn Do no Match");
			return false;
		}
	}
	
	/*
	if (fabs(size - STNSIZE) < DIFF) {
		publish_debug("Match Size of Stn");
		return true;
	} else {
		publish_debug("Size of Stn Do no Match");
		return false;
	}
	*/
}

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {		
	double lr,ll,ln,lt,xp,xm,lx,x2,lc;
	double qr,ql,dr,dl,dt,qt,qb,rr,rl;		
	int il,ir,ic;
	bool redge,ledge,fpoint;
	char buf[100];
	string s1;

	int rd;
	int sx,ex,mx,cn;
	double ax,bx;

	if (laser_use) {		
		laser_use = false;
		bootup = false;
		publish_move_status(1);
		ROS_INFO("laser SG");
		return;
		ln = 2.0;	
		redge = false;
		ledge = false;
		fpoint = false;
		laser_size = lscan->ranges.size();
		cidx = (int)(laser_size / 2.0);
		lidx = cidx + IDXGAP;
		ridx = cidx - IDXGAP;
		//ROS_INFO("Laser Size : %d. Count : %d",laser_size,xcount);	
		sprintf(buf,"Laser Size = %d. LIDX : %d. CIDX : %d. RIDX : %d ",laser_size,lidx,cidx,ridx);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		
		// filter data
		rd = 0;
		for (int i=ridx;i<lidx;i++) {
			if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
				// valid data
				x2 = lscan->ranges[i];
				//ROS_INFO("Laser Scan : i=%d.  lx=%.5f",i,x2);
				if (x2 > MAXMAX) {
					x2 = MAXMAX;
				} else {
					if (x2 < 0) {
						x2 = MAXMAX;
					} 
				}
				lvalue[i] = x2;
			} else {
				lvalue[i] = MAXMAX;
			}
			rd++;
		} 
		sprintf(buf,"No of Laser point  = %d ",rd);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		// filter 
		for (int i=ridx;i<lidx;i++) {
			x2 = lvalue[i];
			if (x2 == MAXMAX) {
				if ((i>RIDX) && (i<(LIDX-1))) {
					x2 = ((lvalue[i-1]+lvalue[i+1])/2.0);
				} else if (i>RIDX) {
					x2 = ((lvalue[i-2]+lvalue[i-1])/2.0);
				} else {
					x2 = ((lvalue[i+1]+lvalue[i+2])/2.0);
				}
			}
			lvalue[i] = x2;
		} 
		// detect edges
		sx = 0;
		ex = 0;
		mx = 0;
		for (int i=ridx;i<lidx;i++) {
			x2 = lvalue[i];
			if (x2 >= MAXDIST) {
         x2 = MAXDIST;
      }
			if (i == ridx) {
      	bx = x2;
        continue;
      }
			ax = x2;
			if (ax < bx) {
      	if ((bx-ax) > STEP) {
         	// start edge of object
         	sx = i;
					sprintf(buf,"Start Edge  = %d ",sx);
					s1.assign(buf,strlen(buf));
					publish_debug(s1);
       	}
      } else {
         if ( ((ax-bx) > STEP) && (sx > ridx) ) {
					// end edge
					ex = i;
					sprintf(buf,"End Edge  = %d ",ex);
					s1.assign(buf,strlen(buf));
					publish_debug(s1);
					mx = (int)((sx + ex) / 2);
         	// check if size close to docking stn
					x2 = lvalue[mx];
					dl = lvalue[ex];
					dr = lvalue[sx];
					dr = (dr + dl)/2.0;
					sprintf(buf,"Mid Point  = %d. Dist = %.5f. Edge Dist : %.5f ",mx,x2,dr);
					s1.assign(buf,strlen(buf));
					publish_debug(s1);
					if (checksize(x2,dr,(ex-sx))) {
						// match 
						break;
					} else {
						// start over
						publish_debug("Start Over");
						ex = 0;
						sx = 0;
						mx = 0;
					}
         }
      }
			bx = ax;
		}
		if (bootup) {
			laser_use = false;
			bootup = false;
			publish_move_status(1);
			return;
		}
		if (mx > 0) {
			// found docking stn
			lc = lvalue[mx];
			//ROS_INFO("Index of Left Edge : %d. Right Edge : %d. Center : %d. DC : %.5f",ex,sx,mx,lc);
			sprintf(buf,"Laser : Index of Left Edge : %d. Right Edge : %d. Center : %d. DC : %.5f",ex,sx,mx,lc);
			s1.assign(buf,strlen(buf));
			publish_debug(s1);
			// calculate angle to turn
			if (mx > cidx) {
				// turn anti-clockwise - left
				qt = (mx-cidx) * degperidx;
				move_to_charge(lc*mdist,qt,speed);
				//ROS_INFO("Turn Left : %.5f degree.",qt);
				publish_debug("Turn Left ");
			} else {
				// turn clockwise - right
				qt = (cidx - mx) * degperidx;
				move_to_charge(lc*mdist,-qt,speed);
				//ROS_INFO("Turn Right : %.5f degree.",qt);
				publish_debug("Turn Right ");
			}
		} else {
			publish_debug("No Docking Stn Detected"); 
			//toffset = 0.0;
			//move_to_charge(0.0,0.0,0.0);
			publish_move_status(7); // abort docking
		}
		laser_use = false;
	}
}

void laserCallBack_org(const sensor_msgs::LaserScan::ConstPtr& lscan) {		
	double lr,ll,ln,lt,xp,xm,lx,x2,lc;
	double qr,ql,dr,dl,dt,qt,qb,rr,rl;		
	int il,ir,ic;
	bool redge,ledge,fpoint;
	char buf[100];
	string s1;

	if (laser_use) {		
		ln = 2.0;	
		redge = false;
		ledge = false;
		fpoint = false;
		laser_size = lscan->ranges.size();
		//ROS_INFO("Laser Size : %d. Count : %d",laser_size,xcount);	
		sprintf(buf,"Laser : Laser Size = %d ",laser_size);
		s1.assign(buf,strlen(buf));
		publish_debug(s1);
		
		// filter data
		for (int i=RIDX;i<LIDX;i++) {
			if(!std::isnan(lscan->ranges[i]) && !std::isinf(lscan->ranges[i])){
				// valid data
				x2 = lscan->ranges[i];
				//ROS_INFO("Laser Scan : i=%d.  lx=%.5f",i,x2);
				if (x2 > MAXDIST) {
					x2 = MAXDIST;
				} else {
					if (x2 < 0) {
						x2 = MAXDIST;
					} 
				}
				lvalue[i] = x2;
			} else {
				lvalue[i] = MAXDIST;
			}
		} 
		// detect edges
		lt = lvalue[RIDX];
		//ROS_INFO("First Scan : i=%d.  lt=%.5f",RIDX,lt);
		for (int i=RIDX+1;i<LIDX;i++) {
			if(!std::isnan(lvalue[i]) && !std::isinf(lvalue[i])){
				lx = lvalue[i];
				//ROS_INFO("Detecting Edge Scan : i=%d.  lx=%.5f",i,lx);
				if (lx <= MAXDIST) {										
					if (!redge) {
						//ROS_INFO("Looking for right edge. i=%d. lx=%.5f. lt=%.5f",i,lx,lt);
						if ((lt - lx) > STEP) {
							lr = lx;
							ir = i;				
							//ROS_INFO("Index of Right Edge : %d. lr=%.5f",ir,lr);	
							sprintf(buf,"Laser : Index of Right Edge : %d. lr=%.5f",ir,lr);
							s1.assign(buf,strlen(buf));
							publish_debug(s1);		
							redge = true;
							lt = lx;
							continue;
						} 						
					}
					if ((redge) && (!ledge)) {
						//ROS_INFO("Checking for left edge. i=%d. lx=%.5f. lt=%.5f",i,lx,lt);
						if ((lx - lt) > STEP) {
							//ROS_INFO("Checking for left edgeA. i=%d. lx=%.5f. lt=%.5f",i,lx,lt);
							ll = lx;
							il = i;
							//ROS_INFO("Index of Left Edge : %d. ll=%.5f",il,ll);
							sprintf(buf,"Laser : Index of Left Edge : %d. ll=%.5f",il,ll);
							s1.assign(buf,strlen(buf));
							publish_debug(s1);
							ledge = true;
							break;
						} 
					}		
					lt = lx;								
				} 
			} else {
				ROS_INFO("Error Data");
			} 
		}	

		if (redge && ledge) {
			ic = (int)((il + ir-1)/2.0);
			lc = lvalue[ic];
			//ROS_INFO("Index of Left Edge : %d. Right Edge : %d. Center : %d. DC : %.5f",il,ir,ic,lc);
			sprintf(buf,"Laser : Index of Left Edge : %d. Right Edge : %d. Center : %d. DC : %.5f",il,ir,ic,lc);
			s1.assign(buf,strlen(buf));
			publish_debug(s1);
			// calculate angle to turn
			if (ic > CIDX) {
				// turn anti-clockwise - left
				qt = (ic-CIDX) * degperidx;
				move_to_charge(lc*mdist,qt,speed);
				//ROS_INFO("Turn Left : %.5f degree.",qt);
				publish_debug("Turn Left ");
			} else {
				// turn clockwise - right
				qt = (CIDX - ic) * degperidx;
				move_to_charge(lc*mdist,-qt,speed);
				//ROS_INFO("Turn Right : %.5f degree.",qt);
				publish_debug("Turn Right ");
			}
				
		}
		//publish_move_status(1);move(const htbot::move::ConstPtr& msg)
		laser_use = false;
	}
}

bool move(const htbot::move::ConstPtr& msg)
{
	ros::NodeHandle nm;
	geometry_msgs::Twist vel;
	geometry_msgs::Twist pos;
  MoveBaseClient ac("move_base", true); // move_base_nys
	move_base_msgs::MoveBaseGoal goal;
  char buf [100];
	string s;
	int cnt;
	ros::Time start_time;
	ros::Time start_delay_time;
	double goal_time;
	bool delay;
	int reached;

	//ac("move_base", true);
	if (msg->opt == 1) {
		// move straight to charger contact until close	
		ROS_INFO(" SendGoal : Move to Docking Station");
		pos.linear.z = 1.0;
		pos_pub.publish(pos);
		return true;
	}
	if (msg->opt == 2) {
		publish_debug("sendgoal : Move to Docking Station activated Stage 2");
		mdist = ffmove;
		speed = 500.0;
		toffset = offset;
		reached = 0;
		nm.setParam("ReachPosition",reached);
		laser_use = true;
		//sleep(7);
		//publish_move_status(1); // movement completed and ok	
		return true;
	}

	if (msg->opt == 4) {
		publish_debug("sendgoal : Move away from Docking Station 0.3m");
		pos.angular.z = 0.0;// 0.0
		pos.linear.x = 0.0; // -0.3
		pos.linear.z = 2.0;
		pos.linear.y = 0.0;  // standard speed
		pos_pub.publish(pos);
		//sleep(3);
		//publish_move_status(1); // movement completed and ok
		return true;
	}

	if (msg->opt == 5) {
		bootup = true;
		laser_use = true;		
		ROS_INFO("SG 5");		
		return true;
	}
	if (msg->opt == 6) {
		pos.angular.z = 0.0;
		pos.linear.x = 0.0;
		pos.linear.z = 8.0;  // 7.0
		pos.linear.y = 100.0;  // standard speed
		pos_pub.publish(pos);
		return true;
	}
	if (msg->opt == 7) {
		pos.linear.z = align_method; //10.0
		pos_pub.publish(pos);
		return true;
	}
	if (msg->opt == 8) {
		pos.linear.z = 1.0;  //
		pos_pub.publish(pos);
		return true;
	}
	// pre move
	if (msg->opt == 9) {
		pos.linear.z = 20.0;  //
		pos.linear.x = msg->pd;
		pos.angular.z = msg->pa;
		pos_pub.publish(pos);
		return true;
	}
	// post move
	if (msg->opt == 10) {
		pos.linear.z = 20.0;  //
		pos.linear.x = msg->pd;
		pos.angular.z = msg->pa;
		pos_pub.publish(pos);
		return true;
	}
	/*
	start_delay_time = ros::Time::now();
	publish_clear();	
	while (true) {
		if ( ros::Time::now() > (start_delay_time + ros::Duration(3.0)) ) {
			break;
		}
	}
	*/
	
	start_time = ros::Time::now();
	delay = false;
	//nm.setParam("/startmove",1);
	nm.setParam("navstatus",7);  //navigation started.
	nm.setParam("lookforObs",1); // look for obstacle in front
	//ROS_INFO("sendgoal_node started");
	cnt = 0;  // counter for number of retry after abort
	while (true) {
		
		if (delay) {
			if ( ros::Time::now() < (start_delay_time + ros::Duration(5.0)) ) {
					//publish_status("sendgoal : delay");
					continue;
			} 
		}
		// testing
		//ROS_INFO("Sendgoal : Goal Reached");
		//publish_debug("Sendgoal : Reached Goal");
		//nm.setParam("moveflag",1);
		//publish_move_status(1); // movement completed and ok
		//nm.setParam("moveok",1);
		//break;
		//publish_debug("Sendgoal : sended Goal");
		goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();
  	goal.target_pose.pose.position.x = msg->x;
  	goal.target_pose.pose.position.y = msg->y;
  	goal.target_pose.pose.position.z = msg->z;
  	goal.target_pose.pose.orientation.x = msg->rx;
  	goal.target_pose.pose.orientation.y = msg->ry;
  	goal.target_pose.pose.orientation.z = msg->rz;
  	goal.target_pose.pose.orientation.w = msg->rw;  

		while(!ac.waitForServer(ros::Duration(5.0))){
    	ROS_INFO("Waiting for the move_base_nys action server to come up");
  	} 

		//ROS_INFO("Goal Server : Sending Goal > X : %lf. Y : %lf",msg->x,msg->y);

  	ac.sendGoal(goal);
		//ac.waitForResult();
		//publish_status("Goal Server : Sent Goal");
		//ROS_INFO("Goal Server : Sending Goal > X : %lf. Y : %lf",msg->x,msg->y);

		if (msg->opt == 3) {
			goal_time = 4.0;
		} else {
			goal_time = 0.0;
		}
		//goal_time = 2.0;
  	if (ac.waitForResult(ros::Duration(goal_time))) {
			// goal completed
			//publish_debug("sendgoal : Goal Completed.");	
			//ROS_INFO("Sendgoal : Goal Completed");
		} else {
			// goals not completed
			//ROS_INFO("Sendgoal : Goal Not Completed");
			publish_debug("sendgoal : Goal Not Completed.");
			nm.getParam("/stop_move",stop_move);
			if (stop_move > 0) {
				// stop and move on
				publish_debug("sendgoal : Goal not completed. Cancel Goal");
				nm.setParam("moveflag",5);
				publish_move_status(1); // movement completed and ok
				nm.setParam("moveok",1);
				stop_move = 0;
				nm.setParam("/stop_move",0);
				ac.cancelGoal();
				break;
			} else {
				//continue with goal
				publish_debug("sendgoal : Goal not completed..Continue");
				continue;
			}
		}
		//sleep(5);
		//publish_status("WaitForResult Back");
		//ROS_INFO("Result is Back");
	
  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  	{
    	//ROS_INFO("Sendgoal : Goal Reached");
			publish_debug("Sendgoal : Reached Goal");
			//nm.setParam("moveflag",1);
			publish_move_status(1); // movement completed and ok
			//nm.setParam("moveok",1);
			break;
		}
  	else 
  	{
			if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
  		{
    		nm.setParam("moveflag",1);
				publish_move_status(2); // movement aborted
				nm.setParam("moveok",1);
				publish_debug("Sendgoal : Goal Aborted");
				break;
				/*
				if (cnt >= 3) {
					nm.setParam("moveflag",1);
					publish_move_status(2); // movement aborted
					nm.setParam("moveok",1);
					publish_debug("Sendgoal : Goal Aborted 3 times");
					break;
				} else {
					publish_debug("Sendgoal : Goal Aborted. Try again");
				}
				*/
				//if (!delay) {
				//	publish_clear();
				//	start_delay_time = ros::Time::now();
				//	delay = true;
				//	publish_debug("Sendgoal : Goal Aborted. Clear Map. Try again");
				//} else {
				//	nm.setParam("moveflag",1);
				//	publish_move_status(2); // movement aborted
				//	nm.setParam("moveok",1);
				//	break;
				//}
				
				//publish_debug("Sendgoal : Goal Aborted");
				//if ( ros::Time::now() > (start_time + ros::Duration(30.0)) ) {
				//	nm.setParam("moveflag",1);
				//	publish_move_status(2); // movement aborted
				//	nm.setParam("moveok",1);
				//	publish_debug("Sendgoal : Goal Aborted");
				//	break;
				//}
				//publish_debug("Sendgoal : Goal Aborted. Clear Map. Try again");
				//ROS_INFO("Sendgoal : Goal Aborted. Clear Map. Try again");
			}
			else
			{
				if(ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    			//ROS_INFO("Sendgoal : Something Wrong");			
					publish_debug("Preempted");
					nm.setParam("moveflag",1);
					publish_move_status(5); // movement cancel
					//if ( ros::Time::now() > (start_time + ros::Duration(120.0)) ) {
					//	nm.setParam("moveflag",1);
					//	publish_move_status(3); // movement aborted
					//	nm.setParam("moveok",1);
					//	publish_debug("Something Very Wrong");
					//	break;
					//}
					//publish_debug("Error in Sending Goal. Try again");
					break;
				} 
			}
		}	
	}
	//nm.setParam("navstatus",0);  //navigation started.
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_to_goal_server");
  ros::NodeHandle n;

	laser_use = false; 
	bootup = false;
	stnsize = 0.0;
	stop_move = 0;
	xcount = 0;
	move_cmd = n.subscribe<htbot::move>("move", 1000, move);  // move_test to test. move is actual
  status_pub = n.advertise<htbot::status>("feedback",100);
	debug_pub = n.advertise<htbot::debug>("debug",100);
	status_mov = n.advertise<htbot::move_status>("move_status",100);
	vel_pub = n.advertise<geometry_msgs::Twist>("joycmd_vel", 1);
	cmdvel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	pos_pub = n.advertise<geometry_msgs::Twist>("cmd_pos", 1);
	//laser_sub= n.subscribe<sensor_msgs::LaserScan>("/scan",1, laserCallBack);	 //scan /scan_align - org
	movx_sub = n.subscribe<htbot::move_status>("move_change", 100,changeCallBack);
	clear_pub = n.advertise<htbot::clear>("clearMap", 100);
	n.getParam("DEGPERIDX",degperidx);
	n.getParam("FIRSTMOVE",fmove);
	n.getParam("FINALMOVE",ffmove);
	n.getParam("OFFSET",offset);
	n.getParam("ALIGN_METHOD",align_method);

  //ROS_INFO("Ready to move.");
  ros::spin();

  return 0;
}



