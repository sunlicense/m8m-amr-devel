#include <pluginlib/class_list_macros.h>
#include "rac_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rac_planner::RacPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rac_planner {

RacPlanner::RacPlanner (){

}

RacPlanner::RacPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	initialize(name, costmap_ros);
}


void RacPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	double px,py,pz,prx,pry,prz,prw,status,st,an,anmax;
	double ex,ey,diff;
	tf2::Quaternion quat;
	geometry_msgs::Quaternion qq;
	int ret,ix;
	FILE *pfp;
	std::string fn;

	if(!initialized_){
		ros::NodeHandle nh;
  	costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
    costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_
		plan_pub_ = nh.advertise<nav_msgs::Path>("racplan", 1);
    
    frame_id_ = costmap_ros_->getGlobalFrameID();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
		//ROS_INFO("------------------ rac planner : initialised. frame_id : %s ---------------------",frame_id_.c_str());
    
		// read path file
		nh.getParam("cleanplanDirectory",cleanplanDirectory);
		nh.getParam("cleanplanfile",cleanplan);
		fn = cleanplanDirectory + cleanplan;
		ROS_INFO("----- RAC Planner : makePlan Fn = %s. --------",fn.c_str());
		pfp = fopen(fn.c_str(), "r");
		if (pfp == NULL) {
  		ROS_INFO("---- RACPlanner initialize : I couldn't open cleanplan file for reading. ----------");    
  		return;
 	 	}
		ret = fscanf(pfp, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &px,&py,&pz,&prx,&pry,&prz,&prw,&status); // start position
		ret = fscanf(pfp, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &px,&py,&pz,&prx,&pry,&prz,&prw,&status); // end position
		ret = fscanf(pfp, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &px,&py,&pz,&prx,&pry,&prz,&prw,&status); // docking station position
		noPP = 0;
		while(true) {
			if (fscanf(pfp,"%lf %lf %lf %lf %lf %lf %lf %lf\n",&px,&py,&pz,&prx,&pry,&prz,&prw,&status) == EOF) {
				break;
			}
			pathInfo[noPP][0] = px;
			pathInfo[noPP][1] = py;
			pathInfo[noPP][2] = px;
			pathInfo[noPP][3] = prx;
			pathInfo[noPP][4] = pry;
			pathInfo[noPP][5] = prz;
			pathInfo[noPP][6] = prw;
			pathInfo[noPP][7] = status; // 0.0=normal. 1.0=segment marker. segment end point
			//if (status == 1.0) {
			//	calcQuat(pathInfo[noPP-1][0],pathInfo[noPP-1][1],px,py);
			//	pathInfo[noPP][3] = qx;
			//	pathInfo[noPP][4] = qy;
			//	pathInfo[noPP][5] = qz;
			//	pathInfo[noPP][6] = qw;
			//	ROS_INFO("---- noPP=%d : x=%.3f. y=%.3f. prz=%.3f. prw=%.3f ----",noPP,px,py,qz,qw);
			//}
			noPP++;
		}
		for (int i=0;i<noPP;i++) {
			status = pathInfo[i][7];
			if (status == 1.0) {
				px = pathInfo[i][0];
				py = pathInfo[i][1];
				if (i < (noPP-1)) {
					ex = pathInfo[i+1][0];
					ey = pathInfo[i+1][1];
					anmax = calcAn(px,py,ex,ey);
					ix = i + 1;
					for (int j=i+2;j<noPP;j++) {					
						ex = pathInfo[j][0];
						ey = pathInfo[j][1];
						an = calcAn(px,py,ex,ey);						
						diff = angles::shortest_angular_distance(anmax, an);
						if (fabs(diff) > 0.08) {
							break;
						}
						ix = j;
						st = pathInfo[j][7];
						if (st == 1.0) {
							break;
						}
					}
					calcQuat(pathInfo[i][0],pathInfo[i][1],pathInfo[ix][0],pathInfo[ix][1]);
					pathInfo[i][3] = qx;
					pathInfo[i][4] = qy;
					pathInfo[i][5] = qz;
					pathInfo[i][6] = qw;
				}
			}
		}
		cleanplanmove = false;
		PPPtr = 0;
		initialized_ = true;
		nh.setParam("PPPtr",PPPtr);
		//ROS_INFO("---- RACPLANNER : Initialisation . noPP = %d. PPPtr = %d. -----------",noPP,PPPtr);
		//for (int i=0;i<noPP;i++) {
		//	ROS_INFO("--- PP=%d : x=%.3f. y=%.3f. Status=%.3f ----",i,pathInfo[i][0],pathInfo[i][1],pathInfo[i][7]);
		//}
		callcount = false;
 	} else {
  	ROS_INFO("---- This planner has already been initialized... doing nothing ----");
	}
}

double RacPlanner::calcAn(double sx, double sy,double ex, double ey)
{
	double an,yawp,diff;
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
	return an;
}

void RacPlanner::calcQuat(double sx, double sy,double ex, double ey)
{
	double an,yawp,diff;
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
	qx = qq.x;
	qy = qq.y;
	qz = qq.z;
	qw = qq.w;
	return;
}

void RacPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_INFO("--- publishPlan : planner has not been initialized yet, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

double RacPlanner::footprintCost(double x_i, double y_i, double theta_i){
	if(!initialized_){
	  ROS_INFO("--- footprintCost : The Rac planner has not been initialized, please call initialize() to use the planner ---");
    return -1.0;
  }

 	std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
  //if we have no footprint... do nothing
  if(footprint.size() < 3)
  	return -1.0;

  //check if the footprint is legal
  double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
  return footprint_cost;
}

bool RacPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
	double x,y,x1,y1;
	double tr,tw;
	double footprint_cost,tyawp;
	ros::NodeHandle nh;
	FILE *pfp;
	std::string fn;
	int ret;
	double px,py,pz,prx,pry,prz,prw;
	geometry_msgs::PoseStamped ngoal;
	tf::Quaternion qp(0.0,0.0,0.0,1.0);
	tyawp = tf::getYaw(qp);	
	int gcount,i;
	double status;
	

	if(!initialized_){
	  ROS_INFO("--- MakePlan : planner has not been initialized, please call initialize() to use the planner ---");
    return -1.0;
  }

	//clear the plan, just in case	
	nh.getParam("racPlanCall",racPlanCall);
	if (!racPlanCall) {
		if (!callcount) {
			ROS_INFO("------------------- RACPlanner : Not Called ----------------------");
			callcount = true;
		}
		return true;
	}
	callcount = false;
  plan.clear();

	std::string global_frame = frame_id_;
	if (goal.header.frame_id != global_frame) {
  	ROS_INFO("--- goal must be in the %s frame.It is instead in the %s frame. ---", global_frame.c_str(), goal.header.frame_id.c_str());
    return false;
  }
	if (start.header.frame_id != global_frame) {
  	ROS_INFO("--- start must be in the %s frame.It is instead in the %s frame. ---", global_frame.c_str(), start.header.frame_id.c_str());
    return false;
  }
	
	double sx = start.pose.position.x;
  double sy = start.pose.position.y;
  unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
  double start_x, start_y, goal_x, goal_y;

	if(!costmap_->worldToMap(sx, sy, start_x_i, start_y_i)) {
  	ROS_INFO("--- robot's start position is off the global costmap. Planning will always fail ---");
    return false;
  }
	
	double wx = goal.pose.position.x;
  double wy = goal.pose.position.y;
	if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
  	ROS_INFO("--- The goal sent to the global planner is off the global costmap.Planning will always fail to this goal. ---");
    return false;
  }
	//racPlanCall = false;
	nh.getParam("cleanplanmove",cleanplanmove);
	nh.getParam("cleanplanmoveObs",cleanplanmoveObs);
	plan.clear();
	//ROS_INFO("--- RAC Planner : cleaning makeplan. noPP=%d. PPPtr=%d. Start X=%.3f. Y=%.3f ---",noPP,PPPtr,sx,sy);
	
	if (cleanplanmove) {
		if (PPPtr == noPP) {
			ROS_INFO("--- RAC Planner : end of goals in cleaning plan ---");
			cleanplanmove = false;
			nh.setParam("cleanplanmove",cleanplanmove);
			return true;
		}
		if (cleanplanmoveObs) {
			nh.getParam("PPPtr",PPPtr);
		}
		plan.push_back(start);
		ros::Time plan_time = ros::Time::now();
		for (i=PPPtr;i<noPP;i++) {		
			status = pathInfo[i][7];
			//if (status == 1.0) {
			//	geometry_msgs::PoseStamped goal_copy = goal;
  		//	goal_copy.header.stamp = ros::Time::now();
			//	ROS_INFO("------ last Goal Inserted : x=%.3f. y=%.3f -----",goal_copy.pose.position.x,goal_copy.pose.position.y);
			//	plan.push_back(goal_copy);
			//	break;
			//}
			ngoal.header.stamp = plan_time;
			ngoal.header.frame_id = global_frame;
			ngoal.pose.position.x = pathInfo[i][0];
			ngoal.pose.position.y = pathInfo[i][1];
			ngoal.pose.position.z = pathInfo[i][2];
			ngoal.pose.orientation.x = pathInfo[i][3];
			ngoal.pose.orientation.y = pathInfo[i][4];
			ngoal.pose.orientation.z = pathInfo[i][5];
			ngoal.pose.orientation.w = pathInfo[i][6];							
			//ROS_INFO("------ Goal Inserted : x=%.3f. y=%.3f -----",ngoal.pose.position.x,ngoal.pose.position.y);
			//ROS_INFO("------ Goal Inserted : rz=%.3f. rw=%.3f -----",ngoal.pose.orientation.z,ngoal.pose.orientation.w);
			plan.push_back(ngoal);		
			if (status == 1.0) {
				//ROS_INFO("------ last Goal Inserted : Ptr=%d. x=%.3f. y=%.3f -----",i,ngoal.pose.position.x,ngoal.pose.position.y);
				break;
			}	
		}
		PPPtr = i+1;
		if (PPPtr == noPP) {
			//ROS_INFO("--- RAC Planner : end of goals in cleaning plan ---");
	 		cleanplanmove = false;
			nh.setParam("cleanplanmove",cleanplanmove);
		}
		publishPlan(plan);
		nh.setParam("PPPtr",PPPtr);
		racPlanCall = false;
		nh.setParam("racPlanCall",racPlanCall);
		//ROS_INFO("----------- rac planner : makePlan Done. PPPtr=%d. noPP=%d -------------",PPPtr,noPP);
	} else {
		return false;
	}
  return true;
}
};
