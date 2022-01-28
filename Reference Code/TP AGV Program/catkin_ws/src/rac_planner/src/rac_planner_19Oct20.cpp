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
	if(!initialized_){
		ros::NodeHandle nh;
  	costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
    costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_
		plan_pub_ = nh.advertise<nav_msgs::Path>("racplan", 1);
    
    frame_id_ = costmap_ros_->getGlobalFrameID();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
		ROS_INFO("------------------ rac planner : initialised. frame_id : %s ---------------------",frame_id_.c_str());
    initialized_ = true;
 	} else {
  	ROS_WARN("This planner has already been initialized... doing nothing");
	}
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
	int gcount;

	if(!initialized_){
	  ROS_INFO("--- MakePlan : planner has not been initialized, please call initialize() to use the planner ---");
    return -1.0;
  }

	//clear the plan, just in case
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
	
	double wx = start.pose.position.x;
  double wy = start.pose.position.y;
  unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
  double start_x, start_y, goal_x, goal_y;

	if(!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
  	ROS_INFO("--- robot's start position is off the global costmap. Planning will always fail ---");
    return false;
  }
	wx = goal.pose.position.x;
  wy = goal.pose.position.y;
	if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
  	ROS_INFO("--- The goal sent to the global planner is off the global costmap.Planning will always fail to this goal. ---");
    return false;
  }

	nh.getParam("cleanplanDirectory",cleanplanDirectory);
	nh.getParam("cleanplanfile",cleanplan);

	fn = cleanplanDirectory + cleanplan;
	ROS_INFO("----------- RAC Planner : makePlan Fn = %s. --------------",fn.c_str());
	pfp = fopen(fn.c_str(), "r");
	if (pfp == NULL) {
  	ROS_INFO("---------- makePlan : I couldn't open cleanplan file for reading. -------------------");    
  	return true;
  }
	plan.clear();
	plan.push_back(start);
	ret = fscanf(pfp, "%lf %lf %lf %lf %lf %lf %lf\n", &px,&py,&pz,&prx,&pry,&prz,&prw); // start position
	ROS_INFO("--- Start Pos : %.3f %.3f %.3f %.3f %.3f %.3f %.3f ----",px,py,pz,prx,pry,prz,prw);
	ret = fscanf(pfp, "%lf %lf %lf %lf %lf %lf %lf\n", &px,&py,&pz,&prx,&pry,&prz,&prw); // end position
	ROS_INFO("--- End Pos : %.3f %.3f %.3f %.3f %.3f %.3f %.3f ----",px,py,pz,prx,pry,prz,prw);
	ret = fscanf(pfp, "%lf %lf %lf %lf %lf %lf %lf\n", &px,&py,&pz,&prx,&pry,&prz,&prw); // docking station position
	ROS_INFO("--- Dock Pos : %.3f %.3f %.3f %.3f %.3f %.3f %.3f ----",px,py,pz,prx,pry,prz,prw);
	ros::Time plan_time = ros::Time::now();
	gcount = 0;
	while(true) {
		if (fscanf(pfp,"%lf %lf\n",&x,&y) == EOF) {
			break;
		}
		gcount++;
		ngoal.header.stamp = plan_time;
		ngoal.header.frame_id = global_frame;
		ngoal.pose.position.x = x;
		ngoal.pose.position.y = y;
		ngoal.pose.position.z = 0.0;
		ngoal.pose.orientation.x = 0.0;
		ngoal.pose.orientation.y = 0.0;
		ngoal.pose.orientation.z = 0.0;
		ngoal.pose.orientation.w = 1.0;		
		footprint_cost = footprintCost(x,y,tyawp);
		ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x,y);
		if (gcount == 1) {
			nh.setParam("checkPlanX",x);
			nh.getParam("checkPlanY",y);
		}
		plan.push_back(ngoal);
	}
	
	ROS_INFO("------------------ rac planner : Load Goal. ---------------------");
	geometry_msgs::PoseStamped goal_copy = goal;
  goal_copy.header.stamp = ros::Time::now();
	plan.push_back(goal_copy);
	publishPlan(plan);
	ROS_INFO("------------------ rac planner : makePlan Done. ---------------------");
  return true;
}
};
