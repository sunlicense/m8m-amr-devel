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
  	costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
    costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_

    // initialize other planner parameters
    //ros::NodeHandle private_nh("~/" + name);
    //private_nh.param("step_size", step_size_, costmap_->getResolution());
    //private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
		ROS_INFO("------------------ rac planner : initialised ---------------------");
    initialized_ = true;
 	} else {
  	ROS_WARN("This planner has already been initialized... doing nothing");
	}
}

double RacPlanner::footprintCost(double x_i, double y_i, double theta_i){
	if(!initialized_){
	  ROS_INFO("The Rac planner has not been initialized, please call initialize() to use the planner");
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
	double footprint_cost,tyawp;
	
	x = start.pose.position.x;
	y = start.pose.position.y;
	
	ROS_INFO("----- rac planner makeplan : start x=%.3f y=%.3f. --------",x,y);
	plan.clear();
	plan.push_back(start);
	geometry_msgs::PoseStamped ngoal;
	tf::Quaternion qp(0.0,0.0,0.0,1.0);
	tyawp = tf::getYaw(qp);	
	x1 = x;
	y1 = y;
	goal_step = 0.3;
	ROS_INFO("----------- Rac Planner : Left -------------------");
	for (int i=0;i<20;i++) {   // left
		x1 = x1 - goal_step;
		ngoal.pose.position.x = x1;
		ngoal.pose.position.y = y1;
		ngoal.pose.orientation.x = 0.0;
		ngoal.pose.orientation.y = 0.0;
		ngoal.pose.orientation.z = 0.0;
		ngoal.pose.orientation.w = 1.0;		
		footprint_cost = footprintCost(x1,y1,tyawp);
		ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
		plan.push_back(ngoal);
	}
	ROS_INFO("----------- Rac Planner : Down -------------------");
	y1 = y1 - goal_step;  // down
	ngoal.pose.position.x = x1;
	ngoal.pose.position.y = y1;
	ngoal.pose.orientation.x = 0.0;
	ngoal.pose.orientation.y = 0.0;
	ngoal.pose.orientation.z = 0.0;
	ngoal.pose.orientation.w = 1.0;
	footprint_cost = footprintCost(x1,y1,tyawp);
  ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
	plan.push_back(ngoal);
	
	ROS_INFO("----------- Rac Planner : Right -------------------");
	for (int i=0;i<20;i++) {   // right
		x1 = x1 + goal_step;
		ngoal.pose.position.x = x1;
		ngoal.pose.position.y = y1;
		ngoal.pose.orientation.x = 0.0;
		ngoal.pose.orientation.y = 0.0;
		ngoal.pose.orientation.z = 0.0;
		ngoal.pose.orientation.w = 1.0;		
		footprint_cost = footprintCost(x1,y1,tyawp);
  	ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
		plan.push_back(ngoal);
	}
	
	ROS_INFO("----------- Rac Planner : Down -------------------");
	y1 = y1 - goal_step;  // down
	ngoal.pose.position.x = x1;
	ngoal.pose.position.y = y1;
	ngoal.pose.orientation.x = 0.0;
	ngoal.pose.orientation.y = 0.0;
	ngoal.pose.orientation.z = 0.0;
	ngoal.pose.orientation.w = 1.0;
	footprint_cost = footprintCost(x1,y1,tyawp);
  ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
	plan.push_back(ngoal);
	ROS_INFO("----------- Rac Planner : Left -------------------");
	for (int i=0;i<20;i++) {   // left
		x1 = x1 - goal_step;
		ngoal.pose.position.x = x1;
		ngoal.pose.position.y = y1;
		ngoal.pose.orientation.x = 0.0;
		ngoal.pose.orientation.y = 0.0;
		ngoal.pose.orientation.z = 0.0;
		ngoal.pose.orientation.w = 1.0;		
		footprint_cost = footprintCost(x1,y1,tyawp);
  	ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
		plan.push_back(ngoal);
	}
	ROS_INFO("----------- Rac Planner : Down -------------------");
	y1 = y1 - goal_step;  // down
	ngoal.pose.position.x = x1;
	ngoal.pose.position.y = y1;
	ngoal.pose.orientation.x = 0.0;
	ngoal.pose.orientation.y = 0.0;
	ngoal.pose.orientation.z = 0.0;
	ngoal.pose.orientation.w = 1.0;
	footprint_cost = footprintCost(x1,y1,tyawp);
  ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
	plan.push_back(ngoal);

	ROS_INFO("----------- Rac Planner : Right -------------------");
	for (int i=0;i<20;i++) {   // right
		x1 = x1 + goal_step;
		ngoal.pose.position.x = x1;
		ngoal.pose.position.y = y1;
		ngoal.pose.orientation.x = 0.0;
		ngoal.pose.orientation.y = 0.0;
		ngoal.pose.orientation.z = 0.0;
		ngoal.pose.orientation.w = 1.0;		
		footprint_cost = footprintCost(x1,y1,tyawp);
  	ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
		plan.push_back(ngoal);
	}
	ROS_INFO("----------- Rac Planner : Down -------------------");
	y1 = y1 - goal_step;  // down
	ngoal.pose.position.x = x1;
	ngoal.pose.position.y = y1;
	ngoal.pose.orientation.x = 0.0;
	ngoal.pose.orientation.y = 0.0;
	ngoal.pose.orientation.z = 0.0;
	ngoal.pose.orientation.w = 1.0;
	footprint_cost = footprintCost(x1,y1,tyawp);
  ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
	plan.push_back(ngoal);
	ROS_INFO("----------- Rac Planner : Left -------------------");
	for (int i=0;i<20;i++) {   // left
		x1 = x1 - goal_step;
		ngoal.pose.position.x = x1;
		ngoal.pose.position.y = y1;
		ngoal.pose.orientation.x = 0.0;
		ngoal.pose.orientation.y = 0.0;
		ngoal.pose.orientation.z = 0.0;
		ngoal.pose.orientation.w = 1.0;		
		footprint_cost = footprintCost(x1,y1,tyawp);
  	ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
		plan.push_back(ngoal);
	}
	ROS_INFO("----------- Rac Planner : Down -------------------");
	y1 = y1 - goal_step;  // down
	ngoal.pose.position.x = x1;
	ngoal.pose.position.y = y1;
	ngoal.pose.orientation.x = 0.0;
	ngoal.pose.orientation.y = 0.0;
	ngoal.pose.orientation.z = 0.0;
	ngoal.pose.orientation.w = 1.0;
	footprint_cost = footprintCost(x1,y1,tyawp);
  ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
	plan.push_back(ngoal);
	ROS_INFO("----------- Rac Planner : Right -------------------");
	for (int i=0;i<20;i++) {   // right
		x1 = x1 + goal_step;
		ngoal.pose.position.x = x1;
		ngoal.pose.position.y = y1;
		ngoal.pose.orientation.x = 0.0;
		ngoal.pose.orientation.y = 0.0;
		ngoal.pose.orientation.z = 0.0;
		ngoal.pose.orientation.w = 1.0;		
		footprint_cost = footprintCost(x1,y1,tyawp);
  	ROS_INFO("----------- Rac Planner : fcost=%.3f. x=%.3f. y=%.3f -------------------",footprint_cost,x1,y1);
		plan.push_back(ngoal);
	}
	ROS_INFO("------------------ rac planner : Load Goal. ---------------------");
	plan.push_back(goal);
	ROS_INFO("------------------ rac planner : makePlan Done. ---------------------");
  return true;
}
};
