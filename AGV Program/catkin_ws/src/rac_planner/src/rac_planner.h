/** include the libraries you need in your planner here */
/** for rac path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <vector>

using std::string;

#ifndef RAC_PLANNER_CPP
#define RAC_PLANNER_CPP
#define MAXPP 500
#define PI 3.141593

namespace rac_planner {

class RacPlanner : public nav_core::BaseGlobalPlanner {
 public:

  RacPlanner();
  RacPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
               );
	double footprintCost(double x_i, double y_i, double theta_i);
	void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
	void calcQuat(double sx, double sy,double ex, double ey);
	double calcAn(double sx, double sy,double ex, double ey);
	void loadPath(void); 
	void loadNormalPath(void);

 private:
 	costmap_2d::Costmap2DROS* costmap_ros_;
 	ros::Publisher plan_pub_;

	double goal_step;
	std::string cleanplanDirectory,cleanplan,racpathDir,racpath;
	std::string frame_id_;
  costmap_2d::Costmap2D* costmap_;
  base_local_planner::WorldModel* world_model_; 
	double pathInfo[MAXPP][8];
	int noPP;
	int PPPtr;
	bool cleanplanmove,cleanplanmoveObs,normalPath;
	bool racPlanCall;
	double qx,qy,qz,qw;
	bool callcount;
 
  //double footprintCost(double x_i, double y_i, double theta_i);
 
  bool initialized_;
};
};
#endif

