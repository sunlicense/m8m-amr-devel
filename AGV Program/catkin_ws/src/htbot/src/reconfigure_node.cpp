/*
 * This node is to dynamically re-configure ros nodes
 */
/* 	History
*		Date Modified : 28.8.2020
*		Changes :
*/

#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include "boost/algorithm/string.hpp"
#include <math.h>
#include "htbot/dyna.h"
#include "htbot/DYNA.h"

using namespace std;

ros::Subscriber dyna_sub;
ros::Publisher footprint_pub;

double d1,d2,d3,d4,d5,d6,d7,d8,d9,e1,e2,e3,minRange;

double gftprint[10][20];
double lftprint[10][20];

// prototype

bool dynamicReConfigure(const htbot::dyna::ConstPtr& msg);
void publish_mapzminRange(void);
void calminRange(void);

void calminRange(void) 
{
	double mr,mrt;
	
	//boost::mutex::scoped_lock lock(publish_mutex_);
	mr = 0.0;
	mrt = sqrt(d1*d1 + d2*d2);
	if (mrt > mr) {
		mr = mrt;
	}
	mrt = sqrt(d3*d3 + d4*d4);
	if (mrt > mr) {
		mr = mrt;
	} 
	mrt = sqrt(d5*d5 + d6*d6);
	if (mrt > mr) {
		mr = mrt;
	} 
	mrt = sqrt(d7*d7 + d8*d8);
	if (mrt > mr) {
		mr = mrt;
	} 
	mrt = sqrt(d9*d9 + e1*e1);
	if (mrt > mr) {
		mr = mrt;
	} 
	mrt = sqrt(e2*e2 + e3*e3);
	if (mrt > mr) {
		mr = mrt;
	}  
	minRange = mr;
	ROS_INFO("------------- Dyna : minRange = %.3f -------------",minRange);
}

void publish_mapzminRange(void)
{
	//boost::mutex::scoped_lock lock(publish_mutex_);
	htbot::dyna fp;
	fp.doubleValue = minRange;
	footprint_pub.publish(fp);
	return;
}

bool dynamicReConfigure(const htbot::dyna::ConstPtr& msg)
{
	int id,ret,iv;
	double dv;
	char buf[300];
	std::string sc,sc1;	
	id = msg->paramid;
	
	//ROS_INFO("---------- Dyna ID : %d. -----------------",id);
	switch (id) {
		case planner_frequency:			
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base planner_frequency %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : planner_frequency -----------------");		
			break;
		case planner_patience:			
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base planner_patience %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());			
			//ROS_INFO("---------- Dyna : planner_patience -----------------");		
			break;
		case controller_frequency:			
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base controller_frequency %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());			
			//ROS_INFO("---------- Dyna : controller_frequency -----------------");		
			break;
		case controller_patience:			
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base controller_patience %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());			
			//ROS_INFO("---------- Dyna : controller_patience -----------------");		
			break;
		case sim_time:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS sim_time %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : sim_time -----------------");	
			break;		
		case path_distance_bias:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS path_distance_bias %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : path_distance_bias -----------------");	
			break;	
		case goal_distance_bias:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS goal_distance_bias %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : goal_distance_bias -----------------");	
			break;	
		case max_vel_x:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS max_vel_x  %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : max_vel_x -----------------");	
			break;
		case min_vel_x:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS min_vel_x %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : min_vel_x %.3f -----------------",dv);	
			break;
		case acc_lim_x:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS acc_lim_x %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : acc_lim_x %.3f -----------------",dv);
			break;
		case acc_lim_y:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS acc_lim_y %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : acc_lim_y %.3f -----------------",dv);
			break;
		case acc_lim_theta:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS acc_lim_theta %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : acc_lim_theta %.3f -----------------",dv);
			break;
		case max_vel_theta:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS max_vel_theta %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : max_vel_theta %.3f -----------------",dv);
			break;
		case min_vel_theta:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS min_vel_theta %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : min_vel_theta %.3f -----------------",dv);
			break;
		case min_in_place_vel_theta:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS min_in_place_vel_theta %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());	
			//ROS_INFO("---------- Dyna : min_in_place_vel_theta %.3f -----------------",dv);	
			break;
		case vx_samples:
			iv = msg->intValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS vx_samples %d &",iv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : vx_samples %d -----------------",iv);	
			break;
		case vtheta_samples:
			iv = msg->intValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS vtheta_samples %d &",iv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : vtheta_samples %d -----------------",iv);	
			break;
		case heading_lookahead:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS heading_lookahead %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : heading_lookahead %.3f -----------------",dv);
			break;
		case escape_vel:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS escape_vel %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : escape_vel %.3f -----------------",dv);
			break;
		case global_costmap_update_frequency:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/global_costmap update_frequency %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : global_costmap_update_frequency %.3f -----------------",dv);
			break;
		case global_costmap_publish_frequency:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/global_costmap publish_frequency %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : global_costmap_publish_frequency %.3f -----------------",dv);
			break;
		case global_costmap_width:
			iv = msg->intValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/global_costmap width %d &",iv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : global_costmap_width %d -----------------",iv);
			break;
		case global_costmap_height:
			iv = msg->intValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/global_costmap height %d &",iv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : global_costmap_height %d -----------------",iv);
			break;
		case global_costmap_robot_radius:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/global_costmap robot_radius %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : global_costmap_robot_radius %.3f -----------------",dv);
			break;
		case global_costmap_footprint_padding:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/global_costmap footprint_padding %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : global_costmap_footprint_padding %.3f -----------------",dv);
			break;
		case global_costmap_inflation_radius:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/global_costmap/inflation_radius inflation_radius %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : /move_base/global_costmap/inflation_radius %.3f -----------------",dv);
			break;
		case local_costmap_update_frequency:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/local_costmap update_frequency %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : local_costmap_update_frequency %.3f -----------------",dv);
			break;
		case local_costmap_publish_frequency:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/local_costmap publish_frequency %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : local_costmap_publish_frequency %.3f -----------------",dv);
			break;
		case local_costmap_width:
			iv = msg->intValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/local_costmap width %d &",iv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : local_costmap_width %d -----------------",iv);
			break;
		case local_costmap_height:
			iv = msg->intValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/local_costmap height %d &",iv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : local_costmap_height %d -----------------",iv);
			break;
		case local_costmap_robot_radius:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/local_costmap robot_radius %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : local_costmap_robot_radius %.3f -----------------",dv);
			break;
		case local_costmap_footprint_padding:
			dv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base/local_costmap footprint_padding %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : local_costmap_footprint_padding %.3f -----------------",dv);
			break;
		case amcl_max_particles:
			iv = msg->intValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /amcl max_particles %d &",iv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : amcl_max_particles %d -----------------",iv);
			break;
		case amcl_min_particles:
			iv = msg->intValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /amcl min_particles %d &",iv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : amcl_min_particles %d -----------------",iv);
			break;
		case amcl_update_min_d:
			iv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /amcl update_min_d %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : amcl_update_min_d %.3f -----------------",dv);
			break;
		case amcl_update_min_a:
			iv = msg->doubleValue;
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /amcl update_min_a %.3f &",dv);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());		
			//ROS_INFO("---------- Dyna : amcl_update_min_a %.3f -----------------",dv);
			break;
		case global_footprint:
			iv = msg->intValue;
			d1 = gftprint[iv][0];
			d2 = gftprint[iv][1];
			d3 = gftprint[iv][2];
			d4 = gftprint[iv][3];
			d5 = gftprint[iv][4];
			d6 = gftprint[iv][5];
			d7 = gftprint[iv][6];
			d8 = gftprint[iv][7];
			d9 = gftprint[iv][8];
			e1 = gftprint[iv][9];
			e2 = gftprint[iv][10];
			e3 = gftprint[iv][11];
			sc = "rosrun dynamic_reconfigure dynparam set /move_base/global_costmap footprint "; 			
			sprintf(buf,"[[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f]] &",d1,d2,d3,d4,d5,d6,d7,d8,d9,e1,e2,e3);
			sc1.assign(buf,strlen(buf));
			sc = sc + sc1;
			//ROS_INFO("---------- gfp : %s --------------",sc.c_str());
			ret = system(sc.c_str());	
			break;
		case local_footprint:
			iv = msg->intValue;
			d1 = lftprint[iv][0];
			d2 = lftprint[iv][1];
			d3 = lftprint[iv][2];
			d4 = lftprint[iv][3];
			d5 = lftprint[iv][4];
			d6 = lftprint[iv][5];
			d7 = lftprint[iv][6];
			d8 = lftprint[iv][7];
			d9 = lftprint[iv][8];
			e1 = lftprint[iv][9];
			e2 = lftprint[iv][10];
			e3 = lftprint[iv][11];
			sc = "rosrun dynamic_reconfigure dynparam set /move_base/local_costmap footprint ";
			sprintf(buf,"[[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f]] &",d1,d2,d3,d4,d5,d6,d7,d8,d9,e1,e2,e3);
			sc1.assign(buf,strlen(buf));
			sc = sc + sc1;
			//ROS_INFO("---------- lfp : %s --------------",sc.c_str());
			ret = system(sc.c_str());	
			calminRange();
			//publish_mapzminRange();
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /mapzone MapObsMinRange %.3f &",minRange);
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());
			break;
		case global_planner:			
			sc1 = msg->strValue;
			ROS_INFO("-------------ReConfigure :  Global Planner : %s--------------",sc1.c_str());
			sprintf(buf,"rosrun dynamic_reconfigure dynparam set /move_base base_global_planner %s &",sc1.c_str());
			sc.assign(buf,strlen(buf));
			ret = system(sc.c_str());	
			break;
	}
	//usleep(100000);
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Dynamic ReConfigure Server");
  ros::NodeHandle n;
	
	dyna_sub = n.subscribe<htbot::dyna>("reconfigure", 100, dynamicReConfigure);  // 
	footprint_pub = n.advertise<htbot::dyna>("mapzonefootprint", 100);

	n.getParam("global_footprint_base0_x",gftprint[0][0]);
	n.getParam("global_footprint_base0_y",gftprint[0][1]);
	n.getParam("global_footprint_base1_x",gftprint[0][2]);
	n.getParam("global_footprint_base1_y",gftprint[0][3]);
	n.getParam("global_footprint_base2_x",gftprint[0][4]);
	n.getParam("global_footprint_base2_y",gftprint[0][5]);
	n.getParam("global_footprint_base3_x",gftprint[0][6]);
	n.getParam("global_footprint_base3_y",gftprint[0][7]);
	n.getParam("global_footprint_base4_x",gftprint[0][8]);
	n.getParam("global_footprint_base4_y",gftprint[0][9]);
	n.getParam("global_footprint_base5_x",gftprint[0][10]);
	n.getParam("global_footprint_base5_y",gftprint[0][11]);

	n.getParam("global_footprint_fpA0_x",gftprint[1][0]);
	n.getParam("global_footprint_fpA0_y",gftprint[1][1]);
	n.getParam("global_footprint_fpA1_x",gftprint[1][2]);
	n.getParam("global_footprint_fpA1_y",gftprint[1][3]);
	n.getParam("global_footprint_fpA2_x",gftprint[1][4]);
	n.getParam("global_footprint_fpA2_y",gftprint[1][5]);
	n.getParam("global_footprint_fpA3_x",gftprint[1][6]);
	n.getParam("global_footprint_fpA3_y",gftprint[1][7]);
	n.getParam("global_footprint_fpA4_x",gftprint[1][8]);
	n.getParam("global_footprint_fpA4_y",gftprint[1][9]);
	n.getParam("global_footprint_fpA5_x",gftprint[1][10]);
	n.getParam("global_footprint_fpA5_y",gftprint[1][11]);

	n.getParam("local_footprint_base0_x",lftprint[0][0]);
	n.getParam("local_footprint_base0_y",lftprint[0][1]);
	n.getParam("local_footprint_base1_x",lftprint[0][2]);
	n.getParam("local_footprint_base1_y",lftprint[0][3]);
	n.getParam("local_footprint_base2_x",lftprint[0][4]);
	n.getParam("local_footprint_base2_y",lftprint[0][5]);
	n.getParam("local_footprint_base3_x",lftprint[0][6]);
	n.getParam("local_footprint_base3_y",lftprint[0][7]);
	n.getParam("local_footprint_base4_x",lftprint[0][8]);
	n.getParam("local_footprint_base4_y",lftprint[0][9]);
	n.getParam("local_footprint_base5_x",lftprint[0][10]);
	n.getParam("local_footprint_base5_y",lftprint[0][11]);

	n.getParam("local_footprint_fpA0_x",lftprint[1][0]);
	n.getParam("local_footprint_fpA0_y",lftprint[1][1]);
	n.getParam("local_footprint_fpA1_x",lftprint[1][2]);
	n.getParam("local_footprint_fpA1_y",lftprint[1][3]);
	n.getParam("local_footprint_fpA2_x",lftprint[1][4]);
	n.getParam("local_footprint_fpA2_y",lftprint[1][5]);
	n.getParam("local_footprint_fpA3_x",lftprint[1][6]);
	n.getParam("local_footprint_fpA3_y",lftprint[1][7]);
	n.getParam("local_footprint_fpA4_x",lftprint[1][8]);
	n.getParam("local_footprint_fpA4_y",lftprint[1][9]);
	n.getParam("local_footprint_fpA5_x",lftprint[1][10]);
	n.getParam("local_footprint_fpA5_y",lftprint[1][11]);

 	n.getParam("global_footprint_padbase",gftprint[9][0]);
	n.getParam("global_footprint_padA",gftprint[9][1]);

 	n.getParam("local_footprint_padbase",lftprint[9][0]);
	n.getParam("local_footprint_padA",lftprint[9][1]);

  ros::spin();

  return 0;
}



