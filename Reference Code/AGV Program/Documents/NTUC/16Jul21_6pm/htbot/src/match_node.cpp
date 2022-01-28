/*
 * This node is to service command request to match 2 laser scans
 */
/* 	History
*		Date Modified : 26.10.2016
*		Changes :
*/

//#define ODROID
//#define SSMC
//#define RAC
//#include "htbot/match_node.h"
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "boost/algorithm/string.hpp"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include "htbot/scanCmd.h"
#include "htbot/scanMcmd.h"
#include "htbot/mqueue.h"
#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max


using namespace std;

ros::Subscriber laser_sub;
ros::Subscriber scanCmd_sub;
ros::Subscriber pose_sub;
ros::Publisher pos_pub;

double rz,rw,angle;
bool laser_use;
int laser_size,cidx,lidx,ridx;
int xcount;
int stop_move;
double fmove,ffmove,mdist,offset,toffset,speed;
double degperidx,stnsize;
bool savescan,match,scan_init,getpose;  // start match the laser scan.
double align_method;
double px,py,pa,prz,prw;
double gx,gy,ga,grz,grw;
double pr_ch_x,pr_ch_y,pr_ch_a;  // 
double outdx,outdy,outda;  // output of scan matching

std::vector<double> a_cos_;
std::vector<double> a_sin_;
std::string base_frame;
std::string fixed_frame;
std::string laser_frame;
tf::Transform f2b;    // fixed-to-base tf (pose of base frame in fixed frame)
tf::Transform f2b_kf; // pose of the last keyframe scan in fixed frame
tf::Transform base_to_laser; // static, cached
tf::Transform laser_to_base; // static, cached, calculated from base_to_laser
htbot::mqueue web_srv;
ros::ServiceClient web_client;

sm_params input;
sm_result output;
LDP prev_ldp_scan;
LDP curr_ldp_scan;
LDP ref_ldp_scan;
ros::Time last_icp_time, delay_time, scan_time;
int mstate;
FILE *fp, *fp1;
ros::ServiceServer scan_match_service;

int GLP,GGP;

// prototype
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);
void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,LDP& ldp);
//void processScan(LDP& curr_ldp_scan, const ros::Time& time);
void processScan();
void scanCMD(const htbot::scanCmd::ConstPtr& msg);
void matchLoop();
bool scanMatchServiceCallback(htbot::scanMcmd::Request &req,htbot::scanMcmd::Response &res);
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
void initParams();
bool getBaseToLaserTf (const std::string& frame_id);
bool dbRequest(int GP, int LP);
void getPrediction();
void ScanMatchAlignment();
void ScanMatchAlignment2();
void ScanMatchAlignment3();


void initParams()
{
  base_frame = "base_link";
	fixed_frame = "map";
	laser_frame = "laser";

  // **** CSM parameters - comments copied from algos.h (by Andrea Censi)
	input.max_angular_correction_deg = 45.0;
	input.max_linear_correction = 0.50;
	input.max_iterations = 40;
	input.epsilon_xy = 0.000001;
	input.epsilon_theta = 0.000001;
	input.max_correspondence_dist = 0.3;
	input.sigma = 0.010;
	input.use_corr_tricks = 1;
	input.restart = 0;
	input.restart_threshold_mean_error = 0.01;
	input.restart_dt = 1.0;
	input.restart_dtheta = 0.1;
	input.clustering_threshold = 0.25;
	input.orientation_neighbourhood = 20;
	input.use_point_to_line_distance = 1;
	input.do_alpha_test = 0;
	input.do_alpha_test_thresholdDeg = 20.0;
	input.outliers_maxPerc = 0.90;
	input.outliers_adaptive_order = 0.7;
	input.outliers_adaptive_mult = 2.0;
	input.do_visibility_test = 0;
	input.outliers_remove_doubles = 1;
	input.do_compute_covariance = 0;
	input.debug_verify_tricks = 0;
	input.use_ml_weights = 0;
	input.use_sigma_weights = 0;

	f2b.setIdentity();
  f2b_kf.setIdentity();

	input.laser[0] = 0.0;
  input.laser[1] = 0.0;
  input.laser[2] = 0.0;

  // Initialize output_ vectors as Null for error-checking
  output.cov_x_m = 0;
  output.dx_dy1_m = 0;
  output.dx_dy2_m = 0;

}



void getPrediction() {
	double yawg,yawp,yd;

	tf::Quaternion qg(0.0,0.0,grz,grw);
	tf::Quaternion qp(0.0,0.0,prz,prw);
	pr_ch_x = px - gx;
	pr_ch_y = py - gy;
	
	yawg = tf::getYaw(qg);
	yawp = tf::getYaw(qp);
	yd = yawp - yawg;
	if (yd >= M_PI) {
		yd -= 2.0 * M_PI;
	} else {
		if (yd < -M_PI) {
			yd += 2.0 * M_PI;
		}
	}
	pr_ch_a = yd;

}



bool dbRequest(int GP, int LP) {
	bool ret;

	web_srv.request.GN = GP;  
	web_srv.request.LP = LP;  		
	web_srv.request.cmd = 32;
			
	//ROS_INFO("dbReequest : 32.  LP : %d. GP : %d ",LP,GP);
		
	if (web_client.call(web_srv)) {
		//ROS_INFO("Received reply from dbase Server");
		if (web_srv.response.status == 32) {	 
			//ROS_ERROR("call dbase service OK");	 		 
			gx = web_srv.response.tx;   	
			gy = web_srv.response.ty;
			grz = web_srv.response.rz;
			grw = web_srv.response.rw;
			ret = true;
			//ROS_INFO("DB Info : gx=%.3f. gy=%.3f. grz=%.3f. grw=%.3f",gx,gy,grz,grw);	
		} else {
	    ROS_INFO("status wrong");	
			ret = false;
	 	}
	} else {
		ROS_INFO("Failed to call dbase service");	
		ret = false;
	}
	return ret;
}



bool getBaseToLaserTf (const std::string& frame_id)
{
	tf::TransformListener    tf_listener;
	//tf::Transform base_to_laser; // static, cached
	//tf::Transform laser_to_base; // static, cached, calculated from base_to_laser_
  ros::Time t = ros::Time::now();

  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener.waitForTransform(base_frame, frame_id, ros::Time(0), ros::Duration(1.0));
    tf_listener.lookupTransform (base_frame, frame_id, ros::Time(0), base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("Could not get initial transform from base to laser frame, %s", ex.what());
    return false;
  }
  base_to_laser = base_to_laser_tf;
  laser_to_base = base_to_laser.inverse();
	//ROS_INFO("BTL : y = %.3f. x = %.3f",base_to_laser.getOrigin().y(),base_to_laser.getOrigin().x());
  return true;
}



void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	double x,y,z,rz,rw;
	tf::Quaternion qq;
	
	if (getpose) {
		px = msg->position.x;
		py = msg->position.y;
		prz = msg->orientation.z;
		prw = msg->orientation.w;
		quaternionMsgToTF(msg->orientation,qq);
		pa = tf::getYaw(qq);
		
		//ROS_INFO("Match : Pose : x=%.2f. y=%.2f. pa=%.2f",px,py,pa);
		getpose = false;
	}
}



void scanCMD(const htbot::scanCmd::ConstPtr& msg)
{
	ros::NodeHandle nm;
	char buf[100];
	string s1;

	switch (msg->cmd) {
		case 0:  // save Reference Laser Scan to file
			//ROS_INFO("Save Ref Scan");
			fp = fopen(msg->file.c_str(), "w");
  		if (fp == NULL) {
  			ROS_INFO("I couldn't open reference laser scan file to write.\n");    
  			return;
  		}
			mstate = 1;  
			break;
		case 1:  // start scan matching alignment
			//ROS_INFO("Load Ref Scan : %s",msg->file.c_str());
			fp = fopen(msg->file.c_str(), "r");
  		if (fp == NULL) {
  			ROS_INFO("I couldn't open reference laser scan file to read.\n");    
  			return;
  		}
			GLP = msg->lp;
			GGP = msg->gp;
			mstate = 2;
			break;
		case 2: // save laser scan to current LDP
			mstate = 3;
			break;
		case 3:  // start scan matching alignment 2nd time
			//ROS_INFO("Load Ref Scan : %s",msg->file.c_str());
			fp = fopen(msg->file.c_str(), "r");
  		if (fp == NULL) {
  			ROS_INFO("I couldn't open reference laser scan file to read.\n");    
  			return;
  		}
			GLP = msg->lp;
			GGP = msg->gp;
			mstate = 4;
			break;
		case 4:  // start scan matching alignment 3rd time
			//ROS_INFO("Load Ref Scan : %s",msg->file.c_str());
			fp = fopen(msg->file.c_str(), "r");
  		if (fp == NULL) {
  			ROS_INFO("I couldn't open reference laser scan file to read.\n");    
  			return;
  		}
			GLP = msg->lp;
			GGP = msg->gp;
			mstate = 5;
			break;
	}
	
}

void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }

  input.min_reading = scan_msg->range_min;
  input.max_reading = scan_msg->range_max;
}

void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);
  //ROS_INFO("Laser Size = %d",n);
  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      // fill in laser scan data

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
			//ROS_INFO("Invalid Reading = %d. range=%.3f",i,r);
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment;

    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan) {		

	
	if (!scan_init) {  // first time scan 
		if (!getBaseToLaserTf(lscan->header.frame_id))
    {
      ROS_INFO("Problem with Base to Laser TF.");
      return;
    }
		scan_init = true;
	}
	

  if (savescan)
  {		
    createCache(lscan);    // caches the sin and cos of all angles
    laserScanToLDP(lscan, curr_ldp_scan);    
    savescan = false;		
  }

	if (match)
  {		
    createCache(lscan);    // caches the sin and cos of all angles		
    laserScanToLDP(lscan, curr_ldp_scan);		
		scan_time = lscan->header.stamp;	
		//ROS_INFO("----------- Match : Scan Match ----------");	
    match = false;
  }
}

//void processScan(LDP& curr_ldp_scan, const ros::Time& time)
void processScan()
{
  ros::WallTime start = ros::WallTime::now();

  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  ref_ldp_scan->odometry[0] = 0.0;
  ref_ldp_scan->odometry[1] = 0.0;

  ref_ldp_scan->odometry[2] = 0.0;

  ref_ldp_scan->estimate[0] = 0.0;
  ref_ldp_scan->estimate[1] = 0.0;
  ref_ldp_scan->estimate[2] = 0.0;

  ref_ldp_scan->true_pose[0] = 0.0;
  ref_ldp_scan->true_pose[1] = 0.0;
  ref_ldp_scan->true_pose[2] = 0.0;

  input.laser_ref  = ref_ldp_scan;
  input.laser_sens = curr_ldp_scan;

	getPrediction();
	//ROS_INFO("Prediction : prx=%.3f. pry=%.3f. pra=%.3f",pr_ch_x,pr_ch_y,pr_ch_a);
  // the predicted change of the laser's position, in the fixed frame

  tf::Transform pr_ch;
  createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

  // account for the change since the last kf, in the fixed frame

  pr_ch = pr_ch * (f2b * f2b_kf.inverse());

  // the predicted change of the laser's position, in the laser frame

  tf::Transform pr_ch_l;
  pr_ch_l = laser_to_base * f2b.inverse() * pr_ch * f2b * base_to_laser ;
	
	input.first_guess[0] = pr_ch_l.getOrigin().getX();
  input.first_guess[1] = pr_ch_l.getOrigin().getY();
  input.first_guess[2] = tf::getYaw(pr_ch_l.getRotation());
	
	//ROS_INFO("1st Guess : px = %.3f. py = %.3f. pa = %.3f",input.first_guess[0],input.first_guess[1],input.first_guess[2]);
  //input.first_guess[0] = pr_ch_x;
  //input.first_guess[1] = pr_ch_y;
  //input.first_guess[2] = pr_ch_a;
	//ROS_INFO("1st Guess : px = %.3f. py = %.3f. pa = %.3f",input.first_guess[0],input.first_guess[1],input.first_guess[2]);
  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output.cov_x_m)
  {
		//ROS_INFO("Here X0");
    gsl_matrix_free(output.cov_x_m);
    output.cov_x_m = 0;
  }
  if (output.dx_dy1_m)
  {
		//ROS_INFO("Here X1");
    gsl_matrix_free(output.dx_dy1_m);
    output.dx_dy1_m = 0;
  }
  if (output.dx_dy2_m)
  {
		//ROS_INFO("Here X2");
    gsl_matrix_free(output.dx_dy2_m);
    output.dx_dy2_m = 0;
  }

  // ***  match - using point to line icp from CSM
	//output.valid = 0;
	//output.iterations = 0;
	//output.x[0]= 1.0;
	//output.x[1]= 2.0;
	//output.x[2]= 0.5;

  sm_icp(&input, &output);
  tf::Transform corr_ch;

  if (output.valid)
  {		
		outdx = output.x[0];
		outdy = output.x[1];
		outda = output.x[2];
		
		//outdy = (output.x[1] + input.first_guess[1]) / 2.0;
		
		// send adjustment to bot_node
		//ROS_INFO("Output : x = %.3f. y = %.3f. angle = %.3f, ite = %d",output.x[0],output.x[1],output.x[2],output.iterations);   								
		//ROS_INFO("Output : x = %.3f. y = %.3f. angle = %.3f",outdx,outdy,outda); 
  }
  else
  {		
		// send error to bot_node. use robot_pose adjustment
    corr_ch.setIdentity();
    ROS_INFO("Error in scan matching");
  }

  double dur = (ros::WallTime::now() - start).toSec() * 1e3;
  //ROS_INFO("Scan matcher total duration: %.1f ms", dur);
}

void ScanMatchAlignment() {
	geometry_msgs::Twist pos;
	double tt;

	if (output.valid) {
		// scan matching successful
		tt = outdx;
		if (tt < 0.0) {
			tt = -tt;
		}
		if (tt > 0.25) {
			pos.linear.z = 10.0;
			pos_pub.publish(pos);
			return;
		}
		tt = outdy;
		if (tt < 0.0) {
			tt = -tt;
		}
		if (tt > 0.25) {
			pos.linear.z = 10.0;
			pos_pub.publish(pos);
			return;
		}
		tt = outda;
		if (tt < 0.0) {
			tt = -tt;
		}
		if (tt > 0.35) {
			pos.linear.z = 10.0;
			pos_pub.publish(pos);
			return;
		}
		//if ( (tx < 0.03) && (ty < 0.03) && (ta < 0.08) ) {
		//	pos.linear.z = 35.5;
		//	pos_pub.publish(pos);
		//	return;
		//}
		pos.linear.z = 35.0;  // scan matching alignment code
		pos.linear.x = outdx; // alignDX
    //pos/angular.x = // alignX. move to vertical
		pos.linear.y = outdy; // alignDY
		//pos/angular.y = // alignY. move to horizontal
		pos.angular.z = outda;  
		  
	} else {
		// scan matching fail. use robot_pose alignment
		pos.linear.z = 10.0;		
	}
	pos_pub.publish(pos);
}


void ScanMatchAlignment2() {
	geometry_msgs::Twist pos;
	double tt;
	double xx,yy,aa;

	if (output.valid) {
		// scan matching successful
		//ROS_INFO("2nd ScanMatch");
		xx = outdx;
		if (xx < 0.0) {
			xx = -xx;
		}
		yy = outdy;
		if (yy < 0.0) {
			yy = -yy;
		}
		aa = outda;
		if (aa < 0.0) {
			aa = -aa;
		}
		//if ( (((xx < 0.2) && (xx > 0.04)) || ((yy < 0.2) && (yy > 0.04))) && (aa < 0.1)) {
		if ( (((xx < 0.2) && (xx > 0.04)) || ((yy < 0.2) && (yy > 0.04))) && (aa > 0.03)) {
			//ROS_INFO("Algin 2nd Time");			
			pos.linear.z = 35.0;  // scan matching alignment code
			pos.linear.x = outdx; // alignDX
    	//pos/angular.x = // alignX. move to vertical
			pos.linear.y = outdy; // alignDY
			//pos/angular.y = // alignY. move to horizontal
			pos.angular.z = outda;			
		} else {
			pos.linear.z = 35.5;
		} 		
	} else {
		pos.linear.z = 35.5;
	}
	pos_pub.publish(pos);
}

void ScanMatchAlignment3() {
	geometry_msgs::Twist pos;
	double tt;
	double xx,yy,aa;

	if (output.valid) {
		// scan matching successful
		//ROS_INFO("3rd ScanMatch");
		aa = outda;
		if (aa < 0.0) {
			aa = -aa;
		}
		//if (aa < 0.1) {
		if (aa > 0.02) {
			//ROS_INFO("Algin 3rd Time");			
			pos.linear.z = 35.0;  // scan matching alignment code
			pos.linear.x = outdx; // alignDX
    	//pos/angular.x = // alignX. move to vertical
			pos.linear.y = outdy; // alignDY
			//pos/angular.y = // alignY. move to horizontal
			pos.angular.z = outda;			
		} else {
			pos.linear.z = 35.5;
		} 		
	} else {
		pos.linear.z = 35.5;
	}
	pos_pub.publish(pos);
}

bool scanMatchServiceCallback(htbot::scanMcmd::Request &req,htbot::scanMcmd::Response &res)
{
	int cmd;

	cmd = req.cmd;
	switch (cmd) {
		case 0: // save reference scan to file
			//ROS_INFO("scan match command 0");
			fp = fopen(req.file.c_str(), "w");
  		if (fp == NULL) {
  			ROS_INFO("I couldn't open reference laser scan file to write.\n");    
  			return true;
  		}
			mstate = 1; 
			res.status = 0;
			break;
		case 1:
			fp = fopen(req.file.c_str(), "r");
  		if (fp == NULL) {
  			ROS_INFO("I couldn't open reference laser scan file to read.\n");    
  			return true;
  		}			
			GLP = req.lp;
			GGP = req.gp;
			mstate = 2;
			res.status = 1;
			break;
		case 2:  // match reference and current scan
			mstate = 3;
			GLP = req.lp;
			GGP = req.gp;
			res.status = 2;
			break;
	}
	return true;
}

void matchLoop() {
	switch (mstate) {
		case 0:
			break;
		case 1:  // catpure laser scan and save into json file
			savescan = true;
			delay_time = ros::Time::now();
			mstate = 10;
			break;
		case 10:
			if ( ros::Time::now() > (delay_time + ros::Duration(0.1)) ) {
				if (!savescan) {
					ld_write_as_json(curr_ldp_scan,fp);
					fclose(fp);	
					mstate = 0;
					ROS_INFO("Saved laser Scan to RefScan File");
				}
			}
			break;
		case 2:  // read laser scan from json file
			ref_ldp_scan = ld_from_json_stream(fp);
			fclose(fp);	
			
			if (!dbRequest(GGP,GLP)) {
				ROS_INFO("Error in getting position of LP");
			}
			
			mstate = 21;
			//ROS_INFO("--------- Match : Read Ref Laser Scan:  nray : %d -------------------",ref_ldp_scan->nrays);
			delay_time = ros::Time::now();
			break;
		case 21: // read laser scan and save a matching LDP
			if ( ros::Time::now() > (delay_time + ros::Duration(0.1)) ) {				
				match = true; 	
				getpose = true;				
				delay_time = ros::Time::now();
				mstate = 22;				
				//ROS_INFO("---------- Match : Read Current laser scan:  nray : %d -------------",prev_ldp_scan->nrays);
			}
			break;
		case 22:
			if ( ros::Time::now() > (delay_time + ros::Duration(0.1)) ) {
				if ((!match) && (!getpose)) {					
					//processScan(ref_ldp_scan,scan_time);
					processScan();
					//ROS_INFO("After Process Scan : px = %.3f. py = %.3f. pa = %.3f.",px,py,pa);	
					ScanMatchAlignment();				
					mstate = 0;
				}
			}
			break;
		case 3:
			if (!dbRequest(GGP,GLP)) {
				ROS_INFO("Error in getting position of LP");
			}
			mstate = 0;
			break;
		case 4:  // read laser scan from json file
			ref_ldp_scan = ld_from_json_stream(fp);
			fclose(fp);	
			
			if (!dbRequest(GGP,GLP)) {
				ROS_INFO("Error in getting position of LP");
			}
			
			mstate = 41;
			//ROS_INFO("Read Ref Laser Scan:  nray : %d",ref_ldp_scan->nrays);
			delay_time = ros::Time::now();
			break;
		case 41: // read laser scan and save a matching LDP
			if ( ros::Time::now() > (delay_time + ros::Duration(0.2)) ) {				
				match = true; 
				getpose = true;		
				delay_time = ros::Time::now();
				mstate = 42;				
				//ROS_INFO("Read Current laser scan:  nray : %d",prev_ldp_scan->nrays);
			}
			break;
		case 42:
			if ( ros::Time::now() > (delay_time + ros::Duration(1.0)) ) {
				if ((!match) && (!getpose)) {					
				//if ((!match)) {					
					processScan();
					ScanMatchAlignment2();				
					mstate = 0;										
				}
			}
			break;
		case 5:  // read laser scan from json file
			ref_ldp_scan = ld_from_json_stream(fp);
			fclose(fp);	
			
			if (!dbRequest(GGP,GLP)) {
				ROS_INFO("Error in getting position of LP");
			}
			
			mstate = 51;
			//ROS_INFO("Read Ref Laser Scan:  nray : %d",ref_ldp_scan->nrays);
			delay_time = ros::Time::now();
			break;
		case 51: // read laser scan and save a matching LDP
			if ( ros::Time::now() > (delay_time + ros::Duration(0.2)) ) {				
				match = true; 
				getpose = true;		
				delay_time = ros::Time::now();
				mstate = 52;				
				//ROS_INFO("Read Current laser scan:  nray : %d",prev_ldp_scan->nrays);
			}
			break;
		case 52:
			if ( ros::Time::now() > (delay_time + ros::Duration(1.0)) ) {
				if ((!match) && (!getpose)) {					
				//if ((!match)) {					
					processScan();
					ScanMatchAlignment3();				
					mstate = 0;										
				}
			}
			break;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "match laser scans");
  ros::NodeHandle n;
	
	savescan = false;
	match = false;
	scan_init = false;
	getpose = false;

	mstate = 0;

	ros::Rate loop_rate(10.0);

	laser_sub= n.subscribe<sensor_msgs::LaserScan>("/refscan",1, laserCallBack);	 //scan /scan_align - org
	scanCmd_sub = n.subscribe<htbot::scanCmd>("scanCmd", 1, scanCMD);  // 
	scan_match_service = n.advertiseService("scanMatch",scanMatchServiceCallback);
	pose_sub = n.subscribe<geometry_msgs::Pose>("/robot_pose", 1, poseCallback);
	web_client = n.serviceClient<htbot::mqueue>("web_cmd");
	pos_pub = n.advertise<geometry_msgs::Twist>("cmd_pos", 1);

	initParams();
	//sleep(2);
	
	
	while (true) {  	  	 	  	
		matchLoop();  
		ros::spinOnce();	
  	loop_rate.sleep();
  }
	

  return 0;
}



