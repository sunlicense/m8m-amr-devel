/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */



#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define MIN_ANGLE 0.0f
#define MAX_ANGLE 90.0f
#define NODE_COUNT 360
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.)

double angle_min, angle_max, Dist2Center,dist_min;
double detection_range, detection_height;
int noscan;

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan);


void publish_scan(ros::Publisher *pub, 
                  rplidar_response_measurement_node_t *nodes, 
                  size_t node_count, ros::Time start,
                  double scan_time, bool inverted, 
                  float angle_min, float angle_max, 
                  std::string frame_id)
{
  static int scan_count = 0;
  sensor_msgs::LaserScan scan_msg;
	size_t node_count1;
	double refr,dist,an,rn,rd,cc;
	int idx;
	node_count1 = 90;
  scan_msg.header.stamp = start;
  scan_msg.header.frame_id = frame_id;
 	scan_count++;

  scan_msg.angle_min = angle_min;
 	scan_msg.angle_max = angle_max;
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count1-1);// node_count

  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (double)(node_count1-1);// node_count

  scan_msg.range_min = 0.05;
  scan_msg.range_max = 6.;

 	scan_msg.ranges.resize(node_count1);
	
	for (size_t i = 0; i < node_count1 ; i++) {
		//scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
		scan_msg.ranges[i] = 6.0;
	}

/*
	scan_msg.intensities.resize(node_count1);
  for (size_t i = 0; i < node_count1; i++) {
  	scan_msg.intensities[i] = (float)0;
  }
*/

 	if (!inverted) { // assumes scan window at the top
		for (int i = 0; i < noscan ; i++) {
    	float read_value = (float) nodes[i].distance_q2/4.0f/1000;
      if (read_value != 0.0) {
				//rd = DEG2RAD(i);
				//cc = cos(rd);
				//cc = sin(rd);
				refr = detection_range / cos(DEG2RAD(i));		
				//refr = detection_height / sin(DEG2RAD(i));			
				if (read_value < refr) {					
					dist = read_value * cos(DEG2RAD(i));					
					an = atan(Dist2Center / dist);
					rn = Dist2Center / sin(an);
					idx = (int) RAD2DEG(an);
					if (dist < dist_min) {
						continue;
					}
					scan_msg.ranges[idx] = rn;
					//scan_msg.intensities[idx] = (float)160;
				}				
			}
		}		
		/*
		for (int i = noscan; i < node_count1 ; i++) {
    	float read_value = (float) nodes[i].distance_q2/4.0f/1000;
      if (read_value != 0.0) {
				//rd = DEG2RAD(i);
				//cc = cos(rd);
				//cc = sin(rd);
				//refr = detection_range / cos(DEG2RAD(i));		
				refr = detection_height / sin(DEG2RAD(i));			
				if (read_value < refr) {					
					dist = read_value * cos(DEG2RAD(i));
					an = atan(Dist2Center / dist);
					rn = Dist2Center / sin(an);
					idx = (int) RAD2DEG(an);
					if (dist < dist_min) {
						continue;
					}
					scan_msg.ranges[idx] = rn;
					//scan_msg.intensities[idx] = (float)160;
				}				
			}
		}
		*/
	} else {
		for (int i = 0; i < noscan; i++) {
    	float read_value = (float)nodes[359-i].distance_q2/4.0f/1000;
      if (read_value != 0.0) {
				//rd = DEG2RAD(i);
				//cc = cos(rd);
				refr = detection_range / cos(DEG2RAD(i));		
				//refr = detection_height / sin(DEG2RAD(i));			
				if (read_value < refr) {	
					//ROS_INFO("i=%d, value=%.3f. refr=%.3f",i,read_value,refr);
					dist = read_value * cos(DEG2RAD(i));
					an = atan(dist / Dist2Center);
					rn = dist / sin(an);
					idx = (int) RAD2DEG(an);
					if (dist < dist_min) {
						continue;
					}
					scan_msg.ranges[idx] = rn;
					//scan_msg.intensities[idx] = (float)160;
				}				
			}
    }
    //for (int i = 0; i < noscan; i++) {
		/*
		for (int i = noscan; i < node_count1 ; i++) {
    	float read_value = (float)nodes[359-i].distance_q2/4.0f/1000;
      if (read_value != 0.0) {
				//rd = DEG2RAD(i);
				//cc = cos(rd);
				//refr = detection_range / cos(DEG2RAD(i));		
				refr = detection_height / sin(DEG2RAD(i));			
				if (read_value < refr) {	
					//ROS_INFO("i=%d, value=%.3f. refr=%.3f",i,read_value,refr);
					dist = read_value * cos(DEG2RAD(i));
					an = atan(dist / Dist2Center);
					rn = dist / sin(an);
					idx = (int) RAD2DEG(an);
					if (dist < dist_min) {
						continue;
					}
					scan_msg.ranges[idx] = rn;
					//scan_msg.intensities[idx] = (float)160;
				}				
			}
    }		
		*/
 	}
	
/*
  scan_msg.intensities.resize(node_count1);
  for (size_t i = 0; i < node_count1; i++) {
  	scan_msg.intensities[i] = (float)0;
  }
*/
	
  pub->publish(scan_msg);
	
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
	u_result     op_result;
  rplidar_response_device_health_t healthinfo;

  op_result = drv->getHealth(healthinfo);
  if (IS_OK(op_result)) { 
  	printf("RPLidar health status : %d\n", healthinfo.status);
        
    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
    	fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
      	return false;
    } else {
        return true;
    }

	} else {
  	fprintf(stderr, "Error, cannot retrieve rplidar health code: %x\n", op_result);
    	return false;
 	}
}

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "laserfilter_node");

 	std::string serial_port;
  int serial_baudrate = 115200;
  std::string frame_id;
  bool inverted = false;
  bool angle_compensate = true;
	

  ros::NodeHandle n;
	ros::Subscriber laser_sub= n.subscribe<sensor_msgs::LaserScan>("/scan_align",1, laserCallBack);
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 2000);

 	// start scan...
  drv->startScan();
  ros::Time start_scan_time;
  ros::Time end_scan_time;
  double scan_duration;
  while (ros::ok()) {
  	rplidar_response_measurement_node_t nodes[NODE_COUNT*2];
    size_t   count = _countof(nodes);
		
    start_scan_time = ros::Time::now();
    op_result = drv->grabScanData(nodes, count);
    end_scan_time = ros::Time::now();
    scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;
		
    if (op_result == RESULT_OK) {
    	op_result = drv->ascendScanData(nodes, count);
      //float angle_min = DEG2RAD(0.0f);
			//float angle_min = DEG2RAD(MIN_ANGLE);
      //float angle_max = DEG2RAD(359.0f);
			//float angle_max = DEG2RAD(MAX_ANGLE);
			
     	if (op_result == RESULT_OK) {
      	if (angle_compensate) {
        	const int angle_compensate_nodes_count = NODE_COUNT;
        	const int angle_compensate_multiple = 1;
          int angle_compensate_offset = -3;
          rplidar_response_measurement_node_t angle_compensate_nodes[angle_compensate_nodes_count];
          memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_t));
					
          int i = 0, j = 0;
          for( ; i < count; i++ ) {
          	if (nodes[i].distance_q2 != 0) {
            	float angle = (float)((nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
              int angle_value = (int)(angle * angle_compensate_multiple);
              if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;							
              for (j = 0; j < angle_compensate_multiple; j++) {
              	angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
              }
          	}
         	}  					
          publish_scan(&scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                        start_scan_time, scan_duration, inverted,  
                        angle_min, angle_max,frame_id);
     		} else {
        	int start_node = 0, end_node = 0;
          int i = 0;
          // find the first valid node and last valid node
          while (nodes[i++].distance_q2 == 0);
          start_node = i-1;
          i = count -1;
          while (nodes[i--].distance_q2 == 0);
          end_node = i+1;

          angle_min = DEG2RAD((float)(nodes[start_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
          angle_max = DEG2RAD((float)(nodes[end_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

          publish_scan(&scan_pub, &nodes[start_node], end_node-start_node +1, 
                       start_scan_time, scan_duration, inverted,  
                       angle_min, angle_max,frame_id);
      	}
  		} else if (op_result == RESULT_OPERATION_FAIL) {
      	// All the data is invalid, just publish them
        //float angle_min = DEG2RAD(0.0f);
        //float angle_max = DEG2RAD(359.0f);				
				//float angle_min = DEG2RAD(MIN_ANGLE);            
				//float angle_max = DEG2RAD(MAX_ANGLE);

        publish_scan(&scan_pub, nodes, count, start_scan_time, scan_duration, inverted,  
                     angle_min, angle_max, frame_id);
     	}
   	}

    ros::spinOnce();
	}

  // done!
  RPlidarDriver::DisposeDriver(drv);
  return 0;
}
