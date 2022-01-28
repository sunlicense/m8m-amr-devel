/*
 * Node to detect tag using camera
 */

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16.h>
#include <time.h>

using namespace cv;
using namespace std;

int nImageWidth;
int nImageHeight;
Ptr<aruco::Dictionary> dictionary;
Mat curImage;

#define TAG_SIZE 0.02			// Tag size 2cm (0.02m)
#define TAG_ID	70				// Using Aruco Index ID 70

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
		curImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e)
  {
  	ROS_ERROR("cv_bridge exception: %s", e.what());
  	return;
  }
}

void visionCb(const std_msgs::Int16& msg)
{
	// New vision task requested for
	// Clone the image Mat from the global image buffer
	Mat newImg = curImage.clone(); 

	// Saving a copy of the image to check
	if (!newImg.empty())
	{	
		time_t timenow = time(0);
		tm *ltm = localtime(&timenow);
		char buffer[255];
		strftime(buffer,255,"/home/rac/catkin_ws/src/htbot/data/tag_images/test_image_%F:%H:%M:%S.jpg",ltm);
		puts(buffer);
		imwrite(buffer,newImg);
	}

	// Look for the aruco markers
	bool bMarkerFound = false;

  vector<int> ids;
  vector<vector<Point2f> > corners;
  aruco::detectMarkers(newImg, dictionary, corners, ids);
	ROS_INFO("----------------- Activated Detection of  Tag --------------------");
	// Check if aruco marker found and the index (by right should only have one or none marker)
	int index = -1;
	for (int i = 0; i < ids.size(); i++)
	{
		if (ids[i] == 70)
		{
			bMarkerFound = true;
			index = i;
			break;
		}
	}

	if (bMarkerFound == false)
	{
		cout << "Unable to find marker" << endl;
		// aruco marker not found
		ros::param::set("vision_valid", -1);
	}
	else
	{
		// aruco marker found
		// sort the corner points according to pixel y-coordinate
		vector<Point2f> sortedPoints;
		for (int j = 0; j < 4; j++)
		{
			sortedPoints.push_back(corners[index][j]);
			
		}

		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3 - j; k++)
			{
					if (sortedPoints[k].y > sortedPoints[k+1].y)
					{
							// Swap element k and k+1
							Point2f temp;
							temp = sortedPoints[k];
							sortedPoints[k] = sortedPoints[k+1];
							sortedPoints[k+1] = temp;
					}
				
			}
		} 

		// Calculate the angle to turn and dx and dy to move robot under and align to tag
		double move_angle;
		double move_dx;
		double move_dy;

		// Use the last 2 points of the sorted points to find the angle to turn
		double deltaX = sortedPoints[3].x - sortedPoints[2].x;
		double deltaY = sortedPoints[3].y - sortedPoints[2].y; 
		double tiltAngle = atan(deltaY/deltaX);

		// The angle to turn to align robot to tag in degrees
		move_angle = tiltAngle*180.0/CV_PI;

		// Find the rectangle/square enclosing the 4 corner points
		RotatedRect tagRect = minAreaRect(corners[index]);

		// Tag must be squareish so if difference between width and height more than 10% then reject and report invalid
		if (abs(tagRect.size.width-tagRect.size.height)/tagRect.size.width > 0.1)
		{
			cout << "Invalid marker" << endl;
			ros::param::set("vision_valid", -1);
			return;
		}
		// Calculate the x and y offsets of the robot from the centre of the tag
		double tagSizeinPixels = (tagRect.size.width+tagRect.size.height)/2;
		double xOffSet = (newImg.rows/2 - tagRect.center.y)*TAG_SIZE/tagSizeinPixels;
		double yOffSet = (newImg.cols/2 - tagRect.center.x)*TAG_SIZE/tagSizeinPixels;
		
		move_dx = -xOffSet;
		move_dy = yOffSet;

		cout << "Robot movement to move under and align to tag: dx: " << move_dx << " dy: " << move_dy << " angle: " << move_angle << endl;

		// Update the detection results in the ros parameters server
		ros::param::set("vision_angle", move_angle);
		ros::param::set("vision_dx", move_dx);
		ros::param::set("vision_dy", move_dy);

		ros::param::set("vision_valid", 1);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detect_tag");
	ros::NodeHandle n;

	// Load aruco 4x4 dictionary
	dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);

	// Subscribe to usb_cam publisher
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub;
	image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCb);

	// Subscribe to vision publisher to wait for vision task
	ros::Subscriber sub = n.subscribe("vision", 1, visionCb);
	ROS_INFO("------------ detectTag Node : started -----------------");

	ros::spin();
  
	return 0;
}
