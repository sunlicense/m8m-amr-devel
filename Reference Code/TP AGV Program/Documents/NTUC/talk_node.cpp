/*
 * This node is to play sound 
 */
/* 	History
*		Date Modified : 15.7.2015
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
#include "htbot/move_status.h"
#include "htbot/status.h"
#include "boost/algorithm/string.hpp"
#include <math.h>
#include "htbot/clear.h"
#include "htbot/sound.h"
#include "std_srvs/Empty.h"
#include <actionlib_msgs/GoalID.h>

using namespace std;

ros::Subscriber play_sub;
ros::Publisher cancel_pub;
//#define XU4
// prototype
bool moveflag,revflag;
int movecount;
std::string sdir,homedir;

void publish_cancel()
{
	actionlib_msgs::GoalID gid;
	gid.id = "Cancel Goal";
	gid.stamp = ros::Time::now();
	cancel_pub.publish(gid);
	ROS_INFO("Cancel Goal");
	return;
}


bool sound(const htbot::sound::ConstPtr& msg)
{
	int id;

	id = msg->id;
	//ROS_INFO("Sound ID : %d",id);
	switch (id) {
		case 1:			
			//system("dbus-launch gnome-terminal -x aplay ~/sound/well.wav &");
			system("gnome-terminal -x aplay ~/sound/well.wav &");
			// sdir = "aplay -c 1 -q -t wav "+homedir+"sound/well.wav &";
			// system(sdir.c_str());	
			sleep(3);
			break;
		case 2:		
			system("gnome-terminal -x aplay ~/sound/good_bad_ugly.wav &");
			sleep(9);
			break;
		case 3:
			system("gnome-terminal -x aplay ~/sound/honk.wav &");
			sleep(2);
			break;
		case 4:
			system("gnome-terminal -x aplay ~/sound/navigation.wav &");
			sleep(3);
			break;
		case 5:
			system("gnome-terminal -x aplay ~/sound/mapping.wav &");
			sleep(3);
			break;
		case 6:
			system("gnome-terminal -x aplay ~/sound/odom.wav &");
			sleep(3);
			break;
		case 7:
			system("gnome-terminal -x aplay ~/sound/obstacle.wav &");
			sleep(3);
			break;
		case 8:
			system("gnome-terminal -x aplay ~/sound/moving.wav &");
			sleep(2);
			break;
		case 9:
			system("gnome-terminal -x aplay ~/sound/clear.wav &");
			sleep(1);
			break;
		case 10:
			system("gnome-terminal -x aplay ~/sound/load.wav &");
			sleep(1);
			break;
		case 11:
			system("gnome-terminal -x aplay ~/sound/unload.wav &");
			sleep(1);
			break;
		case 12:
			system("gnome-terminal -x aplay ~/sound/crash_x.wav &");
			sleep(2);
			break;
		case 13:
			system("gnome-terminal -x aplay ~/sound/emsw.wav &");
			publish_cancel();
			sleep(2);
			break;
		case 14:  // robot need space
			if (moveflag) {
 				sdir = homedir+"nspace.sh &";
				system(sdir.c_str());
				// system("/home/rac-tprf/nspace.sh");
			} else {
				system("gnome-terminal -x aplay ~/sound/needspace.wav &");				
			}	
			sleep(2);		
			break;
		case 15:  // buzzer_x 
			//system("gnome-terminal -x aplay ~/sound/buzzer_x.wav &");
			//system("/home/rac/reverse.sh");
			if (!revflag) {
    				sdir = "gnome-terminal -x "+homedir+"reverse.sh &";
				system(sdir.c_str());
				// system("gnome-terminal -x /home/rac-tprf/reverse.sh &");
				revflag = true;
			}
			//sleep(2);
			break;
		case 16:  // buzzer_x 
			system("gnome-terminal -x aplay ~/sound/Batlow.wav &");
			sleep(2);
			break;
		case 17:  // nav abort 
			system("gnome-terminal -x aplay ~/sound/Abort.wav &");
			sleep(2);
			//system("gnome-terminal -x /home/rac/ssmcend.sh &");
			break;
		case 18:  // estop releaseed 
			system("gnome-terminal -x aplay ~/sound/estoprel.wav &");
			sleep(2);
			break;
		case 19:  // move music 
			moveflag = true;
			//system("/home/rac/ssmc.sh");
			sdir = "gnome-terminal -x "+homedir+"ssmc.sh &";
			system(sdir.c_str());
			// system("gnome-terminal -x /home/rac-tprf/ssmc.sh &");
			//ROS_INFO("done 19");
			//sleep(1);
			break;
		case 20:  // battery low 
			system("gnome-terminal -x aplay ~/sound/Batlow.wav &");
			sleep(2);
			break;
		case 21:  // explosion. Error 
			system("gnome-terminal -x aplay ~/sound/explosion_x.wav &");
			sleep(2);
			break;
		case 22:  // request to open Lobby Door
			system("gnome-terminal -x aplay ~/sound/LDoor.wav &");
			sleep(2);
			break;
		case 23:  // Lobby Door Opened
			system("gnome-terminal -x aplay ~/sound/LDoorOpened.wav &");
			sleep(2);
			break;
		case 24:  // Request for Lift
			system("gnome-terminal -x aplay ~/sound/AskLift.wav &");
			sleep(2);
			break;
		case 25:  // Lift Door Opened
			system("gnome-terminal -x aplay ~/sound/LIftOpened.wav &");
			sleep(2);
			break;
		case 26:  // Move into Lift
			system("gnome-terminal -x aplay ~/sound/MoveIntoLift.wav &");
			sleep(2);
			break;
		case 27:  // Move out of Lift
			system("gnome-terminal -x aplay ~/sound/MoveOutOfLift.wav &");
			sleep(2);
			break;
		case 28:  // Shutdown
			system("gnome-terminal -x aplay ~/sound/Shutdown.wav &");
			sleep(2);
			break;
		case 29: // Lobby Door Cmd Error
			system("gnome-terminal -x aplay ~/sound/LobbyDoorCmdError.wav &");
			sleep(2);
			break;
		case 30: // Lobby Door Controller Error
			system("gnome-terminal -x aplay ~/sound/LobbyDoorControlError.wav &");
			sleep(2);
			break;
		case 89:  // stop reverse 
			//moveflag = false;
			//movecount = 99;
			//system("/home/rac/reverseend.sh");
			if (revflag) {
				sdir = "gnome-terminal -x "+homedir+"reverseend.sh &";
				system(sdir.c_str());
				// system("gnome-terminal -x /home/rac-tprf/reverseend.sh &");
				revflag = false;
			}
			break;
		case 99:  // kill aplay 
			moveflag = false;
			//movecount = 99;
			//ROS_INFO("exec ssmcend.sh");
			//system("/home/rac/ssmcend.sh");
			sdir = "gnome-terminal -x "+homedir+"ssmcend.sh &";
			system(sdir.c_str());
			//system("gnome-terminal -x /home/rac-tprf/ssmcend.sh &");
			//sleep(1);
			break;
	}
		
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Sound Server");
  ros::NodeHandle n;

  moveflag = false;
  revflag = false;
  // n.getParam("home_dir",homedir);
  n.param<std::string>("home_dir",homedir,"/home/rac-tprf/");	
  play_sub = n.subscribe<htbot::sound>("sound", 1, sound);  // 
  cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base_nys/cancel",1);

  ros::spin();

  return 0;
}



