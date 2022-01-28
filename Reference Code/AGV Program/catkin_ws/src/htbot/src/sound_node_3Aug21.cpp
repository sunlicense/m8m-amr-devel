/*
 * This node is to play sound 
 */
/* 	History
*		Date Modified : 15.7.2015
*		Changes :
		28.7.21
			1. 9.55am : try sending command via rosparam.
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
#include "htbot/PLAYSOUND.h"

using namespace std;

ros::Subscriber play_sub;

// prototype
void sound(int id);

int SoundCMD,delaycnt,delay,ret;
bool SoundON,repeat,startcnt,enableSound;

void sound(int id)
{

	ROS_INFO("---------- SoundNode ID : %d.  -----------------",id);
	repeat = false;
	switch (id) {
		case SYSREADY:			//1
			delay = 18;
			ret = system("aplay ~/sound/SysReady.wav &");	
			break;
		case NEEDSPACE:		//2
			delay = 18;
			ret = system("aplay ~/sound/needspace.wav &");		
			break;
		case NAVABORT: //3
			delay = 18;
			ret = system("aplay ~/sound/NavAbort.wav &");
			break;
		case ESTOPREL: //4
			delay = 18;
			ret = system("aplay ~/sound/EstopRel.wav &");	
			break;
		case BATLOW: //5
			delay = 18;
			ret = system("aplay ~/sound/Batterylow.wav &");
			break;		
		case ESTOP: //6
			delay = 18;
			ret = system("aplay ~/sound/Estop.wav &");
			break;
		case REACHED://7
			delay = 18;
			ret = system("aplay ~/sound/Reached.wav &");
			break;
		case FINDPATH://8
			delay = 18;
			ret = system("aplay ~/sound/FindPath.wav &");	
			break;
		case MOVEOUTDOCKSTATION://9
			delay = 18;
			ret = system("aplay ~/sound/MoveOutDockStation.wav &");
			break;
		case MOVETODOCK://10
			ret = system("aplay ~/sound/MoveToDock.wav &");	
			break;
		case NEXTJOB://11
			delay = 18;
			ret = system("aplay ~/sound/NextJob.wav &");
			break;
		case RETURNTOCHARGE://12
			delay = 18;
			ret = system("aplay ~/sound/ReturnToCharge.wav &");
			break;
		case SHUTDOWN://13
			delay = 18;
			ret = system("aplay ~/sound/Shutdown.wav &");
			break;
		case STOCKER://14
			delay = 18;
			ret = system("aplay ~/sound/Stocker.wav &");
			break;
		case WAFESTART://15
			delay = 18;
			ret = system("aplay ~/sound/WafeStart.wav &");	
			break;
		case SSMC://16
			delay = 30;
			repeat = true;
			ret = system("aplay ~/sound/AGVMoving.wav &");			
			break;
		case DETECTCURRENT://17
			delay = 18;
			ret = system("aplay ~/sound/ChargingCurrent.wav &");			
			break;  
		case PREMPTED://18
			delay = 18;
			ret = system("aplay ~/sound/crash_x.wav &");		
			break;
		case LOSTLOCALISE://19
			delay = 18;
			ret = system("aplay ~/sound/LostLocal.wav &");		
			break;
		case RELOCALISE://20
			delay = 18;
			ret = system("aplay ~/sound/ReLocalise.wav &");			
			break;
		case LOCALISE://21
			delay = 18;
			ret = system("aplay ~/sound/Localised.wav &");		
			break;
		case CHECKLOCALISE://22
			delay = 18;
			ret = system("aplay ~/sound/CheckLocalization.wav &");			
			break;
		case LOCALISEREADY://23
			delay = 18;
			ret = system("aplay ~/sound/LocalizedReady.wav &");				
			break;
		case MAPPINGMODE://24
			delay = 18;
			ret = system("aplay ~/sound/MappingModules.wav &");			
			break;
		case NAVIGATIONMODE://25
			delay = 18;
			ret = system("aplay ~/sound/NavigationModules.wav &");			
			break;
		case SYSLOADED://26
			delay = 18;
			ret = system("aplay ~/sound/SysReady.wav &");		
			break;
		case ZEROING://27
			delay = 18;
			ret = system("aplay ~/sound/StartLocalisation.wav &");		
			break;
		case RESTORELOCALISATION://28
			delay = 18;
			ret = system("aplay ~/sound/RestoreLocalisation.wav &");				
			break;
		case SICKLS://29
			delay = 18;
			ret = system("aplay ~/sound/SickLS.wav &");			
			break;
		case RIGHTRS://30
			delay = 18;
			ret = system("aplay ~/sound/RightRS.wav &");			
			break;
		case NETWORK://31
			delay = 18;
			ret = system("aplay ~/sound/Network.wav &");			
			break;
		case MOTOROK://32
			delay = 18;
			ret = system("aplay ~/sound/MotorOk.wav &");		
			break;
		case LEFTRS://33
			delay = 18;
			ret = system("aplay ~/sound/LeftRS.wav &");			
			break;
		case LS3D://34
			delay = 18;
			ret = system("aplay ~/sound/ls3D.wav &");			
			break;
		case REARSICKON://35
			delay = 18;
			ret = system("aplay ~/sound/RearSickOn.wav &");				
			break;
		case REARSICKOFF://36
			delay = 18;
			ret = system("aplay ~/sound/RearSickOff.wav &");				
			break;
		case FRONTSICKOFF://37
			delay = 18;
			ret = system("aplay ~/sound/FrontSickOff.wav &");			
			break;
		case FRONTSICKON://38
			delay = 18;
			ret = system("aplay ~/sound/FrontSickOn.wav &");				
			break;
		case RIGHTRSOFF://39 
			delay = 18;
			ret = system("aplay ~/sound/RightRSOff.wav &");				
			break;
		case REARHDON://40
			delay = 18;
			ret = system("aplay ~/sound/RearHDOn.wav &");		
			break;
		case REARHDOFF://41
			delay = 18;
			ret = system("aplay ~/sound/RearHDOff.wav &");			
			break;		
		case MOTOROFF://42
			delay = 18;
			ret = system("aplay ~/sound/MotorOff.wav &");			
			break;				
		case MIDDLERSON://43
			delay = 18;
			ret = system("aplay ~/sound/MiddleRSOn.wav &");		
			break;
		case MIDDLERSOFF://44
			delay = 18;
			ret = system("aplay ~/sound/MiddleRSOff.wav &");		
			break;
		case LS3DOFF://45
			delay = 18;
			ret = system("aplay ~/sound/LS3DOff.wav &");		
			break;
		case LEFTRSOFF://46
			delay = 18;
			ret = system("aplay ~/sound/LeftRSOff.wav &");			
			break;
		case FRONTHDON://47
			delay = 18;
			ret = system("aplay ~/sound/FrontHDOn.wav &");			
			break;
		case FRONTHDOFF://48
			delay = 18;
			ret = system("aplay ~/sound/FrontHDOff.wav &");			
			break;				
		case BOTTOMRSON://49
			delay = 18;
			ret = system("aplay ~/sound/BottomRSOn.wav &");				
			break;
		case BOTTOMRSOFF://50
			delay = 18;
			ret = system("aplay ~/sound/BottomRSOff.wav &");			
			break;				
		case ALLSYSTEMON://51
			delay = 18;
			ret = system("aplay ~/sound/AllSystemOn.wav &");			
			break;	
		case LOGOFF: //52
			delay = 18;
			ret = system("aplay ~/sound/Logoff.wav &");		
			break;
		case ASKLIFT: //53
			delay = 18;
			ret = system("aplay ~/sound/AskLift.wav &");			
			break;
		case LIFTOPENED://54
			delay = 18;
			ret = system("aplay ~/sound/LIftOpened.wav &");		
			break;
		case MOVEINTOLIFT://55
			delay = 18;
			ret = system("aplay ~/sound/MoveIntoLift.wav &");			
			break;
		case MOVEOUTOFLIFT://56
			delay = 18;
			ret = system("aplay ~/sound/MoveOutOfLift.wav &");			
			break;
		case LOBBYDOORCMDERROR://57
			delay = 18;
			ret = system("aplay ~/sound/LobbyDoorCmdError.wav &");			
			break;
		case LOBBYDOORCONTROLERROR://58
			delay = 18;
			ret = system("aplay ~/sound/LobbyDoorControlError.wav &");			
			break;
		case REQUESTLOBYDOOROPEN://59
			delay = 18;
			ret = system("aplay ~/sound/RequestLobbyDoorOpen.wav &");				
			break;
		case LOBBYDOOROPENED://60
			delay = 18;
			ret = system("aplay ~/sound/LobbyDoorOpened.wav &");		
			break;
		case STOPCLEAN://61
			delay = 18;
			ret = system("aplay ~/sound/StopClean.wav &");					
			break;
		case RESUMECLEAN://62
			delay = 18;
			ret = system("aplay ~/sound/ResumClean.wav &");			
			break;
		case PAUSECLEAN://63
			delay = 18;
			ret = system("aplay ~/sound/PauseClean.wav &");				
			break;
		case MOTORRELEASED://64
			delay = 18;
			ret = system("aplay ~/sound/MotorReleased.wav &");			
			break;
		case MOTORENGAGED://65
			delay = 18;
			ret = system("aplay ~/sound/MotorEngaged.wav &");			
			break;
		case CLEANING://66
			delay = 18;
			ret = system("aplay ~/sound/Cleaning.wav &");			
			break;
		case CLEANREPORT://67
			delay = 18;
			ret = system("aplay ~/sound/CleanReport.wav &");				
			break;
		case CLEANCOMPLETED://68
			delay = 18;
			ret = system("aplay ~/sound/CleanCompleted.wav &");			
			break;
		case SYSTEMBACKUP://69
			delay = 18;
			ret = system("aplay ~/sound/SystemBackup.wav &");		
			break;
		case LIFTARRIVED://70
			delay = 20;
			ret = system("aplay ~/sound/LiftArrived.wav &");		
			break;
		case STOPMOVESOUND:  // kill aplay 
			//ret = system("~/ssmcend.sh");
			repeat = false;
			ROS_INFO("---------- SoundNode Sound : SSMC End ------------");
			break;
	}
	
  return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Sound Server");
  ros::NodeHandle n;
	ros::Rate loop_rate(5.0);
	
	SoundON = false;
	SoundCMD = 0;
	delaycnt = 0;
	startcnt = true;
	enableSound = false;
	delay = 5;
	repeat = false;
	ROS_INFO("----------- Sound Node Started -----------------------");
	while (true) {  
		if (startcnt) {
			delaycnt++;
			if (delaycnt > delay) {
				enableSound = true;
				startcnt = false;
				ROS_INFO("---------- SoundN : sound enabled. ----------------");
				//ret = system("ps aux | grep -i aplay | awk {'print $2'} | xargs kill -9 &");
			}
		}
		if (enableSound) {	
			n.getParam("SoundON",SoundON);
			if (SoundON) {
				SoundON = false;
				n.setParam("SoundON",SoundON);
				n.getParam("SoundCMD",SoundCMD);				
				sound(SoundCMD);
				delaycnt = 0;
				startcnt = true;
				enableSound = false;
				ROS_INFO("---------- SoundN : sound command Recv. ----------------");
			} else {
				if (repeat) {
					n.getParam("SoundCMD",SoundCMD);
					sound(SoundCMD);
					delaycnt = 0;
					startcnt = true;
					enableSound = false;
					ROS_INFO("---------- SoundN : sound repeat. ----------------");
				}
			}			
		}
		//ros::spinOnce();	
  	loop_rate.sleep();
  }
  //ros::spin();

  return 0;
}



