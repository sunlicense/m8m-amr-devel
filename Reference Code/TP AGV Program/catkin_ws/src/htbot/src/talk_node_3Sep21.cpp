/*
 * This node is to play sound 
 */
/* 	History
*		Date Modified : 15.7.2015
*		Changes :
		9.8.21
			1. 10am : remove sleep 
		11.8.21
			1. 10.30am : adjusted the delay
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

#define MAXSOUND 300
#define LOOPRATE 10
#define LOOPRATEX 10
#define HALFSEC 5
#define SEC1	LOOPRATEX * 1
#define SEC1_2	HALFSEC * 3
#define SEC2	LOOPRATEX * 2
#define SEC2_2	HALFSEC * 5
#define SEC3	LOOPRATEX * 3
#define SEC3_2	HALFSEC * 7
#define SEC4	LOOPRATEX * 4
#define SEC4_2	HALFSEC * 9

using namespace std;

ros::Subscriber play_sub;

// prototype
void init();
bool sound(const htbot::sound::ConstPtr& msg);
void playsound(void);
void play(int id);

double restartdelay,startdelay;
int delaycount,delay;
int storage[MAXSOUND];
int cidx,sidx,nidx;
bool ssmcactive;


void init()
{
	cidx = 0;
	sidx = 0;
	nidx = 0;	
	delaycount = 0;
	delay = 0;
	ssmcactive = false;
}

bool sound(const htbot::sound::ConstPtr& msg)
{
	int id;
	id = (int)msg->id;
	storage[sidx++] = id;
	nidx++;
	if (sidx == MAXSOUND) {
		sidx = 0;
	}
}

void playsound(void)
{
	int id;

	if (nidx > 0) {
		id = storage[cidx++];
		if (cidx == MAXSOUND) {
			cidx = 0;
		}
		nidx--;
		play(id);
	}
}

void play(int id) {

	int ret;

	//restartdelay = (double)msg->restartdelay;
	//startdelay = (double)msg->startdelay;
	ROS_INFO("---------- TalkNode Sound ID : %d. -----------------",id);
	switch (id) {
		case SYSREADY:			//1
			ret = system("aplay ~/sound/SysReady.wav &");	
			delay = SEC4_2;			
			break;
		case NEEDSPACE:		//2
			ret = system("aplay ~/sound/needspace.wav &");		
			delay = SEC2;					
			break;
		case NAVABORT: //3
			ret = system("aplay ~/sound/NavAbort.wav &");	
			delay = SEC3;			
			break;
		case ESTOPREL: //4
			ret = system("aplay ~/sound/EstopRel.wav &");
			delay = SEC3;			
			break;
		case BATLOW: //5
			ret = system("aplay ~/sound/Batterylow.wav &");
			delay = SEC3;			
			break;		
		case ESTOP: //6
			ret = system("aplay ~/sound/Estop.wav &");
			delay = SEC3;					
			break;
		case REACHED://7
			ret = system("aplay ~/sound/Reached.wav &");
			delay = SEC3;			
			break;
		case FINDPATH://8
			ret = system("aplay ~/sound/FindPath.wav &");	
			delay = SEC3;		
			break;
		case MOVEOUTDOCKSTATION://9
			ret = system("aplay ~/sound/MoveOutDockStation.wav &");	
			delay = SEC3;
			break;
		case MOVETODOCK://10
			ret = system("aplay ~/sound/MoveToDock.wav &");
			delay = SEC3;
			break;
		case NEXTJOB://11
			ret = system("aplay ~/sound/NextJob.wav &");	
			delay = SEC3;
			break;
		case RETURNTOCHARGE://12
			ret = system("aplay ~/sound/ReturnToCharge.wav &");	
			delay = SEC4;
			break;
		case SHUTDOWN://13
			ret = system("aplay ~/sound/Shutdown.wav &");	
			delay = SEC3;
			break;
		case STOCKER://14
			ret = system("aplay ~/sound/Stocker.wav &");
			delay = SEC2;
			break;
		case WAFESTART://15
			ret = system("aplay ~/sound/WafeStart.wav &");
			delay = SEC2;
			break;
		case SSMC://16
			ssmcactive = true;
			ret = system("~/agv.sh &");
			delay = SEC1;			
			break;
		case DETECTCURRENT://17
			ret = system("aplay ~/sound/ChargingCurrent.wav &");		
			delay = SEC2_2;		
			break;  
		case PREMPTED://18
			ret = system("aplay ~/sound/crash_x.wav &");
			delay = SEC2;				
			break;
		case LOSTLOCALISE://19
			ret = system("aplay ~/sound/LostLocal.wav &");
			delay = SEC3;		
			break;
		case RELOCALISE://20
			ret = system("aplay ~/sound/ReLocalise.wav &");	
			delay = SEC3;		
			break;
		case LOCALISE://21
			ret = system("aplay ~/sound/Localised.wav &");
			delay = SEC2_2;		
			break;
		case CHECKLOCALISE://22
			ret = system("aplay ~/sound/CheckLocalization.wav &");	
			delay = SEC3;				
			break;
		case LOCALISEREADY://23
			ret = system("aplay ~/sound/LocalizedReady.wav &");
			delay = SEC3;		
			break;
		case MAPPINGMODE://24
			ret = system("aplay ~/sound/MappingModules.wav &");	
			delay = SEC3;			
			break;
		case NAVIGATIONMODE://25
			ret = system("aplay ~/sound/NavigationModules.wav &");
			delay = SEC4;			
			break;
		case SYSLOADED://26
			ret = system("aplay ~/sound/SysReady.wav &");
			delay = SEC4_2;			
			break;
		case ZEROING://27
			ret = system("aplay ~/sound/StartLocalisation.wav &");
			delay = SEC3;			
			break;
		case RESTORELOCALISATION://28
			ret = system("aplay ~/sound/RestoreLocalisation.wav &");
			delay = SEC4_2;		
			break;
		case SICKLS://29
			ret = system("aplay ~/sound/SickLS.wav &");	
			delay = SEC2;			
			break;
		case RIGHTRS://30
			ret = system("aplay ~/sound/RightRS.wav &");	
			delay = SEC2;			
			break;
		case NETWORK://31
			ret = system("aplay ~/sound/Network.wav &");	
			delay = SEC2;				
			break;
		case MOTOROK://32
			ret = system("aplay ~/sound/MotorOk.wav &");
			delay = SEC2;		
			break;
		case LEFTRS://33
			ret = system("aplay ~/sound/LeftRS.wav &");	
			delay = SEC2;		
			break;
		case LS3D://34
			ret = system("aplay ~/sound/ls3D.wav &");
			delay = SEC2;		
			break;
		case REARSICKON://35
			ret = system("aplay ~/sound/RearSickOn.wav &");	
			delay = SEC2;			
			break;
		case REARSICKOFF://36
			ret = system("aplay ~/sound/RearSickOff.wav &");	
			delay = SEC3;				
			break;
		case FRONTSICKOFF://37
			ret = system("aplay ~/sound/FrontSickOff.wav &");
			delay = SEC3;			
			break;
		case FRONTSICKON://38
			ret = system("aplay ~/sound/FrontSickOn.wav &");
			delay = SEC2;				
			break;
		case RIGHTRSOFF://39 
			ret = system("aplay ~/sound/RightRSOff.wav &");
			delay = SEC4;			
			break;
		case REARHDON://40
			ret = system("aplay ~/sound/RearHDOn.wav &");
			delay = SEC3;			
			break;
		case REARHDOFF://41
			ret = system("aplay ~/sound/RearHDOff.wav &");	
			delay = SEC4;			
			break;		
		case MOTOROFF://42
			ret = system("aplay ~/sound/MotorOff.wav &");
			delay = SEC3;				
			break;				
		case MIDDLERSON://43
			ret = system("aplay ~/sound/MiddleRSOn.wav &");	
			delay = SEC3;			
			break;
		case MIDDLERSOFF://44
			ret = system("aplay ~/sound/MiddleRSOff.wav &");
			delay = SEC3;			
			break;
		case LS3DOFF://45
			ret = system("aplay ~/sound/LS3DOff.wav &");
			delay = SEC3;			
			break;
		case LEFTRSOFF://46
			ret = system("aplay ~/sound/LeftRSOff.wav &");
			delay = SEC4;			
			break;
		case FRONTHDON://47
			ret = system("aplay ~/sound/FrontHDOn.wav &");	
			delay = SEC3;			
			break;
		case FRONTHDOFF://48
			ret = system("aplay ~/sound/FrontHDOff.wav &");	
			delay = SEC4;			
			break;				
		case BOTTOMRSON://49
			ret = system("aplay ~/sound/BottomRSOn.wav &");	
			delay = SEC3;		
			break;
		case BOTTOMRSOFF://50
			ret = system("aplay ~/sound/BottomRSOff.wav &");
			delay = SEC3;			
			break;				
		case ALLSYSTEMON://51
			ret = system("aplay ~/sound/AllSystemOn.wav &");	
			delay = SEC2;		
			break;	
		case LOGOFF: //52
			ret = system("aplay ~/sound/Logoff.wav &");	
			delay = SEC3;			
			break;
		case ASKLIFT: //53
			ret = system("aplay ~/sound/AskLift.wav &");	
			delay = SEC2;				
			break;
		case LIFTOPENED://54
			ret = system("aplay ~/sound/LIftOpened.wav &");	
			delay = SEC2;		
			break;
		case MOVEINTOLIFT://55
			ret = system("aplay ~/sound/MoveIntoLift.wav &");	
			delay = SEC2;		
			break;
		case MOVEOUTOFLIFT://56
			ret = system("aplay ~/sound/MoveOutOfLift.wav &");
			delay = SEC2;			
			break;
		case LOBBYDOORCMDERROR://57
			ret = system("aplay ~/sound/LobbyDoorCmdError.wav &");	
			delay = SEC3;			
			break;
		case LOBBYDOORCONTROLERROR://58
			ret = system("aplay ~/sound/LobbyDoorControlError.wav &");	
			delay = SEC3;			
			break;
		case REQUESTLOBYDOOROPEN://59
			ret = system("aplay ~/sound/RequestLobbyDoorOpen.wav &");	
			delay = SEC3;			
			break;
		case LOBBYDOOROPENED://60
			ret = system("aplay ~/sound/LobbyDoorOpened.wav &");
			delay = SEC2;			
			break;
		case STOPCLEAN://61
			ret = system("aplay ~/sound/StopClean.wav &");	
			delay = SEC3;			
			break;
		case RESUMECLEAN://62
			ret = system("aplay ~/sound/ResumClean.wav &");	
			delay = SEC3;		
			break;
		case PAUSECLEAN://63
			ret = system("aplay ~/sound/PauseClean.wav &");	
			delay = SEC3;				
			break;
		case MOTORRELEASED://64
			ret = system("aplay ~/sound/MotorReleased.wav &");	
			delay = SEC3;			
			break;
		case MOTORENGAGED://65
			ret = system("aplay ~/sound/MotorEngaged.wav &");	
			delay = SEC3;		
			break;
		case CLEANING://66
			ret = system("aplay ~/sound/Cleaning.wav &");
			delay = SEC3;				
			break;
		case CLEANREPORT://67
			ret = system("aplay ~/sound/CleanReport.wav &");	
			delay = SEC3;				
			break;
		case CLEANCOMPLETED://68
			ret = system("aplay ~/sound/CleanCompleted.wav &");	
			delay = SEC3;		
			break;
		case SYSTEMBACKUP://69
			ret = system("aplay ~/sound/SystemBackup.wav &");	
			delay = SEC3;	
			break;
		case LIFTARRIVED://70
			delay = SEC3;
			ret = system("aplay ~/sound/LiftArrived.wav &");		
			break;
		case STOPMOVESOUND:  // kill aplay 
			if (ssmcactive) {
				ret = system("~/agvend.sh");
				delay = SEC1;	
				ssmcactive = false;
			}
			break;
	}
	delaycount = 0;
  return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Sound Server");
  ros::NodeHandle n;
	ros::Rate loop_rate(LOOPRATE);
	
	init();
	play_sub = n.subscribe<htbot::sound>("sound", 100, sound);  // 
	
	delaycount = 0;
	while (true) {  	
		delaycount++;
		if (delaycount > delay) {
			//ros::spinOnce();	
			//ROS_INFO("--------- delaycount = %d. delay = %d. ---------",delaycount,delay);
			if (delaycount > 100000) {
				delay = 5;
				delaycount = 0;
			}
			playsound();
		}
		ros::spinOnce();	
  	loop_rate.sleep();
  }
  //ros::spin();

  return 0;
}



