/*
 * This node is to play sound 
 */
/* 	History
*		Date Modified : 15.7.2015
*		Changes :
		25.6.21 : 3.25pm
			1. changed the whole algo to checking for aplay still running
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
void init();
bool sound(const htbot::sound::ConstPtr& msg);

bool sysready,needspace,navabort,estop,estoprel,batlow,reached,ssmc,detectcurrent,prempt,localise,relocalise;
bool findpath,moveoutdockstation,movetodock,nextjob,returntocharge,shutdown,stocker,wafestart,lostlocalise;
bool checklocalise,localiseReady,sysloaded,mappingmode,navigationmode,zeroing,restorelocalisation;
bool SickLS,RightRS,Network,MotorOk,LeftRS,ls3D,Logoff;
bool RearSickOn,RearSickOff,FrontSickOff,FrontSickOn,RightRSOff,RearHDOn,RearHDOff,MotorOff,MiddleRSOn,MiddleRSOff;
bool LS3DOff,LeftRSOff,FrontHDOn,FrontHDOff,BottomRSOn,BottomRSOff,AllSystemOn;
bool AskLift,LIftOpened,MoveIntoLift,MoveOutOfLift,LobbyDoorCmdError,LobbyDoorControlError;
bool RequestLobbyDoorOpen,LobbyDoorOpened;
bool StopClean,ResumClean,PauseClean,MotorReleased,MotorEngaged,Cleaning;
bool CleanReport,CleanCompleted,SystemBackup;
bool AplayRunning;

double sysreadyT,needspaceT,navabortT,estopT,estoprelT,batlowT,reachedT,ssmcT,detectcurrentT,premptT,localiseT,relocaliseT;
double findpathT,moveoutdockstationT,movetodockT,nextjobT,returntochargeT,shutdownT,stockerT,wafestartT,lostlocaliseT;
double checklocaliseT,localiseReadyT,sysloadedT,mappingmodeT,navigationmodeT,zeroingT,restorelocalisationT;
double SickLST,RightRST,NetworkT,MotorOkT,LeftRST,ls3DT,LogoffT;
double RearSickOnT,RearSickOffT,FrontSickOffT,FrontSickOnT,RightRSOffT,RearHDOnT,RearHDOffT,MotorOffT,MiddleRSOnT,MiddleRSOffT;
double LS3DOffT,LeftRSOffT,FrontHDOnT,FrontHDOffT,BottomRSOnT,BottomRSOffT,AllSystemOnT;
double AskLiftT,LIftOpenedT,MoveIntoLiftT,MoveOutOfLiftT,LobbyDoorCmdErrorT,LobbyDoorControlErrorT;
double RequestLobbyDoorOpenT,LobbyDoorOpenedT;
double StopCleanT,ResumCleanT,PauseCleanT,MotorReleasedT,MotorEngagedT,CleaningT;
double CleanReportT,CleanCompletedT,SystemBackupT;

ros::Time sysreadyRT,needspaceRT,navabortRT,estopRT,estoprelRT,batlowRT,reachedRT,ssmcRT,detectcurrentRT,premptRT,localiseRT;
ros::Time findpathRT,moveoutdockstationRT,movetodockRT,nextjobRT,returntochargeRT,shutdownRT,stockerRT,wafestartRT,relocaliseRT;
ros::Time lostlocaliseRT,checklocaliseRT,localiseReadyRT,sysloadedRT,mappingmodeRT,navigationmodeRT,zeroingRT,restorelocalisationRT;
ros::Time SickLSRT,RightRSRT,NetworkRT,MotorOkRT,LeftRSRT,ls3DRT,LogoffRT;
ros::Time RearSickOnRT,RearSickOffRT,FrontSickOffRT,FrontSickOnRT,RightRSOffRT,RearHDOnRT,RearHDOffRT,MotorOffRT,MiddleRSOnRT,MiddleRSOffRT;
ros::Time LS3DOffRT,LeftRSOffRT,FrontHDOnRT,FrontHDOffRT,BottomRSOnRT,BottomRSOffRT,AllSystemOnRT;
ros::Time AskLiftRT,LIftOpenedRT,MoveIntoLiftRT,MoveOutOfLiftRT,LobbyDoorCmdErrorRT,LobbyDoorControlErrorRT;
ros::Time RequestLobbyDoorOpenRT,LobbyDoorOpenedRT;
ros::Time StopCleanRT,ResumCleanRT,PauseCleanRT,MotorReleasedRT,MotorEngagedRT,CleaningRT;
ros::Time CleanReportRT,CleanCompletedRT,SystemBackupRT,playStartRT;

double restartdelay,startdelay;


void init()
{
	AplayRunning = false;
	sysready = false;
	needspace = false;
	navabort = false;
	estop= false;
	estoprel = false;
  batlow = false;
	reached = false;
	ssmc = false;
	detectcurrent = false;
	prempt = false;
	findpath = false;
	moveoutdockstation = false;
	movetodock = false;
	nextjob = false;
	returntocharge = false;
	shutdown = false;
	stocker = false;
	wafestart = false;
	localise = false;
	relocalise = false;
	lostlocalise = false;
	checklocalise = false;
	localiseReady = false;
	sysloaded = false;
	mappingmode = false;
	navigationmode = false;
	zeroing= false;
	restorelocalisation = false;
	SickLS = false;
	RightRS = false; 
	RearHDOn = false;
	Network = false;
  MotorOk = false;
  LeftRS = false;
  FrontHDOn = false;
  ls3D = false;
	RearSickOn = false;
	RearSickOff = false;
  FrontSickOff = false;
  FrontSickOn = false;
  RightRSOff = false; 
  RearHDOn = false;
  RearHDOff = false;
  MotorOff = false;
  MiddleRSOn = false;
  MiddleRSOff = false;
	LS3DOff = false;
  LeftRSOff = false;
	FrontHDOn = false;
	FrontHDOff = false;
	BottomRSOn = false;
	BottomRSOff = false;
 	AllSystemOn = false;
	Logoff = false;
	StopClean = false;
	ResumClean = false;
  PauseClean = false;
	MotorReleased = false;
	MotorEngaged = false;
	Cleaning = false;
	CleanReport = false;
	CleanCompleted = false;
	SystemBackup = false;
	AskLift = false;
	LIftOpened = false;
	MoveIntoLift = false;
	MoveOutOfLift = false;
	LobbyDoorCmdError = false;
	LobbyDoorControlError = false;
}

void playsound() {

}

bool sound(const htbot::sound::ConstPtr& msg)
{
	int id,ret;
	ros::NodeHandle nn;
	id = msg->id;
	restartdelay = (double)msg->restartdelay;
	startdelay = (double)msg->startdelay;
	//ROS_INFO("---------- TalkNode Sound ID : %d. RSD=%.3f -----------------",id,restartdelay);
	switch (id) {
		case SYSREADY:			//1
			if (!sysready) {				
				//sleep(startdelay);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
				ret = system("aplay ~/sound/SysReady.wav &");	
				//sysready = true;						
				//sysreadyT = restartdelay;
				//sleep(4);
				//sysreadyRT = ros::Time::now();
				//sleep(2);
			}
			break;
		case NEEDSPACE:		//2
			if (!needspace) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/needspace.wav &");		
				//ROS_INFO("------ Talk Node : Need Space --------");
				//needspace = true;						
				//needspaceT = restartdelay;
				//sleep(3);
				//needspaceRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}				
			break;
		case NAVABORT: //3
			if (!navabort) {		
				//sleep(startdelay);	
				ret = system("aplay ~/sound/NavAbort.wav &");		
				//ROS_INFO("------ Talk Node : Nav Aborted --------");
				//navabort = true;					
				//navabortT = restartdelay;
				//sleep(3);
				//navabortRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case ESTOPREL: //4
			if (!estoprel) {			
				//sleep(startdelay);	
				ret = system("aplay ~/sound/EstopRel.wav &");		
				//ROS_INFO("------ Talk Node : EStop Released --------");
				//estoprel = true;						
				//estoprelT = restartdelay;
				//sleep(3);
				//estoprelRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case BATLOW: //5
			if (!batlow) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/Batterylow.wav &");		
				//ROS_INFO("------ Talk Node : Battery Low --------");
				//batlow = true;						
				//batlowT = restartdelay;
				//sleep(3);
				//batlowRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}	
			break;		
		case ESTOP: //6
			if (!estop) {				
				//sleep(startdelay);
				ret = system("aplay ~/sound/Estop.wav &");		
				//ROS_INFO("------ Talk Node : EStop Activated --------");
				//estop = true;					
				//estopT = restartdelay;
				//sleep(3);
				//estopRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case REACHED://7
			if (!reached) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/Reached.wav &");		
				//ROS_INFO("------ Talk Node : Reached --------");
				//reached = true;						
				//reachedT = restartdelay;
				//sleep(3);
				//reachedRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case FINDPATH://8
			if (!findpath) {			
				//sleep(startdelay);	
				ret = system("aplay ~/sound/FindPath.wav &");		
				//ROS_INFO("------ Talk Node : FindPath --------");
				//findpath = true;						
				//findpathT = restartdelay;
				//sleep(3);
				//findpathRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case MOVEOUTDOCKSTATION://9
			if (!moveoutdockstation) {	
				//sleep(startdelay);			
				ret = system("aplay ~/sound/MoveOutDockStation.wav &");		
				//ROS_INFO("------ Talk Node : MoveOutDockStation --------");
				//moveoutdockstation = true;						
				//moveoutdockstationT = restartdelay;
				//sleep(3);
				//moveoutdockstationRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case MOVETODOCK://10
			if (!movetodock) {			
				//sleep(startdelay);	
				ret = system("aplay ~/sound/MoveToDock.wav &");		
				//ROS_INFO("------ Talk Node : MoveToDock --------");
				//movetodock = true;					
				//movetodockT = restartdelay;
				//sleep(3);
				//movetodockRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case NEXTJOB://11
			if (!nextjob) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/NextJob.wav &");	
				//ROS_INFO("------ Talk Node : NextJob --------");	
				//nextjob = true;				
				//nextjobT = restartdelay;
				//sleep(3);
				//nextjobRT = ros::Time::now();		
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case RETURNTOCHARGE://12
			if (!returntocharge) {	
				//sleep(startdelay);			
				ret = system("aplay ~/sound/ReturnToCharge.wav &");		
				//ROS_INFO("------ Talk Node : ReturnToCharge --------");
				//returntocharge = true;						
				//returntochargeT = restartdelay;
				//sleep(3);
				//returntochargeRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case SHUTDOWN://13
			if (!shutdown) {	
				//sleep(startdelay);			
				ret = system("aplay ~/sound/Shutdown.wav &");		
				//ROS_INFO("------ Talk Node : Shutdown --------");
				//shutdown = true;						
				//shutdownT = restartdelay;
				//sleep(3);
				//shutdownRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case STOCKER://14
			if (!stocker) {				
				//sleep(startdelay);
				ret = system("aplay ~/sound/Stocker.wav &");		
				//ROS_INFO("------ Talk Node : Stocker --------");
				//stocker = true;						
				//stockerT = restartdelay;
				//sleep(3);
				//stockerRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case WAFESTART://15
			if (!wafestart) {		
				//sleep(startdelay);		
				//ret = system("gnome-terminal -x aplay ~/sound/WafeStart.wav");		
				ret = system("aplay ~/sound/WafeStart.wav &");	
				//ROS_INFO("------ Talk Node : WafeStart --------");
				//wafestart = true;						
				//wafestartT = restartdelay;
				//sleep(1);
				//wafestartRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}
			break;
		case SSMC://16
			if (!ssmc) {		
				//sleep(startdelay);		
				ret = system("~/ssmc.sh &");	
				ssmc = true;						
				//ssmcT = restartdelay;
				//sleep(3);
				//ssmcRT = ros::Time::now();
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case DETECTCURRENT://17
			if (!detectcurrent) {				
				//sleep(startdelay);
				ret = system("aplay ~/sound/ChargingCurrent.wav &");	
				//ROS_INFO("------ Talk Node : ChargingCurrent --------");
				//detectcurrent = true;					
				//detectcurrentT = restartdelay;
				//sleep(3);
				//detectcurrentRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;  
		case PREMPTED://18
			if (!prempt) {			
				//sleep(startdelay);	
				ret = system("aplay ~/sound/crash_x.wav &");	
				//prempt = true;					
				//premptT = restartdelay;
				//sleep(3);
				//premptRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LOSTLOCALISE://19
			if (!lostlocalise) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/LostLocal.wav &");	
				//ROS_INFO("------ Talk Node : LostLocal --------");
				//lostlocalise = true;					
				//lostlocaliseT = restartdelay;
				//sleep(3);
				//lostlocaliseRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case RELOCALISE://20
			if (!relocalise) {	
				//sleep(startdelay);			
				ret = system("aplay ~/sound/ReLocalise.wav &");	
				//ROS_INFO("------ Talk Node : ReLocalise --------");
				//relocalise = true;						
				//relocaliseT = restartdelay;
				//sleep(3);
				//relocaliseRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LOCALISE://21
			if (!localise) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/Localised.wav &");	
				//ROS_INFO("------ Talk Node : Localised --------");
				//localise = true;						
				//localiseT = restartdelay;
				//sleep(3);
				//localiseRT = ros::Time::now();
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case CHECKLOCALISE://22
			if (!checklocalise) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/CheckLocalization.wav &");	
				//ROS_INFO("------ Talk Node : CheckLocalization --------");
				//checklocalise = true;
				//checklocaliseRT = ros::Time::now();		
				//checklocaliseT = restartdelay;
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LOCALISEREADY://23
			if (!localiseReady) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/LocalizedReady.wav &");	
				//ROS_INFO("------ Talk Node : LocalizedReady --------");
				//localiseReady = true;					
				//localiseReadyT = restartdelay;
				//sleep(3);
				//localiseReadyRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case MAPPINGMODE://24
			if (!mappingmode) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/MappingModules.wav &");	
				//ROS_INFO("------ Talk Node : MappingMode --------");
				//mappingmode = true;					
				//mappingmodeT = restartdelay;
				//sleep(3);
				//mappingmodeRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case NAVIGATIONMODE://25
			if (!navigationmode) {			
				//sleep(startdelay);	
				ret = system("aplay ~/sound/NavigationModules.wav &");	
				//ROS_INFO("------ Talk Node : NavigationMode --------");
				//navigationmode = true;					
				//navigationmodeT = restartdelay;
				//sleep(3);
				//navigationmodeRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case SYSLOADED://26
			if (!sysloaded) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/SysReady.wav &");	
				//ROS_INFO("------ Talk Node : SysLoaded --------");
				//sysloaded = true;					
				//sysloadedT = restartdelay;
				//sleep(3);
				//sysloadedRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case ZEROING://27
			if (!zeroing) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/StartLocalisation.wav &");	
				//ROS_INFO("------ Talk Node : Zeroing --------");
				//zeroing = true;					
				//zeroingT = restartdelay;
				//sleep(3);
				//zeroingRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case RESTORELOCALISATION://28
			if (!restorelocalisation) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/RestoreLocalisation.wav &");	
				//ROS_INFO("------ Talk Node : Zeroing --------");
				//restorelocalisation = true;					
				//restorelocalisationT = restartdelay;
				//restorelocalisationRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case SICKLS://29
			if (!SickLS) {			
				ret = system("aplay ~/sound/SickLS.wav &");	
				//SickLS = true;					
				//SickLST = restartdelay;
				//SickLSRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case RIGHTRS://30
			if (!RightRS) {		
				ret = system("aplay ~/sound/RightRS.wav &");	
				//RightRS = true;					
				//RightRST = restartdelay;
				//RightRSRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case NETWORK://31
			if (!Network) {		
				ret = system("aplay ~/sound/Network.wav &");	
				//Network = true;					
				//NetworkT = restartdelay;
				//NetworkRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case MOTOROK://32
			if (!MotorOk) {		
				ret = system("aplay ~/sound/MotorOk.wav &");	
				//MotorOk = true;					
				//MotorOkT = restartdelay;
				//MotorOkRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LEFTRS://33
			if (!LeftRS) {		
				ret = system("aplay ~/sound/LeftRS.wav &");	
				//LeftRS = true;					
				//LeftRST = restartdelay;
				//LeftRSRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LS3D://34
			if (!ls3D) {		
				ret = system("aplay ~/sound/ls3D.wav &");	
				//ls3D = true;					
				//ls3DT = restartdelay;
				//ls3DRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case REARSICKON://35
			if (!RearSickOn) {		
				ret = system("aplay ~/sound/RearSickOn.wav &");	
				//RearSickOn = true;					
				//RearSickOnT = restartdelay;
				//RearSickOnRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case REARSICKOFF://36
			if (!RearSickOff) {		
				ret = system("aplay ~/sound/RearSickOff.wav &");	
				//RearSickOff = true;					
				//RearSickOffT = restartdelay;
				//RearSickOffRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case FRONTSICKOFF://37
			if (!FrontSickOff) {		
				ret = system("aplay ~/sound/FrontSickOff.wav &");	
				//FrontSickOff = true;					
				//FrontSickOffT = restartdelay;
				//FrontSickOffRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case FRONTSICKON://38
			if (!FrontSickOn) {		
				ret = system("aplay ~/sound/FrontSickOn.wav &");	
				//FrontSickOn = true;					
				//FrontSickOnT = restartdelay;
				//FrontSickOnRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case RIGHTRSOFF://39 
			if (!RightRSOff) {		
				ret = system("aplay ~/sound/RightRSOff.wav &");	
				//RightRSOff = true;					
				//RightRSOffT = restartdelay;
				//RightRSOffRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case REARHDON://40
			if (!RearHDOn) {		
				ret = system("aplay ~/sound/RearHDOn.wav &");	
				//RearHDOn = true;					
				//RearHDOnT = restartdelay;
				//RearHDOnRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case REARHDOFF://41
			if (!RearHDOff) {		
				ret = system("aplay ~/sound/RearHDOff.wav &");	
				//RearHDOff = true;					
				//RearHDOffT = restartdelay;
				//RearHDOffRT = ros::Time::now();	
				sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;		
		case MOTOROFF://42
			if (!MotorOff) {		
				ret = system("aplay ~/sound/MotorOff.wav &");	
				//MotorOff = true;					
				//MotorOffT = restartdelay;
				//MotorOffRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;				
		case MIDDLERSON://43
			if (!MiddleRSOn) {		
				ret = system("aplay ~/sound/MiddleRSOn.wav &");	
				//MiddleRSOn = true;					
				//MiddleRSOnT = restartdelay;
				//MiddleRSOnRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case MIDDLERSOFF://44
			if (!MiddleRSOff) {		
				ret = system("aplay ~/sound/MiddleRSOff.wav &");	
				//MiddleRSOff = true;					
				//MiddleRSOffT = restartdelay;
				//MiddleRSOffRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LS3DOFF://45
			if (!LS3DOff) {		
				ret = system("aplay ~/sound/LS3DOff.wav &");	
				//LS3DOff = true;					
				//LS3DOffT = restartdelay;
				//LS3DOffRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LEFTRSOFF://46
			if (!LeftRSOff) {		
				ret = system("aplay ~/sound/LeftRSOff.wav &");	
				//LeftRSOff = true;					
				//LeftRSOffT = restartdelay;
				//LeftRSOffRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case FRONTHDON://47
			if (!FrontHDOn) {		
				ret = system("aplay ~/sound/FrontHDOn.wav &");	
				//FrontHDOn = true;					
				//FrontHDOnT = restartdelay;
				//FrontHDOnRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case FRONTHDOFF://48
			if (!FrontHDOff) {		
				ret = system("aplay ~/sound/FrontHDOff.wav &");	
				//FrontHDOff = true;					
				//FrontHDOffT = restartdelay;
				//FrontHDOffRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;				
		case BOTTOMRSON://49
			if (!BottomRSOn) {		
				ret = system("aplay ~/sound/BottomRSOn.wav &");	
				//BottomRSOn = true;					
				//BottomRSOnT = restartdelay;
				//BottomRSOnRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case BOTTOMRSOFF://50
			if (!BottomRSOff) {		
				ret = system("aplay ~/sound/BottomRSOff.wav &");	
				//BottomRSOff = true;					
				//BottomRSOffT = restartdelay;
				//BottomRSOffRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;				
		case ALLSYSTEMON://51
			if (!AllSystemOn) {		
				ret = system("aplay ~/sound/AllSystemOn.wav &");	
				//AllSystemOn = true;					
				//AllSystemOnT = restartdelay;
				//AllSystemOnRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;	
		case LOGOFF: //52
			if (!Logoff) {		
				ret = system("aplay ~/sound/Logoff.wav &");	
				//Logoff = true;					
				//LogoffT = restartdelay;
				//LogoffRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case ASKLIFT: //53
			if (!AskLift) {		
				ret = system("aplay ~/sound/AskLift.wav &");	
				//AskLift = true;					
				//AskLiftT = restartdelay;
				//AskLiftRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LIFTOPENED://54
			if (!LIftOpened) {		
				ret = system("aplay ~/sound/LIftOpened.wav &");	
				//LIftOpened = true;					
				//LIftOpenedT = restartdelay;
				//LIftOpenedRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case MOVEINTOLIFT://55
			if (!MoveIntoLift) {		
				ret = system("aplay ~/sound/MoveIntoLift.wav &");	
				//MoveIntoLift = true;					
				//MoveIntoLiftT = restartdelay;
				//MoveIntoLiftRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case MOVEOUTOFLIFT://56
			if (!MoveOutOfLift) {		
				ret = system("aplay ~/sound/MoveOutOfLift.wav &");	
				//MoveOutOfLift = true;					
				//MoveOutOfLiftT = restartdelay;
				//MoveOutOfLiftRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LOBBYDOORCMDERROR://57
			if (!LobbyDoorCmdError) {		
				ret = system("aplay ~/sound/LobbyDoorCmdError.wav &");	
				//LobbyDoorCmdError = true;					
				//LobbyDoorCmdErrorT = restartdelay;
				//LobbyDoorCmdErrorRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LOBBYDOORCONTROLERROR://58
			if (!LobbyDoorControlError) {		
				ret = system("aplay ~/sound/LobbyDoorControlError.wav &");	
				//LobbyDoorControlError = true;					
				//LobbyDoorControlErrorT = restartdelay;
				//LobbyDoorControlErrorRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case REQUESTLOBYDOOROPEN://59
			if (!RequestLobbyDoorOpen) {		
				ret = system("aplay ~/sound/RequestLobbyDoorOpen.wav &");	
				//RequestLobbyDoorOpen = true;					
				//RequestLobbyDoorOpenT = restartdelay;
				//RequestLobbyDoorOpenRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case LOBBYDOOROPENED://60
			if (!LobbyDoorOpened) {		
				ret = system("aplay ~/sound/LobbyDoorOpened.wav &");	
				//LobbyDoorOpened = true;					
				//LobbyDoorOpenedT = restartdelay;
				//LobbyDoorOpenedRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case STOPCLEAN://61
			if (!StopClean) {		
				ret = system("aplay ~/sound/StopClean.wav &");	
				//StopClean = true;					
				//StopCleanT = restartdelay;
				//StopCleanRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case RESUMECLEAN://62
			if (!ResumClean) {		
				ret = system("aplay ~/sound/ResumClean.wav &");	
				//ResumClean = true;					
				//ResumCleanT = restartdelay;
				//ResumCleanRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case PAUSECLEAN://63
			if (!PauseClean) {		
				ret = system("aplay ~/sound/PauseClean.wav &");	
				//PauseClean = true;					
				//PauseCleanT = restartdelay;
				//PauseCleanRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case MOTORRELEASED://64
			if (!MotorReleased) {		
				ret = system("aplay ~/sound/MotorReleased.wav &");	
				//MotorReleased = true;					
				//MotorReleasedT = restartdelay;
				//MotorReleasedRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case MOTORENGAGED://65
			if (!MotorEngaged) {		
				ret = system("aplay ~/sound/MotorEngaged.wav &");	
				//MotorEngaged = true;					
				//MotorEngagedT = restartdelay;
				//MotorEngagedRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case CLEANING://66
			if (!Cleaning) {		
				ret = system("aplay ~/sound/Cleaning.wav &");	
				//Cleaning = true;					
				//CleaningT = restartdelay;
				//CleaningRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case CLEANREPORT://67
			if (!CleanReport) {		
				ret = system("aplay ~/sound/CleanReport.wav &");	
				//CleanReport = true;					
				//CleanReportT = restartdelay;
				//CleanReportRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case CLEANCOMPLETED://68
			if (!CleanCompleted) {		
				ret = system("aplay ~/sound/CleanCompleted.wav &");	
				//CleanCompleted = true;					
				//CleanCompletedT = restartdelay;
				//CleanCompletedRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case SYSTEMBACKUP://69
			if (!SystemBackup) {		
				ret = system("aplay ~/sound/SystemBackup.wav &");	
				//SystemBackup = true;					
				//SystemBackupT = restartdelay;
				//SystemBackupRT = ros::Time::now();	
				//sleep(2);
				AplayRunning = true;
				nn.setParam("AplayRunning",AplayRunning);
			}			
			break;
		case STOPMOVESOUND:  // kill aplay 
			//ret = system("~/ssmcend.sh");
			ret = system("killall ssmc.sh");
			ret = system("killall aplay");
			//ROS_INFO("------------ Talk : End SSMC -----------------");
			ssmc = false;
			//ret = system("~/checkAplay.sh &");
			break;
	}
	
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Sound Server");
  ros::NodeHandle n;
	ros::Rate loop_rate(2.0);
	bool aplayStop;
	int ret;
	
	init();
	play_sub = n.subscribe<htbot::sound>("sound", 30, sound);  // 
	AplayRunning = false;
	n.setParam("AplayRunning",AplayRunning);
	ret = system("~/checkAplay.sh &");
	while (true) {  	
		/*
		//ROS_INFO(" Talk Node ");
		if (sysloaded) {  	 	
			if ( ros::Time::now() > (sysloadedRT + ros::Duration(sysloadedT)) ) {
				sysloaded = false;				
			}	
		}
		if (mappingmode) {  	 	
			if ( ros::Time::now() > (mappingmodeRT + ros::Duration(mappingmodeT)) ) {
				mappingmode = false;				
			}	
		}
		if (navigationmode) {  	 	
			if ( ros::Time::now() > (navigationmodeRT + ros::Duration(navigationmodeT)) ) {
				navigationmode = false;				
			}	
		}
		if (zeroing) {  	 	
			if ( ros::Time::now() > (zeroingRT + ros::Duration(zeroingT)) ) {
				zeroing = false;				
			}	
		}
		if (sysready) {  	 	
			if ( ros::Time::now() > (sysreadyRT + ros::Duration(sysreadyT)) ) {
				sysready = false;				
			}	
		}
		if (needspace) {  	 	
			if ( ros::Time::now() > (needspaceRT + ros::Duration(needspaceT)) ) {
				needspace = false;				
			}	
		}
		if (navabort) {  	 	
			if ( ros::Time::now() > (navabortRT + ros::Duration(navabortT)) ) {
				navabort = false;		
				//ROS_INFO(" NavAbort ");		
			}	
		}
		if (estop) {  	 	
			if ( ros::Time::now() > (estopRT + ros::Duration(estopT)) ) {
				estop = false;				
			}	
		}
		if (estoprel) {  	 	
			if ( ros::Time::now() > (estoprelRT + ros::Duration(estoprelT)) ) {
				estoprel = false;				
			}	
		}
		if (batlow) {  	 	
			if ( ros::Time::now() > (batlowRT + ros::Duration(batlowT)) ) {
				batlow = false;				
			}	
		}
		if (reached) {  	 	
			if ( ros::Time::now() > (reachedRT + ros::Duration(reachedT)) ) {
				reached = false;				
			}	
		}
		if (findpath) {  	 	
			if ( ros::Time::now() > (findpathRT + ros::Duration(findpathT)) ) {
				findpath = false;				
			}	
		}
		if (moveoutdockstation) {  	 	
			if ( ros::Time::now() > (moveoutdockstationRT + ros::Duration(moveoutdockstationT)) ) {
				moveoutdockstation = false;				
			}	
		}
		if (movetodock) {  	 	
			if ( ros::Time::now() > (movetodockRT + ros::Duration(movetodockT)) ) {
				movetodock = false;				
			}	
		}
		if (nextjob) {  	 	
			if ( ros::Time::now() > (nextjobRT + ros::Duration(nextjobT)) ) {
				nextjob = false;				
			}	
		}
		if (returntocharge) {  	 	
			if ( ros::Time::now() > (returntochargeRT + ros::Duration(returntochargeT)) ) {
				returntocharge = false;				
			}	
		}
		if (shutdown) {  	 	
			if ( ros::Time::now() > (shutdownRT + ros::Duration(shutdownT)) ) {
				shutdown = false;				
			}	
		}
		if (stocker) {  	 	
			if ( ros::Time::now() > (stockerRT + ros::Duration(stockerT)) ) {
				stocker = false;				
			}	
		}
		if (wafestart) {  	 	
			if ( ros::Time::now() > (wafestartRT + ros::Duration(wafestartT)) ) {
				//ROS_INFO("------TalkNode : Restart----------");
				wafestart = false;				
			}	
		}
		if (ssmc) {  	 	
			if ( ros::Time::now() > (ssmcRT + ros::Duration(ssmcT)) ) {
				ssmc = false;				
			}	
		}
		if (detectcurrent) {  	 	
			if ( ros::Time::now() > (detectcurrentRT + ros::Duration(detectcurrentT)) ) {
				detectcurrent = false;				
			}	
		}
		if (localise) {  	 	
			if ( ros::Time::now() > (localiseRT + ros::Duration(localiseT)) ) {
				localise = false;				
			}	
		}
		if (relocalise) {  	 	
			if ( ros::Time::now() > (relocaliseRT + ros::Duration(relocaliseT)) ) {
				relocalise = false;				
			}	
		}
		if (lostlocalise) {  	 	
			if ( ros::Time::now() > (lostlocaliseRT + ros::Duration(lostlocaliseT)) ) {
				lostlocalise = false;				
			}	
		}
		if (checklocalise) {  	 	
			if ( ros::Time::now() > (checklocaliseRT + ros::Duration(checklocaliseT)) ) {
				checklocalise = false;				
			}	
		}
		if (localiseReady) {  	 	
			if ( ros::Time::now() > (localiseReadyRT + ros::Duration(localiseReadyT)) ) {
				localiseReady = false;				
			}	
		}
		if (restorelocalisation) {  	 	
			if ( ros::Time::now() > (restorelocalisationRT + ros::Duration(restorelocalisationT)) ) {
				restorelocalisation = false;				
			}	
		}
		if (SickLS) {  	 	
			if ( ros::Time::now() > (SickLSRT + ros::Duration(SickLST)) ) {
				SickLS = false;				
			}	
		}
		if (RightRS) {  	 	
			if ( ros::Time::now() > (RightRSRT + ros::Duration(RightRST)) ) {
				RightRS = false;				
			}	
		}
		if (RightRSOff) {  	 	
			if ( ros::Time::now() > (RightRSOffRT + ros::Duration(RightRSOffT)) ) {
				RightRSOff = false;				
			}	
		}
		if (RearHDOn) {  	 	
			if ( ros::Time::now() > (RearHDOnRT + ros::Duration(RearHDOnT)) ) {
				RearHDOn = false;				
			}	
		}
		if (RearHDOff) {  	 	
			if ( ros::Time::now() > (RearHDOffRT + ros::Duration(RearHDOffT)) ) {
				RearHDOff = false;				
			}	
		}
		if (Network) {  	 	
			if ( ros::Time::now() > (NetworkRT + ros::Duration(NetworkT)) ) {
				Network = false;				
			}	
		}
		if (MotorOk) {  	 	
			if ( ros::Time::now() > (MotorOkRT + ros::Duration(MotorOkT)) ) {
				MotorOk = false;				
			}	
		}
		if (MotorOff) {  	 	
			if ( ros::Time::now() > (MotorOffRT + ros::Duration(MotorOffT)) ) {
				MotorOff = false;				
			}	
		}
		if (LeftRS) {  	 	
			if ( ros::Time::now() > (LeftRSRT + ros::Duration(LeftRST)) ) {
				LeftRS = false;				
			}	
		} 
		if (LeftRSOff) {  	 	
			if ( ros::Time::now() > (LeftRSOffRT + ros::Duration(LeftRSOffT)) ) {
				LeftRSOff = false;				
			}	
		}
		if (FrontHDOn) {  	 	
			if ( ros::Time::now() > (FrontHDOnRT + ros::Duration(FrontHDOnT)) ) {
				FrontHDOn = false;				
			}	
		}
		if (FrontHDOff) {  	 	
			if ( ros::Time::now() > (FrontHDOffRT + ros::Duration(FrontHDOffT)) ) {
				FrontHDOff = false;				
			}	
		}
		if (ls3D) {  	 	
			if ( ros::Time::now() > (ls3DRT + ros::Duration(ls3DT)) ) {
				ls3D = false;				
			}	
		}
		if (LS3DOff) {  	 	
			if ( ros::Time::now() > (LS3DOffRT + ros::Duration(LS3DOffT)) ) {
				LS3DOff = false;				
			}	
		}
		if (Logoff) {  	 	
			if ( ros::Time::now() > (LogoffRT + ros::Duration(LogoffT)) ) {
				Logoff = false;				
			}	
		}
		if (StopClean) {  	 	
			if ( ros::Time::now() > (StopCleanRT + ros::Duration(StopCleanT)) ) {
				StopClean = false;				
			}	
		}
		if (ResumClean) {  	 	
			if ( ros::Time::now() > (ResumCleanRT + ros::Duration(ResumCleanT)) ) {
				ResumClean = false;				
			}	
		}
		if (MotorReleased) {  	 	
			if ( ros::Time::now() > (MotorReleasedRT + ros::Duration(MotorReleasedT)) ) {
				MotorReleased = false;				
			}	
		}
		if (MotorEngaged) {  	 	
			if ( ros::Time::now() > (MotorEngagedRT + ros::Duration(MotorEngagedT)) ) {
				MotorEngaged = false;				
			}	
		}
		if (Cleaning) {  	 	
			if ( ros::Time::now() > (CleaningRT + ros::Duration(CleaningT)) ) {
				Cleaning = false;				
			}	
		}
		if (CleanReport) {  	 	
			if ( ros::Time::now() > (CleanReportRT + ros::Duration(CleanReportT)) ) {
				CleanReport = false;				
			}	
		}
		if (CleanCompleted) {  	 	
			if ( ros::Time::now() > (CleanCompletedRT + ros::Duration(CleanCompletedT)) ) {
				CleanCompleted = false;				
			}	
		} 
		if (SystemBackup) {  	 	
			if ( ros::Time::now() > (SystemBackupRT + ros::Duration(SystemBackupT)) ) {
				SystemBackup = false;				
			}	
		}
		if (RearSickOn) {  	 	
			if ( ros::Time::now() > (RearSickOnRT + ros::Duration(RearSickOnT)) ) {
				RearSickOn = false;				
			}	
		}
		if (RearSickOff) {  	 	
			if ( ros::Time::now() > (RearSickOffRT + ros::Duration(RearSickOffT)) ) {
				RearSickOff = false;				
			}	
		}
		if (FrontSickOff) {  	 	
			if ( ros::Time::now() > (FrontSickOffRT + ros::Duration(FrontSickOffT)) ) {
				FrontSickOff = false;				
			}	
		}
		if (FrontSickOn) {  	 	
			if ( ros::Time::now() > (FrontSickOnRT + ros::Duration(FrontSickOnT)) ) {
				FrontSickOn = false;				
			}	
		}
		if (MiddleRSOn) {  	 	
			if ( ros::Time::now() > (MiddleRSOnRT + ros::Duration(MiddleRSOnT)) ) {
				MiddleRSOn = false;				
			}	
		}
		if (MiddleRSOff) {  	 	
			if ( ros::Time::now() > (MiddleRSOffRT + ros::Duration(MiddleRSOffT)) ) {
				MiddleRSOff = false;				
			}	
		} 
		if (BottomRSOn) {  	 	
			if ( ros::Time::now() > (BottomRSOnRT + ros::Duration(BottomRSOnT)) ) {
				BottomRSOn = false;				
			}	
		}
		if (BottomRSOff) {  	 	
			if ( ros::Time::now() > (BottomRSOffRT + ros::Duration(BottomRSOffT)) ) {
				BottomRSOff = false;				
			}	
		}
		if (AllSystemOn) {  	 	
			if ( ros::Time::now() > (AllSystemOnRT + ros::Duration(AllSystemOnT)) ) {
				AllSystemOn = false;				
			}	
		}
		if (AskLift) {  	 	
			if ( ros::Time::now() > (AskLiftRT + ros::Duration(AskLiftT)) ) {
				AskLift = false;				
			}	
		}
		if (LIftOpened) {  	 	
			if ( ros::Time::now() > (LIftOpenedRT + ros::Duration(LIftOpenedT)) ) {
				LIftOpened = false;				
			}	
		}
		if (MoveIntoLift) {  	 	
			if ( ros::Time::now() > (MoveIntoLiftRT + ros::Duration(MoveIntoLiftT)) ) {
				MoveIntoLift = false;				
			}	
		}
		if (MoveOutOfLift) {  	 	
			if ( ros::Time::now() > (MoveOutOfLiftRT + ros::Duration(MoveOutOfLiftT)) ) {
				MoveOutOfLift = false;				
			}	
		}
		if (LobbyDoorCmdError) {  	 	
			if ( ros::Time::now() > (LobbyDoorCmdErrorRT + ros::Duration(LobbyDoorCmdErrorT)) ) {
				LobbyDoorCmdError = false;				
			}	
		}
		if (LobbyDoorControlError) {  	 	
			if ( ros::Time::now() > (LobbyDoorControlErrorRT + ros::Duration(LobbyDoorControlErrorT)) ) {
				LobbyDoorControlError = false;				
			}	
		}
		if (RequestLobbyDoorOpen) {  	 	
			if ( ros::Time::now() > (RequestLobbyDoorOpenRT + ros::Duration(RequestLobbyDoorOpenT)) ) {
				RequestLobbyDoorOpen = false;				
			}	
		}
		if (LobbyDoorOpened) {  	 	
			if ( ros::Time::now() > (LobbyDoorOpenedRT + ros::Duration(LobbyDoorOpenedT)) ) {
				LobbyDoorOpened = false;				
			}	
		}
		*/
		n.getParam("AplayRunning",AplayRunning);
		if (!AplayRunning || ssmc) {
			//ROS_INFO("--------- Talk : Aplay not running. -------");
			ros::spinOnce();	
		} 
		//ros::spinOnce();	
  	loop_rate.sleep();
  }
  //ros::spin();

  return 0;
}



