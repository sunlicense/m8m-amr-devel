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

double sysreadyT,needspaceT,navabortT,estopT,estoprelT,batlowT,reachedT,ssmcT,detectcurrentT,premptT,localiseT,relocaliseT;
double findpathT,moveoutdockstationT,movetodockT,nextjobT,returntochargeT,shutdownT,stockerT,wafestartT,lostlocaliseT;
double checklocaliseT,localiseReadyT,sysloadedT,mappingmodeT,navigationmodeT,zeroingT,restorelocalisationT;
double SickLST,RightRST,NetworkT,MotorOkT,LeftRST,ls3DT,LogoffT;
double RearSickOnT,RearSickOffT,FrontSickOffT,FrontSickOnT,RightRSOffT,RearHDOnT,RearHDOffT,MotorOffT,MiddleRSOnT,MiddleRSOffT;
double LS3DOffT,LeftRSOffT,FrontHDOnT,FrontHDOffT,BottomRSOnT,BottomRSOffT,AllSystemOnT;
double AskLiftT,LIftOpenedT,MoveIntoLiftT,MoveOutOfLiftT,LobbyDoorCmdErrorT,LobbyDoorControlErrorT;
double RequestLobbyDoorOpenT,LobbyDoorOpenedT;

ros::Time sysreadyRT,needspaceRT,navabortRT,estopRT,estoprelRT,batlowRT,reachedRT,ssmcRT,detectcurrentRT,premptRT,localiseRT;
ros::Time findpathRT,moveoutdockstationRT,movetodockRT,nextjobRT,returntochargeRT,shutdownRT,stockerRT,wafestartRT,relocaliseRT;
ros::Time lostlocaliseRT,checklocaliseRT,localiseReadyRT,sysloadedRT,mappingmodeRT,navigationmodeRT,zeroingRT,restorelocalisationRT;
ros::Time SickLSRT,RightRSRT,NetworkRT,MotorOkRT,LeftRSRT,ls3DRT,LogoffRT;
ros::Time RearSickOnRT,RearSickOffRT,FrontSickOffRT,FrontSickOnRT,RightRSOffRT,RearHDOnRT,RearHDOffRT,MotorOffRT,MiddleRSOnRT,MiddleRSOffRT;
ros::Time LS3DOffRT,LeftRSOffRT,FrontHDOnRT,FrontHDOffRT,BottomRSOnRT,BottomRSOffRT,AllSystemOnRT;
ros::Time AskLiftRT,LIftOpenedRT,MoveIntoLiftRT,MoveOutOfLiftRT,LobbyDoorCmdErrorRT,LobbyDoorControlErrorRT;
ros::Time RequestLobbyDoorOpenRT,LobbyDoorOpenedRT;

double restartdelay,startdelay;


void init()
{
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

	id = msg->id;
	restartdelay = (double)msg->restartdelay;
	startdelay = (double)msg->startdelay;
	//ROS_INFO("---------- TalkNode Sound ID : %d. RSD=%.3f -----------------",id,restartdelay);
	switch (id) {
		case SYSREADY:			
			if (!sysready) {				
				//sleep(startdelay);
				ret = system("aplay ~/sound/SysReady.wav &");	
				sysready = true;						
				sysreadyT = restartdelay;
				//sleep(4);
				sysreadyRT = ros::Time::now();
				sleep(2);
			}
			break;
		case NEEDSPACE:		
			if (!needspace) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/needspace.wav &");		
				//ROS_INFO("------ Talk Node : Need Space --------");
				needspace = true;						
				needspaceT = restartdelay;
				//sleep(3);
				needspaceRT = ros::Time::now();
				sleep(2);
			}				
			break;
		case NAVABORT:
			if (!navabort) {		
				//sleep(startdelay);	
				ret = system("aplay ~/sound/NavAbort.wav &");		
				//ROS_INFO("------ Talk Node : Nav Aborted --------");
				navabort = true;					
				navabortT = restartdelay;
				//sleep(3);
				navabortRT = ros::Time::now();	
				sleep(2);
			}
			break;
		case ESTOPREL:
			if (!estoprel) {			
				//sleep(startdelay);	
				ret = system("aplay ~/sound/EstopRel.wav &");		
				//ROS_INFO("------ Talk Node : EStop Released --------");
				estoprel = true;						
				estoprelT = restartdelay;
				//sleep(3);
				estoprelRT = ros::Time::now();
				sleep(2);
			}
			break;
		case BATLOW:
			if (!batlow) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/Batterylow.wav &");		
				//ROS_INFO("------ Talk Node : Battery Low --------");
				batlow = true;						
				batlowT = restartdelay;
				//sleep(3);
				batlowRT = ros::Time::now();
				sleep(2);
			}	
			break;		
		case ESTOP:
			if (!estop) {				
				//sleep(startdelay);
				ret = system("aplay ~/sound/Estop.wav &");		
				//ROS_INFO("------ Talk Node : EStop Activated --------");
				estop = true;					
				estopT = restartdelay;
				//sleep(3);
				estopRT = ros::Time::now();	
				sleep(2);
			}
			break;
		case REACHED:
			if (!reached) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/Reached.wav &");		
				//ROS_INFO("------ Talk Node : Reached --------");
				reached = true;						
				reachedT = restartdelay;
				//sleep(3);
				reachedRT = ros::Time::now();
				sleep(2);
			}
			break;
		case FINDPATH:
			if (!findpath) {			
				//sleep(startdelay);	
				ret = system("aplay ~/sound/FindPath.wav &");		
				//ROS_INFO("------ Talk Node : FindPath --------");
				findpath = true;						
				findpathT = restartdelay;
				//sleep(3);
				findpathRT = ros::Time::now();
				sleep(2);
			}
			break;
		case MOVEOUTDOCKSTATION:
			if (!moveoutdockstation) {	
				//sleep(startdelay);			
				ret = system("aplay ~/sound/MoveOutDockStation.wav &");		
				//ROS_INFO("------ Talk Node : MoveOutDockStation --------");
				moveoutdockstation = true;						
				moveoutdockstationT = restartdelay;
				//sleep(3);
				moveoutdockstationRT = ros::Time::now();
				sleep(2);
			}
			break;
		case MOVETODOCK:
			if (!movetodock) {			
				//sleep(startdelay);	
				ret = system("aplay ~/sound/MoveToDock.wav &");		
				//ROS_INFO("------ Talk Node : MoveToDock --------");
				movetodock = true;					
				movetodockT = restartdelay;
				//sleep(3);
				movetodockRT = ros::Time::now();	
				sleep(2);
			}
			break;
		case NEXTJOB:
			if (!nextjob) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/NextJob.wav &");	
				//ROS_INFO("------ Talk Node : NextJob --------");	
				nextjob = true;				
				nextjobT = restartdelay;
				//sleep(3);
				nextjobRT = ros::Time::now();		
				sleep(2);
			}
			break;
		case RETURNTOCHARGE:
			if (!returntocharge) {	
				//sleep(startdelay);			
				ret = system("aplay ~/sound/ReturnToCharge.wav &");		
				//ROS_INFO("------ Talk Node : ReturnToCharge --------");
				returntocharge = true;						
				returntochargeT = restartdelay;
				//sleep(3);
				returntochargeRT = ros::Time::now();
				sleep(2);
			}
			break;
		case SHUTDOWN:
			if (!shutdown) {	
				//sleep(startdelay);			
				ret = system("aplay ~/sound/Shutdown.wav &");		
				//ROS_INFO("------ Talk Node : Shutdown --------");
				shutdown = true;						
				shutdownT = restartdelay;
				//sleep(3);
				shutdownRT = ros::Time::now();
				sleep(2);
			}
			break;
		case STOCKER:
			if (!stocker) {				
				//sleep(startdelay);
				ret = system("aplay ~/sound/Stocker.wav &");		
				//ROS_INFO("------ Talk Node : Stocker --------");
				stocker = true;						
				stockerT = restartdelay;
				//sleep(3);
				stockerRT = ros::Time::now();
				sleep(2);
			}
			break;
		case WAFESTART:
			if (!wafestart) {		
				//sleep(startdelay);		
				//ret = system("gnome-terminal -x aplay ~/sound/WafeStart.wav");		
				ret = system("aplay ~/sound/WafeStart.wav &");	
				//ROS_INFO("------ Talk Node : WafeStart --------");
				wafestart = true;						
				wafestartT = restartdelay;
				//sleep(1);
				wafestartRT = ros::Time::now();
				sleep(2);
			}
			break;
		case SSMC:
			if (!ssmc) {		
				//sleep(startdelay);		
				ret = system("~/ssmc.sh &");	
				ssmc = true;						
				ssmcT = restartdelay;
				//sleep(3);
				ssmcRT = ros::Time::now();
			}			
			break;
		case DETECTCURRENT:
			if (!detectcurrent) {				
				//sleep(startdelay);
				ret = system("aplay ~/sound/ChargingCurrent.wav &");	
				//ROS_INFO("------ Talk Node : ChargingCurrent --------");
				detectcurrent = true;					
				detectcurrentT = restartdelay;
				//sleep(3);
				detectcurrentRT = ros::Time::now();	
				sleep(2);
			}			
			break;  
		case PREMPTED:
			if (!prempt) {			
				//sleep(startdelay);	
				ret = system("aplay ~/sound/crash_x.wav &");	
				prempt = true;					
				premptT = restartdelay;
				//sleep(3);
				premptRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case LOSTLOCALISE:
			if (!lostlocalise) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/LostLocal.wav &");	
				//ROS_INFO("------ Talk Node : LostLocal --------");
				lostlocalise = true;					
				lostlocaliseT = restartdelay;
				//sleep(3);
				lostlocaliseRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case RELOCALISE:
			if (!relocalise) {	
				//sleep(startdelay);			
				ret = system("aplay ~/sound/ReLocalise.wav &");	
				//ROS_INFO("------ Talk Node : ReLocalise --------");
				relocalise = true;						
				relocaliseT = restartdelay;
				//sleep(3);
				relocaliseRT = ros::Time::now();
				sleep(2);
			}			
			break;
		case LOCALISE:
			if (!localise) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/Localised.wav &");	
				//ROS_INFO("------ Talk Node : Localised --------");
				localise = true;						
				localiseT = restartdelay;
				//sleep(3);
				localiseRT = ros::Time::now();
				sleep(2);
			}			
			break;
		case CHECKLOCALISE:
			if (!checklocalise) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/CheckLocalization.wav &");	
				//ROS_INFO("------ Talk Node : CheckLocalization --------");
				checklocalise = true;
				checklocaliseRT = ros::Time::now();		
				checklocaliseT = restartdelay;
				sleep(2);
			}			
			break;
		case LOCALISEREADY:
			if (!localiseReady) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/LocalizedReady.wav &");	
				//ROS_INFO("------ Talk Node : LocalizedReady --------");
				localiseReady = true;					
				localiseReadyT = restartdelay;
				//sleep(3);
				localiseReadyRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case MAPPINGMODE:
			if (!mappingmode) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/MappingModules.wav &");	
				//ROS_INFO("------ Talk Node : MappingMode --------");
				mappingmode = true;					
				mappingmodeT = restartdelay;
				//sleep(3);
				mappingmodeRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case NAVIGATIONMODE:
			if (!navigationmode) {			
				//sleep(startdelay);	
				ret = system("aplay ~/sound/NavigationModules.wav &");	
				//ROS_INFO("------ Talk Node : NavigationMode --------");
				navigationmode = true;					
				navigationmodeT = restartdelay;
				//sleep(3);
				navigationmodeRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case SYSLOADED:
			if (!sysloaded) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/SysReady.wav &");	
				//ROS_INFO("------ Talk Node : SysLoaded --------");
				sysloaded = true;					
				sysloadedT = restartdelay;
				//sleep(3);
				sysloadedRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case ZEROING:
			if (!zeroing) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/StartLocalisation.wav &");	
				//ROS_INFO("------ Talk Node : Zeroing --------");
				zeroing = true;					
				zeroingT = restartdelay;
				//sleep(3);
				zeroingRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case RESTORELOCALISATION:
			if (!restorelocalisation) {		
				//sleep(startdelay);		
				ret = system("aplay ~/sound/RestoreLocalisation.wav &");	
				//ROS_INFO("------ Talk Node : Zeroing --------");
				restorelocalisation = true;					
				restorelocalisationT = restartdelay;
				restorelocalisationRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case SICKLS:
			if (!SickLS) {			
				ret = system("aplay ~/sound/SickLS.wav &");	
				SickLS = true;					
				SickLST = restartdelay;
				SickLSRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case RIGHTRS:
			if (!RightRS) {		
				ret = system("aplay ~/sound/RightRS.wav &");	
				RightRS = true;					
				RightRST = restartdelay;
				RightRSRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case RIGHTRSOFF:
			if (!RightRSOff) {		
				ret = system("aplay ~/sound/RightRSOff.wav &");	
				RightRSOff = true;					
				RightRSOffT = restartdelay;
				RightRSOffRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case REARHDON:
			if (!RearHDOn) {		
				ret = system("aplay ~/sound/RearHDOn.wav &");	
				RearHDOn = true;					
				RearHDOnT = restartdelay;
				RearHDOnRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case REARHDOFF:
			if (!RearHDOff) {		
				ret = system("aplay ~/sound/RearHDOff.wav &");	
				RearHDOff = true;					
				RearHDOffT = restartdelay;
				RearHDOffRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case NETWORK:
			if (!Network) {		
				ret = system("aplay ~/sound/Network.wav &");	
				Network = true;					
				NetworkT = restartdelay;
				NetworkRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case MOTOROK:
			if (!MotorOk) {		
				ret = system("aplay ~/sound/MotorOk.wav &");	
				MotorOk = true;					
				MotorOkT = restartdelay;
				MotorOkRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case MOTOROFF:
			if (!MotorOff) {		
				ret = system("aplay ~/sound/MotorOff.wav &");	
				MotorOff = true;					
				MotorOffT = restartdelay;
				MotorOffRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case LEFTRS:
			if (!LeftRS) {		
				ret = system("aplay ~/sound/LeftRS.wav &");	
				LeftRS = true;					
				LeftRST = restartdelay;
				LeftRSRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case LEFTRSOFF:
			if (!LeftRSOff) {		
				ret = system("aplay ~/sound/LeftRSOff.wav &");	
				LeftRSOff = true;					
				LeftRSOffT = restartdelay;
				LeftRSOffRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case FRONTHDON:
			if (!FrontHDOn) {		
				ret = system("aplay ~/sound/FrontHDOn.wav &");	
				FrontHDOn = true;					
				FrontHDOnT = restartdelay;
				FrontHDOnRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case FRONTHDOFF:
			if (!FrontHDOff) {		
				ret = system("aplay ~/sound/FrontHDOff.wav &");	
				FrontHDOff = true;					
				FrontHDOffT = restartdelay;
				FrontHDOffRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case LS3D:
			if (!ls3D) {		
				ret = system("aplay ~/sound/ls3D.wav &");	
				ls3D = true;					
				ls3DT = restartdelay;
				ls3DRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case LS3DOFF:
			if (!LS3DOff) {		
				ret = system("aplay ~/sound/LS3DOff.wav &");	
				LS3DOff = true;					
				LS3DOffT = restartdelay;
				LS3DOffRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case BOTTOMRSON:
			if (!BottomRSOn) {		
				ret = system("aplay ~/sound/BottomRSOn.wav &");	
				BottomRSOn = true;					
				BottomRSOnT = restartdelay;
				BottomRSOnRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case BOTTOMRSOFF:
			if (!BottomRSOff) {		
				ret = system("aplay ~/sound/BottomRSOff.wav &");	
				BottomRSOff = true;					
				BottomRSOffT = restartdelay;
				BottomRSOffRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case MIDDLERSON:
			if (!MiddleRSOn) {		
				ret = system("aplay ~/sound/MiddleRSOn.wav &");	
				BottomRSOn = true;					
				BottomRSOnT = restartdelay;
				BottomRSOnRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case MIDDLERSOFF:
			if (!MiddleRSOff) {		
				ret = system("aplay ~/sound/MiddleRSOff.wav &");	
				MiddleRSOn = true;					
				MiddleRSOnT = restartdelay;
				MiddleRSOnRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case LOGOFF:
			if (!Logoff) {		
				ret = system("aplay ~/sound/Logoff.wav &");	
				Logoff = true;					
				LogoffT = restartdelay;
				LogoffRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case ASKLIFT:
			if (!AskLift) {		
				ret = system("aplay ~/sound/AskLift.wav &");	
				AskLift = true;					
				AskLiftT = restartdelay;
				AskLiftRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case LIFTOPENED:
			if (!LIftOpened) {		
				ret = system("aplay ~/sound/LIftOpened.wav &");	
				LIftOpened = true;					
				LIftOpenedT = restartdelay;
				LIftOpenedRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case MOVEINTOLIFT:
			if (!MoveIntoLift) {		
				ret = system("aplay ~/sound/MoveIntoLift.wav &");	
				MoveIntoLift = true;					
				MoveIntoLiftT = restartdelay;
				MoveIntoLiftRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case MOVEOUTOFLIFT:
			if (!MoveOutOfLift) {		
				ret = system("aplay ~/sound/MoveOutOfLift.wav &");	
				MoveOutOfLift = true;					
				MoveOutOfLiftT = restartdelay;
				MoveOutOfLiftRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case LOBBYDOORCMDERROR:
			if (!LobbyDoorCmdError) {		
				ret = system("aplay ~/sound/LobbyDoorCmdError.wav &");	
				LobbyDoorCmdError = true;					
				LobbyDoorCmdErrorT = restartdelay;
				LobbyDoorCmdErrorRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case LOBBYDOORCONTROLERROR:
			if (!LobbyDoorControlError) {		
				ret = system("aplay ~/sound/LobbyDoorControlError.wav &");	
				LobbyDoorControlError = true;					
				LobbyDoorControlErrorT = restartdelay;
				LobbyDoorControlErrorRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case REQUESTLOBYDOOROPEN:
			if (!RequestLobbyDoorOpen) {		
				ret = system("aplay ~/sound/RequestLobbyDoorOpen.wav &");	
				RequestLobbyDoorOpen = true;					
				RequestLobbyDoorOpenT = restartdelay;
				RequestLobbyDoorOpenRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case LOBBYDOOROPENED:
			if (!LobbyDoorOpened) {		
				ret = system("aplay ~/sound/LobbyDoorOpened.wav &");	
				LobbyDoorOpened = true;					
				LobbyDoorOpenedT = restartdelay;
				LobbyDoorOpenedRT = ros::Time::now();	
				sleep(2);
			}			
			break;
		case STOPMOVESOUND:  // kill aplay 
			ret = system("~/ssmcend.sh");
			break;
	}
		
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Sound Server");
  ros::NodeHandle n;
	ros::Rate loop_rate(5.0);
	
	init();
	play_sub = n.subscribe<htbot::sound>("sound", 10, sound);  // 
	
	while (true) {  	
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
		if (RearHDOn) {  	 	
			if ( ros::Time::now() > (RearHDOnRT + ros::Duration(RearHDOnT)) ) {
				RearHDOn = false;				
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
		if (LeftRS) {  	 	
			if ( ros::Time::now() > (LeftRSRT + ros::Duration(LeftRST)) ) {
				LeftRS = false;				
			}	
		}
		if (FrontHDOn) {  	 	
			if ( ros::Time::now() > (FrontHDOnRT + ros::Duration(FrontHDOnT)) ) {
				FrontHDOn = false;				
			}	
		}
		if (ls3D) {  	 	
			if ( ros::Time::now() > (ls3DRT + ros::Duration(ls3DT)) ) {
				ls3D = false;				
			}	
		}
		if (Logoff) {  	 	
			if ( ros::Time::now() > (LogoffRT + ros::Duration(LogoffT)) ) {
				Logoff = false;				
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
		ros::spinOnce();	
  	loop_rate.sleep();
  }
  //ros::spin();

  return 0;
}



