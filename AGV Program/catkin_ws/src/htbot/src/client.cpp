//***********************************************
//message type number: 
//	0: pose message of robot
//	1: path message 
//	2: current and voltage message                     
//  3: arrive station message
//***********************************************
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"

#include "beginner/mqueue.h"

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <sys/ioctl.h>

#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h> 
#include <fcntl.h>

#include <signal.h>
#include <string>

#include <sys/time.h>
#include <signal.h>

#define OUTPUT_SIZE	65535
#define INPUT_SIZE	65535

int connecttoserver();
	int timer_count = 0;
	int timer_count10 = 0;
	struct itimerval tval;
	bool timer_ctrl = false;
	bool timer_ctrl10 = false;

const int HOST_NAME_LENGTH = 64;

    struct sockaddr_in serv_addr; 
	int recvNo;
	socklen_t addrlen=sizeof(recvNo);
	int sockfd = 0, n = 0;
    char recvBuff[65535];
    char *sendBuff= new char[65536];
	int retval = 0;
//	char *addr = "192.168.0.104";
	char *addr;// = "192.168.1.105";
	
	fd_set fdset;
	struct timespec tstart={0,0},tend={0,0};
	struct timeval tv, trcv, tsnd;
	int rc;
	
	struct pollfd fds;
	int ret;
		
	int socket_OK = 0;
	
	//uint32_t
	char *message_in;//[INPUT_SIZE];
	char message_out[OUTPUT_SIZE];

	std::string serverAddr;
	int robotNo;

	float vol, curr;
	
	std::string frStation="",toStation=""; 
	int driveCmd = 0;
	
ros::ServiceClient client;
	
void handler(int signo){
//	ROS_INFO("TIMER %d", ++timer_count);
	timer_count++;
	timer_ctrl = true;
	if(timer_count == 5){
		timer_ctrl10 = true;
		timer_count = 0;
	}
}

void sigpipe_handler(int sig0)
{
	printf("SIGPIPE caught\n");
	socket_OK = 1;
		connecttoserver();
}

void currCallback(const std_msgs::Float32::ConstPtr& currMsg)
{
	curr = currMsg->data;
//	ROS_INFO("Receied curr: %f", curr);
//**********************************
if(timer_ctrl10){
	char* sendbuf;// = new char[100];
	sprintf(message_out,"2 %d %.3f %.3f",robotNo, vol, curr);
	sendbuf = message_out;
//	ROS_INFO("curr and vol %s, %d",message_out, strlen(message_out));
//	ROS_INFO("send %s, %d",sendbuf, strlen(sendbuf));
	char sendsize[6]; 
	sprintf(sendsize,"%d",strlen(message_out));

	retval = send(sockfd, sendsize, 6,MSG_EOR);
	send(sockfd, sendbuf, strlen(message_out),MSG_EOR);
	timer_ctrl10 = false;
}
//***********************************
}

void volCallback(const std_msgs::Float32::ConstPtr& volMsg)
{
	vol = volMsg->data;
//	ROS_INFO("Receied vol: %f", vol);
}

void pathCallback(const nav_msgs::Path::ConstPtr& pathMsg)
{
	int length = pathMsg->poses.size();
	char *str = new char[2*6*length+12];
	char sendbuf[30];// = new char[100];
	sprintf(str,"1 %d %d ",robotNo, length);

	for(int i=0; i<length-1; i++){
		sprintf(sendbuf,"%0.3f %0.3f ",pathMsg->poses[i].pose.position.x, pathMsg->poses[i].pose.position.y);
		str = strcat(str, sendbuf) ;
	}
	char sendsize[6]; 
	sprintf(sendsize,"%d",strlen(str));

//	ROS_INFO("Receied path: %d", length);

	retval = send(sockfd, sendsize, 6,MSG_EOR);
	send(sockfd, str, strlen(str),MSG_EOR);
}
  
void poseCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
{
	int message_len=0;   //topic pose
	char *sendbuf;
	sprintf(message_out,"0 %d %.3f %.3f %.3f %.3f",robotNo,poseMsg->position.x,poseMsg->position.y,poseMsg->orientation.z,poseMsg->orientation.w);
	sendbuf = message_out;
//	ROS_INFO("Pose callback %s, %d",message_out, strlen(message_out));
//	ROS_INFO("send %s, %d",sendbuf, strlen(sendbuf));
//**********************************
if(timer_ctrl){
	char sendsize[6]; 
	sprintf(sendsize,"%d",strlen(message_out));

	retval = send(sockfd, sendsize, 6,MSG_EOR);
	send(sockfd, sendbuf, strlen(message_out),MSG_EOR);
	timer_ctrl = false;
}
//**********************************************************
/*	message_in = new char[1024];	
	message_in[0]='O';
  	message_in[1]='K';
	
	fds.fd = sockfd;
	fds.events = POLLIN;
	
	char* str=new char[1024];

	trcv.tv_sec = 5;
	trcv.tv_usec = 0.5;
	do{	
		ret = poll(&fds, 1, 5000);
//		ROS_INFO("POLL result: %d", ret);
		switch(ret){
		case -1:
		ROS_INFO("POLL error");
		break;
		case 0:
		ROS_INFO("POLL time out");
		if(sockfd > 0){
			shutdown(sockfd,SHUT_RDWR);
			close(sockfd);
		}
		break;
		
		default:
//			ROS_INFO("start read;  ");
			recvNo = recv(sockfd, message_in, 2,0) ;
			message_in[recvNo]='\0';
//			ROS_INFO("read  %d, message: %s\n", recvNo, message_in);
fflush(stdin);
			strcpy(str,message_in);
			break;
		}		
	}while(strcmp(str,"OK") != 0 && recvNo != -1);//recvNo <= 0);//
	delete message_in;
*/
//	ROS_INFO("end read\n");
}

void arriveCallback(const std_msgs::String::ConstPtr& msg)
{
	std::string arrivedStation = msg->data;
	char *sendbuf;
	sprintf(message_out,"3 %s",arrivedStation.c_str());
	sendbuf = message_out;
	ROS_INFO("Arrived station: %s", arrivedStation.c_str());
//	ROS_INFO("send %s, %d",sendbuf, strlen(sendbuf));
	char sendsize[6]; 
	sprintf(sendsize,"%d",strlen(message_out));

	retval = send(sockfd, sendsize, 6,MSG_EOR);
	send(sockfd, sendbuf, strlen(message_out),MSG_EOR);
	if(arrivedStation == frStation){
		beginner::mqueue srv;
		srv.request.cmd = driveCmd;
		srv.request.lps = toStation;
		client.call(srv);
		ROS_INFO("Drive motor to: %s", toStation.c_str());
		frStation = "";
	}
	else if(arrivedStation == toStation)
		toStation = "";

}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
/*	retval = send(sockfd, sendBuff, 6,MSG_EOR);
	send(sockfd, msg->data.c_str(),  20,MSG_EOR);
    ROS_INFO("XU4 received: %s; send result: %d",msg->data.c_str(),retval);
		char* str;
	clock_gettime(CLOCK_MONOTONIC, &tstart);
	do{	
		clock_gettime(CLOCK_MONOTONIC, &tend);
		if(((double)tend.tv_sec+1.0e-9*tend.tv_nsec)-((double)tstart.tv_sec+1.0e-9*tstart.tv_nsec)>1)
		{
			ROS_INFO("time out\n");
			retval = -1;
			break;
		}
//			ROS_INFO("start read\n");
//		recvNo = recv(sockfd, recvBuff, sizeof(recvBuff)-1,0) ;
//			ROS_INFO("end read\n");
		recvNo = read(sockfd, recvBuff, sizeof(recvBuff)-1) ;
		recvBuff[recvNo]='\0';
		strcpy(str,recvBuff);
//		printf("byte received:%d, %s\n",recvNo,str);
	}while(strcmp(str,"OK") != 0);//recvNo <= 0);//
//		printf("read in %.5f second\n", (((double)tend.tv_sec+1.0e-9*tend.tv_nsec)-((double)tstart.tv_sec+1.0e-9*tstart.tv_nsec)));
 */}

int connecttoserver(){
 //   struct sockaddr_in serv_addr; 

 	ROS_INFO("memset recvBuff");
    memset(recvBuff, '0',sizeof(recvBuff));
	if(sockfd > 0){
		shutdown(sockfd,SHUT_RDWR);
		close(sockfd);
	}
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        ROS_INFO(" Error : Could not create socket: %d",sockfd);
        ROS_INFO(" Error NO: %d \n",errno);
       return 1;
    } 
 
    memset(&serv_addr, '0', sizeof(serv_addr)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(5150); 

    if(inet_pton(AF_INET, addr, &serv_addr.sin_addr)<=0)
    {
        ROS_INFO("\n inet_pton error occured\n");
        return 1;
    } 

	ROS_INFO("connect");
	
/*	char sName[HOST_NAME_LENGTH+1];
	struct hostent * hostPtr;
	struct in_addr **addr;
	char* localAddr;
	
	memset(sName, 0, sizeof(sName));
	gethostname(sName,HOST_NAME_LENGTH);
	hostPtr = gethostbyname(sName);
//	addr.s_addr = inet_addr(sName);
//	addr = (struct in_addr*)*hostPtr->h_addr;
	localAddr = hostPtr->h_addr;
//	localAddr = inet_ntoa(*addr);
	addr = (struct in_addr**)hostPtr->h_addr_list;
//	localAddr = inet_ntoa(*addr);
	
	ROS_INFO("Host name %s \n", hostPtr->h_name);
	ROS_INFO("Host address %s \n", localAddr);
	for(int i=0;addr[i]!= NULL; i++)
		ROS_INFO("Host address %s \n", inet_ntoa(*addr[i]));
//	fcntl(sockfd, F_SETFL,O_NONBLOCK);
	
	struct addrinfo hints, *result;
	int sfd;
	
	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = AF_UNSPEC;//AF_INET for IPv4, AFINET6 for IPv6
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = 0;
	hints.ai_protocol = 0;
	
	sfd = getaddrinfo(NULL,NULL, &hints, &result);
*/	
    if( rc = connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       ROS_INFO("\n Error : Connect Failed; rc = %d \n", rc);
 //      return 1;
    } 
	
	trcv.tv_sec = 10;
	trcv.tv_usec = 0;
	tsnd.tv_sec = 10;
	tsnd.tv_usec = 0;
	int blocking;
	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &trcv, sizeof(trcv));
	setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tsnd, sizeof(tsnd));
	ioctl(sockfd, FIONBIO, (unsigned long*) &blocking);

	if(rc == -1){
		ROS_INFO("cannot connect");
		return 1;
	}  
	if(rc > 0){
		clock_gettime(CLOCK_MONOTONIC, &tend);
		ROS_INFO("Connect in %.5f second", (((double)tend.tv_sec+1.0e-9*tend.tv_nsec)-((double)tstart.tv_sec+1.0e-9*tstart.tv_nsec)));
		return 0;
	}
}

void driveRobotToDestination(int cmd, std::string frDes, std::string toDes)
{
  	beginner::mqueue srv;
	frStation = frDes;
	toStation = toDes;
	driveCmd = cmd;
  	srv.request.cmd = cmd;
  	srv.request.lps = frDes;
	client.call(srv);
	ROS_INFO("Drive motor to %s", frDes.c_str());
 /* 	srv.request.cmd = cmd;
  	srv.request.lps = toDes;
	client.call(srv);
	ROS_INFO("Drive motor to %s", toDes.c_str());*/
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "client");

  ros::NodeHandle nh;
 	ROS_INFO("Start");
	int connResult=0;

	nh.param<std::string>("server_addr", serverAddr, "192.168.0.104");
	nh.param("robot_no", robotNo, 0);
	
	ROS_INFO("server address: %s",serverAddr.c_str());
	ROS_INFO("robot number: %d",robotNo);

	addr = const_cast<char*>(serverAddr.c_str());

	do{
		connResult = connecttoserver();
		if(connResult == 1){
			ROS_INFO("connect to server failed, retry");
			sleep(20);
		}
	}  
	while(connResult == 1);

	signal(SIGPIPE,sigpipe_handler);
	ROS_INFO("subscribe");

//   ros::Subscriber sub = nh.subscribe("chatter", 2, chatterCallback);
   ros::Subscriber subPose = nh.subscribe("robot_pose", 100, poseCallback);
//   ros::Subscriber path_sub = nh.subscribe("/move_base/NavfnROS/plan", 2, pathCallback);
   ros::Subscriber subCurr = nh.subscribe("current", 2, currCallback);
   ros::Subscriber subVol = nh.subscribe("voltage", 2, volCallback);
   ros::Subscriber subArrive = nh.subscribe("arrive", 2, arriveCallback);

	client = nh.serviceClient<beginner::mqueue>("web_cmd");

	timerclear(&tval.it_interval);
	timerclear(&tval.it_value);

//	tval.it_value.tv_sec =1;
 tval.it_value.tv_sec = 0;
 tval.it_value.tv_usec = 250000;
 /* ... and every 250 msec after that. */
 tval.it_interval.tv_sec = 0;
 tval.it_interval.tv_usec = 250000;
	(void)signal(SIGALRM, handler);

	(void)setitimer(ITIMER_REAL, &tval, NULL);

  ros::Rate loop_rate(3);
  while (ros::ok())
  {
//	ROS_INFO("in the while");
    char ecvBuff[65535];

 //  while ( (n = read(sockfd, ecvBuff, 10)) > 0)
	if((n = read(sockfd, ecvBuff, sizeof(ecvBuff))) > 0)
    {
        ecvBuff[n] = '\0';
        ROS_INFO("%d received: %s", n,ecvBuff);
		std::string recvStr = ecvBuff;
		std::string frStation, toStation;
		std::string token[3];
		std::string delimiter = " ";
		size_t pos = 0;
	//	std::string token;
		int i=0;
		while((pos = recvStr.find(delimiter)) != std::string::npos){
			token[i] = recvStr.substr(0, pos);
			ROS_INFO("token[%d] is: %s",i, token[i].c_str());
			recvStr.erase(0,pos+delimiter.length());
			i++;
/*			frStation = recvStr.substr(0, pos);
			ROS_INFO("frStation is: %s",frStation.c_str());
			toStation=recvStr.erase(0,pos+delimiter.length());
			ROS_INFO("toStation is: %s",toStation.c_str());
			if(atoi(token.c_str()) == 11){
				driveRobotToDestination(11, frStation, toStation);
//				ROS_INFO("Call driveRobotToDestination");

			}*/
		}
		token[i]=recvStr;
		ROS_INFO("token[%d] is: %s",i, token[i].c_str());
		if(atoi(token[0].c_str()) == 11){
			driveRobotToDestination(11, token[1], token[2]);
		}
		fflush(stdin);
    } 

//    if(n < 0)
//    {;
 //       ROS_INFO("\n Read error \n");
 //   }

	int error = 0;
	socklen_t len = sizeof(error);
	//retval = getsockopt(sockfd,SOL_SOCKET,SO_KEEPALIVE,&error, &len);
//	ROS_INFO("\n connect status %d \n",retval);
	if(retval == -1){
		ROS_INFO("\n Error : Connect losed, Reconnect to server.s \n");
		connecttoserver();
		retval=0;
	} 
    ros::spinOnce();
    loop_rate.sleep();
  }
	char sendsize[2] = {'8','0'};
	send(sockfd, sendsize, 2, MSG_EOR);

	char sendsize1[8];// = {'0',' ','5',' ','q','u','i','t'};
	sprintf(sendsize1,"0 %d quit",robotNo);
	send(sockfd, sendsize1, 8, MSG_EOR);
	shutdown(sockfd,2);
}
