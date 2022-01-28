 /*************************************************************************
 * Author: Ong Eng Huat
 * Contact: oenghuat@tp.edu.sg, ready2race@yahoo.com
 * Date: 24/11/2020
 *
 * (1) Listens to ROS message on topic /to_lift (publishes by AGV or webpage simulator via ROSbridge).
 * (2) Interprets request for lift services, sends request to RPi via TCP socket connection.
 * (3) Receives TCP socket message from RPi regarding lift controller statuses.
 * (4) Interprets and publishes appropriate message to topic /fr_lift for AGV or webpage simulator.
 * (5) AGV or webpage simulator listens to topic /fr_lift and takes appropriate action.
 *
 * Open terminal and run Roscore :
 *    roscore
 * Run this ROS node :
 *    rosrun ntuc ntuc_lift_client_node 192.168.1.159 3000 <TCP_socket_server_address, port_no>
 * TCP_socket_address is the IP address of RPi running TCP socket server.
 * port_no is the port number the TCP socket server at RPi is listening at.
 *
 * Could also do 'roslaunch ntuc ntuc_elevator.launch'
 *
 ***************************************************************************/
#include <ros/ros.h>
//#include <stdio.h>
//#include <stdlib.h> 
//#include <unistd.h>
//#include <string.h>
//#include <sys/types.h>
#include <sys/socket.h>	// added for TCP socket
#include <netinet/in.h>	// added for TCP socket
#include <netdb.h> 		// added for TCP socket
#include "std_msgs/String.h"
#include "ntuc/lift.h"

#define MESSAGE_FREQ 1   // 1 Hz
 
int sockfd, n; 
/// bool move_x=false, move_xm=false, move_y=false, move_ym=false, move_z=false, move_zm=false; //6 axes

void error(const char *msg) {
    perror(msg);
    exit(0);
}

class Listener {
	private:
	    char topic_message[256];
	public:
	    void callback(const ntuc::lift::ConstPtr& msg);
	    char* getMessageValue();
	};

	void Listener::callback(const ntuc::lift::ConstPtr& msg) {
	    memset(topic_message, 0, 256);
	 //   strcpy(topic_message, msg);
	    ROS_INFO("I heard cmd:[%d]", msg->cmd);
	    ROS_INFO("I heard cfloor:[%d]", msg->cfloor);
	    ROS_INFO("I heard dfloor:[%d]", msg->dfloor);
	/**********
	  if (!strcmp(topic_message,"pause")) {  // match "pause"
	  		ROS_INFO("It is %s!", topic_message);

		  // send "pause_program()" to C3_cobot
		  n = write(sockfd, "pause_program()", 15);
		  if (n < 0) error("ERROR writing to socket"); 		
	  }
	  else if (!strcmp(topic_message,"resume")) { // match "resume"
	  		ROS_INFO("It is %s!", topic_message);  

		  // send "resume_program()" to C3_cobot
		  n = write(sockfd,"resume_program()", 16);
		  if (n < 0) error("ERROR writing to socket"); 	   		  		
	  }
	**********/
	    
	} //end Listener::callback()

	char* Listener::getMessageValue() {
	    return topic_message;
} //end class Listner


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "ntuc_lift_client_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(MESSAGE_FREQ); // set the rate as defined in the macro MESSAGE_FREQ
	Listener listener;
	ros::Subscriber client_sub = nh.subscribe("/to_lift", 100, &Listener::callback, &listener);
	int portno, choice = 1;
	const char* server_ip="10.42.0.1";	// temporary hardcode ip address
	struct sockaddr_in serv_addr;
	struct hostent *server;
	char buffer[256];
	bool echoMode = false;

    
	// Get TCP socket host IP address from ROS parameter provided in ROS launch file
	///int portno;   // port number  
//	ros::param::get("/server_ip", server_ip);       
	// Get port number from ROS parameter provided in ROS launch file
	portno = 3000;   // temporary hardcode port number  
	// ros::param::get("/port", portno);   
    
    /****************************
    if (argc < 3) {
       fprintf(stderr,"Usage: $ rosrun amc tmc_client_node <hostname> <port> --arguments\nArguments:\n -e : Echo mode\n");
       exit(0);
    }
    if (argc > 3)
		if (strcmp(argv[3], "-e") == 0)
			echoMode = true;
    portno = atoi(argv[2]);
    *********************/
    
////////////////////////////////////////////////////////////////
//////////////// Connect to Socket Server //////////////////////
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
//    server = gethostbyname(argv[1]);

	 server = gethostbyname(server_ip);
	 
	 std::cout << "IP is " << server_ip << " , port is " << portno << ".\n";
	 
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
    std::cout << "Connected Successfully.\n" << "Usage: rostopic pub -1 /to_lift ntuc/lift -- '{cmd: 1, cfloor: 2, dfloor: 3}'\n";
////////////////////////////////////////////////////////////////

	while(ros::ok()) {
	  // reset pause_flag
//	  bool pause_flag=false, resume_flag=false, stop_flag=false;
	  
//	    printf("Inside, rate is %f seconds. \n", 1.0/MESSAGE_FREQ );
	    ros::spinOnce();
	}
	return 0;
}
