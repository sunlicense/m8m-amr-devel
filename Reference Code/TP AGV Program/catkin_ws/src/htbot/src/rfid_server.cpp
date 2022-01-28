/*
 * rfid.cpp
 *
 *  Created on: Jul 7, 2017
 *      Author: lukevan
 *      - Scan 16 potential readers and store all connected readers for checking.
 *      - Poll all connected readers for IR sensor status: item detected or removed ( IN or OUT)
 *      - If item detected, scan, store and display transponder (tag) data.  Connect to TPC/IP server and
 *         send IN, transporterID, positionID, LOT data to server.  Publish same data on ROS topic "rfid"
 *      - If item removed, retreave item data stored earlier.  Connect to TPC/IP server and
 *         send OUT, transporterID, positionID, LOT data to server.  Publish same data on ROS topic "rfid"
 *      - Also check for any reader disconnected from system.
 *
 *      Last updated:
 *      - Re calucate tx_data[6] & tx_data[7] for reader 6 to 9 (18/7/17)
 *      - removed +1 from tx_data[3]+1 for reader number detected. (16/8/17)
 *      - and changed other codes for reader number accordingly (16/8/17)
 *      - tidy up and linktest timeout, data Reply timeout (17/8/17)
 *      - using launch file with param (also from udev rules) (18-21/8/17).
 *      - added Deselect response, separate response (8/9/17)
 *      - added new format for sending data message and receiving data reply (15/9/17)
 *      - added TransporterID read from n.getParam("TransporterID",TransporterID) (15/9/17)
 *      - added startup_or_s1f5 sending data out and then in for current pods (27/9/17)
 *      - added s1f6 reply to s1f5
 *      - added S1F7 send at the end of data after start up or when receiving S1F5. (24/10/17)
 *      - added S1F1 & S1F2 (27/10/2017)
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>

#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include <sys/time.h> //FD_SET, FD_ISSET, FD_ZERO macros
using namespace std;

unsigned char ConvertDigitToAscii(unsigned char Digit);
unsigned char ConvertAsciiToNumber(unsigned char AsciiByte);

#define TRUE   1
#define FALSE  0
#define PORT 5000 //27015 //8888
#define T3 30000  // reply time out for data message (30s)
#define T5 10000 // connection separation time out (10s)
#define T6 10000	// control transaction time out (10s)
#define T7 10000 // not selected time out after tcp/ip accept (10s)

int main(int argc, char *argv[ ] )
{
	unsigned char tx_data[8], rx_data[255];  // to store transmitted or received data
	unsigned char *bufptr;      // Current char in buffer
	int  mbytes=0, nbytes;       // Number of bytes read
	int tries = 0; // to store number of TCPIP tries
	int  readerNum=0; // to store reader number
	unsigned char itemDetected[20], itemRemoved[20]; // 20 to store item deteced / removed flags
	unsigned char itemMemLocation[20]; //20 to store all 16 memory location values
	unsigned char temp=0; // to store temp item memory location
	unsigned char LotData[250]; // 192 to store all 16 lots of 8 bytes data each (16*8 = 128 bytes)
	char LotData2[50]; // 13LOT data + other data to be published or sent to TCPIP client
	unsigned char LotData3[20]; // 13LOT data + other data to be published or sent to TCPIP client
	unsigned char published = 0; // to store published flag
	int result; // return value for read/write/send ...
	//char *SerialComPort; // Serial comport number
	std::string SerialComPort;
	std::string TransporterID;
	//char *TCPIPaddr = "172.30.97.213"; // TCPIP internet address if it is run as client
	//std::string TCPIPaddr;  // TCPIP internet address if it is run as client
	int TCPIPportNum = 27015;  //Internet Port number
	unsigned char readerDet [20]; // to strore reader number detected
	unsigned char readerDetFlag[20]; // Reader detected flag
	unsigned char  readerDisconnectedFlag[20]; // Reader disconnected flag
	unsigned char NumOfReaderDet = 0; // Number of readers detected after start up
	unsigned char readerDisPrinted[20]; // max 16 readers but transporter only has 3 readers
	char sendLotDataIn[5] = {0,0,0,0,0};  // =1 is to send data to server
	char sendLotDataInBackup[5] = {0,0,0,0,0};  // =1 is to send data to server
																				// reader number detected is in readerDet[i] , i = 0, 1, 2.
	char sendLotDataOut[5] = {0,0,0,0,0};  // =1 is to send data to server
	bool startup_or_s1f5 = true; 	//
																				// reader number detected is in readerDet[i] , i = 0, 1, 2.
	char reader=0;


	///SECSGEM declaration - start --------------------------------------------------
	union {
		    unsigned short transactionID_short;
		    unsigned char transactionID_byte[2] = {0,0};
		} transactionID;

	char selectreq[] = {0xFF,0xFF,0,0,0,0x01,0,0,0,0,'\0'};
	char selectrsp[] = {0xFF,0xFF,0,0,0,0x02,0,0,0,0,'\0'};
	char deselectreq[] = {0xFF,0xFF,0,0,0,0x03,0,0,0,0,'\0'};
	char deselectrsp[] = {0xFF,0xFF,0,0,0,0x04,0,0,0,0,'\0'};
	char linktestreq[] = {0xFF,0xFF,0,0,0,0x05,0,0,0,0,'\0'};
	char linktestrsp[] = {0xFF,0xFF,0,0,0,0x06,0,0,0,0,'\0'};
	char separatereq[] = {0xFF,0xFF,0,0,0,0x09,0,0,0,0,'\0'};
	char S18F75header[] = {0xFF,0xFF,0x12,0x4B,0,0,0,0,0,0,'\0'};
	char S1F5header[] = {0xFF,0xFF,0x81,0x05,0,0,0,0,0,0,'\0'};
	char S1F7header[] = {0xFF,0xFF,0x81,0x07,0,0,0,0,0,0,'\0'};
	char S1F1header[] = {0xFF,0xFF,0x81,0x01,0,0,0,0,0,0,'\0'};
	char S1F2header[] = {0x7F,0xFF,0x01,0x02,0,0,0,0,0,0,'\0'};
	char S1F2data[] = {0x01,0x02,0x41,0x00,0x41,0,'\0'};
	char S1F6header[] = {0xFF,0xFF,0x01,0x06,0,0,0,0,0,0,'\0'};
	char msgLen1[] = {0,0,0,0x0A,'\0'}; // for other control message
	char msgLen2[] = {0,0,0,0x22,'\0'};  // for Data message
	char msgLen3[] = {0,0,0,0x10,'\0'};  // for Data message
	char selectdone =0, linktestalive=0, dataReply=0, dataMsgSent=0;
	struct timeval starttime, endtime, starttime_ltreq, endtime_ltreq,starttime_ltrsp, endtime_ltrsp;
	struct timeval endtime_dataReply, starttime_dataReply;
	long elapsedtime_ltreq, elapsedtime_ltrsp, elapsedtime_dataReply, mtime, seconds, useconds;
	// SECSGEM declaration - end ---------------------------------------------

	// tcp/ip declaration - start -------------------------------------------------
	int opt = TRUE;
	int master_socket=0 , addrlen , new_socket , client_socket[5] , max_clients = 5 , i, activity, valread , sd;
	int max_sd;
	struct sockaddr_in address;

	char buffer[50];  //data buffer of 1K
	unsigned char buffer2[50];  //data buffer of 1K

	//set of socket descriptors
	fd_set readfds;
	// tcp/ip declaration - end -------------------------------------------------

	std_msgs::String msg;
	std::stringstream ss;
	ros::init(argc, argv, "rfid");
	//ros::NodeHandle n;
	ros::NodeHandle n("~");
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("rfid", 1000);
	ros::Rate loop_rate(1);

	n.getParam("TCPIPportNum",TCPIPportNum);
	n.getParam("SerialComPort",SerialComPort);
	n.getParam("TransporterID",TransporterID);

	int fd; // file description
	struct termios options;
	/* open the port */
	//fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	//fd = open(SerialComPort, O_RDWR | O_NOCTTY | O_NDELAY);
	fd = open(SerialComPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	fcntl(fd, F_SETFL, 0);

	/* get the current options */
	tcgetattr(fd, &options);

	cfsetispeed(&options, B57600);
	/* set raw input, 1 second timeout */
	options.c_cflag     |= (CLOCAL | CREAD);
	options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag     &= ~OPOST;
	options.c_cc[VMIN]  = 0;
	options.c_cc[VTIME] = 1;

	/* set the options */
	tcsetattr(fd, TCSANOW, &options);

	for (int j = 0; j<20; j++)
	{
		itemDetected[j] = 0;
		itemRemoved[j]=0;
		readerDisPrinted[j]=0;
	}

	printf ("Starting... \n");
	printf("Please wait... \n");
	for (int j=1; j<18; j++)
	{
		printf(".");
		tx_data[0] = 0x02;  // start byte
		tx_data[1] = 0x34;  // '4'
		tx_data[2] = 0X30;  // '0'  (reader number: high byte)
		tx_data[3] = ConvertDigitToAscii(j-1); // from '0' to 'F';  (reader number: Low byte)
		tx_data[4] = 0x31;  // '1'
		tx_data[5] = 0x31;  // '1'

		if (j <=10)
		{
			tx_data[6] = 0x43; // 'C'
			tx_data[7] = ConvertDigitToAscii(j+1); // from '2' to 'B'.
		}
		else if (j > 10 && j <=16)
		{
		tx_data[6] = 0x44; // 'D'
		tx_data[7] = ConvertDigitToAscii(j-8); // from '3' to '8'
		}
		else if (j == 17)
		{
			tx_data[2] = 0x31;  // '1'  (reader number: high byte)
			tx_data[3] = 0x30;  // '0'  (reader number: low byte)
			tx_data[6] = 0x43;  // 'C'
			tx_data[7] = 0x33;  // '3'
		}

		result = write(fd, tx_data, 8);
		sleep(0.1);
		bufptr = rx_data;
		nbytes = 0;

		while ((nbytes = read(fd, bufptr,sizeof(rx_data) - 1)) > 0)
		{
			mbytes = mbytes+nbytes;
			//printf("%d\n", nbytes);
			bufptr += nbytes;
			if (mbytes >8)
			{
				*bufptr = '\0';
				//printf(":%s:%d\n", rx_data, mbytes);
				mbytes =0;
				if (rx_data[5] == 'R' || rx_data[5] == 'F' || rx_data[5] == 'T')
				{
					readerDet[NumOfReaderDet] = ConvertAsciiToNumber(tx_data[3]);         // reader number (removed +1 from tx_data[3]+1
					NumOfReaderDet++;
				}
		  } // if (mbytes >8)
		} // while ((nbytes = read(fd, buf....>0)
		//sleep(1);
	} // for (...)
	printf("\n");
	printf("%d readers connected: \n", NumOfReaderDet);
	if (NumOfReaderDet <1)
	{
		printf("Connect at least one reader and try again\n");
		exit (-1);
	}
	for (int x=0;  x < NumOfReaderDet; x++)
	{
		//printf("Reader %d connected\n",readerDet[x] );
		int rd =readerDet[x];
		std::stringstream ss;
		ss << "Reader " << rd << " connected";
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);
		ros::spinOnce();
		published = 1;
	}
	sleep(3); // 3 seconds delay to see readers detected printed on screen

	//get current time
	gettimeofday(&starttime, NULL);
	gettimeofday(&starttime_ltreq, NULL);
	gettimeofday(&starttime_ltrsp, NULL);
	gettimeofday(&starttime_dataReply, NULL);

	// tcp/ip starting -------------------------------------------
	//initialise all client_socket[] to 0 so not checked
	for (i = 0; i < max_clients; i++)
	{
		client_socket[i] = 0;
	}

	//create a master socket
	if( (master_socket = socket(AF_INET , SOCK_STREAM , 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	//set master socket to allow multiple connections , this is just a good habit, it will work without this
	if( setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) < 0 )
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

	//type of socket created
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons( PORT );

	//bind the socket to localhost port 8888
	if (bind(master_socket, (struct sockaddr *)&address, sizeof(address))<0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	printf("Listener on port %d \n", PORT);

	//try to specify maximum of 3 pending connections for the master socket
	if (listen(master_socket, 5) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}

	//accept the incoming connection
	addrlen = sizeof(address);
	puts("Waiting for connections ...");
	// tcp/ip end -------------------------------------------

	while (ros::ok())
	{
		// rfid reader start -------------------------------------
		readerNum=readerDet[reader]; // current reader i
		/* check reader IR sensor status command */
		tx_data[0] = 0x02;  // start byte
		tx_data[1] = 0x34;  // '4'
		tx_data[2] = 0X30;  // '0'  (reader number: high byte)
		tx_data[3] = ConvertDigitToAscii(readerNum); // from '0' to 'F';  (reader number: Low byte)
		tx_data[4] = 0x31;  // '1'
		tx_data[5] = 0x31;  // '1'
		readerDetFlag[reader]=0;
		reader++; // process next reader reader
		if (readerNum <=9)
		{
			tx_data[6] = 0x43; // 'C'
			tx_data[7] = ConvertDigitToAscii(readerNum+2); // from '2' to 'B'.
		}
		else if (readerNum > 9 && readerNum <=15)
		{
			tx_data[6] = 0x44; // 'D'
			tx_data[7] = ConvertDigitToAscii(readerNum-7); // from '3' to '8'
		}
		else if (readerNum == 16)
		{
			tx_data[2] = 0x31;  // '1'  (reader number: high byte)
			tx_data[3] = 0x30;  // '0'  (reader number: low byte)
			tx_data[6] = 0x43;  // 'C'
			tx_data[7] = 0x33;  // '3'
		}
		result = write(fd, tx_data, 8);

		/* read characters into string buffer */
		bufptr = rx_data;

		while ((nbytes = read(fd, bufptr,sizeof(rx_data) - 1)) > 0)
		{
			readerDetFlag[reader-1]=1;
			readerDisconnectedFlag[reader-1] =0;
			if (readerDisPrinted[reader-1]==1)
			{
				readerDisPrinted[reader-1]=0;
				//printf("Reader % dconnected \n", readerDet[reader-1]);
				int rd =readerDet[reader-1];
				std::stringstream ss;
				ss << "Reader " << rd << " connected";
				msg.data = ss.str();

				ROS_INFO("%s", msg.data.c_str());

				chatter_pub.publish(msg);
				ros::spinOnce();
				published = 1;
			}
			mbytes = mbytes+nbytes;
			//printf("%d\n", nbytes);
			bufptr += nbytes;
			if (mbytes >8)
			{
				*bufptr = '\0'; //nul terminate the string
			   //printf(":%s:%d\n", rx_data, mbytes);
			   mbytes =0;
			   if (rx_data[5] == 'R' && itemDetected[ConvertAsciiToNumber(tx_data[3])]==0) // if item first time detected
				{
				   //printf ("reader number %d:%d:%d\n", rx_data[3], ConvertAsciiToNumber(rx_data[3]), ConvertDigitToAscii(readerNum));

					/* read tag data command */
					tx_data[0] = 0x02;  // start byte
					tx_data[1] = 0x34;  // '4'
					tx_data[2] = 0X30;  // '0'  (reader number: high byte)
					tx_data[3] = tx_data[3] + 0;  // from '0' to 'F';  (reader number: Low byte)
					tx_data[4] = 0x36; // '6'
					tx_data[5] = 0x34; // '4'
					tx_data[6] = tx_data[6] + 0; //'C' or 'D' as before;
					tx_data[7] = tx_data[7] + 15;  // from '2' + 15 to '5' + 15 (50 + 15) to (55 + 15) =  65 to  70
					if (rx_data[3] >='6' && rx_data[3] <='9' )
					{
						tx_data[6] = 0x44; // 'D';
						tx_data[7] = rx_data[3] - '6';
					}
					result = write(fd, tx_data, 8);// send out read tag data command
					mbytes = 0;
					bufptr = rx_data;
					while ((nbytes = read(fd, bufptr,sizeof(rx_data) - 1)) > 0)
					{
						mbytes = mbytes+nbytes;
						//printf("%d\n", nbytes);
						bufptr += nbytes;
						if (mbytes >=31)
						{
							//printf ("reader number %d:%d:%d\n", rx_data[3], ConvertAsciiToNumber(rx_data[3]), ConvertDigitToAscii(readerNum));
							*bufptr = '\0'; //nul terminate the string
							//printf(":%s:%d\n", rx_data, mbytes);
							mbytes =0;
							itemDetected[ConvertAsciiToNumber(rx_data[3])] = 1; // set detected flag
							itemRemoved[ConvertAsciiToNumber(rx_data[3])] = 0;  // clr removed flag
							temp = ConvertAsciiToNumber(rx_data[3])*8;         // reader number * 8 bytes is memory location to be stored
							itemMemLocation[ConvertAsciiToNumber(rx_data[3])] = temp; // save item memory location
							/* store 12 bytes of data starting from item memory location */
							LotData[temp+ 0] = rx_data[5];
							LotData[temp+ 1] = rx_data[6];
							LotData[temp+ 2] = rx_data[7];
							LotData[temp+ 3] = rx_data[8];
							LotData[temp+ 4] = rx_data[9];
							LotData[temp+ 5] = rx_data[10];
							LotData[temp+ 6] = rx_data[11];
							LotData[temp+ 7] = rx_data[12];

							for (int rd = 0; rd < NumOfReaderDet; rd++)
							{
								if (ConvertAsciiToNumber(rx_data[3]) == readerDet[rd])
										sendLotDataIn[rd] = 1;
										sendLotDataInBackup[rd]=1;
							}

							/* Display item tag data */
							LotData3[0] = 'I';
							LotData3[1] = 'N';
							LotData3[2] = '0';
							LotData3[3] = '1';
							if (rx_data[3] <'A')
							{
								LotData3[4] = '0';
								LotData3[5] = rx_data[3]; // rx_data[] = '1' to '9'
							}
							else if (rx_data[3]> '9')
							{
								LotData3[4] = '1';
								LotData3[5] = rx_data[3]-'A'; // = '0' to '6'
							}
							LotData3[6] = LotData[temp+0];
							LotData3[7] = LotData[temp+1];
							LotData3[8] = LotData[temp+2];
							LotData3[9] = LotData[temp+3];
							LotData3[10] = LotData[temp+4];
							LotData3[11] = LotData[temp+5];
							LotData3[12] = LotData[temp+6];
							LotData3[13] = LotData[temp+7];
							LotData3[14] = '\0'; // Null Terminated
							//printf("item detected:%s\n", LotData3);

							std::stringstream ss;
							ss << LotData3;
							msg.data = ss.str();

							ROS_INFO("%s", msg.data.c_str());

							chatter_pub.publish(msg);
							ros::spinOnce();
							published = 1;
							break;
						} // if (mbytes >=31)
				  } // while ((nbytes = read(...))
				} // if (rx_data[5] == 'R'...)
				else if ((rx_data[5] == 'F' || rx_data[5] == 'T' ) && itemRemoved[ConvertAsciiToNumber(rx_data[3])] == 0 && itemDetected[ConvertAsciiToNumber(rx_data[3])] == 1)
				{
					//printf ("reader number %d:%d:%d\n", rx_data[3], ConvertAsciiToNumber(rx_data[3]), readerNum);
					itemRemoved[ConvertAsciiToNumber(rx_data[3])] = 1;  // set removed flag
					itemDetected[ConvertAsciiToNumber(rx_data[3])] = 0; // clr detected flag
					temp = itemMemLocation[ConvertAsciiToNumber(rx_data[3])]; // get item memory location

					for (int rd = 0; rd < NumOfReaderDet; rd++)
					{
						if (ConvertAsciiToNumber(rx_data[3]) == readerDet[rd])
								sendLotDataOut[rd] = 1;
					}

					/* Display item tag data */
					LotData3[0] = 'O';
					LotData3[1] = 'U';
					LotData3[2] = 'T';
					LotData3[3] = '0';
					LotData3[4] = '1';
					if (rx_data[3] <'A')
					{
						LotData3[5] = '0';
						LotData3[6] = rx_data[3]; // rx_data[] = '1' to '9';
					}
					else if (rx_data[3] > '9')
					{
						LotData3[5] = '1';
						LotData3[6] = rx_data[3]-'A'; // = '0' to '6';
					}
					LotData3[7] = LotData[temp+0];
					LotData3[8] = LotData[temp+1];
					LotData3[9] = LotData[temp+2];
					LotData3[10] = LotData[temp+3];
					LotData3[11] = LotData[temp+4];
					LotData3[12] = LotData[temp+5];
					LotData3[13] = LotData[temp+6];
					LotData3[14] = LotData[temp+7];
					LotData3[15] = '\0'; // Null Terminated
					//printf("item removed:%s\n", LotData3);

					std::stringstream ss;
					ss << LotData3;
					msg.data = ss.str();
					ROS_INFO("%s", msg.data.c_str());
					chatter_pub.publish(msg);
					ros::spinOnce();
					published =1;

				} // else if (rx_data[5] ==
			} // if (mbytes > 8)
//			if (published ==1)
//			{
//				  ROS_INFO("%s", msg.data.c_str());
//				  chatter_pub.publish(msg);
//				  ros::spinOnce();
//			}
		//	printf ("Do nothing \n");
		} //  while (nbytes = read(....) >0)
		if (reader>=NumOfReaderDet)
		{
			reader=0;  // re-set to first reader
			for (int k =0; k<NumOfReaderDet; k++)
			{
				if(readerDetFlag[k]==0 )
					readerDisconnectedFlag[k] = readerDisconnectedFlag[k] + 1;
					if (readerDisconnectedFlag[k] >10) // if detected absent for 10 times
						if (readerDisPrinted[k]==0)
						{
							//printf("Reader  %d disconnected/powered off or wrong port/baud rate\n", readerDet[k]);
							readerDisPrinted[k] = 1;
							itemRemoved[readerDet[k]] = 0;  // reset removed flag
							itemDetected[readerDet[k]] = 0; // reset detected flag
							int rd=readerDet[k];
							std::stringstream ss;
							ss << "Reader " << rd << " disconnected/powered off or wrong port/baud rate";
							msg.data = ss.str();

							ROS_INFO("%s", msg.data.c_str());

							chatter_pub.publish(msg);
							ros::spinOnce();
							published = 1;
						}
			}
		}
		// rfid reader end ----------------------------------------------------------------------

		// tcp/ip start  ----------------------------------------------------------------------
		//clear the socket set
		FD_ZERO(&readfds);

		//add master socket to set
		FD_SET(master_socket, &readfds);
		max_sd = master_socket;

		//add child sockets to set
		for ( i = 0 ; i < max_clients ; i++)
		{
			//socket descriptor
			sd = client_socket[i];

			//if valid socket descriptor then add to read list
			if(sd > 0)
				FD_SET( sd , &readfds);

			//highest file descriptor number, need it for the select function
			if(sd > max_sd)
				max_sd = sd;
		}
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 100000;

		//wait for an activity on one of the sockets
		activity = select( max_sd + 1 , &readfds , NULL , NULL , &tv);

		if ((activity <= 0) && (errno!=EINTR))
		{
			//printf("select error\n");
		}

		//If something happened on the master socket , then its an incoming connection
		if (FD_ISSET(master_socket, &readfds))
		{
			if ((new_socket = accept(master_socket, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0)
			{
				perror("accept");
				exit(EXIT_FAILURE);
			}

			//inform user of socket number - used in send and receive commands
			printf("New connection , socket fd is %d , ip is : %s , port : %d \n" , new_socket , inet_ntoa(address.sin_addr) , ntohs(address.sin_port));

			//add new socket to array of sockets
			for (i = 0; i < max_clients; i++)
			{
				//if position is empty
				if( client_socket[i] == 0 )
				{
					client_socket[i] = new_socket;
					printf("Adding to list of sockets as %d\n" , i);

					break;
				}
			}
		}

		//else its some IO operation on some other socket :)
		for (i = 0; i < max_clients; i++)
		{
			sd = client_socket[i];

			if (FD_ISSET( sd , &readfds))
			{
				//Check if it was for closing , and also read the incoming message
				if ((valread = read( sd , buffer, 4)) == 0)
				{
					//Somebody disconnected , get his details and print
					getpeername(sd , (struct sockaddr*)&address , (socklen_t*)&addrlen);
					printf("Host disconnected , ip %s , port %d \n" , inet_ntoa(address.sin_addr) , ntohs(address.sin_port));

					//Close the socket and mark as 0 in list for reuse
					close( sd );
					client_socket[i] = 0;

				}

				//Echo back the message that came in
				else
				{
					valread = read( sd , buffer2, buffer[3]);
					//set the string terminating NULL byte on the end of the data read
					//buffer[4] = '\0';
					if (buffer2[5]==1)
					{
						send(sd , msgLen1, 4 , 0 );
						selectrsp[6]=buffer2[6];
						selectrsp[7]=buffer2[7];
						selectrsp[8]=buffer2[8];
						selectrsp[9]=buffer2[9];
						send(sd , selectrsp , 10 , 0 );
						selectdone = 1;
						linktestalive=1;
						printf("Received Select\n");
					}
					else if (buffer2[5]==5) // Link Test request
					{
						send(sd , msgLen1, 4 , 0 );
						linktestrsp[6]=buffer2[6];
						linktestrsp[7]=buffer2[7];
						linktestrsp[8]=buffer2[8];
						linktestrsp[9]=buffer2[9];
						send(sd , linktestrsp , 10 , 0 );
						printf("Received Link Test request\n");
					}
					else if (buffer2[5]==9) // Separate request
					{
						selectdone = 0;
						linktestalive=0;
						printf("Received Separate request\n");
						close( sd );
					}
					else if (buffer2[5]==3) // Deselect request
					{
						send(sd , msgLen1, 4 , 0 );
						deselectrsp[6]=buffer2[6];
						deselectrsp[7]=buffer2[7];
						deselectrsp[8]=buffer2[8];
						deselectrsp[9]=buffer2[9];
						send(sd , deselectrsp , 10 , 0 );
						selectdone = 0;
						linktestalive=0;
						printf("Received de-Select request\n");
					}
					else if (buffer2[5]==6) // Link Test response
					{
						linktestalive=1;
						printf("Received Link Test response\n");
					}
					else if (buffer2[2] == 0x81 && buffer2[3]==1)  // s1f1
					{
						send(sd , msgLen3, 4 , 0 );
						S1F2header[6]=buffer2[6];
						S1F2header[7]=buffer2[7];
						S1F2header[8]=buffer2[8];
						S1F2header[9]=buffer2[9];
						send(sd , S1F2header,10 , 0 );
						send(sd , S1F2data,6 , 0 );
						printf("Received s1f1 command\n");
					}

					else if (buffer2[2] == 0x81 && buffer2[3]==5)  // s1f5
					{
						send(sd , msgLen1, 4 , 0 );
						S1F6header[6]=buffer2[6];
						S1F6header[7]=buffer2[7];
						S1F6header[8]=buffer2[8];
						S1F6header[9]=buffer2[9];
						send(sd , S1F6header,10 , 0 );
						printf("%s\n", S1F6header);
						startup_or_s1f5=true;
						for (int rd =0; rd<3; rd++)
						{
							sendLotDataIn[rd] = sendLotDataInBackup[rd];
						}
						printf("Received s1f5 command\n");

					}
					else if (buffer2[2]==0x12&& buffer2[3]==0x4C)  // data reply
					{
						if(valread < 15)
							valread = read( sd , buffer2, (15-valread));
						dataReply=1;
						printf("Received data reply \n");
					}

				}// else

			} //  if (FD_ISSET( sd , &readfds))

			// Sending Link Test Request
			gettimeofday(&endtime_ltreq, NULL);
			seconds  = endtime_ltreq.tv_sec  - starttime_ltreq.tv_sec;
			useconds = endtime_ltreq.tv_usec - starttime_ltreq.tv_usec;
			elapsedtime_ltreq = ((seconds) * 1000 + useconds/1000.0) + 0.5;
			//printf("Elapsed time for sending linktestreq: %ld milliseconds\n", elapsedtime_ltreq);

			if ( (selectdone == 1) && elapsedtime_ltreq>=10000)
			{
				gettimeofday(&starttime_ltreq, NULL);
				gettimeofday(&starttime_ltrsp, NULL);
				send(sd , msgLen1, 4 , 0 );
				transactionID.transactionID_short++;
				if (transactionID.transactionID_short > 65534)
				transactionID.transactionID_short = 1;
				linktestreq[8]= transactionID.transactionID_byte[1];
				linktestreq[9]= transactionID.transactionID_byte[0];
				send(sd , linktestreq , 10 , 0 );
				linktestalive=0;
			}

			gettimeofday(&endtime, NULL);

			seconds  = endtime.tv_sec  - starttime.tv_sec;
			useconds = endtime.tv_usec - starttime.tv_usec;
			mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;

			//printf("Elapsed time for sending data: %ld milliseconds\n", mtime);

			if ( (selectdone == 1) && mtime >= 30000 )
			{
				gettimeofday(&starttime, NULL);

			}

			// Checking if there are Data message to be sent out
			if (selectdone ==1 && linktestalive == 1 )
			{
				for (int x = 0; x <NumOfReaderDet; x++ )
				{
					if (startup_or_s1f5==true && sendLotDataInBackup[x]==1)
					{
						n.getParam("TransporterID",TransporterID);
						int TransporterIDLen = strlen(TransporterID.c_str());
						//printf("TransporterID length: %d\n", TransporterIDLen);
						//printf("TransporterID: %s\n", TransporterID.c_str());
						msgLen2[3]=(0x21 + TransporterIDLen);
						send(sd, msgLen2, 4, 0);

						transactionID.transactionID_short++;
						if (transactionID.transactionID_short > 65534)
							transactionID.transactionID_short = 1;
						S18F75header[8]= transactionID.transactionID_byte[1];
						S18F75header[9]= transactionID.transactionID_byte[0];
						send(sd, S18F75header, 10, 0);

						LotData2[0]=0x01;
						LotData2[1]=0x04;
						LotData2[2]=0x41;
						LotData2[3]=0x03;
						LotData2[4] = 'O';
						LotData2[5] = 'U';
						LotData2[6] = 'T';

						LotData2[7]=0x41;
						LotData2[8]=TransporterIDLen;
						char *ptr = &LotData2[9];
						sprintf(ptr,TransporterID.c_str());
						LotData2[9+TransporterIDLen]=0x41;
						LotData2[10+TransporterIDLen]=0x02;

						if (rx_data[3] <'A')

						if (readerDet[x]<10)
						{
							LotData2[11+TransporterIDLen] = '0';
							LotData2[12+TransporterIDLen] = ConvertDigitToAscii (readerDet[x]); // = '1' to '9'
						}
						else if (readerDet[x]> 9)
						{
							LotData2[11+TransporterIDLen] = '1';
							LotData2[12+TransporterIDLen] = ConvertDigitToAscii (readerDet[x])-'A'; // = '0' to '6'
						}

						LotData2[13+TransporterIDLen]=0x41;
						LotData2[14+TransporterIDLen]=0x08;

						temp = readerDet[x]*8;         // reader number * 8 bytes is memory location to be stored

						LotData2[15+TransporterIDLen] = LotData[temp+0];
						LotData2[16+TransporterIDLen] = LotData[temp+1];
						LotData2[17+TransporterIDLen] = LotData[temp+2];
						LotData2[18+TransporterIDLen] = LotData[temp+3];
						LotData2[19+TransporterIDLen] = LotData[temp+4];
						LotData2[20+TransporterIDLen] = LotData[temp+5];
						LotData2[21+TransporterIDLen] = LotData[temp+6];
						LotData2[22+TransporterIDLen] = LotData[temp+7];
						LotData2[23+TransporterIDLen] = '\0'; // Null Terminated

						//send(sd, LotData2, 25, 0);
						send(sd, LotData2,(23+TransporterIDLen), 0);
						dataMsgSent=1;
						dataReply=0;
						gettimeofday(&starttime_dataReply, NULL);
					}
					else if ( sendLotDataOut[x]==1)
					{
						sendLotDataOut[x]=0;
						n.getParam("TransporterID",TransporterID);
						int TransporterIDLen = strlen(TransporterID.c_str());
						//printf("TransporterID length: %d\n", TransporterIDLen);
						//printf("TransporterID: %s\n", TransporterID.c_str());
						msgLen2[3]=(0x21 + TransporterIDLen);
						send(sd, msgLen2, 4, 0);

						transactionID.transactionID_short++;
						if (transactionID.transactionID_short > 65534)
							transactionID.transactionID_short = 1;
						S18F75header[8]= transactionID.transactionID_byte[1];
						S18F75header[9]= transactionID.transactionID_byte[0];
						send(sd, S18F75header, 10, 0);

						LotData2[0]=0x01;
						LotData2[1]=0x04;
						LotData2[2]=0x41;
						LotData2[3]=0x03;
						LotData2[4] = 'O';
						LotData2[5] = 'U';
						LotData2[6] = 'T';

						LotData2[7]=0x41;
						LotData2[8]=TransporterIDLen;
						char *ptr = &LotData2[9];
						sprintf(ptr,TransporterID.c_str());
						LotData2[9+TransporterIDLen]=0x41;
						LotData2[10+TransporterIDLen]=0x02;

						if (rx_data[3] <'A')

						if (readerDet[x]<10)
						{
							LotData2[11+TransporterIDLen] = '0';
							LotData2[12+TransporterIDLen] = ConvertDigitToAscii (readerDet[x]); // = '1' to '9'
						}
						else if (readerDet[x]> 9)
						{
							LotData2[11+TransporterIDLen] = '1';
							LotData2[12+TransporterIDLen] = ConvertDigitToAscii (readerDet[x])-'A'; // = '0' to '6'
						}

						LotData2[13+TransporterIDLen]=0x41;
						LotData2[14+TransporterIDLen]=0x08;

						temp = readerDet[x]*8;         // reader number * 8 bytes is memory location to be stored

						LotData2[15+TransporterIDLen] = LotData[temp+0];
						LotData2[16+TransporterIDLen] = LotData[temp+1];
						LotData2[17+TransporterIDLen] = LotData[temp+2];
						LotData2[18+TransporterIDLen] = LotData[temp+3];
						LotData2[19+TransporterIDLen] = LotData[temp+4];
						LotData2[20+TransporterIDLen] = LotData[temp+5];
						LotData2[21+TransporterIDLen] = LotData[temp+6];
						LotData2[22+TransporterIDLen] = LotData[temp+7];
						LotData2[23+TransporterIDLen] = '\0'; // Null Terminated

						//send(sd, LotData2, 25, 0);
						send(sd, LotData2,(23+TransporterIDLen), 0);
						dataMsgSent=1;
						dataReply=0;
						gettimeofday(&starttime_dataReply, NULL);
					}
				}  //for (int x = 0; x <NumOfReaderDet; x++ )
				//startup_or_s1f5=false;

				for (int x = 0; x <NumOfReaderDet; x++ )
				{
					if (sendLotDataIn[x]==1)
					{
						sendLotDataIn[x]=0;
						n.getParam("TransporterID",TransporterID);
						int TransporterIDLen = strlen(TransporterID.c_str());
						//printf("TransporterID length: %d\n", TransporterIDLen);
						//printf("TransporterID: %s\n", TransporterID.c_str());
						msgLen2[3]=(0x20 + TransporterIDLen);
						send(sd, msgLen2, 4, 0);

						transactionID.transactionID_short++;
						if (transactionID.transactionID_short > 65534)
							transactionID.transactionID_short = 1;
						S18F75header[8]= transactionID.transactionID_byte[1];
						S18F75header[9]= transactionID.transactionID_byte[0];
						send(sd, S18F75header, 10, 0);

						LotData2[0]=0x01;
						LotData2[1]=0x04;
						LotData2[2]=0x41;
						LotData2[3]=0x02;
						LotData2[4] = 'I';
						LotData2[5] = 'N';
						LotData2[6]=0x41;
						LotData2[7]=TransporterIDLen;
						char *ptr = &LotData2[8];
						sprintf(ptr,TransporterID.c_str());
						LotData2[8+TransporterIDLen]=0x41;
						LotData2[9+TransporterIDLen]=0x02;
						if (readerDet[x]<10)
						{
							LotData2[10+TransporterIDLen] = '0';
							LotData2[11+TransporterIDLen] = ConvertDigitToAscii (readerDet[x]); // = '1' to '9'
						}
						else if (readerDet[x]> 9)
						{
							LotData2[10+TransporterIDLen] = '1';
							LotData2[11+TransporterIDLen] = ConvertDigitToAscii (readerDet[x])-'A'; // = '0' to '6'
						}

						LotData2[12+TransporterIDLen]=0x41;
						LotData2[13+TransporterIDLen]=0x08;

						temp = readerDet[x]*8;         // reader number * 8 bytes is memory location to be stored
						LotData2[14+TransporterIDLen] = LotData[temp+0];
						LotData2[15+TransporterIDLen] = LotData[temp+1];
						LotData2[16+TransporterIDLen] = LotData[temp+2];
						LotData2[17+TransporterIDLen] = LotData[temp+3];
						LotData2[18+TransporterIDLen] = LotData[temp+4];
						LotData2[19+TransporterIDLen] = LotData[temp+5];
						LotData2[20+TransporterIDLen] = LotData[temp+6];
						LotData2[21+TransporterIDLen] = LotData[temp+7];
						LotData2[22+TransporterIDLen] = '\0'; // Null Terminated

						//send(sd, LotData2, 24, 0);
						send(sd, LotData2,(22+TransporterIDLen), 0);

						if (startup_or_s1f5== true)
						{
							startup_or_s1f5=false;
							// sending S1F7
							send(sd, msgLen1, 4, 0);

							transactionID.transactionID_short++;
							if (transactionID.transactionID_short > 65534)
								transactionID.transactionID_short = 1;
							S1F7header[8]= transactionID.transactionID_byte[1];
							S1F7header[9]= transactionID.transactionID_byte[0];
							send(sd, S1F7header, 10, 0);
							printf("Sending S1F7: %s\n", S1F7header);
						}

						dataMsgSent=1;
						dataReply=0;
						gettimeofday(&starttime_dataReply, NULL);
					}
				} //for (int x = 0; x <NumOfReaderDet; x++ )
			}

			// Checking Data Reply time out
			gettimeofday(&endtime_dataReply, NULL);
			seconds  = endtime_dataReply.tv_sec  - starttime_dataReply.tv_sec;
			useconds = endtime_dataReply.tv_usec - starttime_dataReply.tv_usec;
			elapsedtime_dataReply = ((seconds) * 1000 + useconds/1000.0) + 0.5;
			//printf("Elapsed time: %ld milliseconds %d:%d\n", elapsedtime_ltreq, dataMsgSent, dataReply);

			if (dataMsgSent==1 && elapsedtime_dataReply >= 15000 && dataReply==0)
			{
				printf ("Data Reply Timeout\n");
				gettimeofday(&starttime_dataReply, NULL);
			}

			// Checking Link Test Response Timeout
			gettimeofday(&endtime_ltrsp, NULL);
			seconds  = endtime_ltrsp.tv_sec  - starttime_ltrsp.tv_sec;
			useconds = endtime_ltrsp.tv_usec - starttime_ltrsp.tv_usec;
			elapsedtime_ltrsp = ((seconds) * 1000 + useconds/1000.0) + 0.5;
			//printf("Elapsed time for receiving linktestrsp: %ld milliseconds\n", elapsedtime_ltrsp);
			if (elapsedtime_ltrsp>=10000 && linktestalive==0)
			{
				//sleep(0.1); // for testing
				// do something about it
				// selectdone =0; // just an example to simulate stop sending
				printf("Link Test Timeout\n");
				gettimeofday(&starttime_ltrsp, NULL);
			}
		} // for
		// tcp/ip end  ----------------------------------------------------------------------
	} // while (1)
} // main()

unsigned char ConvertAsciiToNumber(unsigned char AsciiByte)
{
	switch (AsciiByte)
	{
		// Upper case:
		case 'A': return 10;
		case 'B': return 11;
		case 'C': return 12;
		case 'D': return 13;
		case 'E': return 14;
		case 'F': return 15;

		// Numbers:
		case '0': return 0;
		case '1': return 1;
		case '2': return 2;
		case '3': return 3;
		case '4': return 4;
		case '5': return 5;
		case '6': return 6;
		case '7': return 7;
		case '8': return 8;
		case '9': return 9;

		default: return 0;
	}
}

unsigned char ConvertDigitToAscii(unsigned char Digit)
{
	unsigned char AsciiByte;

	if( (Digit < 10) && (Digit >= 0) )
	{
		AsciiByte = Digit | 0x30;
	}

	switch(Digit)
	{
		case 10:
			AsciiByte = 'A';
			break;

		case 11:
			AsciiByte = 'B';
			break;

		case 12:
			AsciiByte = 'C';
			break;

		case 13:
			AsciiByte = 'D';
			break;

		case 14:
			AsciiByte = 'E';
			break;

		case 15:
			AsciiByte = 'F';
			break;
	}
	return AsciiByte;
}
