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
 *      Last updated: 18/7/17:
 *      - Re calucate tx_data[6] & tx_data[7] for reader 6 to 9
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
using namespace std;

unsigned char ConvertDigitToAscii(unsigned char Digit);
unsigned char ConvertAsciiToNumber(unsigned char AsciiByte);

unsigned char tx_data[8], rx_data[255];  // to store transmitted or received data
unsigned char *bufptr;      // Current char in buffer
int  mbytes=0, nbytes;       // Number of bytes read
int tries = 0; // to store number of TCPIP tries
int  readerNum=0; // to store reader number
unsigned char itemDetected[20], itemRemoved[20]; // to store item deteced / removed flags
unsigned char itemMemLocation[20]; // to store all 16 memory location values
unsigned char temp=0; // to store temp item memory location
unsigned char LotData[192]; // to store all 16 lots of 8 bytes data each (16*8 = 128 bytes)
unsigned char LotData2[13]; // LOT data + other data to be published or sent to TCPIP server
//unsigned char readerDetected = 0; // to store reader detected flag
//unsigned char firstDetected =0; // to store first time reader detected flag
unsigned char published = 0; // to store published flag
int result; // return value for read/write/send ...
//char *SerialComPort; // Serial comport number
std::string SerialComPort; 
//char *TCPIPaddr; // TCPIP internet address
std::string TCPIPaddr;
int TCPIPportNum = 27015;  //Internet Port number
unsigned char readerDet [20]; // to strore reader number detected
unsigned char readerDetFlag[20]; // Reader detected flag
unsigned char  readerDisconnectedFlag[20]; // Reader disconnected flag
unsigned char NumOfReaderDet = 0; // Number of readers detected after start up
int i=0;



int main(int argc, char *argv[ ] )
{
	/* comment out for using n.param in launch file rfid.launch
	if (argc !=4)
	{
		printf("Please enter: Filename SerialComportNum IPaddress IPportNumber\n");
		exit (-1);
	}
	SerialComPort=argv[1];
	TCPIPaddr=argv[2];
	TCPIPportNum=atoi(argv[3]);
	printf("Serial ComPort: %s\n", SerialComPort);
	printf("TCP/IP Address: %s\n", TCPIPaddr);
	printf("TCP/IP Port Number: %d\n",TCPIPportNum);
*/
	ros::init(argc, argv, "rfid");
	ros::NodeHandle n("~");
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("rfid", 1000);
	ros::Rate loop_rate(1);
	int count = 0;

	//n.param("TCPIPaddr",TCPIPaddr, "192.168.1.110");
	n.getParam("TCPIPaddr",TCPIPaddr);
	n.getParam("TCPIPportNum",TCPIPportNum);
	//n.param("SerialComPort",SerialComPort, "/dev/sensors/RFID");
	n.getParam("SerialComPort",SerialComPort);
	//ROS_INFO("Port name %s", TCPIPaddr.c_str());
	int sock; // socket return value
	struct sockaddr_in server;
	int fd; // file description
	struct termios options;
	/* open the port */
	//fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
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

	for (i = 0; i<20; i++)
	{
		itemDetected[i] = 0;
		itemRemoved[i]=0;
	}
	i=0;

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
					readerDet[NumOfReaderDet] = ConvertAsciiToNumber(tx_data[3]+1);         // reader number
					NumOfReaderDet++; //i++;
				}
		  } // if (mbytes >8)
		} // while ((nbytes = read(fd, buf....>0)
		//sleep(1);
	} // for (...)
	printf("\n");
	printf("%d readers detected: \n", NumOfReaderDet);
	if (NumOfReaderDet <1)
	{
		printf("Connect at least one reader and try again\n");
		exit (-1);
	}
	for (int x=0;  x < NumOfReaderDet; x++)
		printf("Reader %d detected\n",readerDet[x] );
	sleep(3); // 3 seconds delay to see readers detected printed on screen
	i=0;

	std_msgs::String msg;
	std::stringstream ss;

	while (ros::ok())
	{
		readerNum=readerDet[i]; // current reader i
		/* check reader IR sensor status command */
		tx_data[0] = 0x02;  // start byte
		tx_data[1] = 0x34;  // '4'
		tx_data[2] = 0X30;  // '0'  (reader number: high byte)
		tx_data[3] = ConvertDigitToAscii(readerNum-1); // from '0' to 'F';  (reader number: Low byte)
		tx_data[4] = 0x31;  // '1'
		tx_data[5] = 0x31;  // '1'
		readerDetFlag[i]=0;
		i++; // process next reader i
		if (readerNum <=10)
		{
			tx_data[6] = 0x43; // 'C'
			tx_data[7] = ConvertDigitToAscii(readerNum+1); // from '2' to 'B'.
		}
		else if (readerNum > 10 && readerNum <=16)
		{
			tx_data[6] = 0x44; // 'D'
			tx_data[7] = ConvertDigitToAscii(readerNum-8); // from '3' to '8'
		}
		else if (readerNum == 17)
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
			readerDetFlag[i-1]=1;
			readerDisconnectedFlag[i-1] =0;
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
							*bufptr = '\0'; //nul terminate the string
							//printf(":%s:%d\n", rx_data, mbytes);
							mbytes =0;

							itemDetected[ConvertAsciiToNumber(tx_data[3])] = 1; // set detected flag
							itemRemoved[ConvertAsciiToNumber(tx_data[3])] = 0;  // clr removed flag
							temp = ConvertAsciiToNumber(tx_data[3])*8;         // reader number * 8 bytes is memory location to be stored
							itemMemLocation[ConvertAsciiToNumber(tx_data[3])] = temp; // save item memory location
							/* store 12 bytes of data starting from item memory location */
							LotData[temp+ 0] = rx_data[5];
							LotData[temp+ 1] = rx_data[6];
							LotData[temp+ 2] = rx_data[7];
							LotData[temp+ 3] = rx_data[8];
							LotData[temp+ 4] = rx_data[9];
							LotData[temp+ 5] = rx_data[10];
							LotData[temp+ 6] = rx_data[11];
							LotData[temp+ 7] = rx_data[12];

							/* Display item tag data */
							LotData2[0] = 'I';
							LotData2[1] = 'N';
							LotData2[2] = '0';
							LotData2[3] = '1';
							if (readerNum <10)
							{
								LotData2[4] = '0';
								LotData2[5] = ConvertDigitToAscii(readerNum);
							}
							else if (readerNum> 9)
							{
								LotData2[4] = '1';
								LotData2[5] = ConvertDigitToAscii(readerNum-10);
							}
							LotData2[6] = LotData[temp+0];
							LotData2[7] = LotData[temp+1];
							LotData2[8] = LotData[temp+2];
							LotData2[9] = LotData[temp+3];
							LotData2[10] = LotData[temp+4];
							LotData2[11] = LotData[temp+5];
							LotData2[12] = LotData[temp+6];
							LotData2[13] = LotData[temp+7];
							LotData2[14] = '\0'; // Null Terminated
							printf("item detected:%s\n", LotData2);
							for (tries = 0; tries < 10; tries++ )
							{
								sock = socket(AF_INET , SOCK_STREAM , 0);
								if (sock == -1)
								{
									printf("Could not create TCP/IP socket");
									sleep(1);
									continue; //break;
								}
								puts("TCP/IP socket created");

								server.sin_addr.s_addr = inet_addr(TCPIPaddr.c_str());
								//server.sin_addr.s_addr = inet_addr("172.30.74.105");
								//server.sin_addr.s_addr = inet_addr("172.30.68.103");
								server.sin_family = AF_INET;
								//server.sin_port = htons( 27015 );
								server.sin_port = htons( TCPIPportNum );

								//Connect to remote server
								if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
								{
									perror("TCP/IP connect failed. Error");
									sleep(1);
									continue; //break; //return 1;
								}

								puts("TCP/IP connected\n");
								if ((result = send(sock, LotData2,14, 0) )<0)
								{
									printf("TCP/IP send failed\n");
									sleep(1);
									continue;
								}
								close(sock);

								std::stringstream ss;
								ss << LotData2;
								msg.data = ss.str();

								ROS_INFO("%s", msg.data.c_str());

								chatter_pub.publish(msg);
								ros::spinOnce();
								published = 1;
								break;
							} // for (...)
							break;
						} // if (mbytes >=31)
				  } // while ((nbytes = read(...))
				} // if (rx_data[5] == 'R'...)
				else if ((rx_data[5] == 'F' || rx_data[5] == 'T' ) && itemRemoved[ConvertAsciiToNumber(rx_data[3])] == 0 && itemDetected[ConvertAsciiToNumber(rx_data[3])] == 1)
				{
					itemRemoved[ConvertAsciiToNumber(rx_data[3])] = 1;  // set removed flag
					itemDetected[ConvertAsciiToNumber(rx_data[3])] = 0; // clr detected flag
					temp = itemMemLocation[ConvertAsciiToNumber(rx_data[3])]; // get item memory location

					/* Display item tag data */
					LotData2[0] = 'O';
					LotData2[1] = 'U';
					LotData2[2] = 'T';
					LotData2[3] = '0';
					LotData2[4] = '1';
					if (readerNum <10)
					{
						LotData2[5] = '0';
						LotData2[6] = ConvertDigitToAscii(readerNum);
					}
					else if (readerNum> 9)
					{
						LotData2[5] = '1';
						LotData2[6] = ConvertDigitToAscii(readerNum-10);
					}
					LotData2[7] = LotData[temp+0];
					LotData2[8] = LotData[temp+1];
					LotData2[9] = LotData[temp+2];
					LotData2[10] = LotData[temp+3];
					LotData2[11] = LotData[temp+4];
					LotData2[12] = LotData[temp+5];
					LotData2[13] = LotData[temp+6];
					LotData2[14] = LotData[temp+7];
					LotData2[15] = '\0'; // Null Terminated
					printf("item removed:%s\n", LotData2);
					for (tries=0; tries<10; tries++)
					{
						//Create socket
						sock = socket(AF_INET , SOCK_STREAM , 0);
						if (sock == -1)
						{
							printf("Could not create TCP/IP socket");
							sleep(1);
							continue;
						}
						puts("TCP/IP Socket created");

						server.sin_addr.s_addr = inet_addr(TCPIPaddr.c_str());
						//server.sin_addr.s_addr = inet_addr("172.30.74.105");
						//server.sin_addr.s_addr = inet_addr("172.30.68.103");
						server.sin_family = AF_INET;
						//server.sin_port = htons( 27015 );
						server.sin_port = htons(TCPIPportNum );

						//Connect to remote server
						if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
						{
							perror("TCP/IP connect failed. Error");
							sleep(1);
							continue; //break; //return 1;
						}
						puts("TCP/IP connected\n");
						if ((result = send(sock, LotData2,15, 0) )<0)
						{
							printf("TCP/IP send failed\n");
							sleep(1);
							continue;
						}
						close(sock);

						std::stringstream ss;
						ss << LotData2;
						msg.data = ss.str();
						ROS_INFO("%s", msg.data.c_str());
						chatter_pub.publish(msg);
						ros::spinOnce();
						published =1;

						break;
					} // for (tries=0; ....)
				} // else if (rx_data[5] ==
			} // if (mbytes > 8)
			/*if (published ==1)
			{
				  ROS_INFO("%s", msg.data.c_str());
				  chatter_pub.publish(msg);
				  ros::spinOnce();
			}*/
			//printf ("Do nothing \n");
		} //  while (nbytes = read(....) >0)
		if (i>=NumOfReaderDet)
		{
			i=0;  // re-set to first reader
			for (int k =0; k<NumOfReaderDet; k++)
			{
				if(readerDetFlag[k]==0)
					readerDisconnectedFlag[k] = readerDisconnectedFlag[k] + 1;
					if (readerDisconnectedFlag[k] >10) // if detected absent for 10 times
						printf("Reader:  %d disconnected/powered off or wrong port/baud rate\n", readerDet[k]);
			}
		}
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
