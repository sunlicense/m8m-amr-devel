#ifndef EPOS2_H
#define EPOS2_H

/*
 * A class for interfacing to the epos controller
 */

#include <fstream>
#include <stdexcept>
#include <stdint.h>

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <stdlib.h>
#include <sys/time.h>
#include <sstream>
#include <fcntl.h>
#include <iostream>

class EPOS
{	
	// serial port 
	int fd;
	const char *port;
	struct termios oldtio,newtio;

public:
	// Enumeration of possible EPOS operational mode
 	enum OperationMode {
		STEPMODE = -6, 
		MASTER_ENCODER=-5, 
		DIAGNOSTIC=-4,
		CURRENT=-3,
		VELOCITY=-2,
		PROFILE_VELOCITY=3,
		PROFILE_POSITION=1,
		HOMING=6
	};

	// Enumeration of possible Motor Type
 	enum MotorType {
		BRUSHED_DC = 1, 
		EC_SINUS = 10, 
		EC_BLOCK = 11
	};

	enum EncoderType {
		INCR_WITH_INDEX=1,
		INCR_NO_INDEX,
		HALL_SENSOR
	};

	enum MotionProfileType {
		TRAPEZOIDAL=0,
		SINUSOIDAL=1
	};

	// epos properties
	short CurrentRegulatorPGain;
 	short CurrentRegulatorIGain;
 	short VelocityPGain;
 	short VelocityIGain;
 	short PositionPGain;
 	short PositionIGain;
 	short PositionDGain;

 	unsigned short RS232BaudRate;
	unsigned short DisableOpOptionCode;
	short CurrentModeSetting;
 	unsigned short ThermalTimeConstantWinding;
	unsigned short MotorMaxContinuousCurrent;
 	unsigned short MotorMaxPeakCurrent;
 	unsigned int ProfileVelocity;
 	unsigned int MaxProfileVelocity;
	unsigned int MaxProfileVelocityMD;
 	unsigned int ProfileAcceleration;
 	unsigned int ProfileDeceleration;
 	unsigned int MaxFollowError;
 	unsigned int PositionWindow;
 	unsigned short PositionWindowTime;

	unsigned short IncrementalEncoderCount; // pulse per turn
 	unsigned short PositionSensorType;
 	unsigned short MotorType;

 	short ActualCurrent;
 	short AverageCurrent;
 	int ActualPosition;
	int ActualVelocity;
 	int PositionProfileTargetPosition;
 	unsigned short StatusWord;
 	unsigned short ControlWord;
 	char OperationMode;
 	char MotionProfileType;
	unsigned char NodeID;

	unsigned short data[10];
	//unsigned char nodeid;

	EPOS();
 	~EPOS();

 	int openPort(const char *port_name,unsigned short baud);
	bool closePort();

	// Basic commands to read and write objects in EPOS70

	// calculate CRC
	unsigned short CRCCalc(unsigned short data[],unsigned short len);
	// check for Acknowledge
	bool checkAck();
	// check for reply opcode
	bool checkReplyOpCode();

	// read data
	bool readData(unsigned char buf[],int len);
	// read object in EPOS70
	bool readObject(unsigned char inxhigh, unsigned char inxlow, unsigned char subindex,unsigned char bparam[],int& lx);
	// write into object in EPOS70
	bool writeObject(unsigned char inxhigh, unsigned char inxlow, unsigned char subindex,unsigned char bparam[]);

	// get error
	unsigned int getError(unsigned char param[]);
	
	bool sendNMTService(unsigned char nidxhigh, unsigned char nidxlow, unsigned char cmdhigh,unsigned char cmdlow);

	// Configuration commands
	void GetCurrentRegulatorPGain();
	void SetCurrentRegulatorPGain(short pgain);
	void GetCurrentRegulatorIGain();
	void SetCurrentRegulatorIGain(short igain);
	void GetEncoderCount();
	void SetEncoderCount(unsigned short count);
	void GetEncoderType();
	void SetEncoderType(unsigned short type);
	void GetMotorType();
	void SetMotorType(unsigned short type);
	void GetMotorMaxContCurrent();
	void SetMotorMaxContCurrent(unsigned short amp);
	void GetMotorMaxPeakCurrent();
	void SetMotorMaxPeakCurrent(unsigned short amp);
	void GetMaxFollowError();
	void SetMaxFollowError(unsigned int error);
	void SetMaxProfileVelocity(unsigned int velocity);
	bool SetMotionProfileType(char profileType);
	void GetPositionPGain();
	void SetPositionPGain(short gain);
	void GetPositionIGain();
	void SetPositionIGain(short gain);
	void GetPositionDGain();
	void SetPositionDGain(short gain);
	void GetVelocityPGain();
	void SetVelocityPGain(short gain);
	void GetVelocityIGain();
	void SetVelocityIGain(short gain);
	bool GetStatusWord();
	void SetStatusWord(unsigned short statuswd);
	bool GetControlWord();
	bool SetControlWord(unsigned short controlwd);
	void GetCurrentModeSetting();
	void SetCurrentModeSetting(short current);
	void GetThermalTimeConstantWinding();
	void SetThermalTimeConstantWinding(unsigned short time);
	void GetRS232BaudRate();
	bool SetRS232BaudRate(unsigned short rate);
	bool SetDisableOpOptionCode(unsigned short code);

	// Motion Information commands
	void GetCurrent();
	void GetAverageCurrent();
	bool GetPosition();
	bool GetVelocity();
	void GetProfileAcceleration();
	void SetProfileAcceleration(unsigned int acceleration);
	void GetProfileDeceleration();
	void SetProfileDeceleration(unsigned int deceleration);

	// Profile Mode commands
	void GetProfileVelocity();
	void SetProfileVelocity(unsigned int velocity);
	void GetPositionWindow();
	void SetPositionWindow(unsigned int window);
	void GetPositionWindowTime();
	void SetPositionWindowTime(unsigned short time);
	bool SetTargetPosition(int target);
	bool SetTargetVelocity(int velocity);

	// State Machine Commands	
	bool MoveToEnableOperation();
	bool SetOperationMode(char mode);
	void GetNodeID();

	//Device Control Commands
	void ShutDown();
	void SwitchOn();
	void SwitchOnEnable();
	void DisableOperation();
	void EnableOperation();
	bool FaultReset();
	bool Store(void);
	void QuickStop();
	void DisableVoltage();
};

#endif
