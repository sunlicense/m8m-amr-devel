#!/usr/bin/env python2
from opcua import Client
import time
import rospy
from htbot.msg import *
from std_msgs.msg import UInt8

##### OPC Server IP address and TCP port #####
address = rospy.get_param("LIFT_OPC_SERVER")	# Get from ROS Launch File
url = "opc.tcp://"+address					# OPC Server URL

##### ROS Publisher from Lift to AGV, topic /fr_lift #####
fr_lift_pub = rospy.Publisher('fr_lift', lift, queue_size=10)
fr_lift_proxy_pub = rospy.Publisher('lift_status_to_FM', UInt8, queue_size=10)

client = Client(url,timeout=3600) ## timeout 60min to throw exception in try-catch when disconnected
##print(url)
client.connect()
rospy.loginfo("Lift --- Connected to Lift OPCUA Server @ %s ---", address)

##name = "OPCUA_LIFT_SERVER"

##ns_index = client.get_namespace_index(name)

root = client.get_root_node()

##objects = client.get_objects_node()
### At least one Lift occupied by AGV ###
liftOccupiedByAGV = False

##### ##### ##### ##### ##### ##### ##### ##### ##### 
##### Establish connection to OPC server objects #####
##### ##### ##### ##### ##### ##### ##### ##### #####
varLiftCommands = root.get_child(["0:Objects", "2:Parameters", "2:LiftCommand"])
varLiftReplies = root.get_child(["0:Objects", "2:Parameters", "2:LiftReply"])
varCarLevel = root.get_child(["0:Objects", "2:Parameters", "2:CarLevel"])
varBeacon = root.get_child(["0:Objects", "2:Parameters", "2:Beacon"])
varProxy = root.get_child(["0:Objects", "2:Parameters", "2:Proxy"])	
""""""
varInUseStatus = root.get_child(["0:Objects", "2:Parameters", "2:InUse"])
varDoorOpenStatus = root.get_child(["0:Objects", "2:Parameters", "2:DoorOpen"])
varDoorCloseStatus = root.get_child(["0:Objects", "2:Parameters", "2:DoorClose"])
varGoingUpStatus = root.get_child(["0:Objects", "2:Parameters", "2:GoingUp"])
varGoingDownStatus = root.get_child(["0:Objects", "2:Parameters", "2:GoingDown"])
varStationaryStatus = root.get_child(["0:Objects", "2:Parameters", "2:Stationary"])
""""""
varServiceAvailable = root.get_child(["0:Objects", "2:Parameters", "2:ServiceAvailable"])
varEmergencyPower = root.get_child(["0:Objects", "2:Parameters", "2:EmergencyPower"])
varFireAlarm = root.get_child(["0:Objects", "2:Parameters", "2:FireAlarm"])

##### ##### ##### ##### ##### ##### ##### 
##### Callback for ROS topic /to_lift #####
##### ##### ##### ##### ##### ##### #####
def to_lift_callback(data):
	## print("to_lift_callback cmd==[%d]" % data.cmd)
	
	if (data.cmd == 0):	## Car Call to Start Level
		if (liftOccupiedByAGV):
				rospy.loginfo("Lift --- Lift usage exceeded, please wait ---")
				pub_msg = lift()				### an instant of ntuc/lift message type
				pub_msg.cmd = serviceAvailable[0]		### cmd==100
				pub_msg.serviceavail = 2			### service busy, temporary not available	
				fr_lift_pub.publish(pub_msg)
				return		
		# Command to call lift to come to a floor
		rospy.loginfo("Lift --- Command lift to COME to level %d ---", data.cfloor)
		try:		
			varLiftCommands.set_value([data.cmd,data.cfloor,data.dfloor,data.inuse])
		except Exception:
			rospy.loginfo("Lift --- Connection Exception while writing cmd==0 to OPCUA ---")
			pub_msg = lift()				### an instant of ntuc/lift message type
			pub_msg.cmd = serviceAvailable[0]		### cmd==100
			pub_msg.serviceavail = 0			### service NOT available	
			fr_lift_pub.publish(pub_msg)
			client.disconnect()	
		## Publish to /lift_status_to_FM to indicate BUSY
		rospy.loginfo("Lift --- Publish Lift Status BUSY to Fleet-Proxy ---")
		pub_msg = UInt8()				### an instant of std_msgs/UInt8 message type
		pub_msg.data = 1				### data==1|2. AGV1 or AGV2	
		fr_lift_proxy_pub.publish(pub_msg)					
		while True:
			replyFromLift = varLiftReplies.get_value()
			if (replyFromLift[0] == 10):
				# Lift reached required floor and door opened
				rospy.loginfo("Lift --- Arrived at level %d. Able to enter lift ---", data.cfloor)
				pub_msg = lift()		### an instant of ntuc/lift message type
				pub_msg.cmd = replyFromLift[0]
				pub_msg.cfloor = replyFromLift[1]
				pub_msg.dfloor = replyFromLift[2]
				fr_lift_pub.publish(pub_msg)
				break
			# Have not reached. 
			time.sleep(1)
	elif (data.cmd == 1):	## AGV inside elevator car
		# Command for lift to close door after AGV gotten inside
		rospy.loginfo("Lift --- Entered, lift door closing ---")
		try:		
			varLiftCommands.set_value([data.cmd,data.cfloor,data.dfloor,data.inuse])
		except Exception:
			rospy.loginfo("Lift --- Connection Exception while writing cmd==1 to OPCUA ---")
			pub_msg = lift()				### an instant of ntuc/lift message type
			pub_msg.cmd = serviceAvailable[0]		### cmd==100
			pub_msg.serviceavail = 0			### service NOT available	
			fr_lift_pub.publish(pub_msg)
			client.disconnect()	
	elif (data.cmd == 2):	## Car Call to End to Level
		# Command to call lift to go to a floor
		rospy.loginfo("Lift --- Command lift to GOTO level %d ---", data.dfloor)
		try:		
			varLiftCommands.set_value([data.cmd,data.cfloor,data.dfloor,data.inuse])
		except Exception:
			rospy.loginfo("Lift --- Connection Exception while writing cmd==2 to OPCUA ---")
			pub_msg = lift()				### an instant of ntuc/lift message type
			pub_msg.cmd = serviceAvailable[0]		### cmd==100
			pub_msg.serviceavail = 0			### service NOT available	
			fr_lift_pub.publish(pub_msg)
			client.disconnect()		
		while True:
			replyFromLift = varLiftReplies.get_value()
			if (replyFromLift[0] == 11):
				# Lift reached required floor and door opened
				rospy.loginfo("Lift --- Reached level %d. Able to exit Lift ---", data.dfloor)
				pub_msg = lift()		### an instant of ntuc/lift message type
				pub_msg.cmd = replyFromLift[0]
				pub_msg.cfloor = replyFromLift[1]
				pub_msg.dfloor = replyFromLift[2]
				fr_lift_pub.publish(pub_msg)				
				break
			# Have not reached. 
			time.sleep(1)
	elif (data.cmd == 3):	## AGV outside elevator car
		# Command for lift to close door after AGV gotten outside
		rospy.loginfo("Lift --- Exited, lift closing door ---")
		try:		
			varLiftCommands.set_value([data.cmd,data.cfloor,data.dfloor,data.inuse])
		except Exception:
			rospy.loginfo("Lift --- Connection Exception while writing cmd==3 to OPCUA ---")
			pub_msg = lift()				### an instant of ntuc/lift message type
			pub_msg.cmd = serviceAvailable[0]		### cmd==100
			pub_msg.serviceavail = 0			### service NOT available	
			fr_lift_pub.publish(pub_msg)
			client.disconnect()	
		## Publish to /lift_status_to_FM to indicate NOT BUSY
		rospy.loginfo("Lift --- Publish Lift Status NOT BUSY to Fleet-Proxy ---")
		pub_msg = UInt8()				### an instant of std_msgs/UInt8 message type
		pub_msg.data = 0				### data==0 is NOT occupied by AGV
		fr_lift_proxy_pub.publish(pub_msg)	
						
	elif (data.cmd == 8):	## AGV activates/deactivates beacon light alarm
		rospy.loginfo("Lift --- Activate/DeActivate Beacon, /to_lift [%d] ---", data.beacon)
		varBeacon.set_value([data.cmd,data.beacon])		
	elif (data.cmd == 9):
		rospy.loginfo("Lift --- Activate/DeActivate In_Use, /to_lift [%d] ---", data.inuse)
		varLiftCommands.set_value([data.cmd,data.cfloor,data.dfloor,data.inuse])
##### end to_lift_callback() #####	

##### ##### ##### ##### ##### ##### ##### 
##### Callback for ROS topic /to_lift2 #####
##### ##### ##### ##### ##### ##### #####
def to_lift2_callback(data):
	if (data.cmd == 8):	## AGV activates/deactivates beacon light alarm
		rospy.loginfo("Lift --- Activate/DeActivate Beacon, /to_lift2 [%d] ---", data.beacon)
		varBeacon.set_value([data.cmd,data.beacon])		
	elif (data.cmd == 9):
		rospy.loginfo("Lift --- Activate/DeActivate In_Use, /to_lift2 [%d] ---", data.inuse)
		varLiftCommands.set_value([data.cmd,data.cfloor,data.dfloor,data.inuse])
##### end to_lift2_callback() #####		
		  	
##### ##### ##### ##### ##### ##### ##### ##### ##### 
##### Callback for ROS topic /lift_status_from_FM #####
##### ##### ##### ##### ##### ##### ##### ##### #####
def lift_status_from_FM_callback(msg):
	global liftOccupiedByAGV
	## update the liftOccupiedByAGV flag
	liftOccupiedByAGV = msg.data
	print('Lift --- Value of liftOccupiedByAGV ---',liftOccupiedByAGV)
##### end lift_status_from_FM_callback() #####	


if __name__ == '__main__':
	rospy.init_node('liftTransporterSide', anonymous=True)
	rospy.Subscriber('to_lift', lift, to_lift_callback)
	rospy.Subscriber('to_lift2', lift, to_lift2_callback)
	rospy.Subscriber('lift_status_from_FM', UInt8, lift_status_from_FM_callback)
	
	### Notify AGV it's connected to OPC server ###
	pub_msg = lift()				### an instant of ntuc/lift message type
	pub_msg.cmd = 100				### cmd==100 (service available)
	pub_msg.serviceavail = 1			### service IS available	
	fr_lift_pub.publish(pub_msg)
	## Proxy is connected to opcua server
	varProxy.set_value(1)
	rospy.loginfo("Lift --- Connected to Lift Controller ---")
	
	##### Remember previous statuses and only publish ROS messages when statuses change #####	
	prev_carLevel=0
	prev_inUse=0
	prev_doorOpen=0
	prev_doorClose=0
	prev_goingUp=0
	prev_goingDown=0
	prev_serviceAvailable=0
	prev_emergencyPowerAlarm=0
	prev_fireSerivceAlarm=0
	stationary_flag=0
	
	
	##### Loop forever in main thread #####
	##### get values from connected server objects, test and publish to /fr_lift topic #####
	while True:
##		replyFromLift = varLiftReplies.get_value()

		##### Ensure connection with OPC Server is intact #####			
		##### Disconnect if unable to get from server object #####
		##### Allow ROS Launch to automatically restart ROS node #####
		""" """
		try:
			carLevel = varCarLevel.get_value()
			inUse = varInUseStatus.get_value()	
			doorOpen = varDoorOpenStatus.get_value()
			doorClose = varDoorCloseStatus.get_value()
			goingUp = varGoingUpStatus.get_value()
			goingDown = varGoingDownStatus.get_value()
			stationary = varStationaryStatus.get_value()
			serviceAvailable = varServiceAvailable.get_value()
			emergencyPowerAlarm = varEmergencyPower.get_value()
			fireSerivceAlarm = varFireAlarm.get_value()				
		except Exception:
			rospy.loginfo("Lift --- Connection Exception while reading from OPCUA ---")
			pub_msg = lift()				### an instant of ntuc/lift message type
			pub_msg.cmd = serviceAvailable[0]		### cmd==100
			pub_msg.serviceavail = 0			### service NOT available	
			fr_lift_pub.publish(pub_msg)
			client.disconnect()
		""" """				
##		carLevel = varCarLevel.get_value()	
		if (carLevel[0] == 40 and carLevel[1]!=prev_carLevel):
			prev_carLevel = carLevel[1]
			##print("Lift Car is at level %d." % carLevel[1])
			pub_msg = lift()		### an instant of ntuc/lift message type
			pub_msg.cmd = carLevel[0]	### cmd==40
			pub_msg.cfloor = carLevel[1]	### current car level
			##fr_lift_pub.publish(pub_msg)

##		inUse = varInUseStatus.get_value()	
		if (inUse[0] == 20 and inUse[1]!=prev_inUse):
			prev_inUse = inUse[1]
			"""		
			if (inUse[1] == 1):
				print("In Use is Active *#*#*#*#*#*")			
			elif (inUse[1] == 0):
				print("NOT In Use")
			"""
			pub_msg = lift()			### an instant of ntuc/lift message type
			pub_msg.cmd = inUse[0]			### cmd==20
			pub_msg.inuse = inUse[1]		### inuse==0/0	
			fr_lift_pub.publish(pub_msg)	

##		doorOpen = varDoorOpenStatus.get_value()
		if (doorOpen[0] == 21 and doorOpen[1]!=prev_doorOpen):
			prev_doorOpen = doorOpen[1]	
			"""	
			if (doorOpen[1] == 1):
				print("Door is Open *#*#*#*#*#*")			
			elif (doorOpen[1] == 0):
				print("NOT Open")
			"""
			pub_msg = lift()				### an instant of ntuc/lift message type
			pub_msg.cmd = doorOpen[0]				### cmd==21
			pub_msg.dooropen = doorOpen[1]		### dooropen==0/1	
			##fr_lift_pub.publish(pub_msg)

##		doorClose = varDoorCloseStatus.get_value()	
		if (doorClose[0] == 22 and doorClose[1]!=prev_doorClose):
			prev_doorClose = doorClose[1]
			"""
			if (doorClose[1] == 1):
				print(	"Door is Close *#*#*#*#*#*")
			elif (doorClose[1] == 0):
				print("NOT Close")
			"""
			pub_msg = lift()				### an instant of ntuc/lift message type
			pub_msg.cmd = doorClose[0]				### cmd==22
			pub_msg.doorclose = doorClose[1]		### doorclose==0/1	
			##fr_lift_pub.publish(pub_msg)

##		goingUp = varGoingUpStatus.get_value()
		if (goingUp[0] == 23 and goingUp[1]!=prev_goingUp):		
			prev_goingUp = goingUp[1]
			"""	
			if (goingUp[1] == 1):
				print("It is going up *#*#*#*#*#*")
			elif (goingUp[1] == 0):
				print("NOT up")
			"""
			pub_msg = lift()					### an instant of ntuc/lift message type
			pub_msg.cmd = goingUp[0]					### cmd==23
			pub_msg.goingup = goingUp[1]				### doorclose==0/1			
			##fr_lift_pub.publish(pub_msg)	

##		goingDown = varGoingDownStatus.get_value()
		if (goingDown[0] == 24 and goingDown[1]!=prev_goingDown):		
			prev_goingDown = goingDown[1]	
			"""		
			if (goingDown[1] == 1):
				print("It is going down *#*#*#*#*#*")
			elif (goingDown[1] == 0):		
				print("NOT down")
			"""
			pub_msg = lift()					### an instant of ntuc/lift message type
			pub_msg.cmd = goingDown[0]						### cmd==24
			pub_msg.goingdown = goingDown[1]				### goingdown==0/1	
			##fr_lift_pub.publish(pub_msg)	
			 
##		stationary = varStationaryStatus.get_value()
		pub_msg = lift()					### an instant of ntuc/lift message type
		pub_msg.cmd = stationary[0]				### cmd==25
		if (goingUp[1]==0 and goingDown[1]==0 and stationary_flag==0):
			stationary_flag=1
			pub_msg.stationary = stationary_flag			### stationary==0/1	
			fr_lift_pub.publish(pub_msg)	
			##print("It is stationary *#*#*#*#*#*")				
		elif ((goingUp[1]==1 or goingDown[1]==1) and stationary_flag==1):
			stationary_flag=0
			##print("NOT Still")
			pub_msg.stationary = stationary_flag			### stationary==0/1	
			##fr_lift_pub.publish(pub_msg)		

##		serviceAvailable = varServiceAvailable.get_value()
		if (serviceAvailable[0] == 100 and serviceAvailable[1]!=prev_serviceAvailable):
			prev_serviceAvailable = serviceAvailable[1]	
			"""			
			if (serviceAvailable[1] == 1):
				print("Service is available *#*#*#*#*#*")				
			elif (serviceAvailable[1] == 0):
				print("NOT SA")
			"""
			pub_msg = lift()				### an instant of ntuc/lift message type
			pub_msg.cmd = serviceAvailable[0]		### cmd==100
			pub_msg.serviceavail = serviceAvailable[1]		### fireservice==0/1	
			fr_lift_pub.publish(pub_msg)	
	
##		emergencyPowerAlarm = varEmergencyPower.get_value()
		if (emergencyPowerAlarm[0] == 101 and emergencyPowerAlarm[1]!=prev_emergencyPowerAlarm):
			prev_emergencyPowerAlarm = emergencyPowerAlarm[1]
			"""
			if (emergencyPowerAlarm[1] == 1):
				print("Emergency power Activated *#*#*#*#*#*")			
			elif (emergencyPowerAlarm[1] == 0):
				print("NOT EP")
			"""
			pub_msg = lift()					### an instant of ntuc/lift message type
			pub_msg.cmd = emergencyPowerAlarm[0]			### cmd==101
			pub_msg.epower = emergencyPowerAlarm[1]		### fireservice==0/1	
			fr_lift_pub.publish(pub_msg)						

##		fireSerivceAlarm = varFireAlarm.get_value()
		if (fireSerivceAlarm[0] == 102 and fireSerivceAlarm[1]!=prev_fireSerivceAlarm):
			prev_fireSerivceAlarm = fireSerivceAlarm[1]
			"""
			if (fireSerivceAlarm[1] == 1):
				print("Fire alarm Activated *#*#*#*#*#*")			
			elif (fireSerivceAlarm[1] == 0):
				print("NOT FS")
			"""
			pub_msg = lift()					### an instant of ntuc/lift message type
			pub_msg.cmd = fireSerivceAlarm[0]				### cmd==102
			pub_msg.fireservice = fireSerivceAlarm[1]		### fireservice==0/1	
			fr_lift_pub.publish(pub_msg)					
								
		time.sleep(1)
		## print("")	
	rospy.spin()
