#!/usr/bin/env python
from opcua import Client
import time
import threading 
import rospy
from htbot.msg import *
# from ntuc.msg import *

##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####  
##### fleet_agv.py ##### Reside in AGV #####  
##### ROS listener in fleet_agv.py only listens to the AGV it resides in #####  
##### -- Read ROS messages from AGV and update to OPCUA server parameters #####
##### -- Read OPC parameters from OPCUA server and send ROS messages to AGV #####
##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### 

##### OPC Server IP address and TCP port #####
address = rospy.get_param("FLEET_OPC_SERVER")	# Get from ROS Launch File
url = "opc.tcp://"+address					# OPC Server URL
print("Fleet --- OPCUA Server URL is @ %s ---" % url)

# url = "opc.tcp://192.168.1.166:4840"
# url = "opc.tcp://10.0.5.2:4845"
# url = "opc.tcp://127.0.0.1:4845"

##### CONSTANTS DEFINITION #####
CONNECTION = 100
HANDSHAKE = 110

##### ROS Publisher to AGV, topic /task #####
fr_fleet_pub = rospy.Publisher('fr_fleet', task, queue_size=100)
to_lift_pub = rospy.Publisher('to_lift', lift, queue_size=100)

pub_task = rospy.Publisher('task', task, queue_size=100)	## dummy publisher, not used here

client = Client(url,timeout=60)	## timeout 60s to throw exception in try-catch when disconnected

client.connect()
print("Fleet --- Connected to OPCUA Server @ %s ---" % url)

## name = "OPCUA_FLEET_SERVER"

## ns_index = client.get_namespace_index(name)

root = client.get_root_node()

## objects = client.get_objects_node()

##### ##### ##### ##### ##### ##### ##### ##### ##### 
##### Establish connection to OPC server objects #####
##### ##### ##### ##### ##### ##### ##### ##### #####
varFireAlarm = root.get_child(["0:Objects", "2:Parameters", "2:FireAlarm"])
varBeaconAlarm = root.get_child(["0:Objects", "2:Parameters", "2:BeaconAlarm"])
varHeartbeat = root.get_child(["0:Objects", "2:Parameters", "2:Heartbeat1"])		#for AGV1
## varHeartbeat = root.get_child(["0:Objects", "2:Parameters", "2:Heartbeat2"])		#for AGV2
varProxy = root.get_child(["0:Objects", "2:Parameters", "2:Proxy1"])					#for AGV1
## varProxy = root.get_child(["0:Objects", "2:Parameters", "2:Proxy2"])					#for AGV2
varTask = root.get_child(["0:Objects", "2:Parameters", "2:Task1"])					#for AGV1
## varTask = root.get_child(["0:Objects", "2:Parameters", "2:Task2"])				#for AGV2
varCompletedTask = root.get_child(["0:Objects", "2:Parameters", "2:TaskCompleted"])
varStatus = root.get_child(["0:Objects", "2:Parameters", "2:Status1"])				#for AGV1
## varStatus = root.get_child(["0:Objects", "2:Parameters", "2:Status2"])			#for AGV2

##### ##### ##### ##### ##### ##### ##### #####
##### Callback for ROS topic /fr_agv, ntuc_fleet/task #####
##### Message sent by AGV core or simulator #####
##### Currently not used #####
##### ##### ##### ##### ##### ##### ##### #####
def fr_agv_callback(msg):
	print("Callback task received cmd==%d" % msg.cmd)
	if (msg.cmd == 3):	## cancel task
		# Completed task
		rospy.loginfo("Fleet --- Cancelled task toLP %d ---" % msg.toLP)
		varCompletedTask.set_value([msg.cmd,msg.time.secs,msg.type,msg.fromLP,msg.toLP])
		varTask.set_value([msg.cmd,msg.time.secs,msg.type,msg.fromLP,msg.toLP,msg.alloc])
	elif (msg.cmd == 2):	## acknowledge new task received
		print("Fleet --- AGV acknowledge new task received ---")
		##republish_task2AGV = False
	elif (msg.cmd == 1):	## re-submit aborted task
		## Command for lift to close door after getting inside lift
		print("Fleet --- Re-submit aborted task ---")
		## varTask.set_value([msg.cmd,msg.time.secs,msg.type,msg.fromLP,msg.toLP,msg.alloc])

rospy.Subscriber('fr_agv', task, fr_agv_callback)  	

##### ##### ##### ##### ##### ##### ##### #####
##### Callback for ROS topic /to_fleet, ntuc_fleet/agv_status #####
##### Message sent by AGV core or simulator #####
##### ##### ##### ##### ##### ##### ##### #####
def to_fleet_callback(msg):
	## print("Callback status received cmd==%d" % msg.cmd)
	if (msg.cmd==1):	## AGV Operational Status
		# AGV operational status received from AGV, try writing to opcua register
		rospy.loginfo("Fleet --- AGV operational status, complete==%d ---" % msg.complete)
		try:
			varStatus.set_value([msg.cmd, rospy.Time.now().secs, msg.status, msg.location, msg.arrival.secs, msg.b_level, msg.e_status, msg.complete, msg.lastLP])				
		except Exception:
			pub_msg = task()				### an instant of ntuc_fleet/task message type
			pub_msg.cmd = HANDSHAKE		### HANDSHAKE==110
			pub_msg.type = 0				### Not written to opcua register
			fr_fleet_pub.publish(pub_msg)
			##time.sleep(5)					### delay killing opcua client and reconnect 
			client.disconnect()			### reconnect to opcua server via (roslaunch respawn)
			 		
		### Successfully written to opcua register . . . Check last msg.complete 2 or 3
		if (msg.complete==0):
			pub_msg = task()				### an instant of ntuc_fleet/task message type
			pub_msg.cmd = HANDSHAKE			### HANDSHAKE==110
			pub_msg.type = 1				### Written to opcua register
			fr_fleet_pub.publish(pub_msg)	

	elif (msg.cmd == 2):	## alert status
		print("Fleet --- Alert Status ---")

	elif (msg.cmd == 3):	## alarm status
		print("Fleet --- Alarm Status ---")

	elif (msg.cmd == 4):	## AGV General Status
		# AGV general status received from AGV, try writing to opcua register	
		rospy.loginfo("Fleet --- AGV general status, complete==%d ---" % msg.complete)
		try:
			varStatus.set_value([msg.cmd, rospy.Time.now().secs, msg.status, msg.location, msg.arrival.secs, 
				msg.b_level, msg.e_status, msg.complete, msg.lastLP])				
		except Exception:
			pub_msg = task()				### an instant of ntuc_fleet/task message type
			pub_msg.cmd = HANDSHAKE		### HANDSHAKE==110
			pub_msg.type = 0				### Not written to opcua register
			fr_fleet_pub.publish(pub_msg)
			##time.sleep(5)					### delay killing opcua client and reconnect 
			##republish_task2AGV==True	### earmark to republish task
			client.disconnect()			### reconnect to opcua server via (roslaunch respawn)
			 		
		### Successfully written to opcua register . . . Check last msg.complete 2 or 3
		pub_msg = task()				### an instant of ntuc_fleet/task message type
		pub_msg.cmd = HANDSHAKE			### HANDSHAKE==110
		pub_msg.type = 1				### Written to opcua register
		fr_fleet_pub.publish(pub_msg)	

rospy.Subscriber('to_fleet', agv_status, to_fleet_callback)  	


##### ##### ##### ##### ##### ##### ##### #####
##### Heartbeat to update OPCUA server at regular interval #####
##### Triggerred via timer interrupt every 5s #####
##### ##### ##### ##### ##### ##### ##### #####
last_time=0
def heartbeat():
	global last_time

	while True:
		varHeartbeat.set_value(rospy.Time.now().secs)
		## print("Interval %d" % (rospy.Time.now().secs-last_time))
		last_time = rospy.Time.now().secs
		time.sleep(5)


##### ##### ##### ##### ##### ##### #####
##### Python main program entry #####
##### ##### ##### ##### ##### ##### #####
if __name__ == '__main__':
	rospy.init_node('fleet_proxy', anonymous=True)

	## start the heartbeat() thread
	threading.Thread(target=heartbeat, args=()).start() 
	last_time=rospy.Time.now().secs

	""" """
	### Notify AGV it's connected to OPC Server ###
	pub_msg = task()		### an instance of ntuc_fleet/task message type
	pub_msg.cmd = CONNECTION		### CONNECTION==100
	pub_msg.type = 1				### connected to opcua server
	fr_fleet_pub.publish(pub_msg)
	""" """
	## Proxy is connected to opcua server
	varProxy.set_value(1)

	curTask=[0,0,0,0,0,0]	## current task
	curBeacon=0			## current beacon state
	loop_count=0
	while True:
		##### Ensure connection with OPC Server is intact #####			
		##### Disconnect if unable to get from server object #####
		##### Allow ROS Launch to automatically restart ROS node #####
		""" """
		try:
			newTask = varTask.get_value()
			## serviceAvailable = varServiceAvailable.get_value()
			newBeacon = varBeaconAlarm.get_value()
			fireAlarm = varFireAlarm.get_value()				
		except Exception:
			rospy.loginfo("Fleet --- Connection Exception ---")	
			pub_msg = task()				### an instant of ntuc_fleet/task message type
			pub_msg.cmd = CONNECTION			### CONNECTION==100
			pub_msg.type = 0				### disconnected from opcua server
			fr_fleet_pub.publish(pub_msg)
			##time.sleep(5)					### delay killing opcua client and reconnect 			
			client.disconnect()			### reconnect to opcua server via (roslaunch respawn) 
		""" """	
		##if (loop_count%5):
		##	print("newTask: ", newTask[0],newTask[1],newTask[2],newTask[3],newTask[4],newTask[5])

		if (curTask[0] != newTask[0] or curTask[1] != newTask[1] or
			curTask[2] != newTask[2] or curTask[3] != newTask[3] or 
			curTask[4] != newTask[4] or curTask[5] != newTask[5]):
				
			rospy.loginfo("Fleet --- Inside if newTask ... %d, %d, %d, %d, %d ---",newTask[0],newTask[1],newTask[3],newTask[4],newTask[5])
		### Task [cmd, time.secs, type, frLP, toLP, pub2AGV] ###
			if (newTask[0]==2 and newTask[5]==0):	## assigned task from call stations ##
				pub_msg = task()			### an instant of ntuc_fleet/task message type
				pub_msg.cmd = newTask[0]					### cmd==2, assigned task
				pub_msg.time.secs = newTask[1]			### time.secs
				pub_msg.type = newTask[2]					### trolley type	
				pub_msg.fromLP = newTask[3]				### fromLP
				pub_msg.toLP = newTask[4]					### toLP
				pub_msg.alloc = newTask[5] = 1			### alloc==1 for AGV1
				fr_fleet_pub.publish(pub_msg)	
				### Update opcua Task parameter to indicate task pub2AGV
				varTask.set_value([newTask[0],newTask[1],newTask[2],newTask[3],newTask[4],newTask[5]])
				##print("published task to /fr_fleet")
			elif (newTask[0]==3):			## acknowledge task abortion request of AGV (ntuc_fleet.agv_status with agv_status.complete==4)
				pub_msg = task()			### an instant of ntuc_fleet/task message type
				pub_msg.cmd = newTask[0]					### cmd==3
				pub_msg.time.secs = newTask[1]			### time.secs
				pub_msg.type = newTask[2]					### trolley type	
				pub_msg.fromLP = newTask[3]				### fromLP
				pub_msg.toLP = newTask[4]					### toLP
				pub_msg.alloc = newTask[5]					### alloc
				fr_fleet_pub.publish(pub_msg)	
				##print("published task to /fr_fleet")
			curTask = list(newTask)		## update curTask

	    	if (fireAlarm == 1):
				rospy.loginfo("Fleet --- Fire alarm activated ---")
		###print("curBeacon:newBeacon",curBeacon,newBeacon)		###20210625###					
		if (curBeacon!=newBeacon):			
			pub_msg = lift()		### an instant of ntuc/lift message type
			pub_msg.cmd = 8			### lift.cmd==8 to turn on/off Beacon Alarm
			pub_msg.beacon = newBeacon	### State set on Fleet Manager opcua
			to_lift_pub.publish(pub_msg)	
			rospy.loginfo("Fleet --- Beacon Alarm activated/deactivated ---")
			curBeacon=newBeacon
		time.sleep(1)
		loop_count+=1
	##rospy.spin()

