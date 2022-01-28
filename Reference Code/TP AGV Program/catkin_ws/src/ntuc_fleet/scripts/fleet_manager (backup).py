#!/usr/bin/env python
# Tkinter tkinter
from Tkinter import Tk, Frame, Label, Button, Entry, IntVar, END, W, E, StringVar, SUNKEN, BOTH, OptionMenu
from opcua import Server, ua
from opcua import Client		## Client to Lift Controller opcua
import time
import threading 
import rospy
import json
import pickle				## file serialization
from ntuc_fleet.msg import *

##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####  
##### fleet_manager.py ##### Reside in fleet manager #####   
##### -- Read ROS messages from call stations and update to OPCUA server parameters #####
##### -- Read OPC parameters from OPCUA server and send ROS messages to call stations #####
##### -- OPCUA client to Lift Controller, activate/deactivate beacon alarm  #####
##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####

##### CONSTANTS DEFINITION #####
DUMMY = 0

##### Initialize ROS Node #####
rospy.init_node('fleet_manager', anonymous=True)

##### Declare global ROS Publishers #####
pub_jobs = rospy.Publisher('rv_jobs', jobs, queue_size=10)
pub_status = rospy.Publisher('to_station', agv_status, queue_size=10)
## pub_task = rospy.Publisher('task', task, queue_size=10)	## dummy publisher, not used here

##### Global Lists to for tasks #####
task_Q = []			## queue of tasks
delete_Q = []		## tasks earmarked for deletion
unassigned_Q = []	## not used
DEFAULT_BUTTON_COLOR = ''

##### Global Variables #####
task1_completed=False	## task assigned to AGV1 completed
ready4task1=False			## AGV1 ready for new task
task2_completed=False	## task assigned to AGV2 completed
ready4task2=False			## AGV2 ready for new task
last_time=rospy.Time.now().secs		## last ROS time
task_Q_changed=False	## task Q changed?


##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
##### Establish connection to OPCUA server objects @ Lift Controller #####
##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
##### OPCUA Server IP address and TCP port #####

address = rospy.get_param("FLEET_OPC_SERVER")	# Get from ROS Launch File
print("FLEET OPCUA Server IP Address == %s " % address )
url = "opc.tcp://"+address					# OPCUA Server URL
"""
client = Client(url)
client.connect()
print("Connected to OPCUA Server @ %s" % address)
root = client.get_root_node()

varBeacon = root.get_child(["0:Objects", "2:Parameters", "2:Beacon"])
"""

##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
##### ROS Listener callback for /rq_jobs topic, ntuc_fleet/jobs #####
##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
def rq_jobs_callback(msg):
	global last_time, task_Q_changed
	## rospy.loginfo("Input Received %d, toLP=%d", msg.cmd, msg.toLP);

	if msg.cmd==1:	## not used
		pass
	elif msg.cmd==2:	## request for task list
		task_Q_changed=True	## force task list to be published to call stations
		print("task_Q_changed=True")
		task_Q_changed=True	## task Q changed?		
			
rospy.Subscriber('rq_jobs', jobs, rq_jobs_callback)	

##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
##### ROS Listener callback for /fr_station topic, ntuc_fleet/task #####
##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
def fr_station_callback(msg):
	global last_time, task_Q_changed, varBeacon
	## rospy.loginfo("Input Received %d, toLP=%d", msg.cmd, msg.toLP);
	ALL_TASKS=99;			##all tasks
	UNALLOCATED=88;			##unallocated tasks only
	ALLOCATED2AGV1=11;		##task allocated to AGV1 only
	ALLOCATED2AGV2=22;		##task allocated to AGV2 only

	if msg.cmd==1:	## new task
		rospy.loginfo("Task: cmd %d, secs %d, type %d, fromLP %d, toLP %d, alloc %d", msg.cmd, msg.time.secs, msg.type,
				msg.fromLP, msg.toLP, msg.alloc);
		## Ensure unique time.secs field
		if (rospy.Time.now().secs<=last_time):
			time.sleep(1)
		## Add new task to task_Q
		task_Q.append([msg.cmd, rospy.Time.now().secs, msg.type, msg.fromLP, msg.toLP, 0])
                # Update new task received
                # my_opc_server.task1_parameter.set_value([msg.cmd, msg.time.secs, msg.type, msg.fromLP, msg.toLP])
		last_time=rospy.Time.now().secs		## update last_time	
	elif msg.cmd==2:	## request for task list (another option)
		task_Q_changed=True	## force task list to be published to call stations
	elif msg.cmd==3:	## delete task(s)
		rospy.loginfo("--- delete task(s): cmd %d, type %d, fromLP %d, toLP %d", msg.cmd, msg.type, msg.fromLP, msg.toLP);
		if (msg.type==ALL_TASKS):	## delete all tasks including allocated tasks
			## print(msg.cmd, msg.type)
			my_opc_server.update("delete_all")	## clear all tasks
		elif (msg.type==UNALLOCATED):	## delete all unallocated tasks
			## print(msg.cmd, msg.type)
			my_opc_server.update("delete_unallocated")	## delete unallocated tasks	
		elif (msg.type==ALLOCATED2AGV1):	## delete task allocated to AGV1
			## print(msg.cmd, msg.type)
			my_opc_server.update("delete_allocated2AGV1")	## delete task allocated to AGV1
		elif (msg.type==ALLOCATED2AGV2):	## delete task allocated to AGV2
			## print(msg.cmd, msg.type)
			my_opc_server.update("delete_allocated2AGV2")	## delete task allocated to AGV2
		else:
			## Add cancelled task to delete_Q
			delete_Q.append([msg.cmd, msg.time.secs, msg.type, msg.fromLP, msg.toLP, 0])	# preserve time
			### my_opc_server.task1_parameter.set_value([msg.cmd, msg.time.secs, msg.type, msg.fromLP, msg.toLP])	
	elif msg.cmd==4:	## deactivate L11 Kitchen beacon alarm 
		rospy.loginfo("Task: cmd %d, secs %d, type %d, fromLP %d, toLP %d, alloc %d", msg.cmd, msg.time.secs, msg.type,
				msg.fromLP, msg.toLP, msg.alloc);
		print("DeActivate Beacon")
		my_opc_server.beaconAlarm_parameter.set_value(0)
		## varBeacon.set_value([8,0])		

	elif msg.cmd==5:	## activate L11 Kitchen beacon alarm
		rospy.loginfo("Task: cmd %d, secs %d, type %d, fromLP %d, toLP %d, alloc %d", msg.cmd, msg.time.secs, msg.type,
				msg.fromLP, msg.toLP, msg.alloc);
		print("Activate Beacon")
		my_opc_server.beaconAlarm_parameter.set_value(1)
		## varBeacon.set_value([8,1])
		
	elif msg.cmd==7:	## move unallocated task UP the list of tasks
		rospy.loginfo("Task: cmd %d, secs %d, type %d, fromLP %d, toLP %d, alloc %d", msg.cmd, msg.time.secs, msg.type,
				msg.fromLP, msg.toLP, msg.alloc);
		task = [1, msg.time.secs, msg.type, msg.fromLP, msg.toLP, 0]		
		move_up_task(task)
		## varBeacon.set_value([8,1])					

rospy.Subscriber('fr_station', task, fr_station_callback)	


##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
##### Move unallocated task UP in the list, swap with the upper one #####
##### ##### ##### ##### ##### ##### ##### ##### ##### ##### ##### #####
def move_up_task(task):
	global task_Q_changed, task_Q
	print("Inside move_up_task()...")	
	index = task_Q.index(task)
	## print("index", index)
	task_Q[index-1], task_Q[index] = task_Q[index], task_Q[index-1]	## swap
	task_Q_changed=True	## task Q changed?			

					
##### ##### ##### ##### ##### ##### ##### #####  ##### 
#####   OPCUA Server Class with Tkinker GUI Frame as argument #####
##### ##### ##### ##### ##### ##### ##### #####  ##### 
class OPCServer(Frame):

##### ##### ##### ##### ##### ##### ##### #####  ##### 
##### Method:  Constructor #####
##### ##### ##### ##### ##### ##### ##### #####  ##### 
	def __init__(self, master=None):
		Frame.__init__(self, master)               
		self.master = master
		self.bCloseThreadFlag = False
		self.bServerStarted = False
#        self.curTask=[0,0,0,0,0]
		self.task1_cmd = IntVar(0)	## Task allocated to AGV1
		self.task1_type = IntVar(0)
		self.task1_fromLP = StringVar(0)
		self.task1_toLP = StringVar(0)
		self.task2_cmd = IntVar(0)	## Task allocated to AGV2
		self.task2_type = IntVar(0)
		self.task2_fromLP = IntVar(0)
		self.task2_toLP = IntVar(0)
		self.updatedFireAlarm = IntVar()
		self.updatedLiftStatus = IntVar()
		self.updatedBeaconAlarm = IntVar()
		self.init_window()
		self.server = Server()
		
		self.url = url
		# self.url = "opc.tcp://192.168.1.166:4840"
		self.server.set_endpoint(self.url)
		self.name = "OPCUA_FLEET_SERVER"
		self.addspace = self.server.register_namespace(self.name)
		self.rootnode = self.server.get_objects_node()                              # root node
		self.parameters = self.rootnode.add_object(self.addspace, "Parameters")     # parameters node
		# OPCUA Parameters
		self.liftStatus_parameter = self.parameters.add_variable(self.addspace, "LiftStatus", 0,  ua.VariantType.Byte) 		
		self.fireAlarm_parameter = self.parameters.add_variable(self.addspace, "FireAlarm", 0,  ua.VariantType.Byte)            # fire alarm parameter
		self.beaconAlarm_parameter = self.parameters.add_variable(self.addspace, "BeaconAlarm", 0, ua.VariantType.Byte)   		# beacon alarm parameter
		self.heartbeat1_parameter = self.parameters.add_variable(self.addspace, "Heartbeat1", 0, ua.VariantType.Byte)   		# heartbeat for AGV1 parameter
		self.heartbeat2_parameter = self.parameters.add_variable(self.addspace, "Heartbeat2", 0, ua.VariantType.Byte)   		# heartbeat for AGV2 parameter
		self.proxy1_parameter = self.parameters.add_variable(self.addspace, "Proxy1", 0, ua.VariantType.Byte)   		# connection status for Proxy1 @ AGV1 parameter
		self.proxy2_parameter = self.parameters.add_variable(self.addspace, "Proxy2", 0, ua.VariantType.Byte)   		# connection status for Proxy2 @ AGV2 parameter	
		### Task [cmd, time.secs, type, frLP, toLP, pub2AGV] ###
		self.task1_parameter = self.parameters.add_variable(self.addspace, "Task1", ua.Variant([0,0,0,0,0,0], ua.VariantType.Byte))	
		self.task2_parameter = self.parameters.add_variable(self.addspace, "Task2", ua.Variant([0,0,0,0,0,0], ua.VariantType.Byte))
		self.taskCompleted_parameter = self.parameters.add_variable(self.addspace, "TaskCompleted", ua.Variant([0,0,0,0,0], ua.VariantType.Byte))
		self.status1_parameter = self.parameters.add_variable(self.addspace, "Status1", ua.Variant([0,0,0,0,0,0,0,0,0], ua.VariantType.Byte))
		self.status2_parameter = self.parameters.add_variable(self.addspace, "Status2", ua.Variant([0,0,0,0,0,0,0,0,0], ua.VariantType.Byte))
		# OPCUA Client Writable Parameters
		self.liftStatus_parameter.set_writable()	## AGV1 & AGV2		
		self.heartbeat1_parameter.set_writable()	## AGV1
		self.heartbeat2_parameter.set_writable()	## AGV2
		self.proxy1_parameter.set_writable()		## AGV1
		self.proxy2_parameter.set_writable()		## AGV2		
		self.task1_parameter.set_writable()			## AGV1
		self.task2_parameter.set_writable()			## AGV1		
		self.taskCompleted_parameter.set_writable()
		self.status1_parameter.set_writable()		## AGV1
		self.status2_parameter.set_writable()		## AGV1

		self.server.start()
		print("FLEET OPCUA Server Started")
		self.bServerStarted = True    

		
##### ##### ##### ##### ##### ##### ##### #####  ##### 
##### Method:  Creation of GUI window #####
##### ##### ##### ##### ##### ##### ##### #####  #####         
	# Creation of init_window
	def init_window(self):
		global DEFAULT_BUTTON_COLOR
		# changing the title of our master widget      
		self.master.title("Fleet Manager OPCUA Server")

		# allowing the widget to take the full space of the root window
		self.pack(fill=BOTH, expand=1)
		self.status = StringVar()
		self.status.set("Ready")

		# Widgets
		self.start_button = Button(self, text="Start OPCUA Server", command=lambda: self.update("start"))
		self.start_button.configure(state = 'disabled')
		DEFAULT_BUTTON_COLOR = self.start_button.cget('bg')
		self.stop_button = Button(self, text="Stop OPCUA Server", command=lambda: self.update("stop"))
		self.stop_button.configure(state = 'active')
		self.close_button = Button(self, text="Close", command=lambda: self.update("close"))

		vcmdFA = self.register(self.validateFA) # we have to wrap the command
		self.fireAlarmLabel = Label(self, text="Fire Alarm")
		self.fireAlarmEntry = Entry(self,  width=2, validate='key', validatecommand=(vcmdFA, '%P'), text=self.updatedFireAlarm)
		vcmdLS = self.register(self.validateLS) # we have to wrap the command
		self.liftStatusLabel = Label(self, text="Lift Status")
		self.liftStatusEntry = Entry(self,  width=2, validate='key', validatecommand=(vcmdLS, '%P'), text=self.updatedLiftStatus)
		vcmdBA = self.register(self.validateBA) # we have to wrap the command
		self.beaconAlarmLabel = Label(self, text="Beacon Alarm")
		self.beaconAlarmEntry = Entry(self,  width=2, validate='key', validatecommand=(vcmdBA, '%P'), text=self.updatedBeaconAlarm)
		self.updatedFireAlarm = 0
		self.updatedLiftStatus = 0
		self.updatedBeaconAlarm = 0
		self.update_button = Button(self, text="Update Parameters", command=lambda: self.update("update"))
		self.update_button.configure(state = 'active')
		self.status_display = Label(self, bd=1, width=55, relief=SUNKEN, anchor=W, textvariable=self.status, font=('arial',10,'normal'))

		self.statusAGV1Label = Label(self, text="AGV1 busy", fg="blue", font="Verdana 10 bold italic")
		self.task1Label = Label(self, text="Task allocated to AGV1:", fg="blue", font="Verdana 10 bold italic")
		self.task1Label_cmd = Label(self, text="cmd")
		self.task1Label_cmd_value = Label(self, textvariable=self.task1_cmd)
		self.task1Label_type = Label(self, text="type")
		self.task1Label_type_value = Label(self, textvariable=self.task1_type)
		self.task1Label_fromLP = Label(self, text="fromLP")
		self.task1Label_fromLP_value = Label(self, textvariable=self.task1_fromLP)
		self.task1Label_toLP = Label(self, text="toLP")
		self.task1Label_toLP_value = Label(self, textvariable=self.task1_toLP)

		self.statusAGV2Label = Label(self, text="AGV2 busy", fg="blue", font="Verdana 10 bold italic")
		self.task2Label = Label(self, text="Task allocated to AGV2:", fg="blue", font="Verdana 10 bold italic")
		self.task2Label_cmd = Label(self, text="cmd")
		self.task2Label_cmd_value = Label(self, textvariable=self.task2_cmd)
		self.task2Label_type = Label(self, text="type")
		self.task2Label_type_value = Label(self, textvariable=self.task2_type)
		self.task2Label_fromLP = Label(self, text="fromLP")
		self.task2Label_fromLP_value = Label(self, textvariable=self.task2_fromLP)
		self.task2Label_toLP = Label(self, text="toLP")
		self.task2Label_toLP_value = Label(self, textvariable=self.task2_toLP)

		self.deleteTasksLabel = Label(self, text="Delete tasks:", fg="blue", font="Verdana 10 bold italic")
		self.delete_all_button = Button(self, text="All", command=lambda: self.update("delete_all"))
		self.delete_all_button.configure(state = 'disabled')
		self.delete_unallocated_button = Button(self, text="Unallocated", command=lambda: self.update("delete_unallocated"))
		self.delete_unallocated_button.configure(state = 'disabled')	
		self.delete_allocated2AGV1_button = Button(self, text="Allocated to AGV1", command=lambda: self.update("delete_allocated2AGV1"))
		self.delete_allocated2AGV1_button.configure(state = 'disabled')	
		self.delete_allocated2AGV2_button = Button(self, text="Allocated to AGV2", command=lambda: self.update("delete_allocated2AGV2"))
		self.delete_allocated2AGV2_button.configure(state = 'disabled')	

		# Layout
		self.start_button.place(x=10, y=10)
		self.stop_button.place(x=250, y=10)
		self.fireAlarmLabel.place(x=10, y=50)
		self.fireAlarmEntry.place(x=140, y=50)
		self.liftStatusLabel.place(x=240, y=80)
		self.liftStatusEntry.place(x=350, y=80)
		self.beaconAlarmLabel.place(x=10, y=80)
		self.beaconAlarmEntry.place(x=140, y=80)
		self.update_button.place(x=10, y=100)

		self.statusAGV1Label.place(x=10, y=150)
		self.task1Label.place(x=10, y=170)
		self.task1Label_cmd.place(x=20, y=190)
		self.task1Label_type.place(x=20, y=210)
		self.task1Label_fromLP.place(x=20, y=230)
		self.task1Label_toLP.place(x=20, y=250)
		self.task1Label_cmd_value.place(x=100, y=190)
		self.task1Label_type_value.place(x=100, y=210)
		self.task1Label_fromLP_value.place(x=100, y=230)
		self.task1Label_toLP_value.place(x=100, y=250)

		self.statusAGV2Label.place(x=210, y=150)
		self.task2Label.place(x=210, y=170)
		self.task2Label_cmd.place(x=220, y=190)
		self.task2Label_type.place(x=220, y=210)
		self.task2Label_fromLP.place(x=220, y=230)
		self.task2Label_toLP.place(x=220, y=250)
		self.task2Label_cmd_value.place(x=300, y=190)
		self.task2Label_type_value.place(x=300, y=210)
		self.task2Label_fromLP_value.place(x=300, y=230)
		self.task2Label_toLP_value.place(x=300, y=250)

		self.deleteTasksLabel.place(x=10, y=270)
		self.delete_all_button.place(x=20, y=290)	
		self.delete_unallocated_button.place(x=70, y=290)
		self.delete_allocated2AGV1_button.place(x=180, y=290)
		self.delete_allocated2AGV2_button.place(x=20, y=320)		

		self.close_button.place(x=320, y=330)
		self.status_display.place(x=10, y=370)

##### ##### ##### ##### ##### ##### ##### #####  ##### 
##### Setup Threads and Start Threads #####
##### ##### ##### ##### ##### ##### ##### #####  ##### 	
      # self.threadGetTasks = threading.Thread(target=self.getTasks, args=(), daemon=True)
		self.threadManageHeartbeats = threading.Thread(target=self.manageHeartbeats, args=()).start()
		## self.threadManageHeartbeats.start() 
		self.threadManageTasks = threading.Thread(target=self.manageTasks, args=()).start()
		## self.threadManageTasks.start() 
		self.threadManageStatus = threading.Thread(target=self.manageStatus, args=())
		self.threadManageStatus.start()          

			
##### ##### ##### ##### ##### ##### ##### #####  ##### 
##### Thread manageHeartbeats  #####
##### Get new heartbeats from fleet_proxy to ascertain connectivity
##### Heartbeat1 and Heartbeat2 parameters for AGV1 and AGV2 respectively
##### ##### ##### ##### ##### ##### ##### #####  ##### 	
	def manageHeartbeats(self):
		global ready4task1, ready4task2
		prev_heartbeat1=rospy.Time.now().secs		## prev ROS time
		prev_heartbeat2=rospy.Time.now().secs		## prev ROS time
		start_time1=start_time2 = time.time()
		heartbeat1_count=heartbeat2_count = 0
		zero_count1=zero_count2=0					## no. of zero heartbeat_count
		prevLiftStatus=0				## to keep track of change

		while True:
			if self.bCloseThreadFlag:
				break
			if self.bServerStarted:
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				### Process Heartbeat for AGV1
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				heartbeat1 = self.heartbeat1_parameter.get_value()			## get parameters from OPCUA server
				if (heartbeat1!=prev_heartbeat1):
					heartbeat1_count+=1												
				else:
					pass
				prev_heartbeat1=heartbeat1


				## print("time.time() - start_time1", time.time() - start_time1)
				if (time.time() - start_time1)>22:
					if (heartbeat1_count>=4):
						print("Heartbeat1 Count %d" % heartbeat1_count)
						zero_count1=0		## reset zero_count
					else:
						rospy.loginfo("Missed Heartbeat1 Count %d, zero_count1 %d", heartbeat1_count, zero_count1)

						if (heartbeat1_count==0):
							zero_count1+=1
						else:
							zero_count1=0		## reset to achieve continous zero_count1 number	
						## print("ready4task1 & zero_count1", ready4task1, zero_count1)
						if (self.task1_parameter.get_value()[0]==2 and zero_count1>=3):	## task allocated to AGV1 and continous 3 zero counts
							rospy.loginfo("--- Task allocated to AGV1 removed ---")
							newStatus1 = self.status1_parameter.get_value()		## status1 parameters from OPCUA server
							newStatus1[1]=rospy.Time.now().secs		## new ROS time
							newStatus1[7]=2			## completed & not ready, earmark for deletion
							newStatus1[8]=0			## prevent audio alarm activation
							self.status1_parameter.set_value(newStatus1)		## status1 parameters from OPCUA server
							##ready4task1=False
							##self.task1_parameter.set_value([0,rospy.Time.now(),0,0,0,0])	## delete allocated job
							zero_count1=0		## reset zero_count
							ready4task1=False		## Assume AGV1 not ready
							##task_Q_changed=True	## task_Q_changed		
					heartbeat1_count=0		## reset heartbeat_count
					start_time1 = time.time()
				
				""" """
				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				### Process Heartbeat for AGV2
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				heartbeat2 = self.heartbeat2_parameter.get_value()			## get parameters from OPCUA server
				if (heartbeat2!=prev_heartbeat2):
					heartbeat2_count+=1												
				else:
					pass
				prev_heartbeat2=heartbeat2

				## print("time.time() - start_time2", time.time() - start_time2)
				if (time.time() - start_time2)>22:
					if (heartbeat2_count>=4):
						print("Heartbeat2 Count %d" % heartbeat2_count)
						zero_count2=0		## reset zero_count
					else:
						rospy.loginfo("Missed Heartbeat2 Count %d, zero_count2 %d", heartbeat2_count, zero_count2)

						if (heartbeat2_count==0):
							zero_count2+=1
						else:
							zero_count2=0		## reset to achieve continous zero_count2 number	
						## print("ready4task2 & zero_count2", ready4task2, zero_count2)
						if (self.task2_parameter.get_value()[0]==2 and zero_count2>=3):	## continous 3 zero counts
							rospy.loginfo("--- Task allocated to AGV2 removed ---")
							newStatus2 = self.status2_parameter.get_value()		## status2 parameters from OPCUA server
							newStatus2[1]=rospy.Time.now().secs		## new ROS time
							newStatus2[7]=2			## completed & not ready, earmark for deletion
							newStatus2[8]=0			## prevent audio alarm activation							
							self.status2_parameter.set_value(newStatus2)		## status2 parameters from OPCUA server
							##ready4task2=False
							##self.task2_parameter.set_value([0,rospy.Time.now(),0,0,0,0])	## delete allocated job
							zero_count2=0		## reset zero_count
							ready4task2=False		## Assume AGV2 not ready							
							##task_Q_changed=True	## task_Q_changed	
					heartbeat2_count=0
					start_time2 = time.time()
				""" """
				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				### Report Proxy Connection for AGV1
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				proxy1 = self.proxy1_parameter.get_value()			## get parameters from OPCUA server
				### print("value", proxy1)
				if proxy1:
					rospy.loginfo("--- AGV1 Fleet Proxy connected ---")
					self.proxy1_parameter.set_value(0)
				""" """
				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				### Report Proxy Connection for AGV2
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				proxy2 = self.proxy2_parameter.get_value()			## get parameters from OPCUA server
				### print("value", proxy2)
				if proxy2:
					rospy.loginfo("--- AGV2 Fleet Proxy connected ---")
					self.proxy2_parameter.set_value(0)
				""" """
				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				### Update GUI Color for Lift Status Label
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				liftStatus = self.liftStatus_parameter.get_value()			## get parameters from OPCUA server
				if (liftStatus!=prevLiftStatus):
					if liftStatus:				## liftStatus!=0
						rospy.loginfo("--- Lift is occupied by AGV%d ---", liftStatus)
						self.liftStatusLabel.configure(bg='salmon')
						self.liftStatusLabel.configure(text='Lift Status ['+str(liftStatus)+']')	
					else:
						## rospy.loginfo("--- One lift already occupied by AGV ---")
						self.liftStatusLabel.configure(bg=DEFAULT_BUTTON_COLOR)
						self.liftStatusLabel.configure(text='Lift Status')					
					prevLiftStatus=liftStatus
				""" """
			time.sleep(1)


##### ##### ##### ##### ##### ##### ##### #####  ##### 
##### Thread manageTasks  #####
##### Get new tasks submitted by call stations and assigned to
##### Task1 and Task2 parameters for AGV1 and AGV2 respectively
##### ##### ##### ##### ##### ##### ##### #####  ##### 	
	def manageTasks(self):
		global ready4task1, ready4task2, task1_completed, task2_completed, unassigned_Q, task_Q_changed, varBeacon
		## global time_allocated2AGV1, time_published2AGV1, curtime_allocated2AGV1
		curTask1=curTask2=[0,0,0,0,0,0]		## initialize temp list
		###curBeacon=0		###20200625###	
		curtime_allocated2AGV1=curtime_allocated2AGV2=0	## saved time task allocated to AGV1/AGV2
		time_allocated2AGV1=time_allocated2AGV2=0		## time task allocated to AGV1/AGV2
		time_published2AGV1=time_published2AGV2=0		## time task published to AGV1/AGV2
		time_count1=time_count2=0			## delay for Fleet Proxy to publish to AGV1/AGV2 and update opcua server			
		taskQsize=0
		fromLP = {73:'L1LRm3', 74:'L1LRm4', 75:'L1LRm5', 76:'L1LRm6', 77:'L1LRm7', 78:'L1LRm8', 79:'L1LRm9',
				3:'L3Ward3', 4:'L4Ward4', 5:'L5Ward5', 6:'L6Ward6', 7:'L7Ward7', 8:'L8Ward8', 9:'L9Ward9', 
				10:'L10StaffDorm', 11:'L11Ktchen', 51:'L1MainStore', 52:'L1NonHalalPrepRoom', 0:'---'}
		toLP = {73:'L1LRm3', 74:'L1LRm4', 75:'L1LRm5', 76:'L1LRm6', 77:'L1LRm7', 78:'L1LRm8', 79:'L1LRm9',
				3:'L3Ward3', 4:'L4Ward4', 5:'L5Ward5', 6:'L6Ward6', 7:'L7Ward7', 8:'L8Ward8', 9:'L9Ward',
				10:'L10StaffDorm', 11:'L11Ktchen', 51:'L1MainStore', 52:'L1NonHalalPrepRoom', 0:'---'}	
		
		while True:
			if self.bCloseThreadFlag:
				break
			if self.bServerStarted:
				## remove deleted tasks from task_Q
				for deleted in delete_Q:
					[task_Q.remove(x) for x in task_Q if deleted[1] in x]	## if time.secs matches
					task_Q_changed=True				## task_Q_changed	
##				task1_completed=False		## may not need
##				task2_completed=False		## may not need
								
##				if task_Q:	## list not empty
##					self.task1_parameter.set_value([task_Q[0][0], task_Q[0][1], task_Q[0][2], task_Q[0][3], task_Q[0][4]])
##					del task_Q[0]	## remove from list

				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				# Assign next task in the queue to AGV1
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				if task_Q and ready4task1:		## list not empty and ready
					if task_Q[0][0]==1:			## first task in queue not assigned
						task_Q[0][0]=2			## set to assigned task
						task_Q[0][5]=1			## preset alloc==1 for task allocated to AGV1
						task_Q_changed=True				## task_Q_changed							
						self.task1_parameter.set_value([task_Q[0][0], task_Q[0][1], task_Q[0][2], task_Q[0][3], task_Q[0][4], 0]) ## alloc==0 initially
						time_allocated2AGV1=rospy.Time.now().secs		## time task allocated to AGV1
						time_published2AGV1=time_allocated2AGV1+10	## unusual long time task published to AGV1
						rospy.loginfo("Time allocated2AGV1 - %d", time_allocated2AGV1)
						ready4task1=False
						print(529,task_Q)	## print only when changed					
					elif len(task_Q)>=2 and task_Q[1][0]==1:		## second task in queue not assigned ##20210715
						task_Q[1][0]=2				## set to assigned task
						task_Q[1][5]=1				## preset alloc==1 for task allocated to AGV1
						task_Q_changed=True			## task_Q_changed							
						self.task1_parameter.set_value([task_Q[1][0], task_Q[1][1], task_Q[1][2], task_Q[1][3], task_Q[1][4], 0]) ## alloc==0 initially
						time_allocated2AGV1=rospy.Time.now().secs		## time task allocated to AGV1
						time_published2AGV1=time_allocated2AGV1+10	## unusual long time task published to AGV1
						rospy.loginfo("Time allocated2AGV1 - %d", time_allocated2AGV1)
						ready4task1=False
						print(539,task_Q)	## print only when changed	

				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				# Assign next task in the queue to AGV2
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				if task_Q and ready4task2:		## list not empty and ready
					if task_Q[0][0]==1:			## first task in queue not assigned
						task_Q[0][0]=2			## set to assigned task
						task_Q[0][5]=2			## preset alloc==2 for task allocated to AGV2
						task_Q_changed=True	## task_Q_changed						
						self.task2_parameter.set_value([task_Q[0][0], task_Q[0][1], task_Q[0][2], task_Q[0][3], task_Q[0][4], 0]) ## alloc==0 initially
						time_allocated2AGV2=rospy.Time.now().secs		## time task allocated to AGV2
						time_published2AGV2=time_allocated2AGV2+10	## unusual long time task published to AGV2
						rospy.loginfo("Time allocated2AGV2 - %d", time_allocated2AGV2)
						ready4task2=False
						print(554,task_Q)	## print only when changed					
					elif len(task_Q)>=2 and task_Q[1][0]==1:		## second task in queue not assigned ##20210715
						task_Q[1][0]=2				## set to assigned task
						task_Q[1][5]=2				## preset alloc==2 for task allocated to AGV2
						task_Q_changed=True			## task_Q_changed						
						self.task2_parameter.set_value([task_Q[1][0], task_Q[1][1], task_Q[1][2], task_Q[1][3], task_Q[1][4], 0]) ## alloc==0 initially)
						time_allocated2AGV2=rospy.Time.now().secs		## time task allocated to AGV2
						time_published2AGV2=time_allocated2AGV2+10	## unusual long time task published to AGV2
						rospy.loginfo("Time allocated2AGV2 - %d", time_allocated2AGV2)
						ready4task2=False
						print(564,task_Q)	## print only when changed					

				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				# Reset GUI display when list is empty 
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				if not task_Q:					## list is empty
					## Reset task1 (allocated to AGV1) on OPC parameters
					self.task1_parameter.set_value([0,0,0,0,0,0])
					## Reset task1 (allocated to AGV1) on GUI ##						
					self.task1_cmd.set(self.task1_parameter.get_value()[0])
					self.task1_type.set(self.task1_parameter.get_value()[2])
					self.task1_fromLP.set(fromLP[self.task1_parameter.get_value()[3]])
					self.task1_toLP.set(toLP[self.task1_parameter.get_value()[4]])		
					## Reset task2 (allocated to AGV2) OPC parameters
					self.task2_parameter.set_value([0,0,0,0,0,0])
					## Reset task2 (allocated to AGV2) on GUI ##						
					self.task2_cmd.set(self.task2_parameter.get_value()[0])
					self.task2_type.set(self.task2_parameter.get_value()[2])
					self.task2_fromLP.set(fromLP[self.task2_parameter.get_value()[3]])
					self.task2_toLP.set(toLP[self.task2_parameter.get_value()[4]])		
				"""
				if completedTask[0]!=0 and not ready4task1:		## not ready received
					## Reset taskCompleted OPC parameters
					self.taskCompleted_parameter.set_value([0,0,0,0,0])
					## Reset taskCompleted on GUI ##
					self.task2_cmd.set(self.taskCompleted_parameter.get_value()[0])
					self.task2_type.set(self.taskCompleted_parameter.get_value()[2])
					self.task2_fromLP.set(self.taskCompleted_parameter.get_value()[3])
					self.task2_toLP.set(self.taskCompleted_parameter.get_value()[4])
				"""
				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				### Update GUI for Task1
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				newTask1 = self.task1_parameter.get_value()			## get task allocated to AGV1 parameters from OPCUA server

				if (curTask1[0] != newTask1[0] or curTask1[1] != newTask1[1] or
					curTask1[2] != newTask1[2] or curTask1[3] != newTask1[3] or 
					curTask1[4] != newTask1[4] or curTask1[5] != newTask1[5]):
				
					if (newTask1[0]==2 and newTask1[5]==1):	## task successfully allocated to AGV1
						##print("newTask1[0] is %d" % newTask1[0])
						## Update task1 (allocated to AGV1) on GUI ##						
						self.task1_cmd.set(newTask1[0])
						self.task1_type.set(newTask1[2])
						self.task1_fromLP.set(fromLP[newTask1[3]])
						self.task1_toLP.set(toLP[newTask1[4]])
						
						rospy.loginfo("Task published2AGV1 ... %d, %d, %d, %d, %d, %d",newTask1[0],newTask1[1],newTask1[2],
								newTask1[3],newTask1[4],newTask1[5])
						
						time_published2AGV1=rospy.Time.now().secs		## time task published to AGV1		
						rospy.loginfo("Time published2AGV1 - %d", time_published2AGV1)

					##elif (newTask1[0]==3 and not task1_completed):	## cancel task
					elif (newTask1[0]==3):	## cancel task
						## print("newTask1[0] is %d" % newTask1[0])
						## Reset task1 (allocated to AGV1) on OPC parameters
						self.task1_parameter.set_value([0,0,0,0,0,0])		## Note cannot set Task1 to NULL at this moment (20210410)
						## Reset task1 (allocated to AGV1) on GUI ##						
						self.task1_cmd.set(0)
						self.task1_type.set(0)
						self.task1_fromLP.set(fromLP[0])
						self.task1_toLP.set(toLP[0])

					elif (newTask1[0]==0):	## No task allocated to AGV1
						## Reset task1 (allocated to AGV1) on GUI ##						
						self.task1_cmd.set(0)
						self.task1_type.set(0)
						self.task1_fromLP.set(fromLP[0])
						self.task1_toLP.set(toLP[0])					
					"""
					elif (newTask1[0]==3 and newTask1[4]==self.task1_toLP.get()):	## cancel task
						## Reset task1 (allocated to AGV1) on OPC parameters
						self.task1_parameter.set_value([0,0,0,0,0,0])
						## Reset task1 (allocated to AGV1) on GUI ##						
						self.task1_cmd.set(self.task1_parameter.get_value()[0])
						self.task1_type.set(self.task1_parameter.get_value()[2])
						self.task1_fromLP.set(self.task1_parameter.get_value()[3])
						self.task1_toLP.set(self.task1_parameter.get_value()[4])
					"""
					curTask1 = list(newTask1)			## update curTask1		
				## end if
				
				""" """
				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				### Update GUI for Task2
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				newTask2 = self.task2_parameter.get_value()			## get task allocated to AGV2 parameters from OPCUA server
				
				if (curTask2[0] != newTask2[0] or curTask2[1] != newTask2[1] or
					curTask2[2] != newTask2[2] or curTask2[3] != newTask2[3] or 
					curTask2[4] != newTask2[4] or curTask2[5] != newTask2[5]):
				
					if (newTask2[0]==2 and newTask2[5]==2):	## successfully allocated to AGV2
						##print("newTask2[0] is %d" % newTask2[0])
						## Update task2 (allocated to AGV2) on GUI ##						
						self.task2_cmd.set(newTask2[0])
						self.task2_type.set(newTask2[2])
						self.task2_fromLP.set(fromLP[newTask2[3]])
						self.task2_toLP.set(toLP[newTask2[4]])
						
						rospy.loginfo("Task published2AGV2 ... %d, %d, %d, %d, %d, %d",newTask2[0],newTask2[1],newTask2[2],
								newTask2[3],newTask2[4],newTask2[5])						
						
						time_published2AGV2=rospy.Time.now().secs		## time task published to AGV2		
						rospy.loginfo("Time published2AGV2 - %d", time_published2AGV2)						

					##elif (newTask2[0]==3 and not task2_completed):	## cancel task
					elif (newTask2[0]==3):	## cancel task
						## print("newTask2[0] is %d" % newTask2[0])
						## Reset task2 (allocated to AGV2) on OPC parameters
						self.task2_parameter.set_value([0,0,0,0,0,0])		## Note cannot set Task2 to NULL at this moment (20210410)
						## Reset task2 (allocated to AGV2) on GUI ##						
						self.task2_cmd.set(0)
						self.task2_type.set(0)
						self.task2_fromLP.set(fromLP[0])
						self.task2_toLP.set(toLP[0])

					elif (newTask2[0]==0):	## No task allocated to AGV2
						self.task2_cmd.set(0)
						self.task2_type.set(0)
						self.task2_fromLP.set(fromLP[0])
						self.task2_toLP.set(toLP[0])
					"""
					elif (newTask2[0]==3 and newTask2[4]==self.task2_toLP.get()):	## cancel task
						## Reset task2 (allocated to AGV2) on OPC parameters
						self.task2_parameter.set_value([0,0,0,0,0,0])
						## Reset task2 (allocated to AGV2) on GUI ##						
						self.task2_cmd.set(self.task2_parameter.get_value()[0])
						self.task2_type.set(self.task2_parameter.get_value()[2])
						self.task2_fromLP.set(self.task2_parameter.get_value()[3])
						self.task2_toLP.set(self.task2_parameter.get_value()[4])
					"""
					curTask2 = list(newTask2)			## update curTask1		
				## end if
				""" """
						
				## Publish when task list not empty and (task_cmd changed or change in task_Q size)
###				if (task_Q and (task_Q_changed or taskQsize!=len(task_Q))):  
				if (task_Q_changed or taskQsize!=len(task_Q)):  ## modify on 20210414
				### Publish all tasks in queue to call stations via ROSbridge	
					###print(task_Q)			
					pub_msg = jobs()							### an instant of ntuc_fleet/jobs message type
					pub_msg.cmd = 1								### cmd==1
					pub_msg.time = rospy.Time.now()				### ROS time
					### Add all tasks to ROS message array				
					for each in task_Q:			 			### build task ROS message for each task in queue
						task_msg = task()
						task_msg.cmd = each[0]
						task_msg.time.secs = each[1]		### only secs is stored in task 
						task_msg.type = each[2]		
						task_msg.fromLP = each[3]	
						task_msg.toLP = each[4]	
						task_msg.alloc = each[5]							
						pub_msg.tasks.append(task_msg)
					pub_jobs.publish(pub_msg)				### publish to ROS topic
					taskQsize = len(task_Q)					### update taskQsize
					task_Q_changed=False					### reset task_Q_changed

					### open output file for writing (default written to .ros @home)
					with open('../catkin_ws/src/ntuc_fleet/data/list_of_tasks.txt', 'w') as filehandle:
						json.dump(task_Q, filehandle)
##					filehandle.close()

					with open('../catkin_ws/src/ntuc_fleet/data/listfile.data', 'wb') as filehandle:
					# store the data as binary data stream
						pickle.dump(task_Q, filehandle)
##					filehandle.close()
				""" ###20200625### ------
				newBeacon = self.beaconAlarm_parameter.get_value()			## get parameters from OPC server
				if (curBeacon != newBeacon):
					## varBeacon.set_value([8,0])
					curBeacon = newBeacon			## update curBeaconn
				"""
				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				### Update GUI for AGV1 and AGV2 availability
				##### ##### ##### ##### ##### ##### ##### #####  ##### 						
				if ready4task1:				## AGV1 ready to received tasks
					self.statusAGV1Label.config(text="AGV1 is available");
				else:
					self.statusAGV1Label.config(text="AGV1 is busy");	
						
				if ready4task2:				## AGV2 ready to received tasks
					self.statusAGV2Label.config(text="AGV2 is available");
				else:
					self.statusAGV2Label.config(text="AGV2 is busy");	

				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				### Activate/Deactivate delete tasks buttons
				##### ##### ##### ##### ##### ##### ##### #####  ##### 						
				if task_Q:				## task_Q is not empty
					self.delete_all_button.configure(state = 'active')
					if newTask1[0]==2:		## AGV1 is allocated a task
						self.delete_allocated2AGV1_button.configure(state = 'active')
					else:
						self.delete_allocated2AGV1_button.configure(state = 'disabled')
					if newTask2[0]==2:		## AGV2 is allocated a task
						self.delete_allocated2AGV2_button.configure(state = 'active')
					else:
						self.delete_allocated2AGV2_button.configure(state = 'disabled')
					""" """
					tempList = [each for each in task_Q if each[0]==1]		## new list of unallocated tasks
					if tempList:			## is tempList is not empty
						self.delete_unallocated_button.configure(state = 'active')
					else:
						self.delete_unallocated_button.configure(state = 'disabled')
					""" """
				else:
					self.delete_all_button.configure(state = 'disabled')
					self.delete_unallocated_button.configure(state = 'disabled')
					self.delete_allocated2AGV1_button.configure(state = 'disabled')
					self.delete_allocated2AGV2_button.configure(state = 'disabled')
						
				### Determine if task was published to AGV1 ###
				if (curtime_allocated2AGV1!=time_allocated2AGV1 and time_count1>3):				
					if (time_allocated2AGV1>0 and (time_published2AGV1-time_allocated2AGV1)==10):
						rospy.loginfo("***** Task not published to AGV1! **** %d", time_count1)
					else:
						rospy.loginfo("***** Task published to AGV1! **** %d", time_count1)			
					curtime_allocated2AGV1=time_allocated2AGV1		## save current time
					time_count1=0
				if (curtime_allocated2AGV1!=time_allocated2AGV1):
					time_count1+=1					## increament time_count1	
				### rospy.loginfo("***** Running time_count1 **** %d", time_count1)		
				""" """
				### Determine if task was published to AGV2 ###
				if (curtime_allocated2AGV2!=time_allocated2AGV2 and time_count2>3):				
					if (time_allocated2AGV2>0 and (time_published2AGV2-time_allocated2AGV2)==10):
						rospy.loginfo("***** Task not published to AGV2! **** %d", time_count2)
					else:
						rospy.loginfo("***** Task published to AGV2! **** %d", time_count2)			
					curtime_allocated2AGV2=time_allocated2AGV2		## save current time
					time_count2=0
				if (curtime_allocated2AGV2!=time_allocated2AGV2):
					time_count2+=1					## increament time_count2	
				### rospy.loginfo("***** Running time_count2 **** %d", time_count2)		
				"""	"""		
			time.sleep(0.5) 
		## end while
	## end manageTasks()  


##### ##### ##### ##### ##### ##### ##### #####  ##### 
##### Thread manageStatus Definition #####
##### ##### ##### ##### ##### ##### ##### #####  ##### 	
	def manageStatus(self):
		global ready4task1, ready4task2, task1_completed, task2_completed, task_Q_changed
		L11Kitchen = 11								# L11Kitchen LP is 11 ###20200625###	
		curStatus1 = [0,0,0,0,0,0,0,0,0]			# AGV1
		curStatus2 = [0,0,0,0,0,0,0,0,0]			# AGV2	
		
		while True:
			if self.bCloseThreadFlag:
				break
			if self.bServerStarted:
				newTask1 = self.task1_parameter.get_value()			## task1 parameters from OPCUA server
				newTask2 = self.task2_parameter.get_value()			## task2 parameters from OPCUA server
				newStatus1 = self.status1_parameter.get_value()		## status1 parameters from OPCUA server
				newStatus2 = self.status2_parameter.get_value()		## status2 parameters from OPCUA server				
				##completedTask = self.taskCompleted_parameter.get_value()	## get parameters from OPCUA server

##				print("Into mangageStatus() curStatus1", curStatus1[0],curStatus1[1],curStatus1[2],curStatus1[3],curStatus1[4],curStatus1[5],curStatus1[6],curStatus1[7])
##				print("Into mangageStatus() newStatus1", newStatus1[0],newStatus1[1],newStatus1[2],newStatus1[3],newStatus1[4],newStatus1[5],newStatus1[6],newStatus1[7])
##				if newStatus1[7]!=curStatus1[7]:
##					print("Inside mangageStatus(), newStatus1[7]", newStatus1[7])	
				
				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				### process status message for AGV1	
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				if (curStatus1[0]!=newStatus1[0] or curStatus1[1]!=newStatus1[1] or curStatus1[2]!=newStatus1[2] or
					curStatus1[3]!=newStatus1[3] or curStatus1[5]!=newStatus1[5] or 
					curStatus1[6]!=newStatus1[6] or curStatus1[7]!=newStatus1[7] or curStatus1[8]!=newStatus1[8]):

##				if (curStatus1[0]!=newStatus1[0] or curStatus1[1]!=newStatus1[1] or curStatus1[2]!=newStatus1[2] or
##					curStatus1[3]!=newStatus1[3] or curStatus1[4]!=newStatus1[4] or curStatus1[5]!=newStatus1[5] or 
##					curStatus1[6]!=newStatus1[6] or curStatus1[7]!=newStatus1[7] or curStatus1[8]!=newStatus1[8]):
										
					print("Inside manageStatus(),agv_status",newStatus1[0],newStatus1[1],newStatus1[2],newStatus1[3],
						newStatus1[4],newStatus1[5],newStatus1[6],newStatus1[7],newStatus1[8])	
				
					if (newStatus1[0]==1):	## operational status

						if (newStatus1[7]==4):					## dispatch task aborted
							if (len(task_Q)>=2):
								if (task_Q[0][1]==newTask1[1]): 		## first task in queue matches ROS time
									task_Q[0][0]=1;						## change first task in the queue to unassigned
									task_Q_changed=True				## task_Q_changed
									##print(task_Q)	
									newTask1[0]=3						## mark for deletion of task1_parameter
									## self.task1_parameter.set_value(newTask1)			## not needed
									##task1_completed=False
									##ready4task1=False

								elif (task_Q[1][1]==newTask1[1]): 		## second task in queue matches ROS time
									task_Q[1][0]=1;						## change second task in the queue to unassigned
									task_Q_changed=True				## task_Q_changed									
									##print(task_Q)	
									newTask1[0]=3						## mark for deletion of task1_parameter
									## self.task1_parameter.set_value(newTask1)			## not needed
									##task1_completed=False
									##ready4task1=False	
							elif (len(task_Q)==1):													
								if (task_Q[0][1]==newTask1[1]): 		## first task in queue matches ROS time
									task_Q[0][0]=1;						## change first task in the queue to unassigned
									task_Q_changed=True				## task_Q_changed									
									##print(task_Q)	
									newTask1[0]=3						## mark for deletion of task1_parameter
									## self.task1_parameter.set_value(newTask1)			## not needed
									##task1_completed=False
									##ready4task1=False

							print(877,task_Q)	
							##task1_completed=False
							ready4task1=False	
																
							## Reset task allocated to AGV1 on GUI ##
							self.task1_cmd.set(0)
							self.task1_type.set(0)
							self.task1_fromLP.set(0)
							self.task1_toLP.set(0)
						#### end if (newStatus1[7]==4):
						elif (newStatus1[7]==3):					## AGV not ready for dispatch task
							print(task_Q)			
							ready4task1=False

						## Assigned Task is completed
						## Update task1_completed and ready4task
						elif (newStatus1[7]==1 or newStatus1[7]==2):				## assigned task completed
						
							if newTask1[0]==2:								## assigned task
								delete_Q.append(newTask1)					## mark for deletion
								##task1_completed=True
							##	newTask1[0]=0								## invalid task
							print(899,task_Q)			
																							
							## Reset task allocated to AGV1 on GUI ##
							self.task1_cmd.set(0)
							self.task1_type.set(0)
							self.task1_fromLP.set(0)
							self.task1_toLP.set(0)
							time.sleep(0.5)						

							## AGV ready for dispatch task
							if (newStatus1[7]==1):			## AGV ready for dispatch task
								###print(task_Q)		###20210625###								
								ready4task1=True
							elif (newStatus1[7]==2):		## AGV not ready for dispatch task
								ready4task1=False							
							### Task allocated to AGV1 removed
							self.task1_parameter.set_value([0,0,0,0,0,0])
							### Set Fleet Manager beaconAlarm_parameter if lastLP is L11Kitchen ###20200625###
							if (newStatus1[8]==11):			## lastLP is L11Kitchen
								self.beaconAlarm_parameter.set_value(1)	
						#### end elif (newStatus1[7]==1 or newStatus1[7]==2):
						
						### Take care of repeat complete==0 status message from AGV1
						elif (newStatus1[7]==0):
							if (newTask1[0]!=2):			## no task allocated 
								pass
							elif (newTask1[0]==2):		## task allocated, assume rebooted from with incompleted task
								delete_Q.append(newTask1)		## mark for deletion	
								self.task1_parameter.set_value([0,0,0,0,0,0])	### remove Task allocated to AGV1			
								##task1_completed=True	
							ready4task1=True									
							print(930,task_Q)								
						#### end if (newStatus1[7]==0):
						
						### Additional checking to prevent errorneous publishing of task completion when no task is assigned
						### to prevent sounding of alarm at call stations when actually empty trolley was not delivered
						if (newStatus1[7]==1 and newTask1[0]!=2):			## No task was assigned, complete should be 0 [Ready]
							newStatus1[7]=0				
						elif (newStatus1[7]==2 and newTask1[0]!=2):			## No task was assigned, complete should be 3 [Not Ready]
							newStatus1[7]=3	

						## Send update to call station via ROSbridge
						pub_msg = agv_status()				### an instant of ntuc_fleet/agv_status message type
						pub_msg.cmd = newStatus1[0]			### cmd==1
						pub_msg.time.secs = newStatus1[1]	### time in secs
						pub_msg.status = newStatus1[2]		### status of AGV	
						pub_msg.location = newStatus1[3]		### location LP
						pub_msg.arrival.secs = newStatus1[4]	### estimated arrival time secs
						pub_msg.b_level = newStatus1[5]		### battery level in percent	
						pub_msg.e_status = newStatus1[6]		### elevator status		
						pub_msg.complete = newStatus1[7]		### task completion and readiness	
						pub_msg.lastLP = newStatus1[8]		### lastLP of just completed task
						pub_status.publish(pub_msg)

					elif (newStatus1[0]==2):	## alert status
						pass
						
					elif (newStatus1[0]==3):	## alarm status
						pass

					elif (newStatus1[0]==4):	## agv status

						## Send update to call station via ROSbridge
						pub_msg = agv_status()				### an instant of ntuc_fleet/agv_status message type
						pub_msg.cmd = newStatus1[0]			### cmd==1
						pub_msg.time.secs = newStatus1[1]	### time in secs
						pub_msg.status = newStatus1[2]		### status of AGV	
						pub_msg.location = newStatus1[3]		### location LP
						pub_msg.arrival.secs = newStatus1[4]	### estimated arrival time secs
						pub_msg.b_level = newStatus1[5]		### battery level in percent	
						pub_msg.e_status = newStatus1[6]		### elevator status		
						pub_msg.complete = newStatus1[7]		### task completion and readiness	
						pub_msg.lastLP = newStatus1[8]		### lastLP of just completed task
						pub_status.publish(pub_msg)	
						
					curStatus1 = list(newStatus1)			## update curStatus1
				
					
				##### ##### ##### ##### ##### ##### ##### #####  ##### 	
				### process status message for AGV2	
				##### ##### ##### ##### ##### ##### ##### #####  ##### 
				if (curStatus2[0]!=newStatus2[0] or curStatus2[1]!=newStatus2[1] or curStatus2[2]!=newStatus2[2] or 
					curStatus2[3]!=newStatus2[3] or curStatus2[4]!=newStatus2[4] or curStatus2[5]!=newStatus2[5] or
					curStatus2[6]!=newStatus2[6] or curStatus2[7]!=newStatus2[7] or curStatus2[8]!=newStatus2[8] ):
									
					print("Inside manageStatus(),agv_status",newStatus2[0],newStatus2[1],newStatus2[2],newStatus2[3],
						newStatus2[4],newStatus2[5],newStatus2[6],newStatus2[7],newStatus2[8])	
				
					if (newStatus2[0]==1):	## operational status

						if (newStatus2[7]==4):					## dispatch task aborted
							if (len(task_Q)>=2):
								if (task_Q[0][1]==newTask2[1]): 		## first task in queue matches ROS time
									task_Q[0][0]=1;						## change first task in the queue to unassigned
									task_Q_changed=True				## task_Q_changed
									##print(task_Q)	
									newTask2[0]=3						## mark for deletion of task2_parameter
									## self.task2_parameter.set_value(newTask2)			## not needed
									##task2_completed=False
									##ready4task2=False

								elif (task_Q[1][1]==newTask2[1]): 		## second task in queue matches ROS time
									task_Q[1][0]=1;						## change second task in the queue to unassigned
									task_Q_changed=True				## task_Q_changed									
									##print(task_Q)	
									newTask2[0]=3						## mark for deletion of task2_parameter
									## self.task2_parameter.set_value(newTask2)			## not needed
									##task2_completed=False
									##ready4task2=False	
							elif (len(task_Q)==1):													
								if (task_Q[0][1]==newTask2[1]): 		## first task in queue matches ROS time
									task_Q[0][0]=1;						## change first task in the queue to unassigned
									task_Q_changed=True				## task_Q_changed									
									##print(task_Q)	
									newTask2[0]=3						## mark for deletion of task2_parameter
									## self.task2_parameter.set_value(newTask2)			## not needed
									##task2_completed=False
									##ready4task2=False

							print(1018,task_Q)	
							##task2_completed=False
							ready4task2=False																	
																
							## Reset task allocated to AGV2 on GUI ##
							self.task2_cmd.set(0)
							self.task2_type.set(0)
							self.task2_fromLP.set(0)
							self.task2_toLP.set(0)
						#### end if (newStatus2[7]==4):
						elif (newStatus2[7]==3):					## AGV2 not ready for dispatch task
							print(task_Q)			
							ready4task2=False

						## Assigned Task is completed
						## Update task2_completed and ready4task
						elif (newStatus2[7]==1 or newStatus2[7]==2):				## assigned task completed
						
							if newTask2[0]==2:								## assigned task
								delete_Q.append(newTask2)					## mark for deletion
								##task2_completed=True
							##	newTask2[0]=0								## invalid task
							print(1040,task_Q)			
							
							## Reset task allocated to AGV2 on GUI ##
							self.task2_cmd.set(0)
							self.task2_type.set(0)
							self.task2_fromLP.set(0)
							self.task2_toLP.set(0)	
							time.sleep(0.5)																		

							## AGV ready for dispatch task
							if (newStatus2[7]==1):			## AGV ready for dispatch task
								###print(task_Q)		###20210625###										
								ready4task2=True
							elif (newStatus2[7]==2):		## AGV not ready for dispatch task
								ready4task2=False
							### Task allocated to AGV2 removed
							self.task2_parameter.set_value([0,0,0,0,0,0])													
							### Set Fleet Manager beaconAlarm_parameter if lastLP is L11Kitchen ###20200625###
							if (newStatus2[8]==L11Kitchen):			## lastLP is L11Kitchen
								self.beaconAlarm_parameter.set_value(1)		
						#### end elif (newStatus2[7]==1 or newStatus2[7]==2):

						### Take care of repeat complete==0 status message from AGV2
						elif (newStatus2[7]==0):
							if (newTask2[0]!=2):			## no task allocated 
								pass
							elif (newTask2[0]==2):		## task allocated, assume rebooted from with incompleted task
								delete_Q.append(newTask2)		## mark for deletion	
								self.task2_parameter.set_value([0,0,0,0,0,0])	### remove Task allocated to AGV2		
								##task2_completed=True	
							ready4task2=True									
							print(1071,task_Q)	
						#### end if (newStatus2[7]==0):

						### Additional checking to prevent errorneous publishing of task completion when no task is assigned
						### to prevent sounding of alarm at call stations when actually empty trolley was not delivered
						if (newStatus2[7]==1 and newTask2[0]!=2):			## No task was assigned, complete should be 0 [Ready]
							newStatus2[7]=0				
						elif (newStatus2[7]==2 and newTask2[0]!=2):			## No task was assigned, complete should be 3 [Not Ready]
							newStatus2[7]=3	

						## Send update to call station via ROSbridge
						pub_msg = agv_status()				### an instant of ntuc_fleet/agv_status message type
						pub_msg.cmd = newStatus2[0]			### cmd==1
						pub_msg.time.secs = newStatus2[1]	### time in secs
						pub_msg.status = newStatus2[2]		### status of AGV	
						pub_msg.location = newStatus2[3]		### location LP
						pub_msg.arrival.secs = newStatus2[4]	### estimated arrival time secs
						pub_msg.b_level = newStatus2[5]		### battery level in percent	
						pub_msg.e_status = newStatus2[6]		### elevator status		
						pub_msg.complete = newStatus2[7]		### task completion and readiness	
						pub_msg.lastLP = newStatus2[8]		### lastLP of just completed task
						pub_status.publish(pub_msg)	

					elif (newStatus2[0]==2):	## alert status
						pass
						
					elif (newStatus2[0]==3):	## alarm status
						pass

					elif (newStatus2[0]==4):	## agv status

						## Send update to call station via ROSbridge
						pub_msg = agv_status()				### an instant of ntuc_fleet/agv_status message type
						pub_msg.cmd = newStatus2[0]			### cmd==1
						pub_msg.time.secs = newStatus2[1]	### time in secs
						pub_msg.status = newStatus2[2]		### status of AGV	
						pub_msg.location = newStatus2[3]		### location LP
						pub_msg.arrival.secs = newStatus2[4]	### estimated arrival time secs
						pub_msg.b_level = newStatus2[5]		### battery level in percent	
						pub_msg.e_status = newStatus2[6]		### elevator status		
						pub_msg.complete = newStatus2[7]		### task completion and readiness	
						pub_msg.lastLP = newStatus2[8]		### lastLP of just completed task
						pub_status.publish(pub_msg)	
						
					curStatus2 = list(newStatus2)			## update curStatus2
			
				## end if
			time.sleep(0.5)
		## end while
 ## end manageStatus()  

                
##### ##### ##### ##### ##### ##### ##### #####  ##### 
##### GUI update method #####
##### ##### ##### ##### ##### ##### ##### #####  #####             
	def update(self, method):
		global task_Q, task1_completed, task2_completed, ready4task1, ready4task2, last_time, task_Q_changed
		if method == "start":
			self.server.start()
			print("Fleet OPCUA Server Started")
			self.start_button.configure(state = 'disabled')
			self.stop_button.configure(state = 'active')
			self.update_button.configure(state = 'active')
			self.bServerStarted = True
		elif method == "stop":
			self.server.stop()
			print("OPCUA Server Stopped")
			self.start_button.configure(state = 'active')
			self.stop_button.configure(state = 'disabled')
			self.update_button.configure(state = 'disabled')
			self.bServerStarted = False
		elif method == "update":		## Update button pressed
            ## Update Fire Service Alarm OPCUA parameter
			self.fireAlarm_parameter.set_value(self.updatedFireAlarm)
	   		## Update color of GUI label of Fire Service Alarm
			if self.updatedFireAlarm:
				self.fireAlarmLabel.configure(bg='salmon')
			else:
				self.fireAlarmLabel.configure(bg=DEFAULT_BUTTON_COLOR)

            ## Update Lift Status OPCUA parameter
			self.liftStatus_parameter.set_value(self.updatedLiftStatus)
			"""
	    	## Update color of GUI label of Lift Status
			if self.updatedLiftStatus:
				self.liftStatusLabel.configure(bg='salmon')
			else:
				self.liftStatusLabel.configure(bg=DEFAULT_BUTTON_COLOR)
			"""
            ## Update Beacon Alarm OPCUA parameter
			self.beaconAlarm_parameter.set_value(self.updatedBeaconAlarm)
	    	## Update color of GUI label of Beacon Alarm
			if self.updatedBeaconAlarm:
				self.beaconAlarmLabel.configure(bg='salmon')
			else:
				self.beaconAlarmLabel.configure(bg=DEFAULT_BUTTON_COLOR)

		elif method == "delete_all":
			self.task1_parameter.set_value([0,0,0,0,0,0])
			self.task2_parameter.set_value([0,0,0,0,0,0])
			##task1_completed=False	## task assigned to AGV1 completed
			ready4task1=False			## AGV1 ready for new task
			##task2_completed=False	## task assigned to AGV2 completed
			ready4task2=False			## AGV2 ready for new task
			task_Q=[]
			## last_time=rospy.Time.now().secs		## last ROS time
			print("All Tasks Deleted . . .")
			task_Q_changed=True	## task Q changed?		
		elif method == "delete_unallocated":
			new_list=[]
			## delete unallocated tasks from task_Q
			for task in task_Q:
				if task[0]==2:
					new_list.append(task)
			task_Q = new_list
			##last_time=rospy.Time.now().secs		## last ROS time
			print("Unallocated Tasks Deleted . . .")
			task_Q_changed=True	## task Q changed?
		elif method == "delete_allocated2AGV1":
			newTask1 = self.task1_parameter.get_value()			## task1 parameters from OPCUA server
			##self.task2_parameter.set_value([0,0,0,0,0,0])
			##task1_completed=False	## task assigned to AGV1 completed
			ready4task1=False			## AGV1 ready for new task
			print("newTask1[1]", newTask1[1] )
			if (newTask1[1]==0):
				pass
			else:
				## remove task allocated to AGV1 from task_Q
				[task_Q.remove(x) for x in task_Q if newTask1[1] in x]	## if time.secs matches
				self.task1_parameter.set_value([0,0,0,0,0,0])
				##last_time=rospy.Time.now().secs		## last ROS time
				print("Task Allocated to AGV1 Deleted . . .")
				task_Q_changed=True	## task Q changed?
		elif method == "delete_allocated2AGV2":
			newTask2 = self.task2_parameter.get_value()			## task1 parameters from OPCUA server			
			##task2_completed=False	## task assigned to AGV2 completed
			ready4task2=False			## AGV2 ready for new task
			print("newTask2[1]", newTask2[1] )
			if (newTask2[1]==0):
				pass
			else:
				## remove task allocated to AGV2 from task_Q
				[task_Q.remove(x) for x in task_Q if newTask2[1] in x]	## if time.secs matches
				self.task2_parameter.set_value([0,0,0,0,0,0])
				##last_time=rospy.Time.now().secs		## last ROS time
				print("Task Allocated to AGV2 Deleted . . .")
				task_Q_changed=True	## task Q changed?										
		elif method == "close":
			# Set close thread flag so that the started thread(s) threads can exit
			self.bCloseThreadFlag = True
			time.sleep(0.1)
			self.master.destroy()
 ## end update() 

##### ##### ##### ##### ##### ##### ##### #####  #####   ##### 
##### Input data validation for GUI textbox fireAlarmEntry  #####
##### Input field fireAlarmEntry accepts only 0 or 1 #####
##### ##### ##### ##### ##### ##### ##### #####  #####   ##### 
	def validateFA(self, new_value):
		if new_value == "":
			self.updatedFireAlarm = 0
			return True
		if str.isdigit(new_value):
			if int(new_value) == 0 or int(new_value) == 1:
				self.updatedFireAlarm = int(new_value)
				return True
			else:
				self.updatedFireAlarm = 0
				return False
		else:
			self.updatedFireAlarm = 0
			return False
 ## end validateFA() 

##### ##### ##### ##### ##### ##### ##### #####  #####   ##### 
##### Input data validation for GUI textbox liftStatusEntry  #####
##### Input field liftStatusEntry accepts only 0 or 1 #####
##### ##### ##### ##### ##### ##### ##### #####  #####   ##### 
	def validateLS(self, new_value):
		if new_value == "":
			self.updatedLiftStatus = 0
			return True
		if str.isdigit(new_value):
			if int(new_value)==0 or int(new_value)==1 or int(new_value)==2:
				self.updatedLiftStatus = int(new_value)
				return True
			else:
				self.updatedLiftStatus = 0
				return False
		else:
			self.updatedLiftStatus = 0
			return False
 ## end validateLS() 

##### ##### ##### ##### ##### ##### ##### #####  #####   ##### 
##### Input data validation for GUI textbox beaconAlarmEntry  #####
##### Input field beaconAlarmEntry accepts only 0 or 1 #####
##### ##### ##### ##### ##### ##### ##### #####  #####   ##### 	
	def validateBA(self, new_value):
		if new_value == "":
			self.updatedBeaconAlarm = 0
			return True
		if str.isdigit(new_value):
			if int(new_value) == 0 or int(new_value) == 1:
				self.updatedBeaconAlarm = int(new_value)
				return True
			else:
				self.updatedBeaconAlarm = 0
				return False
		else:
			self.updatedBeaconAlarm = 0
			return False
 ## end validateBA() 
 ## end class OPCServer
        
root = Tk()			## Initialize root GUI window
root.geometry("400x400")	## size of the window
my_opc_server = OPCServer(root)	## an instance of OPCServer
root.mainloop()			## GUI event loop
##rospy.spin()			## ROS messages event loop
