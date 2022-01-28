##################################################
##### Message type: status.msg
##### Use in Topics: /to_fleet, /to_station
##################################################
int8 cmd			# command code	 
time time		# fleet manager current time
int8 status		# AGV status
int8 location		# AGV location
time arrival		# estimate time of arrival
int8 b_level		# battery level in %
int8 e_status		# elevator status 
int8 complete		# task completion / ready for next task
int8 lastLP			# LP when the last task has just completed

##################################################
## cmd		1:op_status, 2:alert, 3:alarm, 4:agv_status
## time		time in time.secs and and time.nsecs 
## status	1:delivering food, 2:traffic jam, 3:breakdown, 4:fire alarm, 5:charging
## location	0:L1Store,1:L1Laundry,2:L1NonHalal,3:L3,4:L4,5:L5,6:L6,
##				7:L7,8:L8,9:L9,10:L10Dorm,11:L11,12:L1Charging
## arrival	time in time.secs and and time.nsecs
## b_level	percent of battery level
## lift_status	1:okay, 2:service alarm, 3:fire alarm 
## complete	0:ready for task, 1:completed & ready, 2:completed not ready, 3:not ready, 
##				4:job-aborted not ready, 5:job-aborted ready
## lastLP	73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,
##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward,
##				10: L10StaffDorm, 11:L11Ktchen,
##				51:L1MainStore, 52:L1NonHalalPrepRoom
##################################################	
