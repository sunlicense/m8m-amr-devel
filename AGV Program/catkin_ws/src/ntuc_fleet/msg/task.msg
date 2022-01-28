##################################################
##### Message type: task.msg
##### Use in Topics: /fr_station, /fr_fleet, /fr_agv
##################################################
int8 cmd		# command code 
time time		# time task is queued
int8 type		# different trolley type
int8 fromLP		# starting LP
int8 toLP		# ending LP
int8 alloc		# 0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2

##################################################
## cmd		1:new task, 2:assigned task, 3:cancel task
## time		time.secs and time.nsecs (ROS time)
## type		1:Bian Marie, 2:Food Warmer, 3:Linen
## fromLP	70:L1LRm3-9, 73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,
##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward9, 
##				10:L10StaffDorm, 11:L11Ktchen,
##				51:L1MainStore, 52:L1NonHalalPrepRoom
## toLP		73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,
##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward,
##				10: L10StaffDorm, 11:L11Ktchen,
##				51:L1MainStore, 52:L1NonHalalPrepRoom
## alloc	0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2
##################################################	
## LP Range:: 	2-49: Lift Area, 50-69: Lobby Area, 70-100: Laundry Area 
##################################################	
