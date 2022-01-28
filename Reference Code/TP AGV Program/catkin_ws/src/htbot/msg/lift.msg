##################################################
##### Message type: lift.msg
##### Use in Topics: /to_lift, /fr_lift
##################################################
int8 cmd			# command code			[0-102]
int8 cfloor			# start of trip [1-11] or current car level [1-11]
int8 dfloor			# end level of trip	[1-11]
int8 inuse			# request for in_use activation [0/1] or in_use status [0/1]
int8 dooropen		# door open status 	[0/1]
int8 doorclose		# door close status 	[0/1]
int8 goingup		# going up status 	[0/1]
int8 goingdown		# going down status 	[0/1]
int8 stationary		# stationary status 	[0/1]
int8 serviceavail	# service available status	[0/1]
int8 epower			# emergency power status 	[0/1]
int8 fireservice	# fire service status 		[0/1]
int8 beacon			# beacon light alarm 		[0/1], 1 to trigger, 0 to turn off

##################################################
##### cmd value usage:
##### 0:carCallStartLevel, 1:enteredLift, 2:carCallEndLevel, 3:exitedLift
##### 8:beacon, 9:inUse, 10:okay2Enter, 11:okay2Exit
##### 20:inUse, 21:doorOpen, 22:doorClose, 23:goingUp, 24:goingDown, 25:stationary 
##### 40:carLevel, 100:serviceAvailable, 101: emergencyPower, 102:fireServiceAlarm
##################################################
