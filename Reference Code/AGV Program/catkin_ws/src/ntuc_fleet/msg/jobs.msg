##################################################
##### Message type: jobs.msg
##### Use in Topics: /rq_jobs, /rv_jobs
##################################################
int8 cmd				# command code	 
time time			# time queue is updated
task[] tasks		# list of task.msg (of type task.msg)

##################################################
## cmd		command code
## time		time in time.secs and and time.nsecs 
## task[]	list of pending tasks (of type task.msg)
##################################################
