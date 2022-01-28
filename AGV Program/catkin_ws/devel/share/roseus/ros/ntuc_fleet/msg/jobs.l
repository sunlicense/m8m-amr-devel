;; Auto-generated. Do not edit!


(when (boundp 'ntuc_fleet::jobs)
  (if (not (find-package "NTUC_FLEET"))
    (make-package "NTUC_FLEET"))
  (shadow 'jobs (find-package "NTUC_FLEET")))
(unless (find-package "NTUC_FLEET::JOBS")
  (make-package "NTUC_FLEET::JOBS"))

(in-package "ROS")
;;//! \htmlinclude jobs.msg.html


(defclass ntuc_fleet::jobs
  :super ros::object
  :slots (_cmd _time _tasks ))

(defmethod ntuc_fleet::jobs
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:time __time) (instance ros::time :init))
    ((:tasks __tasks) (let (r) (dotimes (i 0) (push (instance ntuc_fleet::task :init) r)) r))
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _time __time)
   (setq _tasks __tasks)
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:tasks
   (&rest __tasks)
   (if (keywordp (car __tasks))
       (send* _tasks __tasks)
     (progn
       (if __tasks (setq _tasks (car __tasks)))
       _tasks)))
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ;; time _time
    8
    ;; ntuc_fleet/task[] _tasks
    (apply #'+ (send-all _tasks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _cmd
       (write-byte _cmd s)
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; ntuc_fleet/task[] _tasks
     (write-long (length _tasks) s)
     (dolist (elem _tasks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ntuc_fleet/task[] _tasks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tasks (let (r) (dotimes (i n) (push (instance ntuc_fleet::task :init) r)) r))
     (dolist (elem- _tasks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ntuc_fleet::jobs :md5sum-) "d7b1ebb1ce6d0c3bb74e63fcb9b9e5cb")
(setf (get ntuc_fleet::jobs :datatype-) "ntuc_fleet/jobs")
(setf (get ntuc_fleet::jobs :definition-)
      "##################################################
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

================================================================================
MSG: ntuc_fleet/task
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

")



(provide :ntuc_fleet/jobs "d7b1ebb1ce6d0c3bb74e63fcb9b9e5cb")


