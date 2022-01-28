; Auto-generated. Do not edit!


(cl:in-package ntuc_fleet-msg)


;//! \htmlinclude jobs.msg.html

(cl:defclass <jobs> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:real
    :initform 0)
   (tasks
    :reader tasks
    :initarg :tasks
    :type (cl:vector ntuc_fleet-msg:task)
   :initform (cl:make-array 0 :element-type 'ntuc_fleet-msg:task :initial-element (cl:make-instance 'ntuc_fleet-msg:task))))
)

(cl:defclass jobs (<jobs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <jobs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'jobs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ntuc_fleet-msg:<jobs> is deprecated: use ntuc_fleet-msg:jobs instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <jobs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc_fleet-msg:cmd-val is deprecated.  Use ntuc_fleet-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <jobs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc_fleet-msg:time-val is deprecated.  Use ntuc_fleet-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'tasks-val :lambda-list '(m))
(cl:defmethod tasks-val ((m <jobs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc_fleet-msg:tasks-val is deprecated.  Use ntuc_fleet-msg:tasks instead.")
  (tasks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <jobs>) ostream)
  "Serializes a message object of type '<jobs>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time) (cl:floor (cl:slot-value msg 'time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tasks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tasks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <jobs>) istream)
  "Deserializes a message object of type '<jobs>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tasks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tasks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ntuc_fleet-msg:task))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<jobs>)))
  "Returns string type for a message object of type '<jobs>"
  "ntuc_fleet/jobs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'jobs)))
  "Returns string type for a message object of type 'jobs"
  "ntuc_fleet/jobs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<jobs>)))
  "Returns md5sum for a message object of type '<jobs>"
  "d7b1ebb1ce6d0c3bb74e63fcb9b9e5cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'jobs)))
  "Returns md5sum for a message object of type 'jobs"
  "d7b1ebb1ce6d0c3bb74e63fcb9b9e5cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<jobs>)))
  "Returns full string definition for message of type '<jobs>"
  (cl:format cl:nil "##################################################~%##### Message type: jobs.msg~%##### Use in Topics: /rq_jobs, /rv_jobs~%##################################################~%int8 cmd				# command code	 ~%time time			# time queue is updated~%task[] tasks		# list of task.msg (of type task.msg)~%~%##################################################~%## cmd		command code~%## time		time in time.secs and and time.nsecs ~%## task[]	list of pending tasks (of type task.msg)~%##################################################~%~%================================================================================~%MSG: ntuc_fleet/task~%##################################################~%##### Message type: task.msg~%##### Use in Topics: /fr_station, /fr_fleet, /fr_agv~%##################################################~%int8 cmd		# command code ~%time time		# time task is queued~%int8 type		# different trolley type~%int8 fromLP		# starting LP~%int8 toLP		# ending LP~%int8 alloc		# 0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2~%~%##################################################~%## cmd		1:new task, 2:assigned task, 3:cancel task~%## time		time.secs and time.nsecs (ROS time)~%## type		1:Bian Marie, 2:Food Warmer, 3:Linen~%## fromLP	70:L1LRm3-9, 73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,~%##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward9, ~%##				10:L10StaffDorm, 11:L11Ktchen,~%##				51:L1MainStore, 52:L1NonHalalPrepRoom~%## toLP		73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,~%##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward,~%##				10: L10StaffDorm, 11:L11Ktchen,~%##				51:L1MainStore, 52:L1NonHalalPrepRoom~%## alloc	0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2~%##################################################	~%## LP Range:: 	2-49: Lift Area, 50-69: Lobby Area, 70-100: Laundry Area ~%##################################################	~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'jobs)))
  "Returns full string definition for message of type 'jobs"
  (cl:format cl:nil "##################################################~%##### Message type: jobs.msg~%##### Use in Topics: /rq_jobs, /rv_jobs~%##################################################~%int8 cmd				# command code	 ~%time time			# time queue is updated~%task[] tasks		# list of task.msg (of type task.msg)~%~%##################################################~%## cmd		command code~%## time		time in time.secs and and time.nsecs ~%## task[]	list of pending tasks (of type task.msg)~%##################################################~%~%================================================================================~%MSG: ntuc_fleet/task~%##################################################~%##### Message type: task.msg~%##### Use in Topics: /fr_station, /fr_fleet, /fr_agv~%##################################################~%int8 cmd		# command code ~%time time		# time task is queued~%int8 type		# different trolley type~%int8 fromLP		# starting LP~%int8 toLP		# ending LP~%int8 alloc		# 0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2~%~%##################################################~%## cmd		1:new task, 2:assigned task, 3:cancel task~%## time		time.secs and time.nsecs (ROS time)~%## type		1:Bian Marie, 2:Food Warmer, 3:Linen~%## fromLP	70:L1LRm3-9, 73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,~%##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward9, ~%##				10:L10StaffDorm, 11:L11Ktchen,~%##				51:L1MainStore, 52:L1NonHalalPrepRoom~%## toLP		73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,~%##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward,~%##				10: L10StaffDorm, 11:L11Ktchen,~%##				51:L1MainStore, 52:L1NonHalalPrepRoom~%## alloc	0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2~%##################################################	~%## LP Range:: 	2-49: Lift Area, 50-69: Lobby Area, 70-100: Laundry Area ~%##################################################	~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <jobs>))
  (cl:+ 0
     1
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tasks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <jobs>))
  "Converts a ROS message object to a list"
  (cl:list 'jobs
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':time (time msg))
    (cl:cons ':tasks (tasks msg))
))
