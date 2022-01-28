; Auto-generated. Do not edit!


(cl:in-package ntuc_fleet-msg)


;//! \htmlinclude task.msg.html

(cl:defclass <task> (roslisp-msg-protocol:ros-message)
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
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (fromLP
    :reader fromLP
    :initarg :fromLP
    :type cl:fixnum
    :initform 0)
   (toLP
    :reader toLP
    :initarg :toLP
    :type cl:fixnum
    :initform 0)
   (alloc
    :reader alloc
    :initarg :alloc
    :type cl:fixnum
    :initform 0))
)

(cl:defclass task (<task>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <task>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'task)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ntuc_fleet-msg:<task> is deprecated: use ntuc_fleet-msg:task instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc_fleet-msg:cmd-val is deprecated.  Use ntuc_fleet-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc_fleet-msg:time-val is deprecated.  Use ntuc_fleet-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc_fleet-msg:type-val is deprecated.  Use ntuc_fleet-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'fromLP-val :lambda-list '(m))
(cl:defmethod fromLP-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc_fleet-msg:fromLP-val is deprecated.  Use ntuc_fleet-msg:fromLP instead.")
  (fromLP m))

(cl:ensure-generic-function 'toLP-val :lambda-list '(m))
(cl:defmethod toLP-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc_fleet-msg:toLP-val is deprecated.  Use ntuc_fleet-msg:toLP instead.")
  (toLP m))

(cl:ensure-generic-function 'alloc-val :lambda-list '(m))
(cl:defmethod alloc-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc_fleet-msg:alloc-val is deprecated.  Use ntuc_fleet-msg:alloc instead.")
  (alloc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <task>) ostream)
  "Serializes a message object of type '<task>"
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
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'fromLP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'toLP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'alloc)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <task>) istream)
  "Deserializes a message object of type '<task>"
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fromLP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'toLP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'alloc) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<task>)))
  "Returns string type for a message object of type '<task>"
  "ntuc_fleet/task")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'task)))
  "Returns string type for a message object of type 'task"
  "ntuc_fleet/task")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<task>)))
  "Returns md5sum for a message object of type '<task>"
  "d9335104b860e9530a386a51f33ebb59")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'task)))
  "Returns md5sum for a message object of type 'task"
  "d9335104b860e9530a386a51f33ebb59")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<task>)))
  "Returns full string definition for message of type '<task>"
  (cl:format cl:nil "##################################################~%##### Message type: task.msg~%##### Use in Topics: /fr_station, /fr_fleet, /fr_agv~%##################################################~%int8 cmd		# command code ~%time time		# time task is queued~%int8 type		# different trolley type~%int8 fromLP		# starting LP~%int8 toLP		# ending LP~%int8 alloc		# 0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2~%~%##################################################~%## cmd		1:new task, 2:assigned task, 3:cancel task~%## time		time.secs and time.nsecs (ROS time)~%## type		1:Bian Marie, 2:Food Warmer, 3:Linen~%## fromLP	70:L1LRm3-9, 73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,~%##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward9, ~%##				10:L10StaffDorm, 11:L11Ktchen,~%##				51:L1MainStore, 52:L1NonHalalPrepRoom~%## toLP		73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,~%##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward,~%##				10: L10StaffDorm, 11:L11Ktchen,~%##				51:L1MainStore, 52:L1NonHalalPrepRoom~%## alloc	0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2~%##################################################	~%## LP Range:: 	2-49: Lift Area, 50-69: Lobby Area, 70-100: Laundry Area ~%##################################################	~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'task)))
  "Returns full string definition for message of type 'task"
  (cl:format cl:nil "##################################################~%##### Message type: task.msg~%##### Use in Topics: /fr_station, /fr_fleet, /fr_agv~%##################################################~%int8 cmd		# command code ~%time time		# time task is queued~%int8 type		# different trolley type~%int8 fromLP		# starting LP~%int8 toLP		# ending LP~%int8 alloc		# 0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2~%~%##################################################~%## cmd		1:new task, 2:assigned task, 3:cancel task~%## time		time.secs and time.nsecs (ROS time)~%## type		1:Bian Marie, 2:Food Warmer, 3:Linen~%## fromLP	70:L1LRm3-9, 73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,~%##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward9, ~%##				10:L10StaffDorm, 11:L11Ktchen,~%##				51:L1MainStore, 52:L1NonHalalPrepRoom~%## toLP		73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,~%##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward,~%##				10: L10StaffDorm, 11:L11Ktchen,~%##				51:L1MainStore, 52:L1NonHalalPrepRoom~%## alloc	0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2~%##################################################	~%## LP Range:: 	2-49: Lift Area, 50-69: Lobby Area, 70-100: Laundry Area ~%##################################################	~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <task>))
  (cl:+ 0
     1
     8
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <task>))
  "Converts a ROS message object to a list"
  (cl:list 'task
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':time (time msg))
    (cl:cons ':type (type msg))
    (cl:cons ':fromLP (fromLP msg))
    (cl:cons ':toLP (toLP msg))
    (cl:cons ':alloc (alloc msg))
))
