; Auto-generated. Do not edit!


(cl:in-package ntuc-msg)


;//! \htmlinclude lift.msg.html

(cl:defclass <lift> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0)
   (cfloor
    :reader cfloor
    :initarg :cfloor
    :type cl:fixnum
    :initform 0)
   (dfloor
    :reader dfloor
    :initarg :dfloor
    :type cl:fixnum
    :initform 0)
   (inuse
    :reader inuse
    :initarg :inuse
    :type cl:fixnum
    :initform 0)
   (dooropen
    :reader dooropen
    :initarg :dooropen
    :type cl:fixnum
    :initform 0)
   (doorclose
    :reader doorclose
    :initarg :doorclose
    :type cl:fixnum
    :initform 0)
   (goingup
    :reader goingup
    :initarg :goingup
    :type cl:fixnum
    :initform 0)
   (goingdown
    :reader goingdown
    :initarg :goingdown
    :type cl:fixnum
    :initform 0)
   (stationary
    :reader stationary
    :initarg :stationary
    :type cl:fixnum
    :initform 0)
   (serviceavail
    :reader serviceavail
    :initarg :serviceavail
    :type cl:fixnum
    :initform 0)
   (epower
    :reader epower
    :initarg :epower
    :type cl:fixnum
    :initform 0)
   (fireservice
    :reader fireservice
    :initarg :fireservice
    :type cl:fixnum
    :initform 0)
   (beacon
    :reader beacon
    :initarg :beacon
    :type cl:fixnum
    :initform 0))
)

(cl:defclass lift (<lift>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lift>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lift)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ntuc-msg:<lift> is deprecated: use ntuc-msg:lift instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:cmd-val is deprecated.  Use ntuc-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'cfloor-val :lambda-list '(m))
(cl:defmethod cfloor-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:cfloor-val is deprecated.  Use ntuc-msg:cfloor instead.")
  (cfloor m))

(cl:ensure-generic-function 'dfloor-val :lambda-list '(m))
(cl:defmethod dfloor-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:dfloor-val is deprecated.  Use ntuc-msg:dfloor instead.")
  (dfloor m))

(cl:ensure-generic-function 'inuse-val :lambda-list '(m))
(cl:defmethod inuse-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:inuse-val is deprecated.  Use ntuc-msg:inuse instead.")
  (inuse m))

(cl:ensure-generic-function 'dooropen-val :lambda-list '(m))
(cl:defmethod dooropen-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:dooropen-val is deprecated.  Use ntuc-msg:dooropen instead.")
  (dooropen m))

(cl:ensure-generic-function 'doorclose-val :lambda-list '(m))
(cl:defmethod doorclose-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:doorclose-val is deprecated.  Use ntuc-msg:doorclose instead.")
  (doorclose m))

(cl:ensure-generic-function 'goingup-val :lambda-list '(m))
(cl:defmethod goingup-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:goingup-val is deprecated.  Use ntuc-msg:goingup instead.")
  (goingup m))

(cl:ensure-generic-function 'goingdown-val :lambda-list '(m))
(cl:defmethod goingdown-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:goingdown-val is deprecated.  Use ntuc-msg:goingdown instead.")
  (goingdown m))

(cl:ensure-generic-function 'stationary-val :lambda-list '(m))
(cl:defmethod stationary-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:stationary-val is deprecated.  Use ntuc-msg:stationary instead.")
  (stationary m))

(cl:ensure-generic-function 'serviceavail-val :lambda-list '(m))
(cl:defmethod serviceavail-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:serviceavail-val is deprecated.  Use ntuc-msg:serviceavail instead.")
  (serviceavail m))

(cl:ensure-generic-function 'epower-val :lambda-list '(m))
(cl:defmethod epower-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:epower-val is deprecated.  Use ntuc-msg:epower instead.")
  (epower m))

(cl:ensure-generic-function 'fireservice-val :lambda-list '(m))
(cl:defmethod fireservice-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:fireservice-val is deprecated.  Use ntuc-msg:fireservice instead.")
  (fireservice m))

(cl:ensure-generic-function 'beacon-val :lambda-list '(m))
(cl:defmethod beacon-val ((m <lift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ntuc-msg:beacon-val is deprecated.  Use ntuc-msg:beacon instead.")
  (beacon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lift>) ostream)
  "Serializes a message object of type '<lift>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cfloor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dfloor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'inuse)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dooropen)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'doorclose)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'goingup)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'goingdown)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stationary)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'serviceavail)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'epower)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'fireservice)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'beacon)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lift>) istream)
  "Deserializes a message object of type '<lift>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cfloor) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dfloor) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'inuse) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dooropen) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'doorclose) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'goingup) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'goingdown) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stationary) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'serviceavail) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'epower) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fireservice) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'beacon) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lift>)))
  "Returns string type for a message object of type '<lift>"
  "ntuc/lift")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lift)))
  "Returns string type for a message object of type 'lift"
  "ntuc/lift")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lift>)))
  "Returns md5sum for a message object of type '<lift>"
  "e5880338d440c98c58ce479e42b36bed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lift)))
  "Returns md5sum for a message object of type 'lift"
  "e5880338d440c98c58ce479e42b36bed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lift>)))
  "Returns full string definition for message of type '<lift>"
  (cl:format cl:nil "##################################################~%##### Message type: lift.msg~%##### Use in Topics: /to_lift, /fr_lift~%##################################################~%int8 cmd			# command code			[0-102]~%int8 cfloor			# start of trip [1-11] or current car level [1-11]~%int8 dfloor			# end level of trip	[1-11]~%int8 inuse			# request for in_use activation [0/1] or in_use status [0/1]~%int8 dooropen		# door open status 	[0/1]~%int8 doorclose		# door close status 	[0/1]~%int8 goingup		# going up status 	[0/1]~%int8 goingdown		# going down status 	[0/1]~%int8 stationary		# stationary status 	[0/1]~%int8 serviceavail	# service available status	[0/1]~%int8 epower			# emergency power status 	[0/1]~%int8 fireservice	# fire service status 		[0/1]~%int8 beacon			# beacon light alarm 		[0/1], 1 to trigger, 0 to turn off~%~%##################################################~%##### cmd value usage:~%##### 0:carCallStartLevel, 1:enteredLift, 2:carCallEndLevel, 3:exitedLift~%##### 8:beacon, 9:inUse, 10:okay2Enter, 11:okay2Exit~%##### 20:inUse, 21:doorOpen, 22:doorClose, 23:goingUp, 24:goingDown, 25:stationary ~%##### 40:carLevel, 100:serviceAvailable, 101: emergencyPower, 102:fireServiceAlarm~%##################################################~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lift)))
  "Returns full string definition for message of type 'lift"
  (cl:format cl:nil "##################################################~%##### Message type: lift.msg~%##### Use in Topics: /to_lift, /fr_lift~%##################################################~%int8 cmd			# command code			[0-102]~%int8 cfloor			# start of trip [1-11] or current car level [1-11]~%int8 dfloor			# end level of trip	[1-11]~%int8 inuse			# request for in_use activation [0/1] or in_use status [0/1]~%int8 dooropen		# door open status 	[0/1]~%int8 doorclose		# door close status 	[0/1]~%int8 goingup		# going up status 	[0/1]~%int8 goingdown		# going down status 	[0/1]~%int8 stationary		# stationary status 	[0/1]~%int8 serviceavail	# service available status	[0/1]~%int8 epower			# emergency power status 	[0/1]~%int8 fireservice	# fire service status 		[0/1]~%int8 beacon			# beacon light alarm 		[0/1], 1 to trigger, 0 to turn off~%~%##################################################~%##### cmd value usage:~%##### 0:carCallStartLevel, 1:enteredLift, 2:carCallEndLevel, 3:exitedLift~%##### 8:beacon, 9:inUse, 10:okay2Enter, 11:okay2Exit~%##### 20:inUse, 21:doorOpen, 22:doorClose, 23:goingUp, 24:goingDown, 25:stationary ~%##### 40:carLevel, 100:serviceAvailable, 101: emergencyPower, 102:fireServiceAlarm~%##################################################~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lift>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lift>))
  "Converts a ROS message object to a list"
  (cl:list 'lift
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':cfloor (cfloor msg))
    (cl:cons ':dfloor (dfloor msg))
    (cl:cons ':inuse (inuse msg))
    (cl:cons ':dooropen (dooropen msg))
    (cl:cons ':doorclose (doorclose msg))
    (cl:cons ':goingup (goingup msg))
    (cl:cons ':goingdown (goingdown msg))
    (cl:cons ':stationary (stationary msg))
    (cl:cons ':serviceavail (serviceavail msg))
    (cl:cons ':epower (epower msg))
    (cl:cons ':fireservice (fireservice msg))
    (cl:cons ':beacon (beacon msg))
))
