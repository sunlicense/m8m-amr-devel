; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude goal.msg.html

(cl:defclass <goal> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0)
   (startidx
    :reader startidx
    :initarg :startidx
    :type cl:fixnum
    :initform 0)
   (lastidx
    :reader lastidx
    :initarg :lastidx
    :type cl:fixnum
    :initform 0)
   (numgoal
    :reader numgoal
    :initarg :numgoal
    :type cl:fixnum
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (rx
    :reader rx
    :initarg :rx
    :type cl:float
    :initform 0.0)
   (ry
    :reader ry
    :initarg :ry
    :type cl:float
    :initform 0.0)
   (rz
    :reader rz
    :initarg :rz
    :type cl:float
    :initform 0.0)
   (rw
    :reader rw
    :initarg :rw
    :type cl:float
    :initform 0.0)
   (pd
    :reader pd
    :initarg :pd
    :type cl:float
    :initform 0.0)
   (pa
    :reader pa
    :initarg :pa
    :type cl:float
    :initform 0.0)
   (opt
    :reader opt
    :initarg :opt
    :type cl:fixnum
    :initform 0))
)

(cl:defclass goal (<goal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <goal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'goal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<goal> is deprecated: use htbot-msg:goal instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:cmd-val is deprecated.  Use htbot-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'startidx-val :lambda-list '(m))
(cl:defmethod startidx-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:startidx-val is deprecated.  Use htbot-msg:startidx instead.")
  (startidx m))

(cl:ensure-generic-function 'lastidx-val :lambda-list '(m))
(cl:defmethod lastidx-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:lastidx-val is deprecated.  Use htbot-msg:lastidx instead.")
  (lastidx m))

(cl:ensure-generic-function 'numgoal-val :lambda-list '(m))
(cl:defmethod numgoal-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:numgoal-val is deprecated.  Use htbot-msg:numgoal instead.")
  (numgoal m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:x-val is deprecated.  Use htbot-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:y-val is deprecated.  Use htbot-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:z-val is deprecated.  Use htbot-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'rx-val :lambda-list '(m))
(cl:defmethod rx-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:rx-val is deprecated.  Use htbot-msg:rx instead.")
  (rx m))

(cl:ensure-generic-function 'ry-val :lambda-list '(m))
(cl:defmethod ry-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:ry-val is deprecated.  Use htbot-msg:ry instead.")
  (ry m))

(cl:ensure-generic-function 'rz-val :lambda-list '(m))
(cl:defmethod rz-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:rz-val is deprecated.  Use htbot-msg:rz instead.")
  (rz m))

(cl:ensure-generic-function 'rw-val :lambda-list '(m))
(cl:defmethod rw-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:rw-val is deprecated.  Use htbot-msg:rw instead.")
  (rw m))

(cl:ensure-generic-function 'pd-val :lambda-list '(m))
(cl:defmethod pd-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:pd-val is deprecated.  Use htbot-msg:pd instead.")
  (pd m))

(cl:ensure-generic-function 'pa-val :lambda-list '(m))
(cl:defmethod pa-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:pa-val is deprecated.  Use htbot-msg:pa instead.")
  (pa m))

(cl:ensure-generic-function 'opt-val :lambda-list '(m))
(cl:defmethod opt-val ((m <goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:opt-val is deprecated.  Use htbot-msg:opt instead.")
  (opt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <goal>) ostream)
  "Serializes a message object of type '<goal>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'startidx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'lastidx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'numgoal)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ry))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pa))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'opt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <goal>) istream)
  "Deserializes a message object of type '<goal>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'startidx) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lastidx) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'numgoal) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ry) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pa) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opt) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<goal>)))
  "Returns string type for a message object of type '<goal>"
  "htbot/goal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'goal)))
  "Returns string type for a message object of type 'goal"
  "htbot/goal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<goal>)))
  "Returns md5sum for a message object of type '<goal>"
  "3d9db86a8255735881de2fc33b96e154")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'goal)))
  "Returns md5sum for a message object of type 'goal"
  "3d9db86a8255735881de2fc33b96e154")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<goal>)))
  "Returns full string definition for message of type '<goal>"
  (cl:format cl:nil "int8 cmd~%int16 startidx~%int16 lastidx~%int16 numgoal~%float32 x~%float32 y~%float32 z~%float32 rx~%float32 ry~%float32 rz~%float32 rw~%float32 pd~%float32 pa~%int8 opt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'goal)))
  "Returns full string definition for message of type 'goal"
  (cl:format cl:nil "int8 cmd~%int16 startidx~%int16 lastidx~%int16 numgoal~%float32 x~%float32 y~%float32 z~%float32 rx~%float32 ry~%float32 rz~%float32 rw~%float32 pd~%float32 pa~%int8 opt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <goal>))
  (cl:+ 0
     1
     2
     2
     2
     4
     4
     4
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <goal>))
  "Converts a ROS message object to a list"
  (cl:list 'goal
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':startidx (startidx msg))
    (cl:cons ':lastidx (lastidx msg))
    (cl:cons ':numgoal (numgoal msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':rx (rx msg))
    (cl:cons ':ry (ry msg))
    (cl:cons ':rz (rz msg))
    (cl:cons ':rw (rw msg))
    (cl:cons ':pd (pd msg))
    (cl:cons ':pa (pa msg))
    (cl:cons ':opt (opt msg))
))
