; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


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
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<task> is deprecated: use htbot-msg:task instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:cmd-val is deprecated.  Use htbot-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:time-val is deprecated.  Use htbot-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:type-val is deprecated.  Use htbot-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'fromLP-val :lambda-list '(m))
(cl:defmethod fromLP-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:fromLP-val is deprecated.  Use htbot-msg:fromLP instead.")
  (fromLP m))

(cl:ensure-generic-function 'toLP-val :lambda-list '(m))
(cl:defmethod toLP-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:toLP-val is deprecated.  Use htbot-msg:toLP instead.")
  (toLP m))

(cl:ensure-generic-function 'alloc-val :lambda-list '(m))
(cl:defmethod alloc-val ((m <task>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:alloc-val is deprecated.  Use htbot-msg:alloc instead.")
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
  "htbot/task")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'task)))
  "Returns string type for a message object of type 'task"
  "htbot/task")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<task>)))
  "Returns md5sum for a message object of type '<task>"
  "d9335104b860e9530a386a51f33ebb59")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'task)))
  "Returns md5sum for a message object of type 'task"
  "d9335104b860e9530a386a51f33ebb59")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<task>)))
  "Returns full string definition for message of type '<task>"
  (cl:format cl:nil "int8 cmd~%time time~%int8 type~%int8 fromLP~%int8 toLP~%int8 alloc~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'task)))
  "Returns full string definition for message of type 'task"
  (cl:format cl:nil "int8 cmd~%time time~%int8 type~%int8 fromLP~%int8 toLP~%int8 alloc~%~%~%"))
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
