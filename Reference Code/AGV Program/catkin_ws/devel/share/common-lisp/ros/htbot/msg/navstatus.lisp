; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude navstatus.msg.html

(cl:defclass <navstatus> (roslisp-msg-protocol:ros-message)
  ((stn0
    :reader stn0
    :initarg :stn0
    :type cl:fixnum
    :initform 0)
   (stn1
    :reader stn1
    :initarg :stn1
    :type cl:fixnum
    :initform 0)
   (stn2
    :reader stn2
    :initarg :stn2
    :type cl:fixnum
    :initform 0)
   (stn3
    :reader stn3
    :initarg :stn3
    :type cl:fixnum
    :initform 0)
   (stn4
    :reader stn4
    :initarg :stn4
    :type cl:fixnum
    :initform 0)
   (stn5
    :reader stn5
    :initarg :stn5
    :type cl:fixnum
    :initform 0)
   (stn6
    :reader stn6
    :initarg :stn6
    :type cl:fixnum
    :initform 0)
   (stn7
    :reader stn7
    :initarg :stn7
    :type cl:fixnum
    :initform 0)
   (stn8
    :reader stn8
    :initarg :stn8
    :type cl:fixnum
    :initform 0)
   (stn9
    :reader stn9
    :initarg :stn9
    :type cl:fixnum
    :initform 0))
)

(cl:defclass navstatus (<navstatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <navstatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'navstatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<navstatus> is deprecated: use htbot-msg:navstatus instead.")))

(cl:ensure-generic-function 'stn0-val :lambda-list '(m))
(cl:defmethod stn0-val ((m <navstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn0-val is deprecated.  Use htbot-msg:stn0 instead.")
  (stn0 m))

(cl:ensure-generic-function 'stn1-val :lambda-list '(m))
(cl:defmethod stn1-val ((m <navstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn1-val is deprecated.  Use htbot-msg:stn1 instead.")
  (stn1 m))

(cl:ensure-generic-function 'stn2-val :lambda-list '(m))
(cl:defmethod stn2-val ((m <navstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn2-val is deprecated.  Use htbot-msg:stn2 instead.")
  (stn2 m))

(cl:ensure-generic-function 'stn3-val :lambda-list '(m))
(cl:defmethod stn3-val ((m <navstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn3-val is deprecated.  Use htbot-msg:stn3 instead.")
  (stn3 m))

(cl:ensure-generic-function 'stn4-val :lambda-list '(m))
(cl:defmethod stn4-val ((m <navstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn4-val is deprecated.  Use htbot-msg:stn4 instead.")
  (stn4 m))

(cl:ensure-generic-function 'stn5-val :lambda-list '(m))
(cl:defmethod stn5-val ((m <navstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn5-val is deprecated.  Use htbot-msg:stn5 instead.")
  (stn5 m))

(cl:ensure-generic-function 'stn6-val :lambda-list '(m))
(cl:defmethod stn6-val ((m <navstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn6-val is deprecated.  Use htbot-msg:stn6 instead.")
  (stn6 m))

(cl:ensure-generic-function 'stn7-val :lambda-list '(m))
(cl:defmethod stn7-val ((m <navstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn7-val is deprecated.  Use htbot-msg:stn7 instead.")
  (stn7 m))

(cl:ensure-generic-function 'stn8-val :lambda-list '(m))
(cl:defmethod stn8-val ((m <navstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn8-val is deprecated.  Use htbot-msg:stn8 instead.")
  (stn8 m))

(cl:ensure-generic-function 'stn9-val :lambda-list '(m))
(cl:defmethod stn9-val ((m <navstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn9-val is deprecated.  Use htbot-msg:stn9 instead.")
  (stn9 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <navstatus>) ostream)
  "Serializes a message object of type '<navstatus>"
  (cl:let* ((signed (cl:slot-value msg 'stn0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stn1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stn2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stn3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stn4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stn5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stn6)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stn7)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stn8)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stn9)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <navstatus>) istream)
  "Deserializes a message object of type '<navstatus>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stn0) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stn1) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stn2) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stn3) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stn4) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stn5) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stn6) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stn7) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stn8) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stn9) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<navstatus>)))
  "Returns string type for a message object of type '<navstatus>"
  "htbot/navstatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'navstatus)))
  "Returns string type for a message object of type 'navstatus"
  "htbot/navstatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<navstatus>)))
  "Returns md5sum for a message object of type '<navstatus>"
  "cc229716d82952abdbbbe7b0e8dff271")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'navstatus)))
  "Returns md5sum for a message object of type 'navstatus"
  "cc229716d82952abdbbbe7b0e8dff271")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<navstatus>)))
  "Returns full string definition for message of type '<navstatus>"
  (cl:format cl:nil "int8 stn0~%int8 stn1~%int8 stn2~%int8 stn3~%int8 stn4~%int8 stn5~%int8 stn6~%int8 stn7~%int8 stn8~%int8 stn9~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'navstatus)))
  "Returns full string definition for message of type 'navstatus"
  (cl:format cl:nil "int8 stn0~%int8 stn1~%int8 stn2~%int8 stn3~%int8 stn4~%int8 stn5~%int8 stn6~%int8 stn7~%int8 stn8~%int8 stn9~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <navstatus>))
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
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <navstatus>))
  "Converts a ROS message object to a list"
  (cl:list 'navstatus
    (cl:cons ':stn0 (stn0 msg))
    (cl:cons ':stn1 (stn1 msg))
    (cl:cons ':stn2 (stn2 msg))
    (cl:cons ':stn3 (stn3 msg))
    (cl:cons ':stn4 (stn4 msg))
    (cl:cons ':stn5 (stn5 msg))
    (cl:cons ':stn6 (stn6 msg))
    (cl:cons ':stn7 (stn7 msg))
    (cl:cons ':stn8 (stn8 msg))
    (cl:cons ':stn9 (stn9 msg))
))
