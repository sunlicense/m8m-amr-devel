; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude lumstatus.msg.html

(cl:defclass <lumstatus> (roslisp-msg-protocol:ros-message)
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
    :initform 0)
   (batlevel
    :reader batlevel
    :initarg :batlevel
    :type cl:float
    :initform 0.0)
   (laserstatus
    :reader laserstatus
    :initarg :laserstatus
    :type cl:string
    :initform "")
   (errorlog
    :reader errorlog
    :initarg :errorlog
    :type cl:string
    :initform ""))
)

(cl:defclass lumstatus (<lumstatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lumstatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lumstatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<lumstatus> is deprecated: use htbot-msg:lumstatus instead.")))

(cl:ensure-generic-function 'stn0-val :lambda-list '(m))
(cl:defmethod stn0-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn0-val is deprecated.  Use htbot-msg:stn0 instead.")
  (stn0 m))

(cl:ensure-generic-function 'stn1-val :lambda-list '(m))
(cl:defmethod stn1-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn1-val is deprecated.  Use htbot-msg:stn1 instead.")
  (stn1 m))

(cl:ensure-generic-function 'stn2-val :lambda-list '(m))
(cl:defmethod stn2-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn2-val is deprecated.  Use htbot-msg:stn2 instead.")
  (stn2 m))

(cl:ensure-generic-function 'stn3-val :lambda-list '(m))
(cl:defmethod stn3-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn3-val is deprecated.  Use htbot-msg:stn3 instead.")
  (stn3 m))

(cl:ensure-generic-function 'stn4-val :lambda-list '(m))
(cl:defmethod stn4-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn4-val is deprecated.  Use htbot-msg:stn4 instead.")
  (stn4 m))

(cl:ensure-generic-function 'stn5-val :lambda-list '(m))
(cl:defmethod stn5-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn5-val is deprecated.  Use htbot-msg:stn5 instead.")
  (stn5 m))

(cl:ensure-generic-function 'stn6-val :lambda-list '(m))
(cl:defmethod stn6-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn6-val is deprecated.  Use htbot-msg:stn6 instead.")
  (stn6 m))

(cl:ensure-generic-function 'stn7-val :lambda-list '(m))
(cl:defmethod stn7-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn7-val is deprecated.  Use htbot-msg:stn7 instead.")
  (stn7 m))

(cl:ensure-generic-function 'stn8-val :lambda-list '(m))
(cl:defmethod stn8-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn8-val is deprecated.  Use htbot-msg:stn8 instead.")
  (stn8 m))

(cl:ensure-generic-function 'stn9-val :lambda-list '(m))
(cl:defmethod stn9-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stn9-val is deprecated.  Use htbot-msg:stn9 instead.")
  (stn9 m))

(cl:ensure-generic-function 'batlevel-val :lambda-list '(m))
(cl:defmethod batlevel-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:batlevel-val is deprecated.  Use htbot-msg:batlevel instead.")
  (batlevel m))

(cl:ensure-generic-function 'laserstatus-val :lambda-list '(m))
(cl:defmethod laserstatus-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:laserstatus-val is deprecated.  Use htbot-msg:laserstatus instead.")
  (laserstatus m))

(cl:ensure-generic-function 'errorlog-val :lambda-list '(m))
(cl:defmethod errorlog-val ((m <lumstatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:errorlog-val is deprecated.  Use htbot-msg:errorlog instead.")
  (errorlog m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lumstatus>) ostream)
  "Serializes a message object of type '<lumstatus>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'batlevel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'laserstatus))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'laserstatus))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'errorlog))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'errorlog))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lumstatus>) istream)
  "Deserializes a message object of type '<lumstatus>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'batlevel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'laserstatus) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'laserstatus) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'errorlog) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'errorlog) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lumstatus>)))
  "Returns string type for a message object of type '<lumstatus>"
  "htbot/lumstatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lumstatus)))
  "Returns string type for a message object of type 'lumstatus"
  "htbot/lumstatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lumstatus>)))
  "Returns md5sum for a message object of type '<lumstatus>"
  "f8b3c1afcfd978c9fc81e921d6f4109c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lumstatus)))
  "Returns md5sum for a message object of type 'lumstatus"
  "f8b3c1afcfd978c9fc81e921d6f4109c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lumstatus>)))
  "Returns full string definition for message of type '<lumstatus>"
  (cl:format cl:nil "int8 stn0~%int8 stn1~%int8 stn2~%int8 stn3~%int8 stn4~%int8 stn5~%int8 stn6~%int8 stn7~%int8 stn8~%int8 stn9~%float32 batlevel~%string laserstatus~%string errorlog~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lumstatus)))
  "Returns full string definition for message of type 'lumstatus"
  (cl:format cl:nil "int8 stn0~%int8 stn1~%int8 stn2~%int8 stn3~%int8 stn4~%int8 stn5~%int8 stn6~%int8 stn7~%int8 stn8~%int8 stn9~%float32 batlevel~%string laserstatus~%string errorlog~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lumstatus>))
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
     4
     4 (cl:length (cl:slot-value msg 'laserstatus))
     4 (cl:length (cl:slot-value msg 'errorlog))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lumstatus>))
  "Converts a ROS message object to a list"
  (cl:list 'lumstatus
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
    (cl:cons ':batlevel (batlevel msg))
    (cl:cons ':laserstatus (laserstatus msg))
    (cl:cons ':errorlog (errorlog msg))
))
