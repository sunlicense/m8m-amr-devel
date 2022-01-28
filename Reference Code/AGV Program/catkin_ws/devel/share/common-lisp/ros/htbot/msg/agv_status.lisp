; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude agv_status.msg.html

(cl:defclass <agv_status> (roslisp-msg-protocol:ros-message)
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
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (location
    :reader location
    :initarg :location
    :type cl:fixnum
    :initform 0)
   (arrival
    :reader arrival
    :initarg :arrival
    :type cl:real
    :initform 0)
   (b_level
    :reader b_level
    :initarg :b_level
    :type cl:fixnum
    :initform 0)
   (e_status
    :reader e_status
    :initarg :e_status
    :type cl:fixnum
    :initform 0)
   (complete
    :reader complete
    :initarg :complete
    :type cl:fixnum
    :initform 0)
   (lastLP
    :reader lastLP
    :initarg :lastLP
    :type cl:fixnum
    :initform 0))
)

(cl:defclass agv_status (<agv_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <agv_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'agv_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<agv_status> is deprecated: use htbot-msg:agv_status instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <agv_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:cmd-val is deprecated.  Use htbot-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <agv_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:time-val is deprecated.  Use htbot-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <agv_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:status-val is deprecated.  Use htbot-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <agv_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:location-val is deprecated.  Use htbot-msg:location instead.")
  (location m))

(cl:ensure-generic-function 'arrival-val :lambda-list '(m))
(cl:defmethod arrival-val ((m <agv_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:arrival-val is deprecated.  Use htbot-msg:arrival instead.")
  (arrival m))

(cl:ensure-generic-function 'b_level-val :lambda-list '(m))
(cl:defmethod b_level-val ((m <agv_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:b_level-val is deprecated.  Use htbot-msg:b_level instead.")
  (b_level m))

(cl:ensure-generic-function 'e_status-val :lambda-list '(m))
(cl:defmethod e_status-val ((m <agv_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:e_status-val is deprecated.  Use htbot-msg:e_status instead.")
  (e_status m))

(cl:ensure-generic-function 'complete-val :lambda-list '(m))
(cl:defmethod complete-val ((m <agv_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:complete-val is deprecated.  Use htbot-msg:complete instead.")
  (complete m))

(cl:ensure-generic-function 'lastLP-val :lambda-list '(m))
(cl:defmethod lastLP-val ((m <agv_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:lastLP-val is deprecated.  Use htbot-msg:lastLP instead.")
  (lastLP m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <agv_status>) ostream)
  "Serializes a message object of type '<agv_status>"
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
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'location)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'arrival)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'arrival) (cl:floor (cl:slot-value msg 'arrival)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'b_level)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'e_status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'complete)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'lastLP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <agv_status>) istream)
  "Deserializes a message object of type '<agv_status>"
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
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arrival) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b_level) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'e_status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'complete) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lastLP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<agv_status>)))
  "Returns string type for a message object of type '<agv_status>"
  "htbot/agv_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'agv_status)))
  "Returns string type for a message object of type 'agv_status"
  "htbot/agv_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<agv_status>)))
  "Returns md5sum for a message object of type '<agv_status>"
  "0b47d165e9912a2afc0c77fd2d5b0bfa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'agv_status)))
  "Returns md5sum for a message object of type 'agv_status"
  "0b47d165e9912a2afc0c77fd2d5b0bfa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<agv_status>)))
  "Returns full string definition for message of type '<agv_status>"
  (cl:format cl:nil "int8 cmd~%time time~%int8 status~%int8 location~%time arrival~%int8 b_level~%int8 e_status~%int8 complete~%int8 lastLP~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'agv_status)))
  "Returns full string definition for message of type 'agv_status"
  (cl:format cl:nil "int8 cmd~%time time~%int8 status~%int8 location~%time arrival~%int8 b_level~%int8 e_status~%int8 complete~%int8 lastLP~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <agv_status>))
  (cl:+ 0
     1
     8
     1
     1
     8
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <agv_status>))
  "Converts a ROS message object to a list"
  (cl:list 'agv_status
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':time (time msg))
    (cl:cons ':status (status msg))
    (cl:cons ':location (location msg))
    (cl:cons ':arrival (arrival msg))
    (cl:cons ':b_level (b_level msg))
    (cl:cons ':e_status (e_status msg))
    (cl:cons ':complete (complete msg))
    (cl:cons ':lastLP (lastLP msg))
))
