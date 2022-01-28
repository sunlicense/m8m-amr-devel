; Auto-generated. Do not edit!


(cl:in-package htbot-srv)


;//! \htmlinclude sendgoal-request.msg.html

(cl:defclass <sendgoal-request> (roslisp-msg-protocol:ros-message)
  ((x
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
    :initform 0.0))
)

(cl:defclass sendgoal-request (<sendgoal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sendgoal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sendgoal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-srv:<sendgoal-request> is deprecated: use htbot-srv:sendgoal-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <sendgoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:x-val is deprecated.  Use htbot-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <sendgoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:y-val is deprecated.  Use htbot-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <sendgoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:z-val is deprecated.  Use htbot-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'rx-val :lambda-list '(m))
(cl:defmethod rx-val ((m <sendgoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:rx-val is deprecated.  Use htbot-srv:rx instead.")
  (rx m))

(cl:ensure-generic-function 'ry-val :lambda-list '(m))
(cl:defmethod ry-val ((m <sendgoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:ry-val is deprecated.  Use htbot-srv:ry instead.")
  (ry m))

(cl:ensure-generic-function 'rz-val :lambda-list '(m))
(cl:defmethod rz-val ((m <sendgoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:rz-val is deprecated.  Use htbot-srv:rz instead.")
  (rz m))

(cl:ensure-generic-function 'rw-val :lambda-list '(m))
(cl:defmethod rw-val ((m <sendgoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:rw-val is deprecated.  Use htbot-srv:rw instead.")
  (rw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sendgoal-request>) ostream)
  "Serializes a message object of type '<sendgoal-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sendgoal-request>) istream)
  "Deserializes a message object of type '<sendgoal-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sendgoal-request>)))
  "Returns string type for a service object of type '<sendgoal-request>"
  "htbot/sendgoalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sendgoal-request)))
  "Returns string type for a service object of type 'sendgoal-request"
  "htbot/sendgoalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sendgoal-request>)))
  "Returns md5sum for a message object of type '<sendgoal-request>"
  "e870aaccbc4db5d4afb5053d631034b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sendgoal-request)))
  "Returns md5sum for a message object of type 'sendgoal-request"
  "e870aaccbc4db5d4afb5053d631034b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sendgoal-request>)))
  "Returns full string definition for message of type '<sendgoal-request>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 rx~%float32 ry~%float32 rz~%float32 rw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sendgoal-request)))
  "Returns full string definition for message of type 'sendgoal-request"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 rx~%float32 ry~%float32 rz~%float32 rw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sendgoal-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sendgoal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'sendgoal-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':rx (rx msg))
    (cl:cons ':ry (ry msg))
    (cl:cons ':rz (rz msg))
    (cl:cons ':rw (rw msg))
))
;//! \htmlinclude sendgoal-response.msg.html

(cl:defclass <sendgoal-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass sendgoal-response (<sendgoal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sendgoal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sendgoal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-srv:<sendgoal-response> is deprecated: use htbot-srv:sendgoal-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <sendgoal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:status-val is deprecated.  Use htbot-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sendgoal-response>) ostream)
  "Serializes a message object of type '<sendgoal-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sendgoal-response>) istream)
  "Deserializes a message object of type '<sendgoal-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sendgoal-response>)))
  "Returns string type for a service object of type '<sendgoal-response>"
  "htbot/sendgoalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sendgoal-response)))
  "Returns string type for a service object of type 'sendgoal-response"
  "htbot/sendgoalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sendgoal-response>)))
  "Returns md5sum for a message object of type '<sendgoal-response>"
  "e870aaccbc4db5d4afb5053d631034b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sendgoal-response)))
  "Returns md5sum for a message object of type 'sendgoal-response"
  "e870aaccbc4db5d4afb5053d631034b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sendgoal-response>)))
  "Returns full string definition for message of type '<sendgoal-response>"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sendgoal-response)))
  "Returns full string definition for message of type 'sendgoal-response"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sendgoal-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sendgoal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'sendgoal-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'sendgoal)))
  'sendgoal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'sendgoal)))
  'sendgoal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sendgoal)))
  "Returns string type for a service object of type '<sendgoal>"
  "htbot/sendgoal")