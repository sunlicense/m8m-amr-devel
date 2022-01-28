; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude motorcmd.msg.html

(cl:defclass <motorcmd> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (dist
    :reader dist
    :initarg :dist
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (linear
    :reader linear
    :initarg :linear
    :type cl:float
    :initform 0.0)
   (angular
    :reader angular
    :initarg :angular
    :type cl:float
    :initform 0.0))
)

(cl:defclass motorcmd (<motorcmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motorcmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motorcmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<motorcmd> is deprecated: use htbot-msg:motorcmd instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <motorcmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:type-val is deprecated.  Use htbot-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'dist-val :lambda-list '(m))
(cl:defmethod dist-val ((m <motorcmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:dist-val is deprecated.  Use htbot-msg:dist instead.")
  (dist m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <motorcmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:angle-val is deprecated.  Use htbot-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <motorcmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:linear-val is deprecated.  Use htbot-msg:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <motorcmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:angular-val is deprecated.  Use htbot-msg:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motorcmd>) ostream)
  "Serializes a message object of type '<motorcmd>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motorcmd>) istream)
  "Deserializes a message object of type '<motorcmd>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motorcmd>)))
  "Returns string type for a message object of type '<motorcmd>"
  "htbot/motorcmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motorcmd)))
  "Returns string type for a message object of type 'motorcmd"
  "htbot/motorcmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motorcmd>)))
  "Returns md5sum for a message object of type '<motorcmd>"
  "f00942b3a4df0d60ea8f3f655f2d737a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motorcmd)))
  "Returns md5sum for a message object of type 'motorcmd"
  "f00942b3a4df0d60ea8f3f655f2d737a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motorcmd>)))
  "Returns full string definition for message of type '<motorcmd>"
  (cl:format cl:nil "int8 type~%float32 dist~%float32 angle~%float32 linear~%float32 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motorcmd)))
  "Returns full string definition for message of type 'motorcmd"
  (cl:format cl:nil "int8 type~%float32 dist~%float32 angle~%float32 linear~%float32 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motorcmd>))
  (cl:+ 0
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motorcmd>))
  "Converts a ROS message object to a list"
  (cl:list 'motorcmd
    (cl:cons ':type (type msg))
    (cl:cons ':dist (dist msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
))
