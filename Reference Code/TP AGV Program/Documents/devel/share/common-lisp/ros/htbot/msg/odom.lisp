; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude odom.msg.html

(cl:defclass <odom> (roslisp-msg-protocol:ros-message)
  ((dist
    :reader dist
    :initarg :dist
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass odom (<odom>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <odom>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'odom)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<odom> is deprecated: use htbot-msg:odom instead.")))

(cl:ensure-generic-function 'dist-val :lambda-list '(m))
(cl:defmethod dist-val ((m <odom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:dist-val is deprecated.  Use htbot-msg:dist instead.")
  (dist m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <odom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:angle-val is deprecated.  Use htbot-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <odom>) ostream)
  "Serializes a message object of type '<odom>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <odom>) istream)
  "Deserializes a message object of type '<odom>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<odom>)))
  "Returns string type for a message object of type '<odom>"
  "htbot/odom")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'odom)))
  "Returns string type for a message object of type 'odom"
  "htbot/odom")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<odom>)))
  "Returns md5sum for a message object of type '<odom>"
  "c05404bebfe29b1ba1f590bcefdda9bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'odom)))
  "Returns md5sum for a message object of type 'odom"
  "c05404bebfe29b1ba1f590bcefdda9bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<odom>)))
  "Returns full string definition for message of type '<odom>"
  (cl:format cl:nil "float32 dist~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'odom)))
  "Returns full string definition for message of type 'odom"
  (cl:format cl:nil "float32 dist~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <odom>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <odom>))
  "Converts a ROS message object to a list"
  (cl:list 'odom
    (cl:cons ':dist (dist msg))
    (cl:cons ':angle (angle msg))
))
