; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude move_status.msg.html

(cl:defclass <move_status> (roslisp-msg-protocol:ros-message)
  ((stat
    :reader stat
    :initarg :stat
    :type cl:fixnum
    :initform 0))
)

(cl:defclass move_status (<move_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <move_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'move_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<move_status> is deprecated: use htbot-msg:move_status instead.")))

(cl:ensure-generic-function 'stat-val :lambda-list '(m))
(cl:defmethod stat-val ((m <move_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:stat-val is deprecated.  Use htbot-msg:stat instead.")
  (stat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <move_status>) ostream)
  "Serializes a message object of type '<move_status>"
  (cl:let* ((signed (cl:slot-value msg 'stat)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <move_status>) istream)
  "Deserializes a message object of type '<move_status>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stat) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<move_status>)))
  "Returns string type for a message object of type '<move_status>"
  "htbot/move_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move_status)))
  "Returns string type for a message object of type 'move_status"
  "htbot/move_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<move_status>)))
  "Returns md5sum for a message object of type '<move_status>"
  "85998e8afa5502f501182cfd6840bd64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'move_status)))
  "Returns md5sum for a message object of type 'move_status"
  "85998e8afa5502f501182cfd6840bd64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<move_status>)))
  "Returns full string definition for message of type '<move_status>"
  (cl:format cl:nil "int8 stat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'move_status)))
  "Returns full string definition for message of type 'move_status"
  (cl:format cl:nil "int8 stat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <move_status>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <move_status>))
  "Converts a ROS message object to a list"
  (cl:list 'move_status
    (cl:cons ':stat (stat msg))
))
