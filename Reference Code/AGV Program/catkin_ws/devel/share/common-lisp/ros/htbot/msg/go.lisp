; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude go.msg.html

(cl:defclass <go> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0))
)

(cl:defclass go (<go>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <go>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'go)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<go> is deprecated: use htbot-msg:go instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <go>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:cmd-val is deprecated.  Use htbot-msg:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <go>) ostream)
  "Serializes a message object of type '<go>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <go>) istream)
  "Deserializes a message object of type '<go>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<go>)))
  "Returns string type for a message object of type '<go>"
  "htbot/go")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'go)))
  "Returns string type for a message object of type 'go"
  "htbot/go")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<go>)))
  "Returns md5sum for a message object of type '<go>"
  "26e2d5a54557d558b8243da339e9952c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'go)))
  "Returns md5sum for a message object of type 'go"
  "26e2d5a54557d558b8243da339e9952c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<go>)))
  "Returns full string definition for message of type '<go>"
  (cl:format cl:nil "int8 cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'go)))
  "Returns full string definition for message of type 'go"
  (cl:format cl:nil "int8 cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <go>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <go>))
  "Converts a ROS message object to a list"
  (cl:list 'go
    (cl:cons ':cmd (cmd msg))
))
