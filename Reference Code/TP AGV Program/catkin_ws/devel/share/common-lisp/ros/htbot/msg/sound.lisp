; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude sound.msg.html

(cl:defclass <sound> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (startdelay
    :reader startdelay
    :initarg :startdelay
    :type cl:fixnum
    :initform 0)
   (restartdelay
    :reader restartdelay
    :initarg :restartdelay
    :type cl:fixnum
    :initform 0))
)

(cl:defclass sound (<sound>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sound>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sound)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<sound> is deprecated: use htbot-msg:sound instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <sound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:id-val is deprecated.  Use htbot-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'startdelay-val :lambda-list '(m))
(cl:defmethod startdelay-val ((m <sound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:startdelay-val is deprecated.  Use htbot-msg:startdelay instead.")
  (startdelay m))

(cl:ensure-generic-function 'restartdelay-val :lambda-list '(m))
(cl:defmethod restartdelay-val ((m <sound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:restartdelay-val is deprecated.  Use htbot-msg:restartdelay instead.")
  (restartdelay m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sound>) ostream)
  "Serializes a message object of type '<sound>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'startdelay)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'restartdelay)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sound>) istream)
  "Deserializes a message object of type '<sound>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'startdelay) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'restartdelay) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sound>)))
  "Returns string type for a message object of type '<sound>"
  "htbot/sound")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sound)))
  "Returns string type for a message object of type 'sound"
  "htbot/sound")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sound>)))
  "Returns md5sum for a message object of type '<sound>"
  "ca5aa2e17ae07703f6b975c8e29b6503")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sound)))
  "Returns md5sum for a message object of type 'sound"
  "ca5aa2e17ae07703f6b975c8e29b6503")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sound>)))
  "Returns full string definition for message of type '<sound>"
  (cl:format cl:nil "int16 id~%int8 startdelay~%int8 restartdelay~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sound)))
  "Returns full string definition for message of type 'sound"
  (cl:format cl:nil "int16 id~%int8 startdelay~%int8 restartdelay~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sound>))
  (cl:+ 0
     2
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sound>))
  "Converts a ROS message object to a list"
  (cl:list 'sound
    (cl:cons ':id (id msg))
    (cl:cons ':startdelay (startdelay msg))
    (cl:cons ':restartdelay (restartdelay msg))
))
