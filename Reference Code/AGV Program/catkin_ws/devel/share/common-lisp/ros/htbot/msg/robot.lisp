; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude robot.msg.html

(cl:defclass <robot> (roslisp-msg-protocol:ros-message)
  ((amclx
    :reader amclx
    :initarg :amclx
    :type cl:float
    :initform 0.0)
   (amcly
    :reader amcly
    :initarg :amcly
    :type cl:float
    :initform 0.0)
   (amcla
    :reader amcla
    :initarg :amcla
    :type cl:float
    :initform 0.0)
   (robotposediff
    :reader robotposediff
    :initarg :robotposediff
    :type cl:float
    :initform 0.0)
   (localisationstatus
    :reader localisationstatus
    :initarg :localisationstatus
    :type cl:string
    :initform ""))
)

(cl:defclass robot (<robot>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<robot> is deprecated: use htbot-msg:robot instead.")))

(cl:ensure-generic-function 'amclx-val :lambda-list '(m))
(cl:defmethod amclx-val ((m <robot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:amclx-val is deprecated.  Use htbot-msg:amclx instead.")
  (amclx m))

(cl:ensure-generic-function 'amcly-val :lambda-list '(m))
(cl:defmethod amcly-val ((m <robot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:amcly-val is deprecated.  Use htbot-msg:amcly instead.")
  (amcly m))

(cl:ensure-generic-function 'amcla-val :lambda-list '(m))
(cl:defmethod amcla-val ((m <robot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:amcla-val is deprecated.  Use htbot-msg:amcla instead.")
  (amcla m))

(cl:ensure-generic-function 'robotposediff-val :lambda-list '(m))
(cl:defmethod robotposediff-val ((m <robot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:robotposediff-val is deprecated.  Use htbot-msg:robotposediff instead.")
  (robotposediff m))

(cl:ensure-generic-function 'localisationstatus-val :lambda-list '(m))
(cl:defmethod localisationstatus-val ((m <robot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:localisationstatus-val is deprecated.  Use htbot-msg:localisationstatus instead.")
  (localisationstatus m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot>) ostream)
  "Serializes a message object of type '<robot>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'amclx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'amcly))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'amcla))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'robotposediff))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'localisationstatus))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'localisationstatus))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot>) istream)
  "Deserializes a message object of type '<robot>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amclx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amcly) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amcla) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robotposediff) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'localisationstatus) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'localisationstatus) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot>)))
  "Returns string type for a message object of type '<robot>"
  "htbot/robot")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot)))
  "Returns string type for a message object of type 'robot"
  "htbot/robot")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot>)))
  "Returns md5sum for a message object of type '<robot>"
  "9472043a148e005ed77686c11e7067c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot)))
  "Returns md5sum for a message object of type 'robot"
  "9472043a148e005ed77686c11e7067c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot>)))
  "Returns full string definition for message of type '<robot>"
  (cl:format cl:nil "float32 amclx~%float32 amcly~%float32 amcla~%float32 robotposediff~%string localisationstatus~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot)))
  "Returns full string definition for message of type 'robot"
  (cl:format cl:nil "float32 amclx~%float32 amcly~%float32 amcla~%float32 robotposediff~%string localisationstatus~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot>))
  (cl:+ 0
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'localisationstatus))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot>))
  "Converts a ROS message object to a list"
  (cl:list 'robot
    (cl:cons ':amclx (amclx msg))
    (cl:cons ':amcly (amcly msg))
    (cl:cons ':amcla (amcla msg))
    (cl:cons ':robotposediff (robotposediff msg))
    (cl:cons ':localisationstatus (localisationstatus msg))
))
