; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude ultraSS.msg.html

(cl:defclass <ultraSS> (roslisp-msg-protocol:ros-message)
  ((uFR
    :reader uFR
    :initarg :uFR
    :type cl:float
    :initform 0.0)
   (uFL
    :reader uFL
    :initarg :uFL
    :type cl:float
    :initform 0.0)
   (uRF
    :reader uRF
    :initarg :uRF
    :type cl:float
    :initform 0.0)
   (uRR
    :reader uRR
    :initarg :uRR
    :type cl:float
    :initform 0.0)
   (uLF
    :reader uLF
    :initarg :uLF
    :type cl:float
    :initform 0.0)
   (uLR
    :reader uLR
    :initarg :uLR
    :type cl:float
    :initform 0.0))
)

(cl:defclass ultraSS (<ultraSS>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ultraSS>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ultraSS)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<ultraSS> is deprecated: use htbot-msg:ultraSS instead.")))

(cl:ensure-generic-function 'uFR-val :lambda-list '(m))
(cl:defmethod uFR-val ((m <ultraSS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:uFR-val is deprecated.  Use htbot-msg:uFR instead.")
  (uFR m))

(cl:ensure-generic-function 'uFL-val :lambda-list '(m))
(cl:defmethod uFL-val ((m <ultraSS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:uFL-val is deprecated.  Use htbot-msg:uFL instead.")
  (uFL m))

(cl:ensure-generic-function 'uRF-val :lambda-list '(m))
(cl:defmethod uRF-val ((m <ultraSS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:uRF-val is deprecated.  Use htbot-msg:uRF instead.")
  (uRF m))

(cl:ensure-generic-function 'uRR-val :lambda-list '(m))
(cl:defmethod uRR-val ((m <ultraSS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:uRR-val is deprecated.  Use htbot-msg:uRR instead.")
  (uRR m))

(cl:ensure-generic-function 'uLF-val :lambda-list '(m))
(cl:defmethod uLF-val ((m <ultraSS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:uLF-val is deprecated.  Use htbot-msg:uLF instead.")
  (uLF m))

(cl:ensure-generic-function 'uLR-val :lambda-list '(m))
(cl:defmethod uLR-val ((m <ultraSS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:uLR-val is deprecated.  Use htbot-msg:uLR instead.")
  (uLR m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ultraSS>) ostream)
  "Serializes a message object of type '<ultraSS>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uFR))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uFL))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uRF))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uRR))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uLF))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uLR))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ultraSS>) istream)
  "Deserializes a message object of type '<ultraSS>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uFR) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uFL) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uRF) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uRR) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uLF) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uLR) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ultraSS>)))
  "Returns string type for a message object of type '<ultraSS>"
  "htbot/ultraSS")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ultraSS)))
  "Returns string type for a message object of type 'ultraSS"
  "htbot/ultraSS")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ultraSS>)))
  "Returns md5sum for a message object of type '<ultraSS>"
  "9863d3f832b0ad0ccba6621fbb52a2e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ultraSS)))
  "Returns md5sum for a message object of type 'ultraSS"
  "9863d3f832b0ad0ccba6621fbb52a2e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ultraSS>)))
  "Returns full string definition for message of type '<ultraSS>"
  (cl:format cl:nil "float32 uFR~%float32 uFL~%float32 uRF~%float32 uRR~%float32 uLF~%float32 uLR~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ultraSS)))
  "Returns full string definition for message of type 'ultraSS"
  (cl:format cl:nil "float32 uFR~%float32 uFL~%float32 uRF~%float32 uRR~%float32 uLF~%float32 uLR~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ultraSS>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ultraSS>))
  "Converts a ROS message object to a list"
  (cl:list 'ultraSS
    (cl:cons ':uFR (uFR msg))
    (cl:cons ':uFL (uFL msg))
    (cl:cons ':uRF (uRF msg))
    (cl:cons ':uRR (uRR msg))
    (cl:cons ':uLF (uLF msg))
    (cl:cons ':uLR (uLR msg))
))
