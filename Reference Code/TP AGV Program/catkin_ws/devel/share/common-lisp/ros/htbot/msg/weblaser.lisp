; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude weblaser.msg.html

(cl:defclass <weblaser> (roslisp-msg-protocol:ros-message)
  ((info
    :reader info
    :initarg :info
    :type cl:fixnum
    :initform 0)
   (size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0)
   (px
    :reader px
    :initarg :px
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (py
    :reader py
    :initarg :py
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass weblaser (<weblaser>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <weblaser>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'weblaser)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<weblaser> is deprecated: use htbot-msg:weblaser instead.")))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <weblaser>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:info-val is deprecated.  Use htbot-msg:info instead.")
  (info m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <weblaser>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:size-val is deprecated.  Use htbot-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'px-val :lambda-list '(m))
(cl:defmethod px-val ((m <weblaser>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:px-val is deprecated.  Use htbot-msg:px instead.")
  (px m))

(cl:ensure-generic-function 'py-val :lambda-list '(m))
(cl:defmethod py-val ((m <weblaser>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:py-val is deprecated.  Use htbot-msg:py instead.")
  (py m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <weblaser>) ostream)
  "Serializes a message object of type '<weblaser>"
  (cl:let* ((signed (cl:slot-value msg 'info)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'px))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'py))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'py))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <weblaser>) istream)
  "Deserializes a message object of type '<weblaser>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'info) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'px) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'px)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'py) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'py)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<weblaser>)))
  "Returns string type for a message object of type '<weblaser>"
  "htbot/weblaser")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'weblaser)))
  "Returns string type for a message object of type 'weblaser"
  "htbot/weblaser")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<weblaser>)))
  "Returns md5sum for a message object of type '<weblaser>"
  "f7e0272dce619f1a82242dcdb545a8bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'weblaser)))
  "Returns md5sum for a message object of type 'weblaser"
  "f7e0272dce619f1a82242dcdb545a8bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<weblaser>)))
  "Returns full string definition for message of type '<weblaser>"
  (cl:format cl:nil "int8 info~%int32 size~%float32[] px~%float32[] py~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'weblaser)))
  "Returns full string definition for message of type 'weblaser"
  (cl:format cl:nil "int8 info~%int32 size~%float32[] px~%float32[] py~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <weblaser>))
  (cl:+ 0
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'px) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'py) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <weblaser>))
  "Converts a ROS message object to a list"
  (cl:list 'weblaser
    (cl:cons ':info (info msg))
    (cl:cons ':size (size msg))
    (cl:cons ':px (px msg))
    (cl:cons ':py (py msg))
))
