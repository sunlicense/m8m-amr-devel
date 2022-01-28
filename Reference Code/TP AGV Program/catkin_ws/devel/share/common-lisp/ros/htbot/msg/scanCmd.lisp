; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude scanCmd.msg.html

(cl:defclass <scanCmd> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0)
   (file
    :reader file
    :initarg :file
    :type cl:string
    :initform "")
   (f1
    :reader f1
    :initarg :f1
    :type cl:float
    :initform 0.0)
   (f2
    :reader f2
    :initarg :f2
    :type cl:float
    :initform 0.0)
   (f3
    :reader f3
    :initarg :f3
    :type cl:float
    :initform 0.0)
   (lp
    :reader lp
    :initarg :lp
    :type cl:fixnum
    :initform 0)
   (gp
    :reader gp
    :initarg :gp
    :type cl:fixnum
    :initform 0)
   (opt
    :reader opt
    :initarg :opt
    :type cl:fixnum
    :initform 0))
)

(cl:defclass scanCmd (<scanCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <scanCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'scanCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<scanCmd> is deprecated: use htbot-msg:scanCmd instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <scanCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:cmd-val is deprecated.  Use htbot-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <scanCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:file-val is deprecated.  Use htbot-msg:file instead.")
  (file m))

(cl:ensure-generic-function 'f1-val :lambda-list '(m))
(cl:defmethod f1-val ((m <scanCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:f1-val is deprecated.  Use htbot-msg:f1 instead.")
  (f1 m))

(cl:ensure-generic-function 'f2-val :lambda-list '(m))
(cl:defmethod f2-val ((m <scanCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:f2-val is deprecated.  Use htbot-msg:f2 instead.")
  (f2 m))

(cl:ensure-generic-function 'f3-val :lambda-list '(m))
(cl:defmethod f3-val ((m <scanCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:f3-val is deprecated.  Use htbot-msg:f3 instead.")
  (f3 m))

(cl:ensure-generic-function 'lp-val :lambda-list '(m))
(cl:defmethod lp-val ((m <scanCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:lp-val is deprecated.  Use htbot-msg:lp instead.")
  (lp m))

(cl:ensure-generic-function 'gp-val :lambda-list '(m))
(cl:defmethod gp-val ((m <scanCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:gp-val is deprecated.  Use htbot-msg:gp instead.")
  (gp m))

(cl:ensure-generic-function 'opt-val :lambda-list '(m))
(cl:defmethod opt-val ((m <scanCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:opt-val is deprecated.  Use htbot-msg:opt instead.")
  (opt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <scanCmd>) ostream)
  "Serializes a message object of type '<scanCmd>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'f1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'f2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'f3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'lp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'opt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <scanCmd>) istream)
  "Deserializes a message object of type '<scanCmd>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'f1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'f2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'f3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lp) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gp) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opt) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<scanCmd>)))
  "Returns string type for a message object of type '<scanCmd>"
  "htbot/scanCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'scanCmd)))
  "Returns string type for a message object of type 'scanCmd"
  "htbot/scanCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<scanCmd>)))
  "Returns md5sum for a message object of type '<scanCmd>"
  "009679f12681386d87811bbbbfbf00db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'scanCmd)))
  "Returns md5sum for a message object of type 'scanCmd"
  "009679f12681386d87811bbbbfbf00db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<scanCmd>)))
  "Returns full string definition for message of type '<scanCmd>"
  (cl:format cl:nil "int8 cmd~%string file~%float32 f1~%float32 f2~%float32 f3~%int8 lp~%int8 gp~%int8 opt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'scanCmd)))
  "Returns full string definition for message of type 'scanCmd"
  (cl:format cl:nil "int8 cmd~%string file~%float32 f1~%float32 f2~%float32 f3~%int8 lp~%int8 gp~%int8 opt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <scanCmd>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'file))
     4
     4
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <scanCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'scanCmd
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':file (file msg))
    (cl:cons ':f1 (f1 msg))
    (cl:cons ':f2 (f2 msg))
    (cl:cons ':f3 (f3 msg))
    (cl:cons ':lp (lp msg))
    (cl:cons ':gp (gp msg))
    (cl:cons ':opt (opt msg))
))
