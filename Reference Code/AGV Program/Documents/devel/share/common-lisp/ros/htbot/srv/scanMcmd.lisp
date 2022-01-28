; Auto-generated. Do not edit!


(cl:in-package htbot-srv)


;//! \htmlinclude scanMcmd-request.msg.html

(cl:defclass <scanMcmd-request> (roslisp-msg-protocol:ros-message)
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
   (lp
    :reader lp
    :initarg :lp
    :type cl:fixnum
    :initform 0)
   (gp
    :reader gp
    :initarg :gp
    :type cl:fixnum
    :initform 0))
)

(cl:defclass scanMcmd-request (<scanMcmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <scanMcmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'scanMcmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-srv:<scanMcmd-request> is deprecated: use htbot-srv:scanMcmd-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <scanMcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:cmd-val is deprecated.  Use htbot-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <scanMcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:file-val is deprecated.  Use htbot-srv:file instead.")
  (file m))

(cl:ensure-generic-function 'lp-val :lambda-list '(m))
(cl:defmethod lp-val ((m <scanMcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lp-val is deprecated.  Use htbot-srv:lp instead.")
  (lp m))

(cl:ensure-generic-function 'gp-val :lambda-list '(m))
(cl:defmethod gp-val ((m <scanMcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:gp-val is deprecated.  Use htbot-srv:gp instead.")
  (gp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <scanMcmd-request>) ostream)
  "Serializes a message object of type '<scanMcmd-request>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file))
  (cl:let* ((signed (cl:slot-value msg 'lp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <scanMcmd-request>) istream)
  "Deserializes a message object of type '<scanMcmd-request>"
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lp) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gp) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<scanMcmd-request>)))
  "Returns string type for a service object of type '<scanMcmd-request>"
  "htbot/scanMcmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'scanMcmd-request)))
  "Returns string type for a service object of type 'scanMcmd-request"
  "htbot/scanMcmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<scanMcmd-request>)))
  "Returns md5sum for a message object of type '<scanMcmd-request>"
  "fe363af63be51788db46af922590fcac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'scanMcmd-request)))
  "Returns md5sum for a message object of type 'scanMcmd-request"
  "fe363af63be51788db46af922590fcac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<scanMcmd-request>)))
  "Returns full string definition for message of type '<scanMcmd-request>"
  (cl:format cl:nil "int8 cmd~%string file~%int8 lp~%int8 gp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'scanMcmd-request)))
  "Returns full string definition for message of type 'scanMcmd-request"
  (cl:format cl:nil "int8 cmd~%string file~%int8 lp~%int8 gp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <scanMcmd-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'file))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <scanMcmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'scanMcmd-request
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':file (file msg))
    (cl:cons ':lp (lp msg))
    (cl:cons ':gp (gp msg))
))
;//! \htmlinclude scanMcmd-response.msg.html

(cl:defclass <scanMcmd-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (s1
    :reader s1
    :initarg :s1
    :type cl:string
    :initform "")
   (s2
    :reader s2
    :initarg :s2
    :type cl:string
    :initform "")
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (an
    :reader an
    :initarg :an
    :type cl:float
    :initform 0.0))
)

(cl:defclass scanMcmd-response (<scanMcmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <scanMcmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'scanMcmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-srv:<scanMcmd-response> is deprecated: use htbot-srv:scanMcmd-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <scanMcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:status-val is deprecated.  Use htbot-srv:status instead.")
  (status m))

(cl:ensure-generic-function 's1-val :lambda-list '(m))
(cl:defmethod s1-val ((m <scanMcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:s1-val is deprecated.  Use htbot-srv:s1 instead.")
  (s1 m))

(cl:ensure-generic-function 's2-val :lambda-list '(m))
(cl:defmethod s2-val ((m <scanMcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:s2-val is deprecated.  Use htbot-srv:s2 instead.")
  (s2 m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <scanMcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:x-val is deprecated.  Use htbot-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <scanMcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:y-val is deprecated.  Use htbot-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'an-val :lambda-list '(m))
(cl:defmethod an-val ((m <scanMcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:an-val is deprecated.  Use htbot-srv:an instead.")
  (an m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <scanMcmd-response>) ostream)
  "Serializes a message object of type '<scanMcmd-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 's1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 's1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 's2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 's2))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'an))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <scanMcmd-response>) istream)
  "Deserializes a message object of type '<scanMcmd-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 's1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 's1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 's2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 's2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
    (cl:setf (cl:slot-value msg 'an) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<scanMcmd-response>)))
  "Returns string type for a service object of type '<scanMcmd-response>"
  "htbot/scanMcmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'scanMcmd-response)))
  "Returns string type for a service object of type 'scanMcmd-response"
  "htbot/scanMcmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<scanMcmd-response>)))
  "Returns md5sum for a message object of type '<scanMcmd-response>"
  "fe363af63be51788db46af922590fcac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'scanMcmd-response)))
  "Returns md5sum for a message object of type 'scanMcmd-response"
  "fe363af63be51788db46af922590fcac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<scanMcmd-response>)))
  "Returns full string definition for message of type '<scanMcmd-response>"
  (cl:format cl:nil "int8 status~%string s1~%string s2~%float32 x~%float32 y~%float32 an~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'scanMcmd-response)))
  "Returns full string definition for message of type 'scanMcmd-response"
  (cl:format cl:nil "int8 status~%string s1~%string s2~%float32 x~%float32 y~%float32 an~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <scanMcmd-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 's1))
     4 (cl:length (cl:slot-value msg 's2))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <scanMcmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'scanMcmd-response
    (cl:cons ':status (status msg))
    (cl:cons ':s1 (s1 msg))
    (cl:cons ':s2 (s2 msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':an (an msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'scanMcmd)))
  'scanMcmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'scanMcmd)))
  'scanMcmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'scanMcmd)))
  "Returns string type for a service object of type '<scanMcmd>"
  "htbot/scanMcmd")