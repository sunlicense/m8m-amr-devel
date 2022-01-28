; Auto-generated. Do not edit!


(cl:in-package htbot-srv)


;//! \htmlinclude srvcmd-request.msg.html

(cl:defclass <srvcmd-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0)
   (fromLP
    :reader fromLP
    :initarg :fromLP
    :type cl:string
    :initform "")
   (toLP
    :reader toLP
    :initarg :toLP
    :type cl:string
    :initform "")
   (cGP
    :reader cGP
    :initarg :cGP
    :type cl:fixnum
    :initform 0)
   (cLP
    :reader cLP
    :initarg :cLP
    :type cl:fixnum
    :initform 0)
   (fLP
    :reader fLP
    :initarg :fLP
    :type cl:fixnum
    :initform 0)
   (tLP
    :reader tLP
    :initarg :tLP
    :type cl:fixnum
    :initform 0))
)

(cl:defclass srvcmd-request (<srvcmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srvcmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srvcmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-srv:<srvcmd-request> is deprecated: use htbot-srv:srvcmd-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <srvcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:cmd-val is deprecated.  Use htbot-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'fromLP-val :lambda-list '(m))
(cl:defmethod fromLP-val ((m <srvcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:fromLP-val is deprecated.  Use htbot-srv:fromLP instead.")
  (fromLP m))

(cl:ensure-generic-function 'toLP-val :lambda-list '(m))
(cl:defmethod toLP-val ((m <srvcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:toLP-val is deprecated.  Use htbot-srv:toLP instead.")
  (toLP m))

(cl:ensure-generic-function 'cGP-val :lambda-list '(m))
(cl:defmethod cGP-val ((m <srvcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:cGP-val is deprecated.  Use htbot-srv:cGP instead.")
  (cGP m))

(cl:ensure-generic-function 'cLP-val :lambda-list '(m))
(cl:defmethod cLP-val ((m <srvcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:cLP-val is deprecated.  Use htbot-srv:cLP instead.")
  (cLP m))

(cl:ensure-generic-function 'fLP-val :lambda-list '(m))
(cl:defmethod fLP-val ((m <srvcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:fLP-val is deprecated.  Use htbot-srv:fLP instead.")
  (fLP m))

(cl:ensure-generic-function 'tLP-val :lambda-list '(m))
(cl:defmethod tLP-val ((m <srvcmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:tLP-val is deprecated.  Use htbot-srv:tLP instead.")
  (tLP m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srvcmd-request>) ostream)
  "Serializes a message object of type '<srvcmd-request>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fromLP))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fromLP))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'toLP))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'toLP))
  (cl:let* ((signed (cl:slot-value msg 'cGP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cLP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'fLP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tLP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srvcmd-request>) istream)
  "Deserializes a message object of type '<srvcmd-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fromLP) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fromLP) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'toLP) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'toLP) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cGP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cLP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fLP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tLP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srvcmd-request>)))
  "Returns string type for a service object of type '<srvcmd-request>"
  "htbot/srvcmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srvcmd-request)))
  "Returns string type for a service object of type 'srvcmd-request"
  "htbot/srvcmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srvcmd-request>)))
  "Returns md5sum for a message object of type '<srvcmd-request>"
  "30eadfa966628646a567b1a8c11f77b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srvcmd-request)))
  "Returns md5sum for a message object of type 'srvcmd-request"
  "30eadfa966628646a567b1a8c11f77b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srvcmd-request>)))
  "Returns full string definition for message of type '<srvcmd-request>"
  (cl:format cl:nil "int8 cmd~%string fromLP~%string toLP~%int8 cGP~%int8 cLP~%int8 fLP~%int8 tLP~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srvcmd-request)))
  "Returns full string definition for message of type 'srvcmd-request"
  (cl:format cl:nil "int8 cmd~%string fromLP~%string toLP~%int8 cGP~%int8 cLP~%int8 fLP~%int8 tLP~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srvcmd-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'fromLP))
     4 (cl:length (cl:slot-value msg 'toLP))
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srvcmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srvcmd-request
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':fromLP (fromLP msg))
    (cl:cons ':toLP (toLP msg))
    (cl:cons ':cGP (cGP msg))
    (cl:cons ':cLP (cLP msg))
    (cl:cons ':fLP (fLP msg))
    (cl:cons ':tLP (tLP msg))
))
;//! \htmlinclude srvcmd-response.msg.html

(cl:defclass <srvcmd-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass srvcmd-response (<srvcmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srvcmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srvcmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-srv:<srvcmd-response> is deprecated: use htbot-srv:srvcmd-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <srvcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:status-val is deprecated.  Use htbot-srv:status instead.")
  (status m))

(cl:ensure-generic-function 's1-val :lambda-list '(m))
(cl:defmethod s1-val ((m <srvcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:s1-val is deprecated.  Use htbot-srv:s1 instead.")
  (s1 m))

(cl:ensure-generic-function 's2-val :lambda-list '(m))
(cl:defmethod s2-val ((m <srvcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:s2-val is deprecated.  Use htbot-srv:s2 instead.")
  (s2 m))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <srvcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:linear-val is deprecated.  Use htbot-srv:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <srvcmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:angular-val is deprecated.  Use htbot-srv:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srvcmd-response>) ostream)
  "Serializes a message object of type '<srvcmd-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srvcmd-response>) istream)
  "Deserializes a message object of type '<srvcmd-response>"
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
    (cl:setf (cl:slot-value msg 'linear) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srvcmd-response>)))
  "Returns string type for a service object of type '<srvcmd-response>"
  "htbot/srvcmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srvcmd-response)))
  "Returns string type for a service object of type 'srvcmd-response"
  "htbot/srvcmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srvcmd-response>)))
  "Returns md5sum for a message object of type '<srvcmd-response>"
  "30eadfa966628646a567b1a8c11f77b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srvcmd-response)))
  "Returns md5sum for a message object of type 'srvcmd-response"
  "30eadfa966628646a567b1a8c11f77b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srvcmd-response>)))
  "Returns full string definition for message of type '<srvcmd-response>"
  (cl:format cl:nil "int8 status~%string s1~%string s2~%float32 linear~%float32 angular~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srvcmd-response)))
  "Returns full string definition for message of type 'srvcmd-response"
  (cl:format cl:nil "int8 status~%string s1~%string s2~%float32 linear~%float32 angular~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srvcmd-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 's1))
     4 (cl:length (cl:slot-value msg 's2))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srvcmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srvcmd-response
    (cl:cons ':status (status msg))
    (cl:cons ':s1 (s1 msg))
    (cl:cons ':s2 (s2 msg))
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srvcmd)))
  'srvcmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srvcmd)))
  'srvcmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srvcmd)))
  "Returns string type for a service object of type '<srvcmd>"
  "htbot/srvcmd")