; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude queue.msg.html

(cl:defclass <queue> (roslisp-msg-protocol:ros-message)
  ((noQ
    :reader noQ
    :initarg :noQ
    :type cl:fixnum
    :initform 0)
   (fLP1
    :reader fLP1
    :initarg :fLP1
    :type cl:string
    :initform "")
   (tLP1
    :reader tLP1
    :initarg :tLP1
    :type cl:string
    :initform "")
   (fLP2
    :reader fLP2
    :initarg :fLP2
    :type cl:string
    :initform "")
   (tLP2
    :reader tLP2
    :initarg :tLP2
    :type cl:string
    :initform "")
   (fLP3
    :reader fLP3
    :initarg :fLP3
    :type cl:string
    :initform "")
   (tLP3
    :reader tLP3
    :initarg :tLP3
    :type cl:string
    :initform "")
   (fLP4
    :reader fLP4
    :initarg :fLP4
    :type cl:string
    :initform "")
   (tLP4
    :reader tLP4
    :initarg :tLP4
    :type cl:string
    :initform "")
   (LPName
    :reader LPName
    :initarg :LPName
    :type cl:string
    :initform "")
   (LPInfo
    :reader LPInfo
    :initarg :LPInfo
    :type cl:string
    :initform ""))
)

(cl:defclass queue (<queue>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <queue>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'queue)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<queue> is deprecated: use htbot-msg:queue instead.")))

(cl:ensure-generic-function 'noQ-val :lambda-list '(m))
(cl:defmethod noQ-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:noQ-val is deprecated.  Use htbot-msg:noQ instead.")
  (noQ m))

(cl:ensure-generic-function 'fLP1-val :lambda-list '(m))
(cl:defmethod fLP1-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:fLP1-val is deprecated.  Use htbot-msg:fLP1 instead.")
  (fLP1 m))

(cl:ensure-generic-function 'tLP1-val :lambda-list '(m))
(cl:defmethod tLP1-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:tLP1-val is deprecated.  Use htbot-msg:tLP1 instead.")
  (tLP1 m))

(cl:ensure-generic-function 'fLP2-val :lambda-list '(m))
(cl:defmethod fLP2-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:fLP2-val is deprecated.  Use htbot-msg:fLP2 instead.")
  (fLP2 m))

(cl:ensure-generic-function 'tLP2-val :lambda-list '(m))
(cl:defmethod tLP2-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:tLP2-val is deprecated.  Use htbot-msg:tLP2 instead.")
  (tLP2 m))

(cl:ensure-generic-function 'fLP3-val :lambda-list '(m))
(cl:defmethod fLP3-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:fLP3-val is deprecated.  Use htbot-msg:fLP3 instead.")
  (fLP3 m))

(cl:ensure-generic-function 'tLP3-val :lambda-list '(m))
(cl:defmethod tLP3-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:tLP3-val is deprecated.  Use htbot-msg:tLP3 instead.")
  (tLP3 m))

(cl:ensure-generic-function 'fLP4-val :lambda-list '(m))
(cl:defmethod fLP4-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:fLP4-val is deprecated.  Use htbot-msg:fLP4 instead.")
  (fLP4 m))

(cl:ensure-generic-function 'tLP4-val :lambda-list '(m))
(cl:defmethod tLP4-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:tLP4-val is deprecated.  Use htbot-msg:tLP4 instead.")
  (tLP4 m))

(cl:ensure-generic-function 'LPName-val :lambda-list '(m))
(cl:defmethod LPName-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:LPName-val is deprecated.  Use htbot-msg:LPName instead.")
  (LPName m))

(cl:ensure-generic-function 'LPInfo-val :lambda-list '(m))
(cl:defmethod LPInfo-val ((m <queue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:LPInfo-val is deprecated.  Use htbot-msg:LPInfo instead.")
  (LPInfo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <queue>) ostream)
  "Serializes a message object of type '<queue>"
  (cl:let* ((signed (cl:slot-value msg 'noQ)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fLP1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fLP1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tLP1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tLP1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fLP2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fLP2))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tLP2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tLP2))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fLP3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fLP3))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tLP3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tLP3))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fLP4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fLP4))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tLP4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tLP4))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'LPName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'LPName))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'LPInfo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'LPInfo))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <queue>) istream)
  "Deserializes a message object of type '<queue>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'noQ) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fLP1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fLP1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tLP1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tLP1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fLP2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fLP2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tLP2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tLP2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fLP3) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fLP3) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tLP3) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tLP3) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fLP4) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fLP4) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tLP4) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tLP4) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'LPName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'LPName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'LPInfo) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'LPInfo) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<queue>)))
  "Returns string type for a message object of type '<queue>"
  "htbot/queue")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'queue)))
  "Returns string type for a message object of type 'queue"
  "htbot/queue")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<queue>)))
  "Returns md5sum for a message object of type '<queue>"
  "1ad64500a08450047cb508b267d3d903")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'queue)))
  "Returns md5sum for a message object of type 'queue"
  "1ad64500a08450047cb508b267d3d903")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<queue>)))
  "Returns full string definition for message of type '<queue>"
  (cl:format cl:nil "int8 noQ~%string fLP1~%string tLP1~%string fLP2~%string tLP2~%string fLP3~%string tLP3~%string fLP4~%string tLP4~%string LPName~%string LPInfo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'queue)))
  "Returns full string definition for message of type 'queue"
  (cl:format cl:nil "int8 noQ~%string fLP1~%string tLP1~%string fLP2~%string tLP2~%string fLP3~%string tLP3~%string fLP4~%string tLP4~%string LPName~%string LPInfo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <queue>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'fLP1))
     4 (cl:length (cl:slot-value msg 'tLP1))
     4 (cl:length (cl:slot-value msg 'fLP2))
     4 (cl:length (cl:slot-value msg 'tLP2))
     4 (cl:length (cl:slot-value msg 'fLP3))
     4 (cl:length (cl:slot-value msg 'tLP3))
     4 (cl:length (cl:slot-value msg 'fLP4))
     4 (cl:length (cl:slot-value msg 'tLP4))
     4 (cl:length (cl:slot-value msg 'LPName))
     4 (cl:length (cl:slot-value msg 'LPInfo))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <queue>))
  "Converts a ROS message object to a list"
  (cl:list 'queue
    (cl:cons ':noQ (noQ msg))
    (cl:cons ':fLP1 (fLP1 msg))
    (cl:cons ':tLP1 (tLP1 msg))
    (cl:cons ':fLP2 (fLP2 msg))
    (cl:cons ':tLP2 (tLP2 msg))
    (cl:cons ':fLP3 (fLP3 msg))
    (cl:cons ':tLP3 (tLP3 msg))
    (cl:cons ':fLP4 (fLP4 msg))
    (cl:cons ':tLP4 (tLP4 msg))
    (cl:cons ':LPName (LPName msg))
    (cl:cons ':LPInfo (LPInfo msg))
))
