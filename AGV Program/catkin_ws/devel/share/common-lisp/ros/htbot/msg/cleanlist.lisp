; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude cleanlist.msg.html

(cl:defclass <cleanlist> (roslisp-msg-protocol:ros-message)
  ((info
    :reader info
    :initarg :info
    :type cl:fixnum
    :initform 0)
   (mapno
    :reader mapno
    :initarg :mapno
    :type cl:integer
    :initform 0)
   (cplanno
    :reader cplanno
    :initarg :cplanno
    :type cl:integer
    :initform 0)
   (mapdir
    :reader mapdir
    :initarg :mapdir
    :type cl:string
    :initform "")
   (mapname
    :reader mapname
    :initarg :mapname
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (cplandir
    :reader cplandir
    :initarg :cplandir
    :type cl:string
    :initform "")
   (cleanplan
    :reader cleanplan
    :initarg :cleanplan
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass cleanlist (<cleanlist>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cleanlist>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cleanlist)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<cleanlist> is deprecated: use htbot-msg:cleanlist instead.")))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <cleanlist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:info-val is deprecated.  Use htbot-msg:info instead.")
  (info m))

(cl:ensure-generic-function 'mapno-val :lambda-list '(m))
(cl:defmethod mapno-val ((m <cleanlist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:mapno-val is deprecated.  Use htbot-msg:mapno instead.")
  (mapno m))

(cl:ensure-generic-function 'cplanno-val :lambda-list '(m))
(cl:defmethod cplanno-val ((m <cleanlist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:cplanno-val is deprecated.  Use htbot-msg:cplanno instead.")
  (cplanno m))

(cl:ensure-generic-function 'mapdir-val :lambda-list '(m))
(cl:defmethod mapdir-val ((m <cleanlist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:mapdir-val is deprecated.  Use htbot-msg:mapdir instead.")
  (mapdir m))

(cl:ensure-generic-function 'mapname-val :lambda-list '(m))
(cl:defmethod mapname-val ((m <cleanlist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:mapname-val is deprecated.  Use htbot-msg:mapname instead.")
  (mapname m))

(cl:ensure-generic-function 'cplandir-val :lambda-list '(m))
(cl:defmethod cplandir-val ((m <cleanlist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:cplandir-val is deprecated.  Use htbot-msg:cplandir instead.")
  (cplandir m))

(cl:ensure-generic-function 'cleanplan-val :lambda-list '(m))
(cl:defmethod cleanplan-val ((m <cleanlist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:cleanplan-val is deprecated.  Use htbot-msg:cleanplan instead.")
  (cleanplan m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cleanlist>) ostream)
  "Serializes a message object of type '<cleanlist>"
  (cl:let* ((signed (cl:slot-value msg 'info)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mapno)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cplanno)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mapdir))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mapdir))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mapname))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'mapname))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cplandir))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cplandir))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cleanplan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'cleanplan))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cleanlist>) istream)
  "Deserializes a message object of type '<cleanlist>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'info) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mapno) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cplanno) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mapdir) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mapdir) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mapname) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mapname)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cplandir) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cplandir) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cleanplan) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cleanplan)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cleanlist>)))
  "Returns string type for a message object of type '<cleanlist>"
  "htbot/cleanlist")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cleanlist)))
  "Returns string type for a message object of type 'cleanlist"
  "htbot/cleanlist")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cleanlist>)))
  "Returns md5sum for a message object of type '<cleanlist>"
  "975117791827cbfacd76c6c1c424923b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cleanlist)))
  "Returns md5sum for a message object of type 'cleanlist"
  "975117791827cbfacd76c6c1c424923b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cleanlist>)))
  "Returns full string definition for message of type '<cleanlist>"
  (cl:format cl:nil "int8 info~%int32 mapno~%int32 cplanno~%string mapdir~%string[] mapname~%string cplandir~%string[] cleanplan~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cleanlist)))
  "Returns full string definition for message of type 'cleanlist"
  (cl:format cl:nil "int8 info~%int32 mapno~%int32 cplanno~%string mapdir~%string[] mapname~%string cplandir~%string[] cleanplan~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cleanlist>))
  (cl:+ 0
     1
     4
     4
     4 (cl:length (cl:slot-value msg 'mapdir))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mapname) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'cplandir))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cleanplan) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cleanlist>))
  "Converts a ROS message object to a list"
  (cl:list 'cleanlist
    (cl:cons ':info (info msg))
    (cl:cons ':mapno (mapno msg))
    (cl:cons ':cplanno (cplanno msg))
    (cl:cons ':mapdir (mapdir msg))
    (cl:cons ':mapname (mapname msg))
    (cl:cons ':cplandir (cplandir msg))
    (cl:cons ':cleanplan (cleanplan msg))
))
