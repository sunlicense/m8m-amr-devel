; Auto-generated. Do not edit!


(cl:in-package htbot-srv)


;//! \htmlinclude mqueue-request.msg.html

(cl:defclass <mqueue-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0)
   (LP
    :reader LP
    :initarg :LP
    :type cl:fixnum
    :initform 0)
   (GN
    :reader GN
    :initarg :GN
    :type cl:fixnum
    :initform 0)
   (gps
    :reader gps
    :initarg :gps
    :type cl:string
    :initform "")
   (lps
    :reader lps
    :initarg :lps
    :type cl:string
    :initform "")
   (pw
    :reader pw
    :initarg :pw
    :type cl:string
    :initform "")
   (tx
    :reader tx
    :initarg :tx
    :type cl:float
    :initform 0.0)
   (ty
    :reader ty
    :initarg :ty
    :type cl:float
    :initform 0.0)
   (tz
    :reader tz
    :initarg :tz
    :type cl:float
    :initform 0.0)
   (rx
    :reader rx
    :initarg :rx
    :type cl:float
    :initform 0.0)
   (ry
    :reader ry
    :initarg :ry
    :type cl:float
    :initform 0.0)
   (rz
    :reader rz
    :initarg :rz
    :type cl:float
    :initform 0.0)
   (rw
    :reader rw
    :initarg :rw
    :type cl:float
    :initform 0.0)
   (prd
    :reader prd
    :initarg :prd
    :type cl:float
    :initform 0.0)
   (pra
    :reader pra
    :initarg :pra
    :type cl:float
    :initform 0.0)
   (psd
    :reader psd
    :initarg :psd
    :type cl:float
    :initform 0.0)
   (psa
    :reader psa
    :initarg :psa
    :type cl:float
    :initform 0.0)
   (prd1
    :reader prd1
    :initarg :prd1
    :type cl:float
    :initform 0.0)
   (pra1
    :reader pra1
    :initarg :pra1
    :type cl:float
    :initform 0.0)
   (psd1
    :reader psd1
    :initarg :psd1
    :type cl:float
    :initform 0.0)
   (psa1
    :reader psa1
    :initarg :psa1
    :type cl:float
    :initform 0.0)
   (align
    :reader align
    :initarg :align
    :type cl:float
    :initform 0.0)
   (autostart
    :reader autostart
    :initarg :autostart
    :type cl:float
    :initform 0.0))
)

(cl:defclass mqueue-request (<mqueue-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mqueue-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mqueue-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-srv:<mqueue-request> is deprecated: use htbot-srv:mqueue-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:cmd-val is deprecated.  Use htbot-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'LP-val :lambda-list '(m))
(cl:defmethod LP-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:LP-val is deprecated.  Use htbot-srv:LP instead.")
  (LP m))

(cl:ensure-generic-function 'GN-val :lambda-list '(m))
(cl:defmethod GN-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:GN-val is deprecated.  Use htbot-srv:GN instead.")
  (GN m))

(cl:ensure-generic-function 'gps-val :lambda-list '(m))
(cl:defmethod gps-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:gps-val is deprecated.  Use htbot-srv:gps instead.")
  (gps m))

(cl:ensure-generic-function 'lps-val :lambda-list '(m))
(cl:defmethod lps-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lps-val is deprecated.  Use htbot-srv:lps instead.")
  (lps m))

(cl:ensure-generic-function 'pw-val :lambda-list '(m))
(cl:defmethod pw-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:pw-val is deprecated.  Use htbot-srv:pw instead.")
  (pw m))

(cl:ensure-generic-function 'tx-val :lambda-list '(m))
(cl:defmethod tx-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:tx-val is deprecated.  Use htbot-srv:tx instead.")
  (tx m))

(cl:ensure-generic-function 'ty-val :lambda-list '(m))
(cl:defmethod ty-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:ty-val is deprecated.  Use htbot-srv:ty instead.")
  (ty m))

(cl:ensure-generic-function 'tz-val :lambda-list '(m))
(cl:defmethod tz-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:tz-val is deprecated.  Use htbot-srv:tz instead.")
  (tz m))

(cl:ensure-generic-function 'rx-val :lambda-list '(m))
(cl:defmethod rx-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:rx-val is deprecated.  Use htbot-srv:rx instead.")
  (rx m))

(cl:ensure-generic-function 'ry-val :lambda-list '(m))
(cl:defmethod ry-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:ry-val is deprecated.  Use htbot-srv:ry instead.")
  (ry m))

(cl:ensure-generic-function 'rz-val :lambda-list '(m))
(cl:defmethod rz-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:rz-val is deprecated.  Use htbot-srv:rz instead.")
  (rz m))

(cl:ensure-generic-function 'rw-val :lambda-list '(m))
(cl:defmethod rw-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:rw-val is deprecated.  Use htbot-srv:rw instead.")
  (rw m))

(cl:ensure-generic-function 'prd-val :lambda-list '(m))
(cl:defmethod prd-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:prd-val is deprecated.  Use htbot-srv:prd instead.")
  (prd m))

(cl:ensure-generic-function 'pra-val :lambda-list '(m))
(cl:defmethod pra-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:pra-val is deprecated.  Use htbot-srv:pra instead.")
  (pra m))

(cl:ensure-generic-function 'psd-val :lambda-list '(m))
(cl:defmethod psd-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:psd-val is deprecated.  Use htbot-srv:psd instead.")
  (psd m))

(cl:ensure-generic-function 'psa-val :lambda-list '(m))
(cl:defmethod psa-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:psa-val is deprecated.  Use htbot-srv:psa instead.")
  (psa m))

(cl:ensure-generic-function 'prd1-val :lambda-list '(m))
(cl:defmethod prd1-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:prd1-val is deprecated.  Use htbot-srv:prd1 instead.")
  (prd1 m))

(cl:ensure-generic-function 'pra1-val :lambda-list '(m))
(cl:defmethod pra1-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:pra1-val is deprecated.  Use htbot-srv:pra1 instead.")
  (pra1 m))

(cl:ensure-generic-function 'psd1-val :lambda-list '(m))
(cl:defmethod psd1-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:psd1-val is deprecated.  Use htbot-srv:psd1 instead.")
  (psd1 m))

(cl:ensure-generic-function 'psa1-val :lambda-list '(m))
(cl:defmethod psa1-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:psa1-val is deprecated.  Use htbot-srv:psa1 instead.")
  (psa1 m))

(cl:ensure-generic-function 'align-val :lambda-list '(m))
(cl:defmethod align-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:align-val is deprecated.  Use htbot-srv:align instead.")
  (align m))

(cl:ensure-generic-function 'autostart-val :lambda-list '(m))
(cl:defmethod autostart-val ((m <mqueue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:autostart-val is deprecated.  Use htbot-srv:autostart instead.")
  (autostart m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mqueue-request>) ostream)
  "Serializes a message object of type '<mqueue-request>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'LP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'GN)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gps))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lps))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pw))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ty))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ry))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'prd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pra))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psa))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'prd1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pra1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psd1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psa1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'align))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'autostart))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mqueue-request>) istream)
  "Deserializes a message object of type '<mqueue-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'LP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'GN) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gps) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'gps) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lps) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lps) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pw) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pw) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ty) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ry) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'prd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pra) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psa) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'prd1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pra1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psd1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psa1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'align) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'autostart) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mqueue-request>)))
  "Returns string type for a service object of type '<mqueue-request>"
  "htbot/mqueueRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mqueue-request)))
  "Returns string type for a service object of type 'mqueue-request"
  "htbot/mqueueRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mqueue-request>)))
  "Returns md5sum for a message object of type '<mqueue-request>"
  "8063f205c633b3b5b9493f7d4b8b6575")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mqueue-request)))
  "Returns md5sum for a message object of type 'mqueue-request"
  "8063f205c633b3b5b9493f7d4b8b6575")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mqueue-request>)))
  "Returns full string definition for message of type '<mqueue-request>"
  (cl:format cl:nil "int8 cmd~%int8 LP~%int8 GN~%string gps~%string lps~%string pw~%float32 tx~%float32 ty~%float32 tz~%float32 rx~%float32 ry~%float32 rz~%float32 rw~%float32 prd~%float32 pra~%float32 psd~%float32 psa~%float32 prd1~%float32 pra1~%float32 psd1~%float32 psa1~%float32 align~%float32 autostart~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mqueue-request)))
  "Returns full string definition for message of type 'mqueue-request"
  (cl:format cl:nil "int8 cmd~%int8 LP~%int8 GN~%string gps~%string lps~%string pw~%float32 tx~%float32 ty~%float32 tz~%float32 rx~%float32 ry~%float32 rz~%float32 rw~%float32 prd~%float32 pra~%float32 psd~%float32 psa~%float32 prd1~%float32 pra1~%float32 psd1~%float32 psa1~%float32 align~%float32 autostart~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mqueue-request>))
  (cl:+ 0
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'gps))
     4 (cl:length (cl:slot-value msg 'lps))
     4 (cl:length (cl:slot-value msg 'pw))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mqueue-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mqueue-request
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':LP (LP msg))
    (cl:cons ':GN (GN msg))
    (cl:cons ':gps (gps msg))
    (cl:cons ':lps (lps msg))
    (cl:cons ':pw (pw msg))
    (cl:cons ':tx (tx msg))
    (cl:cons ':ty (ty msg))
    (cl:cons ':tz (tz msg))
    (cl:cons ':rx (rx msg))
    (cl:cons ':ry (ry msg))
    (cl:cons ':rz (rz msg))
    (cl:cons ':rw (rw msg))
    (cl:cons ':prd (prd msg))
    (cl:cons ':pra (pra msg))
    (cl:cons ':psd (psd msg))
    (cl:cons ':psa (psa msg))
    (cl:cons ':prd1 (prd1 msg))
    (cl:cons ':pra1 (pra1 msg))
    (cl:cons ':psd1 (psd1 msg))
    (cl:cons ':psa1 (psa1 msg))
    (cl:cons ':align (align msg))
    (cl:cons ':autostart (autostart msg))
))
;//! \htmlinclude mqueue-response.msg.html

(cl:defclass <mqueue-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (tx
    :reader tx
    :initarg :tx
    :type cl:float
    :initform 0.0)
   (ty
    :reader ty
    :initarg :ty
    :type cl:float
    :initform 0.0)
   (tz
    :reader tz
    :initarg :tz
    :type cl:float
    :initform 0.0)
   (rx
    :reader rx
    :initarg :rx
    :type cl:float
    :initform 0.0)
   (ry
    :reader ry
    :initarg :ry
    :type cl:float
    :initform 0.0)
   (rz
    :reader rz
    :initarg :rz
    :type cl:float
    :initform 0.0)
   (rw
    :reader rw
    :initarg :rw
    :type cl:float
    :initform 0.0)
   (prd
    :reader prd
    :initarg :prd
    :type cl:float
    :initform 0.0)
   (pra
    :reader pra
    :initarg :pra
    :type cl:float
    :initform 0.0)
   (psd
    :reader psd
    :initarg :psd
    :type cl:float
    :initform 0.0)
   (psa
    :reader psa
    :initarg :psa
    :type cl:float
    :initform 0.0)
   (prd1
    :reader prd1
    :initarg :prd1
    :type cl:float
    :initform 0.0)
   (pra1
    :reader pra1
    :initarg :pra1
    :type cl:float
    :initform 0.0)
   (psd1
    :reader psd1
    :initarg :psd1
    :type cl:float
    :initform 0.0)
   (psa1
    :reader psa1
    :initarg :psa1
    :type cl:float
    :initform 0.0)
   (LP
    :reader LP
    :initarg :LP
    :type cl:fixnum
    :initform 0)
   (cLP
    :reader cLP
    :initarg :cLP
    :type cl:fixnum
    :initform 0)
   (nGP
    :reader nGP
    :initarg :nGP
    :type cl:fixnum
    :initform 0)
   (cGN
    :reader cGN
    :initarg :cGN
    :type cl:fixnum
    :initform 0)
   (nIQ
    :reader nIQ
    :initarg :nIQ
    :type cl:fixnum
    :initform 0)
   (marked
    :reader marked
    :initarg :marked
    :type cl:fixnum
    :initform 0)
   (gps
    :reader gps
    :initarg :gps
    :type cl:string
    :initform "")
   (lps
    :reader lps
    :initarg :lps
    :type cl:string
    :initform "")
   (lps1
    :reader lps1
    :initarg :lps1
    :type cl:string
    :initform "")
   (lps2
    :reader lps2
    :initarg :lps2
    :type cl:string
    :initform "")
   (lps3
    :reader lps3
    :initarg :lps3
    :type cl:string
    :initform "")
   (lps4
    :reader lps4
    :initarg :lps4
    :type cl:string
    :initform "")
   (lps5
    :reader lps5
    :initarg :lps5
    :type cl:string
    :initform "")
   (lps6
    :reader lps6
    :initarg :lps6
    :type cl:string
    :initform "")
   (lps7
    :reader lps7
    :initarg :lps7
    :type cl:string
    :initform "")
   (lps8
    :reader lps8
    :initarg :lps8
    :type cl:string
    :initform "")
   (marked1
    :reader marked1
    :initarg :marked1
    :type cl:fixnum
    :initform 0)
   (marked2
    :reader marked2
    :initarg :marked2
    :type cl:fixnum
    :initform 0)
   (marked3
    :reader marked3
    :initarg :marked3
    :type cl:fixnum
    :initform 0)
   (marked4
    :reader marked4
    :initarg :marked4
    :type cl:fixnum
    :initform 0)
   (marked5
    :reader marked5
    :initarg :marked5
    :type cl:fixnum
    :initform 0)
   (marked6
    :reader marked6
    :initarg :marked6
    :type cl:fixnum
    :initform 0)
   (marked7
    :reader marked7
    :initarg :marked7
    :type cl:fixnum
    :initform 0)
   (marked8
    :reader marked8
    :initarg :marked8
    :type cl:fixnum
    :initform 0)
   (align
    :reader align
    :initarg :align
    :type cl:float
    :initform 0.0)
   (autostart
    :reader autostart
    :initarg :autostart
    :type cl:float
    :initform 0.0))
)

(cl:defclass mqueue-response (<mqueue-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mqueue-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mqueue-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-srv:<mqueue-response> is deprecated: use htbot-srv:mqueue-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:status-val is deprecated.  Use htbot-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'tx-val :lambda-list '(m))
(cl:defmethod tx-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:tx-val is deprecated.  Use htbot-srv:tx instead.")
  (tx m))

(cl:ensure-generic-function 'ty-val :lambda-list '(m))
(cl:defmethod ty-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:ty-val is deprecated.  Use htbot-srv:ty instead.")
  (ty m))

(cl:ensure-generic-function 'tz-val :lambda-list '(m))
(cl:defmethod tz-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:tz-val is deprecated.  Use htbot-srv:tz instead.")
  (tz m))

(cl:ensure-generic-function 'rx-val :lambda-list '(m))
(cl:defmethod rx-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:rx-val is deprecated.  Use htbot-srv:rx instead.")
  (rx m))

(cl:ensure-generic-function 'ry-val :lambda-list '(m))
(cl:defmethod ry-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:ry-val is deprecated.  Use htbot-srv:ry instead.")
  (ry m))

(cl:ensure-generic-function 'rz-val :lambda-list '(m))
(cl:defmethod rz-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:rz-val is deprecated.  Use htbot-srv:rz instead.")
  (rz m))

(cl:ensure-generic-function 'rw-val :lambda-list '(m))
(cl:defmethod rw-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:rw-val is deprecated.  Use htbot-srv:rw instead.")
  (rw m))

(cl:ensure-generic-function 'prd-val :lambda-list '(m))
(cl:defmethod prd-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:prd-val is deprecated.  Use htbot-srv:prd instead.")
  (prd m))

(cl:ensure-generic-function 'pra-val :lambda-list '(m))
(cl:defmethod pra-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:pra-val is deprecated.  Use htbot-srv:pra instead.")
  (pra m))

(cl:ensure-generic-function 'psd-val :lambda-list '(m))
(cl:defmethod psd-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:psd-val is deprecated.  Use htbot-srv:psd instead.")
  (psd m))

(cl:ensure-generic-function 'psa-val :lambda-list '(m))
(cl:defmethod psa-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:psa-val is deprecated.  Use htbot-srv:psa instead.")
  (psa m))

(cl:ensure-generic-function 'prd1-val :lambda-list '(m))
(cl:defmethod prd1-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:prd1-val is deprecated.  Use htbot-srv:prd1 instead.")
  (prd1 m))

(cl:ensure-generic-function 'pra1-val :lambda-list '(m))
(cl:defmethod pra1-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:pra1-val is deprecated.  Use htbot-srv:pra1 instead.")
  (pra1 m))

(cl:ensure-generic-function 'psd1-val :lambda-list '(m))
(cl:defmethod psd1-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:psd1-val is deprecated.  Use htbot-srv:psd1 instead.")
  (psd1 m))

(cl:ensure-generic-function 'psa1-val :lambda-list '(m))
(cl:defmethod psa1-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:psa1-val is deprecated.  Use htbot-srv:psa1 instead.")
  (psa1 m))

(cl:ensure-generic-function 'LP-val :lambda-list '(m))
(cl:defmethod LP-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:LP-val is deprecated.  Use htbot-srv:LP instead.")
  (LP m))

(cl:ensure-generic-function 'cLP-val :lambda-list '(m))
(cl:defmethod cLP-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:cLP-val is deprecated.  Use htbot-srv:cLP instead.")
  (cLP m))

(cl:ensure-generic-function 'nGP-val :lambda-list '(m))
(cl:defmethod nGP-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:nGP-val is deprecated.  Use htbot-srv:nGP instead.")
  (nGP m))

(cl:ensure-generic-function 'cGN-val :lambda-list '(m))
(cl:defmethod cGN-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:cGN-val is deprecated.  Use htbot-srv:cGN instead.")
  (cGN m))

(cl:ensure-generic-function 'nIQ-val :lambda-list '(m))
(cl:defmethod nIQ-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:nIQ-val is deprecated.  Use htbot-srv:nIQ instead.")
  (nIQ m))

(cl:ensure-generic-function 'marked-val :lambda-list '(m))
(cl:defmethod marked-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:marked-val is deprecated.  Use htbot-srv:marked instead.")
  (marked m))

(cl:ensure-generic-function 'gps-val :lambda-list '(m))
(cl:defmethod gps-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:gps-val is deprecated.  Use htbot-srv:gps instead.")
  (gps m))

(cl:ensure-generic-function 'lps-val :lambda-list '(m))
(cl:defmethod lps-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lps-val is deprecated.  Use htbot-srv:lps instead.")
  (lps m))

(cl:ensure-generic-function 'lps1-val :lambda-list '(m))
(cl:defmethod lps1-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lps1-val is deprecated.  Use htbot-srv:lps1 instead.")
  (lps1 m))

(cl:ensure-generic-function 'lps2-val :lambda-list '(m))
(cl:defmethod lps2-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lps2-val is deprecated.  Use htbot-srv:lps2 instead.")
  (lps2 m))

(cl:ensure-generic-function 'lps3-val :lambda-list '(m))
(cl:defmethod lps3-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lps3-val is deprecated.  Use htbot-srv:lps3 instead.")
  (lps3 m))

(cl:ensure-generic-function 'lps4-val :lambda-list '(m))
(cl:defmethod lps4-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lps4-val is deprecated.  Use htbot-srv:lps4 instead.")
  (lps4 m))

(cl:ensure-generic-function 'lps5-val :lambda-list '(m))
(cl:defmethod lps5-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lps5-val is deprecated.  Use htbot-srv:lps5 instead.")
  (lps5 m))

(cl:ensure-generic-function 'lps6-val :lambda-list '(m))
(cl:defmethod lps6-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lps6-val is deprecated.  Use htbot-srv:lps6 instead.")
  (lps6 m))

(cl:ensure-generic-function 'lps7-val :lambda-list '(m))
(cl:defmethod lps7-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lps7-val is deprecated.  Use htbot-srv:lps7 instead.")
  (lps7 m))

(cl:ensure-generic-function 'lps8-val :lambda-list '(m))
(cl:defmethod lps8-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:lps8-val is deprecated.  Use htbot-srv:lps8 instead.")
  (lps8 m))

(cl:ensure-generic-function 'marked1-val :lambda-list '(m))
(cl:defmethod marked1-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:marked1-val is deprecated.  Use htbot-srv:marked1 instead.")
  (marked1 m))

(cl:ensure-generic-function 'marked2-val :lambda-list '(m))
(cl:defmethod marked2-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:marked2-val is deprecated.  Use htbot-srv:marked2 instead.")
  (marked2 m))

(cl:ensure-generic-function 'marked3-val :lambda-list '(m))
(cl:defmethod marked3-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:marked3-val is deprecated.  Use htbot-srv:marked3 instead.")
  (marked3 m))

(cl:ensure-generic-function 'marked4-val :lambda-list '(m))
(cl:defmethod marked4-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:marked4-val is deprecated.  Use htbot-srv:marked4 instead.")
  (marked4 m))

(cl:ensure-generic-function 'marked5-val :lambda-list '(m))
(cl:defmethod marked5-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:marked5-val is deprecated.  Use htbot-srv:marked5 instead.")
  (marked5 m))

(cl:ensure-generic-function 'marked6-val :lambda-list '(m))
(cl:defmethod marked6-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:marked6-val is deprecated.  Use htbot-srv:marked6 instead.")
  (marked6 m))

(cl:ensure-generic-function 'marked7-val :lambda-list '(m))
(cl:defmethod marked7-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:marked7-val is deprecated.  Use htbot-srv:marked7 instead.")
  (marked7 m))

(cl:ensure-generic-function 'marked8-val :lambda-list '(m))
(cl:defmethod marked8-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:marked8-val is deprecated.  Use htbot-srv:marked8 instead.")
  (marked8 m))

(cl:ensure-generic-function 'align-val :lambda-list '(m))
(cl:defmethod align-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:align-val is deprecated.  Use htbot-srv:align instead.")
  (align m))

(cl:ensure-generic-function 'autostart-val :lambda-list '(m))
(cl:defmethod autostart-val ((m <mqueue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-srv:autostart-val is deprecated.  Use htbot-srv:autostart instead.")
  (autostart m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mqueue-response>) ostream)
  "Serializes a message object of type '<mqueue-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ty))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ry))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'prd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pra))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psa))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'prd1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pra1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psd1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psa1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'LP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cLP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'nGP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cGN)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'nIQ)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'marked)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gps))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lps))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lps1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lps1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lps2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lps2))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lps3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lps3))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lps4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lps4))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lps5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lps5))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lps6))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lps6))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lps7))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lps7))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lps8))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lps8))
  (cl:let* ((signed (cl:slot-value msg 'marked1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'marked2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'marked3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'marked4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'marked5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'marked6)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'marked7)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'marked8)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'align))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'autostart))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mqueue-response>) istream)
  "Deserializes a message object of type '<mqueue-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ty) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ry) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'prd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pra) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psa) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'prd1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pra1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psd1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psa1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'LP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cLP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nGP) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cGN) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nIQ) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marked) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gps) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'gps) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lps) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lps) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lps1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lps1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lps2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lps2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lps3) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lps3) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lps4) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lps4) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lps5) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lps5) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lps6) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lps6) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lps7) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lps7) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lps8) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lps8) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marked1) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marked2) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marked3) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marked4) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marked5) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marked6) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marked7) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marked8) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'align) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'autostart) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mqueue-response>)))
  "Returns string type for a service object of type '<mqueue-response>"
  "htbot/mqueueResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mqueue-response)))
  "Returns string type for a service object of type 'mqueue-response"
  "htbot/mqueueResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mqueue-response>)))
  "Returns md5sum for a message object of type '<mqueue-response>"
  "8063f205c633b3b5b9493f7d4b8b6575")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mqueue-response)))
  "Returns md5sum for a message object of type 'mqueue-response"
  "8063f205c633b3b5b9493f7d4b8b6575")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mqueue-response>)))
  "Returns full string definition for message of type '<mqueue-response>"
  (cl:format cl:nil "int8 status~%float32 tx~%float32 ty~%float32 tz~%float32 rx~%float32 ry~%float32 rz~%float32 rw~%float32 prd~%float32 pra~%float32 psd~%float32 psa~%float32 prd1~%float32 pra1~%float32 psd1~%float32 psa1~%int8 LP~%int8 cLP~%int8 nGP~%int8 cGN~%int8 nIQ~%int8 marked~%string gps~%string lps~%string lps1~%string lps2~%string lps3~%string lps4~%string lps5~%string lps6~%string lps7~%string lps8~%int8 marked1~%int8 marked2~%int8 marked3~%int8 marked4~%int8 marked5~%int8 marked6~%int8 marked7~%int8 marked8~%float32 align~%float32 autostart~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mqueue-response)))
  "Returns full string definition for message of type 'mqueue-response"
  (cl:format cl:nil "int8 status~%float32 tx~%float32 ty~%float32 tz~%float32 rx~%float32 ry~%float32 rz~%float32 rw~%float32 prd~%float32 pra~%float32 psd~%float32 psa~%float32 prd1~%float32 pra1~%float32 psd1~%float32 psa1~%int8 LP~%int8 cLP~%int8 nGP~%int8 cGN~%int8 nIQ~%int8 marked~%string gps~%string lps~%string lps1~%string lps2~%string lps3~%string lps4~%string lps5~%string lps6~%string lps7~%string lps8~%int8 marked1~%int8 marked2~%int8 marked3~%int8 marked4~%int8 marked5~%int8 marked6~%int8 marked7~%int8 marked8~%float32 align~%float32 autostart~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mqueue-response>))
  (cl:+ 0
     1
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     1
     1
     1
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'gps))
     4 (cl:length (cl:slot-value msg 'lps))
     4 (cl:length (cl:slot-value msg 'lps1))
     4 (cl:length (cl:slot-value msg 'lps2))
     4 (cl:length (cl:slot-value msg 'lps3))
     4 (cl:length (cl:slot-value msg 'lps4))
     4 (cl:length (cl:slot-value msg 'lps5))
     4 (cl:length (cl:slot-value msg 'lps6))
     4 (cl:length (cl:slot-value msg 'lps7))
     4 (cl:length (cl:slot-value msg 'lps8))
     1
     1
     1
     1
     1
     1
     1
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mqueue-response>))
  "Converts a ROS message object to a list"
  (cl:list 'mqueue-response
    (cl:cons ':status (status msg))
    (cl:cons ':tx (tx msg))
    (cl:cons ':ty (ty msg))
    (cl:cons ':tz (tz msg))
    (cl:cons ':rx (rx msg))
    (cl:cons ':ry (ry msg))
    (cl:cons ':rz (rz msg))
    (cl:cons ':rw (rw msg))
    (cl:cons ':prd (prd msg))
    (cl:cons ':pra (pra msg))
    (cl:cons ':psd (psd msg))
    (cl:cons ':psa (psa msg))
    (cl:cons ':prd1 (prd1 msg))
    (cl:cons ':pra1 (pra1 msg))
    (cl:cons ':psd1 (psd1 msg))
    (cl:cons ':psa1 (psa1 msg))
    (cl:cons ':LP (LP msg))
    (cl:cons ':cLP (cLP msg))
    (cl:cons ':nGP (nGP msg))
    (cl:cons ':cGN (cGN msg))
    (cl:cons ':nIQ (nIQ msg))
    (cl:cons ':marked (marked msg))
    (cl:cons ':gps (gps msg))
    (cl:cons ':lps (lps msg))
    (cl:cons ':lps1 (lps1 msg))
    (cl:cons ':lps2 (lps2 msg))
    (cl:cons ':lps3 (lps3 msg))
    (cl:cons ':lps4 (lps4 msg))
    (cl:cons ':lps5 (lps5 msg))
    (cl:cons ':lps6 (lps6 msg))
    (cl:cons ':lps7 (lps7 msg))
    (cl:cons ':lps8 (lps8 msg))
    (cl:cons ':marked1 (marked1 msg))
    (cl:cons ':marked2 (marked2 msg))
    (cl:cons ':marked3 (marked3 msg))
    (cl:cons ':marked4 (marked4 msg))
    (cl:cons ':marked5 (marked5 msg))
    (cl:cons ':marked6 (marked6 msg))
    (cl:cons ':marked7 (marked7 msg))
    (cl:cons ':marked8 (marked8 msg))
    (cl:cons ':align (align msg))
    (cl:cons ':autostart (autostart msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'mqueue)))
  'mqueue-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'mqueue)))
  'mqueue-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mqueue)))
  "Returns string type for a service object of type '<mqueue>"
  "htbot/mqueue")