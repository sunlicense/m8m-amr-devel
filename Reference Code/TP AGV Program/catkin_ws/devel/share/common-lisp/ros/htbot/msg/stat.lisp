; Auto-generated. Do not edit!


(cl:in-package htbot-msg)


;//! \htmlinclude stat.msg.html

(cl:defclass <stat> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0)
   (trip
    :reader trip
    :initarg :trip
    :type cl:fixnum
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:fixnum
    :initform 0)
   (maxspeed
    :reader maxspeed
    :initarg :maxspeed
    :type cl:float
    :initform 0.0)
   (minspeed
    :reader minspeed
    :initarg :minspeed
    :type cl:float
    :initform 0.0)
   (avgspeed
    :reader avgspeed
    :initarg :avgspeed
    :type cl:float
    :initform 0.0)
   (curspeed
    :reader curspeed
    :initarg :curspeed
    :type cl:float
    :initform 0.0)
   (totalruntime
    :reader totalruntime
    :initarg :totalruntime
    :type cl:float
    :initform 0.0)
   (totaltriptime
    :reader totaltriptime
    :initarg :totaltriptime
    :type cl:float
    :initform 0.0)
   (totaldowntime
    :reader totaldowntime
    :initarg :totaldowntime
    :type cl:float
    :initform 0.0)
   (totalidletime
    :reader totalidletime
    :initarg :totalidletime
    :type cl:float
    :initform 0.0)
   (uptime
    :reader uptime
    :initarg :uptime
    :type cl:float
    :initform 0.0)
   (totaltripdist
    :reader totaltripdist
    :initarg :totaltripdist
    :type cl:float
    :initform 0.0)
   (avgvolt
    :reader avgvolt
    :initarg :avgvolt
    :type cl:float
    :initform 0.0)
   (avgcurr
    :reader avgcurr
    :initarg :avgcurr
    :type cl:float
    :initform 0.0)
   (maxcurr
    :reader maxcurr
    :initarg :maxcurr
    :type cl:float
    :initform 0.0)
   (mincurr
    :reader mincurr
    :initarg :mincurr
    :type cl:float
    :initform 0.0)
   (amphr
    :reader amphr
    :initarg :amphr
    :type cl:float
    :initform 0.0)
   (batlevel
    :reader batlevel
    :initarg :batlevel
    :type cl:float
    :initform 0.0)
   (tnumtrip
    :reader tnumtrip
    :initarg :tnumtrip
    :type cl:fixnum
    :initform 0)
   (clearops
    :reader clearops
    :initarg :clearops
    :type cl:fixnum
    :initform 0)
   (estop
    :reader estop
    :initarg :estop
    :type cl:fixnum
    :initform 0)
   (motordisable
    :reader motordisable
    :initarg :motordisable
    :type cl:fixnum
    :initform 0)
   (alarmlog
    :reader alarmlog
    :initarg :alarmlog
    :type cl:string
    :initform "")
   (heartbeat
    :reader heartbeat
    :initarg :heartbeat
    :type cl:fixnum
    :initform 0))
)

(cl:defclass stat (<stat>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <stat>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'stat)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name htbot-msg:<stat> is deprecated: use htbot-msg:stat instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:speed-val is deprecated.  Use htbot-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'trip-val :lambda-list '(m))
(cl:defmethod trip-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:trip-val is deprecated.  Use htbot-msg:trip instead.")
  (trip m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:time-val is deprecated.  Use htbot-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'maxspeed-val :lambda-list '(m))
(cl:defmethod maxspeed-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:maxspeed-val is deprecated.  Use htbot-msg:maxspeed instead.")
  (maxspeed m))

(cl:ensure-generic-function 'minspeed-val :lambda-list '(m))
(cl:defmethod minspeed-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:minspeed-val is deprecated.  Use htbot-msg:minspeed instead.")
  (minspeed m))

(cl:ensure-generic-function 'avgspeed-val :lambda-list '(m))
(cl:defmethod avgspeed-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:avgspeed-val is deprecated.  Use htbot-msg:avgspeed instead.")
  (avgspeed m))

(cl:ensure-generic-function 'curspeed-val :lambda-list '(m))
(cl:defmethod curspeed-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:curspeed-val is deprecated.  Use htbot-msg:curspeed instead.")
  (curspeed m))

(cl:ensure-generic-function 'totalruntime-val :lambda-list '(m))
(cl:defmethod totalruntime-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:totalruntime-val is deprecated.  Use htbot-msg:totalruntime instead.")
  (totalruntime m))

(cl:ensure-generic-function 'totaltriptime-val :lambda-list '(m))
(cl:defmethod totaltriptime-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:totaltriptime-val is deprecated.  Use htbot-msg:totaltriptime instead.")
  (totaltriptime m))

(cl:ensure-generic-function 'totaldowntime-val :lambda-list '(m))
(cl:defmethod totaldowntime-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:totaldowntime-val is deprecated.  Use htbot-msg:totaldowntime instead.")
  (totaldowntime m))

(cl:ensure-generic-function 'totalidletime-val :lambda-list '(m))
(cl:defmethod totalidletime-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:totalidletime-val is deprecated.  Use htbot-msg:totalidletime instead.")
  (totalidletime m))

(cl:ensure-generic-function 'uptime-val :lambda-list '(m))
(cl:defmethod uptime-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:uptime-val is deprecated.  Use htbot-msg:uptime instead.")
  (uptime m))

(cl:ensure-generic-function 'totaltripdist-val :lambda-list '(m))
(cl:defmethod totaltripdist-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:totaltripdist-val is deprecated.  Use htbot-msg:totaltripdist instead.")
  (totaltripdist m))

(cl:ensure-generic-function 'avgvolt-val :lambda-list '(m))
(cl:defmethod avgvolt-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:avgvolt-val is deprecated.  Use htbot-msg:avgvolt instead.")
  (avgvolt m))

(cl:ensure-generic-function 'avgcurr-val :lambda-list '(m))
(cl:defmethod avgcurr-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:avgcurr-val is deprecated.  Use htbot-msg:avgcurr instead.")
  (avgcurr m))

(cl:ensure-generic-function 'maxcurr-val :lambda-list '(m))
(cl:defmethod maxcurr-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:maxcurr-val is deprecated.  Use htbot-msg:maxcurr instead.")
  (maxcurr m))

(cl:ensure-generic-function 'mincurr-val :lambda-list '(m))
(cl:defmethod mincurr-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:mincurr-val is deprecated.  Use htbot-msg:mincurr instead.")
  (mincurr m))

(cl:ensure-generic-function 'amphr-val :lambda-list '(m))
(cl:defmethod amphr-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:amphr-val is deprecated.  Use htbot-msg:amphr instead.")
  (amphr m))

(cl:ensure-generic-function 'batlevel-val :lambda-list '(m))
(cl:defmethod batlevel-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:batlevel-val is deprecated.  Use htbot-msg:batlevel instead.")
  (batlevel m))

(cl:ensure-generic-function 'tnumtrip-val :lambda-list '(m))
(cl:defmethod tnumtrip-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:tnumtrip-val is deprecated.  Use htbot-msg:tnumtrip instead.")
  (tnumtrip m))

(cl:ensure-generic-function 'clearops-val :lambda-list '(m))
(cl:defmethod clearops-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:clearops-val is deprecated.  Use htbot-msg:clearops instead.")
  (clearops m))

(cl:ensure-generic-function 'estop-val :lambda-list '(m))
(cl:defmethod estop-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:estop-val is deprecated.  Use htbot-msg:estop instead.")
  (estop m))

(cl:ensure-generic-function 'motordisable-val :lambda-list '(m))
(cl:defmethod motordisable-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:motordisable-val is deprecated.  Use htbot-msg:motordisable instead.")
  (motordisable m))

(cl:ensure-generic-function 'alarmlog-val :lambda-list '(m))
(cl:defmethod alarmlog-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:alarmlog-val is deprecated.  Use htbot-msg:alarmlog instead.")
  (alarmlog m))

(cl:ensure-generic-function 'heartbeat-val :lambda-list '(m))
(cl:defmethod heartbeat-val ((m <stat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader htbot-msg:heartbeat-val is deprecated.  Use htbot-msg:heartbeat instead.")
  (heartbeat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <stat>) ostream)
  "Serializes a message object of type '<stat>"
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'trip)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'time)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maxspeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'minspeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'avgspeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'curspeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'totalruntime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'totaltriptime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'totaldowntime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'totalidletime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uptime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'totaltripdist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'avgvolt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'avgcurr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maxcurr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mincurr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'amphr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'batlevel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'tnumtrip)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'clearops)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'estop)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motordisable)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'alarmlog))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'alarmlog))
  (cl:let* ((signed (cl:slot-value msg 'heartbeat)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <stat>) istream)
  "Deserializes a message object of type '<stat>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'trip) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxspeed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minspeed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'avgspeed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'curspeed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'totalruntime) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'totaltriptime) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'totaldowntime) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'totalidletime) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uptime) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'totaltripdist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'avgvolt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'avgcurr) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxcurr) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mincurr) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amphr) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'batlevel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tnumtrip) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'clearops) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'estop) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motordisable) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'alarmlog) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'alarmlog) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'heartbeat) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<stat>)))
  "Returns string type for a message object of type '<stat>"
  "htbot/stat")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'stat)))
  "Returns string type for a message object of type 'stat"
  "htbot/stat")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<stat>)))
  "Returns md5sum for a message object of type '<stat>"
  "af7104aa5ac2b7ecb587449c2a96bf9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'stat)))
  "Returns md5sum for a message object of type 'stat"
  "af7104aa5ac2b7ecb587449c2a96bf9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<stat>)))
  "Returns full string definition for message of type '<stat>"
  (cl:format cl:nil "int8 speed~%int8 trip~%int8 time~%float32 maxspeed~%float32 minspeed~%float32 avgspeed~%float32 curspeed~%float32 totalruntime~%float32 totaltriptime~%float32 totaldowntime~%float32 totalidletime~%float32 uptime~%float32 totaltripdist~%float32 avgvolt~%float32 avgcurr~%float32 maxcurr~%float32 mincurr~%float32 amphr~%float32 batlevel~%int16 tnumtrip~%int16 clearops~%int16 estop~%int16 motordisable~%string alarmlog~%int16 heartbeat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'stat)))
  "Returns full string definition for message of type 'stat"
  (cl:format cl:nil "int8 speed~%int8 trip~%int8 time~%float32 maxspeed~%float32 minspeed~%float32 avgspeed~%float32 curspeed~%float32 totalruntime~%float32 totaltriptime~%float32 totaldowntime~%float32 totalidletime~%float32 uptime~%float32 totaltripdist~%float32 avgvolt~%float32 avgcurr~%float32 maxcurr~%float32 mincurr~%float32 amphr~%float32 batlevel~%int16 tnumtrip~%int16 clearops~%int16 estop~%int16 motordisable~%string alarmlog~%int16 heartbeat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <stat>))
  (cl:+ 0
     1
     1
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
     4
     2
     2
     2
     2
     4 (cl:length (cl:slot-value msg 'alarmlog))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <stat>))
  "Converts a ROS message object to a list"
  (cl:list 'stat
    (cl:cons ':speed (speed msg))
    (cl:cons ':trip (trip msg))
    (cl:cons ':time (time msg))
    (cl:cons ':maxspeed (maxspeed msg))
    (cl:cons ':minspeed (minspeed msg))
    (cl:cons ':avgspeed (avgspeed msg))
    (cl:cons ':curspeed (curspeed msg))
    (cl:cons ':totalruntime (totalruntime msg))
    (cl:cons ':totaltriptime (totaltriptime msg))
    (cl:cons ':totaldowntime (totaldowntime msg))
    (cl:cons ':totalidletime (totalidletime msg))
    (cl:cons ':uptime (uptime msg))
    (cl:cons ':totaltripdist (totaltripdist msg))
    (cl:cons ':avgvolt (avgvolt msg))
    (cl:cons ':avgcurr (avgcurr msg))
    (cl:cons ':maxcurr (maxcurr msg))
    (cl:cons ':mincurr (mincurr msg))
    (cl:cons ':amphr (amphr msg))
    (cl:cons ':batlevel (batlevel msg))
    (cl:cons ':tnumtrip (tnumtrip msg))
    (cl:cons ':clearops (clearops msg))
    (cl:cons ':estop (estop msg))
    (cl:cons ':motordisable (motordisable msg))
    (cl:cons ':alarmlog (alarmlog msg))
    (cl:cons ':heartbeat (heartbeat msg))
))
