;; Auto-generated. Do not edit!


(when (boundp 'htbot::stat)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'stat (find-package "HTBOT")))
(unless (find-package "HTBOT::STAT")
  (make-package "HTBOT::STAT"))

(in-package "ROS")
;;//! \htmlinclude stat.msg.html


(defclass htbot::stat
  :super ros::object
  :slots (_speed _trip _time _maxspeed _minspeed _avgspeed _curspeed _totalruntime _totaltriptime _totaldowntime _totalidletime _uptime _totaltripdist _avgvolt _avgcurr _maxcurr _mincurr _amphr _batlevel _tnumtrip _clearops _estop _motordisable _alarmlog _heartbeat ))

(defmethod htbot::stat
  (:init
   (&key
    ((:speed __speed) 0)
    ((:trip __trip) 0)
    ((:time __time) 0)
    ((:maxspeed __maxspeed) 0.0)
    ((:minspeed __minspeed) 0.0)
    ((:avgspeed __avgspeed) 0.0)
    ((:curspeed __curspeed) 0.0)
    ((:totalruntime __totalruntime) 0.0)
    ((:totaltriptime __totaltriptime) 0.0)
    ((:totaldowntime __totaldowntime) 0.0)
    ((:totalidletime __totalidletime) 0.0)
    ((:uptime __uptime) 0.0)
    ((:totaltripdist __totaltripdist) 0.0)
    ((:avgvolt __avgvolt) 0.0)
    ((:avgcurr __avgcurr) 0.0)
    ((:maxcurr __maxcurr) 0.0)
    ((:mincurr __mincurr) 0.0)
    ((:amphr __amphr) 0.0)
    ((:batlevel __batlevel) 0.0)
    ((:tnumtrip __tnumtrip) 0)
    ((:clearops __clearops) 0)
    ((:estop __estop) 0)
    ((:motordisable __motordisable) 0)
    ((:alarmlog __alarmlog) "")
    ((:heartbeat __heartbeat) 0)
    )
   (send-super :init)
   (setq _speed (round __speed))
   (setq _trip (round __trip))
   (setq _time (round __time))
   (setq _maxspeed (float __maxspeed))
   (setq _minspeed (float __minspeed))
   (setq _avgspeed (float __avgspeed))
   (setq _curspeed (float __curspeed))
   (setq _totalruntime (float __totalruntime))
   (setq _totaltriptime (float __totaltriptime))
   (setq _totaldowntime (float __totaldowntime))
   (setq _totalidletime (float __totalidletime))
   (setq _uptime (float __uptime))
   (setq _totaltripdist (float __totaltripdist))
   (setq _avgvolt (float __avgvolt))
   (setq _avgcurr (float __avgcurr))
   (setq _maxcurr (float __maxcurr))
   (setq _mincurr (float __mincurr))
   (setq _amphr (float __amphr))
   (setq _batlevel (float __batlevel))
   (setq _tnumtrip (round __tnumtrip))
   (setq _clearops (round __clearops))
   (setq _estop (round __estop))
   (setq _motordisable (round __motordisable))
   (setq _alarmlog (string __alarmlog))
   (setq _heartbeat (round __heartbeat))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:trip
   (&optional __trip)
   (if __trip (setq _trip __trip)) _trip)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:maxspeed
   (&optional __maxspeed)
   (if __maxspeed (setq _maxspeed __maxspeed)) _maxspeed)
  (:minspeed
   (&optional __minspeed)
   (if __minspeed (setq _minspeed __minspeed)) _minspeed)
  (:avgspeed
   (&optional __avgspeed)
   (if __avgspeed (setq _avgspeed __avgspeed)) _avgspeed)
  (:curspeed
   (&optional __curspeed)
   (if __curspeed (setq _curspeed __curspeed)) _curspeed)
  (:totalruntime
   (&optional __totalruntime)
   (if __totalruntime (setq _totalruntime __totalruntime)) _totalruntime)
  (:totaltriptime
   (&optional __totaltriptime)
   (if __totaltriptime (setq _totaltriptime __totaltriptime)) _totaltriptime)
  (:totaldowntime
   (&optional __totaldowntime)
   (if __totaldowntime (setq _totaldowntime __totaldowntime)) _totaldowntime)
  (:totalidletime
   (&optional __totalidletime)
   (if __totalidletime (setq _totalidletime __totalidletime)) _totalidletime)
  (:uptime
   (&optional __uptime)
   (if __uptime (setq _uptime __uptime)) _uptime)
  (:totaltripdist
   (&optional __totaltripdist)
   (if __totaltripdist (setq _totaltripdist __totaltripdist)) _totaltripdist)
  (:avgvolt
   (&optional __avgvolt)
   (if __avgvolt (setq _avgvolt __avgvolt)) _avgvolt)
  (:avgcurr
   (&optional __avgcurr)
   (if __avgcurr (setq _avgcurr __avgcurr)) _avgcurr)
  (:maxcurr
   (&optional __maxcurr)
   (if __maxcurr (setq _maxcurr __maxcurr)) _maxcurr)
  (:mincurr
   (&optional __mincurr)
   (if __mincurr (setq _mincurr __mincurr)) _mincurr)
  (:amphr
   (&optional __amphr)
   (if __amphr (setq _amphr __amphr)) _amphr)
  (:batlevel
   (&optional __batlevel)
   (if __batlevel (setq _batlevel __batlevel)) _batlevel)
  (:tnumtrip
   (&optional __tnumtrip)
   (if __tnumtrip (setq _tnumtrip __tnumtrip)) _tnumtrip)
  (:clearops
   (&optional __clearops)
   (if __clearops (setq _clearops __clearops)) _clearops)
  (:estop
   (&optional __estop)
   (if __estop (setq _estop __estop)) _estop)
  (:motordisable
   (&optional __motordisable)
   (if __motordisable (setq _motordisable __motordisable)) _motordisable)
  (:alarmlog
   (&optional __alarmlog)
   (if __alarmlog (setq _alarmlog __alarmlog)) _alarmlog)
  (:heartbeat
   (&optional __heartbeat)
   (if __heartbeat (setq _heartbeat __heartbeat)) _heartbeat)
  (:serialization-length
   ()
   (+
    ;; int8 _speed
    1
    ;; int8 _trip
    1
    ;; int8 _time
    1
    ;; float32 _maxspeed
    4
    ;; float32 _minspeed
    4
    ;; float32 _avgspeed
    4
    ;; float32 _curspeed
    4
    ;; float32 _totalruntime
    4
    ;; float32 _totaltriptime
    4
    ;; float32 _totaldowntime
    4
    ;; float32 _totalidletime
    4
    ;; float32 _uptime
    4
    ;; float32 _totaltripdist
    4
    ;; float32 _avgvolt
    4
    ;; float32 _avgcurr
    4
    ;; float32 _maxcurr
    4
    ;; float32 _mincurr
    4
    ;; float32 _amphr
    4
    ;; float32 _batlevel
    4
    ;; int16 _tnumtrip
    2
    ;; int16 _clearops
    2
    ;; int16 _estop
    2
    ;; int16 _motordisable
    2
    ;; string _alarmlog
    4 (length _alarmlog)
    ;; int16 _heartbeat
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _speed
       (write-byte _speed s)
     ;; int8 _trip
       (write-byte _trip s)
     ;; int8 _time
       (write-byte _time s)
     ;; float32 _maxspeed
       (sys::poke _maxspeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _minspeed
       (sys::poke _minspeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _avgspeed
       (sys::poke _avgspeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _curspeed
       (sys::poke _curspeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _totalruntime
       (sys::poke _totalruntime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _totaltriptime
       (sys::poke _totaltriptime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _totaldowntime
       (sys::poke _totaldowntime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _totalidletime
       (sys::poke _totalidletime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _uptime
       (sys::poke _uptime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _totaltripdist
       (sys::poke _totaltripdist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _avgvolt
       (sys::poke _avgvolt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _avgcurr
       (sys::poke _avgcurr (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _maxcurr
       (sys::poke _maxcurr (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mincurr
       (sys::poke _mincurr (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _amphr
       (sys::poke _amphr (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _batlevel
       (sys::poke _batlevel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int16 _tnumtrip
       (write-word _tnumtrip s)
     ;; int16 _clearops
       (write-word _clearops s)
     ;; int16 _estop
       (write-word _estop s)
     ;; int16 _motordisable
       (write-word _motordisable s)
     ;; string _alarmlog
       (write-long (length _alarmlog) s) (princ _alarmlog s)
     ;; int16 _heartbeat
       (write-word _heartbeat s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _speed
     (setq _speed (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _speed 127) (setq _speed (- _speed 256)))
   ;; int8 _trip
     (setq _trip (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _trip 127) (setq _trip (- _trip 256)))
   ;; int8 _time
     (setq _time (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _time 127) (setq _time (- _time 256)))
   ;; float32 _maxspeed
     (setq _maxspeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _minspeed
     (setq _minspeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _avgspeed
     (setq _avgspeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _curspeed
     (setq _curspeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _totalruntime
     (setq _totalruntime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _totaltriptime
     (setq _totaltriptime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _totaldowntime
     (setq _totaldowntime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _totalidletime
     (setq _totalidletime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _uptime
     (setq _uptime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _totaltripdist
     (setq _totaltripdist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _avgvolt
     (setq _avgvolt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _avgcurr
     (setq _avgcurr (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _maxcurr
     (setq _maxcurr (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mincurr
     (setq _mincurr (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _amphr
     (setq _amphr (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _batlevel
     (setq _batlevel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int16 _tnumtrip
     (setq _tnumtrip (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _clearops
     (setq _clearops (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _estop
     (setq _estop (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motordisable
     (setq _motordisable (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _alarmlog
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _alarmlog (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _heartbeat
     (setq _heartbeat (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get htbot::stat :md5sum-) "af7104aa5ac2b7ecb587449c2a96bf9f")
(setf (get htbot::stat :datatype-) "htbot/stat")
(setf (get htbot::stat :definition-)
      "int8 speed
int8 trip
int8 time
float32 maxspeed
float32 minspeed
float32 avgspeed
float32 curspeed
float32 totalruntime
float32 totaltriptime
float32 totaldowntime
float32 totalidletime
float32 uptime
float32 totaltripdist
float32 avgvolt
float32 avgcurr
float32 maxcurr
float32 mincurr
float32 amphr
float32 batlevel
int16 tnumtrip
int16 clearops
int16 estop
int16 motordisable
string alarmlog
int16 heartbeat

")



(provide :htbot/stat "af7104aa5ac2b7ecb587449c2a96bf9f")


