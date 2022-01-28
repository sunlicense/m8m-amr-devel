;; Auto-generated. Do not edit!


(when (boundp 'videocontrol::dyna)
  (if (not (find-package "VIDEOCONTROL"))
    (make-package "VIDEOCONTROL"))
  (shadow 'dyna (find-package "VIDEOCONTROL")))
(unless (find-package "VIDEOCONTROL::DYNA")
  (make-package "VIDEOCONTROL::DYNA"))

(in-package "ROS")
;;//! \htmlinclude dyna.msg.html


(defclass videocontrol::dyna
  :super ros::object
  :slots (_paramid _intValue _doubleValue _strValue _boolValue ))

(defmethod videocontrol::dyna
  (:init
   (&key
    ((:paramid __paramid) 0)
    ((:intValue __intValue) 0)
    ((:doubleValue __doubleValue) 0.0)
    ((:strValue __strValue) "")
    ((:boolValue __boolValue) nil)
    )
   (send-super :init)
   (setq _paramid (round __paramid))
   (setq _intValue (round __intValue))
   (setq _doubleValue (float __doubleValue))
   (setq _strValue (string __strValue))
   (setq _boolValue __boolValue)
   self)
  (:paramid
   (&optional __paramid)
   (if __paramid (setq _paramid __paramid)) _paramid)
  (:intValue
   (&optional __intValue)
   (if __intValue (setq _intValue __intValue)) _intValue)
  (:doubleValue
   (&optional __doubleValue)
   (if __doubleValue (setq _doubleValue __doubleValue)) _doubleValue)
  (:strValue
   (&optional __strValue)
   (if __strValue (setq _strValue __strValue)) _strValue)
  (:boolValue
   (&optional __boolValue)
   (if __boolValue (setq _boolValue __boolValue)) _boolValue)
  (:serialization-length
   ()
   (+
    ;; int8 _paramid
    1
    ;; int32 _intValue
    4
    ;; float64 _doubleValue
    8
    ;; string _strValue
    4 (length _strValue)
    ;; bool _boolValue
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _paramid
       (write-byte _paramid s)
     ;; int32 _intValue
       (write-long _intValue s)
     ;; float64 _doubleValue
       (sys::poke _doubleValue (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _strValue
       (write-long (length _strValue) s) (princ _strValue s)
     ;; bool _boolValue
       (if _boolValue (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _paramid
     (setq _paramid (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _paramid 127) (setq _paramid (- _paramid 256)))
   ;; int32 _intValue
     (setq _intValue (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _doubleValue
     (setq _doubleValue (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _strValue
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _strValue (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _boolValue
     (setq _boolValue (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get videocontrol::dyna :md5sum-) "6d03e42ff56dea7eeef77a3e537361b9")
(setf (get videocontrol::dyna :datatype-) "videocontrol/dyna")
(setf (get videocontrol::dyna :definition-)
      "int8 paramid
int32 intValue
float64 doubleValue
string strValue
bool boolValue

")



(provide :videocontrol/dyna "6d03e42ff56dea7eeef77a3e537361b9")


