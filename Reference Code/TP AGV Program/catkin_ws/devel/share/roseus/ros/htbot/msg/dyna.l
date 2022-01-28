;; Auto-generated. Do not edit!


(when (boundp 'htbot::dyna)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'dyna (find-package "HTBOT")))
(unless (find-package "HTBOT::DYNA")
  (make-package "HTBOT::DYNA"))

(in-package "ROS")
;;//! \htmlinclude dyna.msg.html


(defclass htbot::dyna
  :super ros::object
  :slots (_paramid _intValue _doubleValue _strValue _boolValue _ftprintno _footprintlist ))

(defmethod htbot::dyna
  (:init
   (&key
    ((:paramid __paramid) 0)
    ((:intValue __intValue) 0)
    ((:doubleValue __doubleValue) 0.0)
    ((:strValue __strValue) "")
    ((:boolValue __boolValue) nil)
    ((:ftprintno __ftprintno) 0)
    ((:footprintlist __footprintlist) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _paramid (round __paramid))
   (setq _intValue (round __intValue))
   (setq _doubleValue (float __doubleValue))
   (setq _strValue (string __strValue))
   (setq _boolValue __boolValue)
   (setq _ftprintno (round __ftprintno))
   (setq _footprintlist __footprintlist)
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
  (:ftprintno
   (&optional __ftprintno)
   (if __ftprintno (setq _ftprintno __ftprintno)) _ftprintno)
  (:footprintlist
   (&optional __footprintlist)
   (if __footprintlist (setq _footprintlist __footprintlist)) _footprintlist)
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
    ;; int8 _ftprintno
    1
    ;; float64[] _footprintlist
    (* 8    (length _footprintlist)) 4
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
     ;; int8 _ftprintno
       (write-byte _ftprintno s)
     ;; float64[] _footprintlist
     (write-long (length _footprintlist) s)
     (dotimes (i (length _footprintlist))
       (sys::poke (elt _footprintlist i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
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
   ;; int8 _ftprintno
     (setq _ftprintno (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ftprintno 127) (setq _ftprintno (- _ftprintno 256)))
   ;; float64[] _footprintlist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _footprintlist (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _footprintlist i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get htbot::dyna :md5sum-) "aeca87cb1dcfc788fa047b80da0c718b")
(setf (get htbot::dyna :datatype-) "htbot/dyna")
(setf (get htbot::dyna :definition-)
      "int8 paramid
int32 intValue
float64 doubleValue
string strValue
bool boolValue
int8 ftprintno
float64[] footprintlist

")



(provide :htbot/dyna "aeca87cb1dcfc788fa047b80da0c718b")


