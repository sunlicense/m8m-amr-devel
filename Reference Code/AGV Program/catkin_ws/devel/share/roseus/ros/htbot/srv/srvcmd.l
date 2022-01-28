;; Auto-generated. Do not edit!


(when (boundp 'htbot::srvcmd)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'srvcmd (find-package "HTBOT")))
(unless (find-package "HTBOT::SRVCMD")
  (make-package "HTBOT::SRVCMD"))
(unless (find-package "HTBOT::SRVCMDREQUEST")
  (make-package "HTBOT::SRVCMDREQUEST"))
(unless (find-package "HTBOT::SRVCMDRESPONSE")
  (make-package "HTBOT::SRVCMDRESPONSE"))

(in-package "ROS")





(defclass htbot::srvcmdRequest
  :super ros::object
  :slots (_cmd _fromLP _toLP _cGP _cLP _fLP _tLP ))

(defmethod htbot::srvcmdRequest
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:fromLP __fromLP) "")
    ((:toLP __toLP) "")
    ((:cGP __cGP) 0)
    ((:cLP __cLP) 0)
    ((:fLP __fLP) 0)
    ((:tLP __tLP) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _fromLP (string __fromLP))
   (setq _toLP (string __toLP))
   (setq _cGP (round __cGP))
   (setq _cLP (round __cLP))
   (setq _fLP (round __fLP))
   (setq _tLP (round __tLP))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:fromLP
   (&optional __fromLP)
   (if __fromLP (setq _fromLP __fromLP)) _fromLP)
  (:toLP
   (&optional __toLP)
   (if __toLP (setq _toLP __toLP)) _toLP)
  (:cGP
   (&optional __cGP)
   (if __cGP (setq _cGP __cGP)) _cGP)
  (:cLP
   (&optional __cLP)
   (if __cLP (setq _cLP __cLP)) _cLP)
  (:fLP
   (&optional __fLP)
   (if __fLP (setq _fLP __fLP)) _fLP)
  (:tLP
   (&optional __tLP)
   (if __tLP (setq _tLP __tLP)) _tLP)
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ;; string _fromLP
    4 (length _fromLP)
    ;; string _toLP
    4 (length _toLP)
    ;; int8 _cGP
    1
    ;; int8 _cLP
    1
    ;; int8 _fLP
    1
    ;; int8 _tLP
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _cmd
       (write-byte _cmd s)
     ;; string _fromLP
       (write-long (length _fromLP) s) (princ _fromLP s)
     ;; string _toLP
       (write-long (length _toLP) s) (princ _toLP s)
     ;; int8 _cGP
       (write-byte _cGP s)
     ;; int8 _cLP
       (write-byte _cLP s)
     ;; int8 _fLP
       (write-byte _fLP s)
     ;; int8 _tLP
       (write-byte _tLP s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;; string _fromLP
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fromLP (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _toLP
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _toLP (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _cGP
     (setq _cGP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cGP 127) (setq _cGP (- _cGP 256)))
   ;; int8 _cLP
     (setq _cLP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cLP 127) (setq _cLP (- _cLP 256)))
   ;; int8 _fLP
     (setq _fLP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _fLP 127) (setq _fLP (- _fLP 256)))
   ;; int8 _tLP
     (setq _tLP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _tLP 127) (setq _tLP (- _tLP 256)))
   ;;
   self)
  )

(defclass htbot::srvcmdResponse
  :super ros::object
  :slots (_status _s1 _s2 _linear _angular ))

(defmethod htbot::srvcmdResponse
  (:init
   (&key
    ((:status __status) 0)
    ((:s1 __s1) "")
    ((:s2 __s2) "")
    ((:linear __linear) 0.0)
    ((:angular __angular) 0.0)
    )
   (send-super :init)
   (setq _status (round __status))
   (setq _s1 (string __s1))
   (setq _s2 (string __s2))
   (setq _linear (float __linear))
   (setq _angular (float __angular))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:s1
   (&optional __s1)
   (if __s1 (setq _s1 __s1)) _s1)
  (:s2
   (&optional __s2)
   (if __s2 (setq _s2 __s2)) _s2)
  (:linear
   (&optional __linear)
   (if __linear (setq _linear __linear)) _linear)
  (:angular
   (&optional __angular)
   (if __angular (setq _angular __angular)) _angular)
  (:serialization-length
   ()
   (+
    ;; int8 _status
    1
    ;; string _s1
    4 (length _s1)
    ;; string _s2
    4 (length _s2)
    ;; float32 _linear
    4
    ;; float32 _angular
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _status
       (write-byte _status s)
     ;; string _s1
       (write-long (length _s1) s) (princ _s1 s)
     ;; string _s2
       (write-long (length _s2) s) (princ _s2 s)
     ;; float32 _linear
       (sys::poke _linear (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular
       (sys::poke _angular (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _status 127) (setq _status (- _status 256)))
   ;; string _s1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _s1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _s2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _s2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _linear
     (setq _linear (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular
     (setq _angular (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass htbot::srvcmd
  :super ros::object
  :slots ())

(setf (get htbot::srvcmd :md5sum-) "30eadfa966628646a567b1a8c11f77b8")
(setf (get htbot::srvcmd :datatype-) "htbot/srvcmd")
(setf (get htbot::srvcmd :request) htbot::srvcmdRequest)
(setf (get htbot::srvcmd :response) htbot::srvcmdResponse)

(defmethod htbot::srvcmdRequest
  (:response () (instance htbot::srvcmdResponse :init)))

(setf (get htbot::srvcmdRequest :md5sum-) "30eadfa966628646a567b1a8c11f77b8")
(setf (get htbot::srvcmdRequest :datatype-) "htbot/srvcmdRequest")
(setf (get htbot::srvcmdRequest :definition-)
      "int8 cmd
string fromLP
string toLP
int8 cGP
int8 cLP
int8 fLP
int8 tLP
---
int8 status
string s1
string s2
float32 linear
float32 angular

")

(setf (get htbot::srvcmdResponse :md5sum-) "30eadfa966628646a567b1a8c11f77b8")
(setf (get htbot::srvcmdResponse :datatype-) "htbot/srvcmdResponse")
(setf (get htbot::srvcmdResponse :definition-)
      "int8 cmd
string fromLP
string toLP
int8 cGP
int8 cLP
int8 fLP
int8 tLP
---
int8 status
string s1
string s2
float32 linear
float32 angular

")



(provide :htbot/srvcmd "30eadfa966628646a567b1a8c11f77b8")


