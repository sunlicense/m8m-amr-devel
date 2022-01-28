;; Auto-generated. Do not edit!


(when (boundp 'htbot::scanMcmd)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'scanMcmd (find-package "HTBOT")))
(unless (find-package "HTBOT::SCANMCMD")
  (make-package "HTBOT::SCANMCMD"))
(unless (find-package "HTBOT::SCANMCMDREQUEST")
  (make-package "HTBOT::SCANMCMDREQUEST"))
(unless (find-package "HTBOT::SCANMCMDRESPONSE")
  (make-package "HTBOT::SCANMCMDRESPONSE"))

(in-package "ROS")





(defclass htbot::scanMcmdRequest
  :super ros::object
  :slots (_cmd _file _lp _gp ))

(defmethod htbot::scanMcmdRequest
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:file __file) "")
    ((:lp __lp) 0)
    ((:gp __gp) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _file (string __file))
   (setq _lp (round __lp))
   (setq _gp (round __gp))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:file
   (&optional __file)
   (if __file (setq _file __file)) _file)
  (:lp
   (&optional __lp)
   (if __lp (setq _lp __lp)) _lp)
  (:gp
   (&optional __gp)
   (if __gp (setq _gp __gp)) _gp)
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ;; string _file
    4 (length _file)
    ;; int8 _lp
    1
    ;; int8 _gp
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _cmd
       (write-byte _cmd s)
     ;; string _file
       (write-long (length _file) s) (princ _file s)
     ;; int8 _lp
       (write-byte _lp s)
     ;; int8 _gp
       (write-byte _gp s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;; string _file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _lp
     (setq _lp (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _lp 127) (setq _lp (- _lp 256)))
   ;; int8 _gp
     (setq _gp (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _gp 127) (setq _gp (- _gp 256)))
   ;;
   self)
  )

(defclass htbot::scanMcmdResponse
  :super ros::object
  :slots (_status _s1 _s2 _x _y _an ))

(defmethod htbot::scanMcmdResponse
  (:init
   (&key
    ((:status __status) 0)
    ((:s1 __s1) "")
    ((:s2 __s2) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:an __an) 0.0)
    )
   (send-super :init)
   (setq _status (round __status))
   (setq _s1 (string __s1))
   (setq _s2 (string __s2))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _an (float __an))
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
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:an
   (&optional __an)
   (if __an (setq _an __an)) _an)
  (:serialization-length
   ()
   (+
    ;; int8 _status
    1
    ;; string _s1
    4 (length _s1)
    ;; string _s2
    4 (length _s2)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _an
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
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _an
       (sys::poke _an (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _an
     (setq _an (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass htbot::scanMcmd
  :super ros::object
  :slots ())

(setf (get htbot::scanMcmd :md5sum-) "fe363af63be51788db46af922590fcac")
(setf (get htbot::scanMcmd :datatype-) "htbot/scanMcmd")
(setf (get htbot::scanMcmd :request) htbot::scanMcmdRequest)
(setf (get htbot::scanMcmd :response) htbot::scanMcmdResponse)

(defmethod htbot::scanMcmdRequest
  (:response () (instance htbot::scanMcmdResponse :init)))

(setf (get htbot::scanMcmdRequest :md5sum-) "fe363af63be51788db46af922590fcac")
(setf (get htbot::scanMcmdRequest :datatype-) "htbot/scanMcmdRequest")
(setf (get htbot::scanMcmdRequest :definition-)
      "int8 cmd
string file
int8 lp
int8 gp
---
int8 status
string s1
string s2
float32 x
float32 y
float32 an

")

(setf (get htbot::scanMcmdResponse :md5sum-) "fe363af63be51788db46af922590fcac")
(setf (get htbot::scanMcmdResponse :datatype-) "htbot/scanMcmdResponse")
(setf (get htbot::scanMcmdResponse :definition-)
      "int8 cmd
string file
int8 lp
int8 gp
---
int8 status
string s1
string s2
float32 x
float32 y
float32 an

")



(provide :htbot/scanMcmd "fe363af63be51788db46af922590fcac")


