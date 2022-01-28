;; Auto-generated. Do not edit!


(when (boundp 'htbot::sendgoal)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'sendgoal (find-package "HTBOT")))
(unless (find-package "HTBOT::SENDGOAL")
  (make-package "HTBOT::SENDGOAL"))
(unless (find-package "HTBOT::SENDGOALREQUEST")
  (make-package "HTBOT::SENDGOALREQUEST"))
(unless (find-package "HTBOT::SENDGOALRESPONSE")
  (make-package "HTBOT::SENDGOALRESPONSE"))

(in-package "ROS")





(defclass htbot::sendgoalRequest
  :super ros::object
  :slots (_x _y _z _rx _ry _rz _rw ))

(defmethod htbot::sendgoalRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:rx __rx) 0.0)
    ((:ry __ry) 0.0)
    ((:rz __rz) 0.0)
    ((:rw __rw) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _rx (float __rx))
   (setq _ry (float __ry))
   (setq _rz (float __rz))
   (setq _rw (float __rw))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:rx
   (&optional __rx)
   (if __rx (setq _rx __rx)) _rx)
  (:ry
   (&optional __ry)
   (if __ry (setq _ry __ry)) _ry)
  (:rz
   (&optional __rz)
   (if __rz (setq _rz __rz)) _rz)
  (:rw
   (&optional __rw)
   (if __rw (setq _rw __rw)) _rw)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _rx
    4
    ;; float32 _ry
    4
    ;; float32 _rz
    4
    ;; float32 _rw
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rx
       (sys::poke _rx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ry
       (sys::poke _ry (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rz
       (sys::poke _rz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rw
       (sys::poke _rw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rx
     (setq _rx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ry
     (setq _ry (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rz
     (setq _rz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rw
     (setq _rw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass htbot::sendgoalResponse
  :super ros::object
  :slots (_status ))

(defmethod htbot::sendgoalResponse
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; int8 _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _status 127) (setq _status (- _status 256)))
   ;;
   self)
  )

(defclass htbot::sendgoal
  :super ros::object
  :slots ())

(setf (get htbot::sendgoal :md5sum-) "e870aaccbc4db5d4afb5053d631034b4")
(setf (get htbot::sendgoal :datatype-) "htbot/sendgoal")
(setf (get htbot::sendgoal :request) htbot::sendgoalRequest)
(setf (get htbot::sendgoal :response) htbot::sendgoalResponse)

(defmethod htbot::sendgoalRequest
  (:response () (instance htbot::sendgoalResponse :init)))

(setf (get htbot::sendgoalRequest :md5sum-) "e870aaccbc4db5d4afb5053d631034b4")
(setf (get htbot::sendgoalRequest :datatype-) "htbot/sendgoalRequest")
(setf (get htbot::sendgoalRequest :definition-)
      "float32 x
float32 y
float32 z
float32 rx
float32 ry
float32 rz
float32 rw
---
int8 status

")

(setf (get htbot::sendgoalResponse :md5sum-) "e870aaccbc4db5d4afb5053d631034b4")
(setf (get htbot::sendgoalResponse :datatype-) "htbot/sendgoalResponse")
(setf (get htbot::sendgoalResponse :definition-)
      "float32 x
float32 y
float32 z
float32 rx
float32 ry
float32 rz
float32 rw
---
int8 status

")



(provide :htbot/sendgoal "e870aaccbc4db5d4afb5053d631034b4")


