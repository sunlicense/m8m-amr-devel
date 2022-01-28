;; Auto-generated. Do not edit!


(when (boundp 'htbot::move)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'move (find-package "HTBOT")))
(unless (find-package "HTBOT::MOVE")
  (make-package "HTBOT::MOVE"))

(in-package "ROS")
;;//! \htmlinclude move.msg.html


(defclass htbot::move
  :super ros::object
  :slots (_x _y _z _rx _ry _rz _rw _pd _pa _gap _slp _elp _opt ))

(defmethod htbot::move
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:rx __rx) 0.0)
    ((:ry __ry) 0.0)
    ((:rz __rz) 0.0)
    ((:rw __rw) 0.0)
    ((:pd __pd) 0.0)
    ((:pa __pa) 0.0)
    ((:gap __gap) 0.0)
    ((:slp __slp) 0)
    ((:elp __elp) 0)
    ((:opt __opt) 0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _rx (float __rx))
   (setq _ry (float __ry))
   (setq _rz (float __rz))
   (setq _rw (float __rw))
   (setq _pd (float __pd))
   (setq _pa (float __pa))
   (setq _gap (float __gap))
   (setq _slp (round __slp))
   (setq _elp (round __elp))
   (setq _opt (round __opt))
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
  (:pd
   (&optional __pd)
   (if __pd (setq _pd __pd)) _pd)
  (:pa
   (&optional __pa)
   (if __pa (setq _pa __pa)) _pa)
  (:gap
   (&optional __gap)
   (if __gap (setq _gap __gap)) _gap)
  (:slp
   (&optional __slp)
   (if __slp (setq _slp __slp)) _slp)
  (:elp
   (&optional __elp)
   (if __elp (setq _elp __elp)) _elp)
  (:opt
   (&optional __opt)
   (if __opt (setq _opt __opt)) _opt)
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
    ;; float32 _pd
    4
    ;; float32 _pa
    4
    ;; float32 _gap
    4
    ;; int8 _slp
    1
    ;; int8 _elp
    1
    ;; int8 _opt
    1
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
     ;; float32 _pd
       (sys::poke _pd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pa
       (sys::poke _pa (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gap
       (sys::poke _gap (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _slp
       (write-byte _slp s)
     ;; int8 _elp
       (write-byte _elp s)
     ;; int8 _opt
       (write-byte _opt s)
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
   ;; float32 _pd
     (setq _pd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pa
     (setq _pa (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gap
     (setq _gap (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _slp
     (setq _slp (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _slp 127) (setq _slp (- _slp 256)))
   ;; int8 _elp
     (setq _elp (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _elp 127) (setq _elp (- _elp 256)))
   ;; int8 _opt
     (setq _opt (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _opt 127) (setq _opt (- _opt 256)))
   ;;
   self)
  )

(setf (get htbot::move :md5sum-) "05f4bf5bce5c30ffe01fb09c18cbbd86")
(setf (get htbot::move :datatype-) "htbot/move")
(setf (get htbot::move :definition-)
      "float32 x
float32 y
float32 z
float32 rx
float32 ry
float32 rz
float32 rw
float32 pd
float32 pa
float32 gap
int8 slp
int8 elp
int8 opt

")



(provide :htbot/move "05f4bf5bce5c30ffe01fb09c18cbbd86")


