;; Auto-generated. Do not edit!


(when (boundp 'htbot::motorcmd)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'motorcmd (find-package "HTBOT")))
(unless (find-package "HTBOT::MOTORCMD")
  (make-package "HTBOT::MOTORCMD"))

(in-package "ROS")
;;//! \htmlinclude motorcmd.msg.html


(defclass htbot::motorcmd
  :super ros::object
  :slots (_type _dist _angle _linear _angular ))

(defmethod htbot::motorcmd
  (:init
   (&key
    ((:type __type) 0)
    ((:dist __dist) 0.0)
    ((:angle __angle) 0.0)
    ((:linear __linear) 0.0)
    ((:angular __angular) 0.0)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _dist (float __dist))
   (setq _angle (float __angle))
   (setq _linear (float __linear))
   (setq _angular (float __angular))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:dist
   (&optional __dist)
   (if __dist (setq _dist __dist)) _dist)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:linear
   (&optional __linear)
   (if __linear (setq _linear __linear)) _linear)
  (:angular
   (&optional __angular)
   (if __angular (setq _angular __angular)) _angular)
  (:serialization-length
   ()
   (+
    ;; int8 _type
    1
    ;; float32 _dist
    4
    ;; float32 _angle
    4
    ;; float32 _linear
    4
    ;; float32 _angular
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _type
       (write-byte _type s)
     ;; float32 _dist
       (sys::poke _dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linear
       (sys::poke _linear (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular
       (sys::poke _angular (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _type 127) (setq _type (- _type 256)))
   ;; float32 _dist
     (setq _dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linear
     (setq _linear (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular
     (setq _angular (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get htbot::motorcmd :md5sum-) "f00942b3a4df0d60ea8f3f655f2d737a")
(setf (get htbot::motorcmd :datatype-) "htbot/motorcmd")
(setf (get htbot::motorcmd :definition-)
      "int8 type
float32 dist
float32 angle
float32 linear
float32 angular

")



(provide :htbot/motorcmd "f00942b3a4df0d60ea8f3f655f2d737a")


