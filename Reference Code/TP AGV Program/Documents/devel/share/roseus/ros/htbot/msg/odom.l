;; Auto-generated. Do not edit!


(when (boundp 'htbot::odom)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'odom (find-package "HTBOT")))
(unless (find-package "HTBOT::ODOM")
  (make-package "HTBOT::ODOM"))

(in-package "ROS")
;;//! \htmlinclude odom.msg.html


(defclass htbot::odom
  :super ros::object
  :slots (_dist _angle ))

(defmethod htbot::odom
  (:init
   (&key
    ((:dist __dist) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _dist (float __dist))
   (setq _angle (float __angle))
   self)
  (:dist
   (&optional __dist)
   (if __dist (setq _dist __dist)) _dist)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32 _dist
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _dist
       (sys::poke _dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _dist
     (setq _dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get htbot::odom :md5sum-) "c05404bebfe29b1ba1f590bcefdda9bc")
(setf (get htbot::odom :datatype-) "htbot/odom")
(setf (get htbot::odom :definition-)
      "float32 dist
float32 angle

")



(provide :htbot/odom "c05404bebfe29b1ba1f590bcefdda9bc")


