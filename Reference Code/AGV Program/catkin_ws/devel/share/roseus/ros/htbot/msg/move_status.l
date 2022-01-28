;; Auto-generated. Do not edit!


(when (boundp 'htbot::move_status)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'move_status (find-package "HTBOT")))
(unless (find-package "HTBOT::MOVE_STATUS")
  (make-package "HTBOT::MOVE_STATUS"))

(in-package "ROS")
;;//! \htmlinclude move_status.msg.html


(defclass htbot::move_status
  :super ros::object
  :slots (_stat ))

(defmethod htbot::move_status
  (:init
   (&key
    ((:stat __stat) 0)
    )
   (send-super :init)
   (setq _stat (round __stat))
   self)
  (:stat
   (&optional __stat)
   (if __stat (setq _stat __stat)) _stat)
  (:serialization-length
   ()
   (+
    ;; int8 _stat
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _stat
       (write-byte _stat s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _stat
     (setq _stat (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stat 127) (setq _stat (- _stat 256)))
   ;;
   self)
  )

(setf (get htbot::move_status :md5sum-) "85998e8afa5502f501182cfd6840bd64")
(setf (get htbot::move_status :datatype-) "htbot/move_status")
(setf (get htbot::move_status :definition-)
      "int8 stat

")



(provide :htbot/move_status "85998e8afa5502f501182cfd6840bd64")


