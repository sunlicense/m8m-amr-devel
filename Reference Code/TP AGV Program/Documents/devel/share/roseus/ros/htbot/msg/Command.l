;; Auto-generated. Do not edit!


(when (boundp 'htbot::Command)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'Command (find-package "HTBOT")))
(unless (find-package "HTBOT::COMMAND")
  (make-package "HTBOT::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude Command.msg.html


(defclass htbot::Command
  :super ros::object
  :slots (_cmd _param1 _param2 ))

(defmethod htbot::Command
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:param1 __param1) 0)
    ((:param2 __param2) 0.0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _param1 (round __param1))
   (setq _param2 (float __param2))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:param1
   (&optional __param1)
   (if __param1 (setq _param1 __param1)) _param1)
  (:param2
   (&optional __param2)
   (if __param2 (setq _param2 __param2)) _param2)
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ;; int16 _param1
    2
    ;; float64 _param2
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _cmd
       (write-byte _cmd s)
     ;; int16 _param1
       (write-word _param1 s)
     ;; float64 _param2
       (sys::poke _param2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;; int16 _param1
     (setq _param1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _param2
     (setq _param2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get htbot::Command :md5sum-) "37e11590ded60e1e4263d0a06e0c9539")
(setf (get htbot::Command :datatype-) "htbot/Command")
(setf (get htbot::Command :definition-)
      "int8 cmd
int16 param1
float64 param2 

")



(provide :htbot/Command "37e11590ded60e1e4263d0a06e0c9539")


