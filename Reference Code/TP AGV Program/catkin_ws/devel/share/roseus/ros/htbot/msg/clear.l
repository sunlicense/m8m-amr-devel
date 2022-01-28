;; Auto-generated. Do not edit!


(when (boundp 'htbot::clear)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'clear (find-package "HTBOT")))
(unless (find-package "HTBOT::CLEAR")
  (make-package "HTBOT::CLEAR"))

(in-package "ROS")
;;//! \htmlinclude clear.msg.html


(defclass htbot::clear
  :super ros::object
  :slots (_cmd ))

(defmethod htbot::clear
  (:init
   (&key
    ((:cmd __cmd) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _cmd
       (write-byte _cmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;;
   self)
  )

(setf (get htbot::clear :md5sum-) "26e2d5a54557d558b8243da339e9952c")
(setf (get htbot::clear :datatype-) "htbot/clear")
(setf (get htbot::clear :definition-)
      "int8 cmd

")



(provide :htbot/clear "26e2d5a54557d558b8243da339e9952c")


