;; Auto-generated. Do not edit!


(when (boundp 'htbot::go)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'go (find-package "HTBOT")))
(unless (find-package "HTBOT::GO")
  (make-package "HTBOT::GO"))

(in-package "ROS")
;;//! \htmlinclude go.msg.html


(defclass htbot::go
  :super ros::object
  :slots (_cmd ))

(defmethod htbot::go
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

(setf (get htbot::go :md5sum-) "26e2d5a54557d558b8243da339e9952c")
(setf (get htbot::go :datatype-) "htbot/go")
(setf (get htbot::go :definition-)
      "int8 cmd

")



(provide :htbot/go "26e2d5a54557d558b8243da339e9952c")


