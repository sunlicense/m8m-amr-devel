;; Auto-generated. Do not edit!


(when (boundp 'htbot::debug)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'debug (find-package "HTBOT")))
(unless (find-package "HTBOT::DEBUG")
  (make-package "HTBOT::DEBUG"))

(in-package "ROS")
;;//! \htmlinclude debug.msg.html


(defclass htbot::debug
  :super ros::object
  :slots (_msg ))

(defmethod htbot::debug
  (:init
   (&key
    ((:msg __msg) "")
    )
   (send-super :init)
   (setq _msg (string __msg))
   self)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:serialization-length
   ()
   (+
    ;; string _msg
    4 (length _msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get htbot::debug :md5sum-) "7d96ed730776804754140b85e64c862e")
(setf (get htbot::debug :datatype-) "htbot/debug")
(setf (get htbot::debug :definition-)
      "string msg
")



(provide :htbot/debug "7d96ed730776804754140b85e64c862e")


