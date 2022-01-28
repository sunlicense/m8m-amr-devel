;; Auto-generated. Do not edit!


(when (boundp 'htbot::status)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'status (find-package "HTBOT")))
(unless (find-package "HTBOT::STATUS")
  (make-package "HTBOT::STATUS"))

(in-package "ROS")
;;//! \htmlinclude status.msg.html


(defclass htbot::status
  :super ros::object
  :slots (_msg ))

(defmethod htbot::status
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

(setf (get htbot::status :md5sum-) "7d96ed730776804754140b85e64c862e")
(setf (get htbot::status :datatype-) "htbot/status")
(setf (get htbot::status :definition-)
      "string msg
")



(provide :htbot/status "7d96ed730776804754140b85e64c862e")


