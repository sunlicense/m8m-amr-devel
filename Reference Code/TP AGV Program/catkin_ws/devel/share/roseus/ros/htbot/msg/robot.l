;; Auto-generated. Do not edit!


(when (boundp 'htbot::robot)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'robot (find-package "HTBOT")))
(unless (find-package "HTBOT::ROBOT")
  (make-package "HTBOT::ROBOT"))

(in-package "ROS")
;;//! \htmlinclude robot.msg.html


(defclass htbot::robot
  :super ros::object
  :slots (_amclx _amcly _amcla _robotposediff _localisationstatus ))

(defmethod htbot::robot
  (:init
   (&key
    ((:amclx __amclx) 0.0)
    ((:amcly __amcly) 0.0)
    ((:amcla __amcla) 0.0)
    ((:robotposediff __robotposediff) 0.0)
    ((:localisationstatus __localisationstatus) "")
    )
   (send-super :init)
   (setq _amclx (float __amclx))
   (setq _amcly (float __amcly))
   (setq _amcla (float __amcla))
   (setq _robotposediff (float __robotposediff))
   (setq _localisationstatus (string __localisationstatus))
   self)
  (:amclx
   (&optional __amclx)
   (if __amclx (setq _amclx __amclx)) _amclx)
  (:amcly
   (&optional __amcly)
   (if __amcly (setq _amcly __amcly)) _amcly)
  (:amcla
   (&optional __amcla)
   (if __amcla (setq _amcla __amcla)) _amcla)
  (:robotposediff
   (&optional __robotposediff)
   (if __robotposediff (setq _robotposediff __robotposediff)) _robotposediff)
  (:localisationstatus
   (&optional __localisationstatus)
   (if __localisationstatus (setq _localisationstatus __localisationstatus)) _localisationstatus)
  (:serialization-length
   ()
   (+
    ;; float32 _amclx
    4
    ;; float32 _amcly
    4
    ;; float32 _amcla
    4
    ;; float32 _robotposediff
    4
    ;; string _localisationstatus
    4 (length _localisationstatus)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _amclx
       (sys::poke _amclx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _amcly
       (sys::poke _amcly (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _amcla
       (sys::poke _amcla (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _robotposediff
       (sys::poke _robotposediff (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _localisationstatus
       (write-long (length _localisationstatus) s) (princ _localisationstatus s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _amclx
     (setq _amclx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _amcly
     (setq _amcly (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _amcla
     (setq _amcla (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _robotposediff
     (setq _robotposediff (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _localisationstatus
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _localisationstatus (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get htbot::robot :md5sum-) "9472043a148e005ed77686c11e7067c6")
(setf (get htbot::robot :datatype-) "htbot/robot")
(setf (get htbot::robot :definition-)
      "float32 amclx
float32 amcly
float32 amcla
float32 robotposediff
string localisationstatus

")



(provide :htbot/robot "9472043a148e005ed77686c11e7067c6")


