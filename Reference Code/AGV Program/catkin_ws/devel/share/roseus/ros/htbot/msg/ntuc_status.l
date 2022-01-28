;; Auto-generated. Do not edit!


(when (boundp 'htbot::ntuc_status)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'ntuc_status (find-package "HTBOT")))
(unless (find-package "HTBOT::NTUC_STATUS")
  (make-package "HTBOT::NTUC_STATUS"))

(in-package "ROS")
;;//! \htmlinclude ntuc_status.msg.html


(defclass htbot::ntuc_status
  :super ros::object
  :slots (_cmd _time _status _location _arrival _b_level _lift_status _complete ))

(defmethod htbot::ntuc_status
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:time __time) (instance ros::time :init))
    ((:status __status) 0)
    ((:location __location) 0)
    ((:arrival __arrival) (instance ros::time :init))
    ((:b_level __b_level) 0)
    ((:lift_status __lift_status) 0)
    ((:complete __complete) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _time __time)
   (setq _status (round __status))
   (setq _location (round __location))
   (setq _arrival __arrival)
   (setq _b_level (round __b_level))
   (setq _lift_status (round __lift_status))
   (setq _complete (round __complete))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:location
   (&optional __location)
   (if __location (setq _location __location)) _location)
  (:arrival
   (&optional __arrival)
   (if __arrival (setq _arrival __arrival)) _arrival)
  (:b_level
   (&optional __b_level)
   (if __b_level (setq _b_level __b_level)) _b_level)
  (:lift_status
   (&optional __lift_status)
   (if __lift_status (setq _lift_status __lift_status)) _lift_status)
  (:complete
   (&optional __complete)
   (if __complete (setq _complete __complete)) _complete)
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ;; time _time
    8
    ;; int8 _status
    1
    ;; int8 _location
    1
    ;; time _arrival
    8
    ;; int8 _b_level
    1
    ;; int8 _lift_status
    1
    ;; int8 _complete
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _cmd
       (write-byte _cmd s)
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; int8 _status
       (write-byte _status s)
     ;; int8 _location
       (write-byte _location s)
     ;; time _arrival
       (write-long (send _arrival :sec) s) (write-long (send _arrival :nsec) s)
     ;; int8 _b_level
       (write-byte _b_level s)
     ;; int8 _lift_status
       (write-byte _lift_status s)
     ;; int8 _complete
       (write-byte _complete s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _status 127) (setq _status (- _status 256)))
   ;; int8 _location
     (setq _location (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _location 127) (setq _location (- _location 256)))
   ;; time _arrival
     (send _arrival :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _arrival :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8 _b_level
     (setq _b_level (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _b_level 127) (setq _b_level (- _b_level 256)))
   ;; int8 _lift_status
     (setq _lift_status (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _lift_status 127) (setq _lift_status (- _lift_status 256)))
   ;; int8 _complete
     (setq _complete (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _complete 127) (setq _complete (- _complete 256)))
   ;;
   self)
  )

(setf (get htbot::ntuc_status :md5sum-) "ad6efd2f44b02d2eda310e45f4e16a44")
(setf (get htbot::ntuc_status :datatype-) "htbot/ntuc_status")
(setf (get htbot::ntuc_status :definition-)
      "int8 cmd
time time
int8 status
int8 location
time arrival
int8 b_level
int8 lift_status
int8 complete

")



(provide :htbot/ntuc_status "ad6efd2f44b02d2eda310e45f4e16a44")


