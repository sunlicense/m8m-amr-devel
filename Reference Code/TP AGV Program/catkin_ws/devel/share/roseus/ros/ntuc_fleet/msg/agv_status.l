;; Auto-generated. Do not edit!


(when (boundp 'ntuc_fleet::agv_status)
  (if (not (find-package "NTUC_FLEET"))
    (make-package "NTUC_FLEET"))
  (shadow 'agv_status (find-package "NTUC_FLEET")))
(unless (find-package "NTUC_FLEET::AGV_STATUS")
  (make-package "NTUC_FLEET::AGV_STATUS"))

(in-package "ROS")
;;//! \htmlinclude agv_status.msg.html


(defclass ntuc_fleet::agv_status
  :super ros::object
  :slots (_cmd _time _status _location _arrival _b_level _e_status _complete _lastLP ))

(defmethod ntuc_fleet::agv_status
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:time __time) (instance ros::time :init))
    ((:status __status) 0)
    ((:location __location) 0)
    ((:arrival __arrival) (instance ros::time :init))
    ((:b_level __b_level) 0)
    ((:e_status __e_status) 0)
    ((:complete __complete) 0)
    ((:lastLP __lastLP) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _time __time)
   (setq _status (round __status))
   (setq _location (round __location))
   (setq _arrival __arrival)
   (setq _b_level (round __b_level))
   (setq _e_status (round __e_status))
   (setq _complete (round __complete))
   (setq _lastLP (round __lastLP))
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
  (:e_status
   (&optional __e_status)
   (if __e_status (setq _e_status __e_status)) _e_status)
  (:complete
   (&optional __complete)
   (if __complete (setq _complete __complete)) _complete)
  (:lastLP
   (&optional __lastLP)
   (if __lastLP (setq _lastLP __lastLP)) _lastLP)
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
    ;; int8 _e_status
    1
    ;; int8 _complete
    1
    ;; int8 _lastLP
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
     ;; int8 _e_status
       (write-byte _e_status s)
     ;; int8 _complete
       (write-byte _complete s)
     ;; int8 _lastLP
       (write-byte _lastLP s)
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
   ;; int8 _e_status
     (setq _e_status (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _e_status 127) (setq _e_status (- _e_status 256)))
   ;; int8 _complete
     (setq _complete (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _complete 127) (setq _complete (- _complete 256)))
   ;; int8 _lastLP
     (setq _lastLP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _lastLP 127) (setq _lastLP (- _lastLP 256)))
   ;;
   self)
  )

(setf (get ntuc_fleet::agv_status :md5sum-) "0b47d165e9912a2afc0c77fd2d5b0bfa")
(setf (get ntuc_fleet::agv_status :datatype-) "ntuc_fleet/agv_status")
(setf (get ntuc_fleet::agv_status :definition-)
      "##################################################
##### Message type: status.msg
##### Use in Topics: /to_fleet, /to_station
##################################################
int8 cmd			# command code	 
time time		# fleet manager current time
int8 status		# AGV status
int8 location		# AGV location
time arrival		# estimate time of arrival
int8 b_level		# battery level in %
int8 e_status		# elevator status 
int8 complete		# task completion / ready for next task
int8 lastLP			# LP when the last task has just completed

##################################################
## cmd		1:op_status, 2:alert, 3:alarm, 4:agv_status
## time		time in time.secs and and time.nsecs 
## status	1:delivering food, 2:traffic jam, 3:breakdown, 4:fire alarm, 5:charging
## location	0:L1Store,1:L1Laundry,2:L1NonHalal,3:L3,4:L4,5:L5,6:L6,
##				7:L7,8:L8,9:L9,10:L10Dorm,11:L11,12:L1Charging
## arrival	time in time.secs and and time.nsecs
## b_level	percent of battery level
## lift_status	1:okay, 2:service alarm, 3:fire alarm 
## complete	0:ready for task, 1:completed & ready, 2:completed not ready, 3:not ready, 
##				4:job-aborted not ready, 5:job-aborted ready
## lastLP	73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,
##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward,
##				10: L10StaffDorm, 11:L11Ktchen,
##				51:L1MainStore, 52:L1NonHalalPrepRoom
##################################################	

")



(provide :ntuc_fleet/agv_status "0b47d165e9912a2afc0c77fd2d5b0bfa")


