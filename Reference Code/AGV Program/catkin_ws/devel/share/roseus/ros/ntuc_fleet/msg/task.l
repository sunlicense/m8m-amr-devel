;; Auto-generated. Do not edit!


(when (boundp 'ntuc_fleet::task)
  (if (not (find-package "NTUC_FLEET"))
    (make-package "NTUC_FLEET"))
  (shadow 'task (find-package "NTUC_FLEET")))
(unless (find-package "NTUC_FLEET::TASK")
  (make-package "NTUC_FLEET::TASK"))

(in-package "ROS")
;;//! \htmlinclude task.msg.html


(defclass ntuc_fleet::task
  :super ros::object
  :slots (_cmd _time _type _fromLP _toLP _alloc ))

(defmethod ntuc_fleet::task
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:time __time) (instance ros::time :init))
    ((:type __type) 0)
    ((:fromLP __fromLP) 0)
    ((:toLP __toLP) 0)
    ((:alloc __alloc) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _time __time)
   (setq _type (round __type))
   (setq _fromLP (round __fromLP))
   (setq _toLP (round __toLP))
   (setq _alloc (round __alloc))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:fromLP
   (&optional __fromLP)
   (if __fromLP (setq _fromLP __fromLP)) _fromLP)
  (:toLP
   (&optional __toLP)
   (if __toLP (setq _toLP __toLP)) _toLP)
  (:alloc
   (&optional __alloc)
   (if __alloc (setq _alloc __alloc)) _alloc)
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ;; time _time
    8
    ;; int8 _type
    1
    ;; int8 _fromLP
    1
    ;; int8 _toLP
    1
    ;; int8 _alloc
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
     ;; int8 _type
       (write-byte _type s)
     ;; int8 _fromLP
       (write-byte _fromLP s)
     ;; int8 _toLP
       (write-byte _toLP s)
     ;; int8 _alloc
       (write-byte _alloc s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _type 127) (setq _type (- _type 256)))
   ;; int8 _fromLP
     (setq _fromLP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _fromLP 127) (setq _fromLP (- _fromLP 256)))
   ;; int8 _toLP
     (setq _toLP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _toLP 127) (setq _toLP (- _toLP 256)))
   ;; int8 _alloc
     (setq _alloc (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _alloc 127) (setq _alloc (- _alloc 256)))
   ;;
   self)
  )

(setf (get ntuc_fleet::task :md5sum-) "d9335104b860e9530a386a51f33ebb59")
(setf (get ntuc_fleet::task :datatype-) "ntuc_fleet/task")
(setf (get ntuc_fleet::task :definition-)
      "##################################################
##### Message type: task.msg
##### Use in Topics: /fr_station, /fr_fleet, /fr_agv
##################################################
int8 cmd		# command code 
time time		# time task is queued
int8 type		# different trolley type
int8 fromLP		# starting LP
int8 toLP		# ending LP
int8 alloc		# 0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2

##################################################
## cmd		1:new task, 2:assigned task, 3:cancel task
## time		time.secs and time.nsecs (ROS time)
## type		1:Bian Marie, 2:Food Warmer, 3:Linen
## fromLP	70:L1LRm3-9, 73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,
##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward9, 
##				10:L10StaffDorm, 11:L11Ktchen,
##				51:L1MainStore, 52:L1NonHalalPrepRoom
## toLP		73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,
##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward,
##				10: L10StaffDorm, 11:L11Ktchen,
##				51:L1MainStore, 52:L1NonHalalPrepRoom
## alloc	0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2
##################################################	
## LP Range:: 	2-49: Lift Area, 50-69: Lobby Area, 70-100: Laundry Area 
##################################################	

")



(provide :ntuc_fleet/task "d9335104b860e9530a386a51f33ebb59")


