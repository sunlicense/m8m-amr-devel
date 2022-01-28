;; Auto-generated. Do not edit!


(when (boundp 'htbot::lift)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'lift (find-package "HTBOT")))
(unless (find-package "HTBOT::LIFT")
  (make-package "HTBOT::LIFT"))

(in-package "ROS")
;;//! \htmlinclude lift.msg.html


(defclass htbot::lift
  :super ros::object
  :slots (_cmd _cfloor _dfloor _inuse _dooropen _doorclose _goingup _goingdown _stationary _serviceavail _epower _fireservice _beacon ))

(defmethod htbot::lift
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:cfloor __cfloor) 0)
    ((:dfloor __dfloor) 0)
    ((:inuse __inuse) 0)
    ((:dooropen __dooropen) 0)
    ((:doorclose __doorclose) 0)
    ((:goingup __goingup) 0)
    ((:goingdown __goingdown) 0)
    ((:stationary __stationary) 0)
    ((:serviceavail __serviceavail) 0)
    ((:epower __epower) 0)
    ((:fireservice __fireservice) 0)
    ((:beacon __beacon) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _cfloor (round __cfloor))
   (setq _dfloor (round __dfloor))
   (setq _inuse (round __inuse))
   (setq _dooropen (round __dooropen))
   (setq _doorclose (round __doorclose))
   (setq _goingup (round __goingup))
   (setq _goingdown (round __goingdown))
   (setq _stationary (round __stationary))
   (setq _serviceavail (round __serviceavail))
   (setq _epower (round __epower))
   (setq _fireservice (round __fireservice))
   (setq _beacon (round __beacon))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:cfloor
   (&optional __cfloor)
   (if __cfloor (setq _cfloor __cfloor)) _cfloor)
  (:dfloor
   (&optional __dfloor)
   (if __dfloor (setq _dfloor __dfloor)) _dfloor)
  (:inuse
   (&optional __inuse)
   (if __inuse (setq _inuse __inuse)) _inuse)
  (:dooropen
   (&optional __dooropen)
   (if __dooropen (setq _dooropen __dooropen)) _dooropen)
  (:doorclose
   (&optional __doorclose)
   (if __doorclose (setq _doorclose __doorclose)) _doorclose)
  (:goingup
   (&optional __goingup)
   (if __goingup (setq _goingup __goingup)) _goingup)
  (:goingdown
   (&optional __goingdown)
   (if __goingdown (setq _goingdown __goingdown)) _goingdown)
  (:stationary
   (&optional __stationary)
   (if __stationary (setq _stationary __stationary)) _stationary)
  (:serviceavail
   (&optional __serviceavail)
   (if __serviceavail (setq _serviceavail __serviceavail)) _serviceavail)
  (:epower
   (&optional __epower)
   (if __epower (setq _epower __epower)) _epower)
  (:fireservice
   (&optional __fireservice)
   (if __fireservice (setq _fireservice __fireservice)) _fireservice)
  (:beacon
   (&optional __beacon)
   (if __beacon (setq _beacon __beacon)) _beacon)
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ;; int8 _cfloor
    1
    ;; int8 _dfloor
    1
    ;; int8 _inuse
    1
    ;; int8 _dooropen
    1
    ;; int8 _doorclose
    1
    ;; int8 _goingup
    1
    ;; int8 _goingdown
    1
    ;; int8 _stationary
    1
    ;; int8 _serviceavail
    1
    ;; int8 _epower
    1
    ;; int8 _fireservice
    1
    ;; int8 _beacon
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _cmd
       (write-byte _cmd s)
     ;; int8 _cfloor
       (write-byte _cfloor s)
     ;; int8 _dfloor
       (write-byte _dfloor s)
     ;; int8 _inuse
       (write-byte _inuse s)
     ;; int8 _dooropen
       (write-byte _dooropen s)
     ;; int8 _doorclose
       (write-byte _doorclose s)
     ;; int8 _goingup
       (write-byte _goingup s)
     ;; int8 _goingdown
       (write-byte _goingdown s)
     ;; int8 _stationary
       (write-byte _stationary s)
     ;; int8 _serviceavail
       (write-byte _serviceavail s)
     ;; int8 _epower
       (write-byte _epower s)
     ;; int8 _fireservice
       (write-byte _fireservice s)
     ;; int8 _beacon
       (write-byte _beacon s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;; int8 _cfloor
     (setq _cfloor (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cfloor 127) (setq _cfloor (- _cfloor 256)))
   ;; int8 _dfloor
     (setq _dfloor (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _dfloor 127) (setq _dfloor (- _dfloor 256)))
   ;; int8 _inuse
     (setq _inuse (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _inuse 127) (setq _inuse (- _inuse 256)))
   ;; int8 _dooropen
     (setq _dooropen (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _dooropen 127) (setq _dooropen (- _dooropen 256)))
   ;; int8 _doorclose
     (setq _doorclose (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _doorclose 127) (setq _doorclose (- _doorclose 256)))
   ;; int8 _goingup
     (setq _goingup (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _goingup 127) (setq _goingup (- _goingup 256)))
   ;; int8 _goingdown
     (setq _goingdown (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _goingdown 127) (setq _goingdown (- _goingdown 256)))
   ;; int8 _stationary
     (setq _stationary (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stationary 127) (setq _stationary (- _stationary 256)))
   ;; int8 _serviceavail
     (setq _serviceavail (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _serviceavail 127) (setq _serviceavail (- _serviceavail 256)))
   ;; int8 _epower
     (setq _epower (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _epower 127) (setq _epower (- _epower 256)))
   ;; int8 _fireservice
     (setq _fireservice (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _fireservice 127) (setq _fireservice (- _fireservice 256)))
   ;; int8 _beacon
     (setq _beacon (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _beacon 127) (setq _beacon (- _beacon 256)))
   ;;
   self)
  )

(setf (get htbot::lift :md5sum-) "e5880338d440c98c58ce479e42b36bed")
(setf (get htbot::lift :datatype-) "htbot/lift")
(setf (get htbot::lift :definition-)
      "##################################################
##### Message type: lift.msg
##### Use in Topics: /to_lift, /fr_lift
##################################################
int8 cmd			# command code			[0-102]
int8 cfloor			# start of trip [1-11] or current car level [1-11]
int8 dfloor			# end level of trip	[1-11]
int8 inuse			# request for in_use activation [0/1] or in_use status [0/1]
int8 dooropen		# door open status 	[0/1]
int8 doorclose		# door close status 	[0/1]
int8 goingup		# going up status 	[0/1]
int8 goingdown		# going down status 	[0/1]
int8 stationary		# stationary status 	[0/1]
int8 serviceavail	# service available status	[0/1]
int8 epower			# emergency power status 	[0/1]
int8 fireservice	# fire service status 		[0/1]
int8 beacon			# beacon light alarm 		[0/1], 1 to trigger, 0 to turn off

##################################################
##### cmd value usage:
##### 0:carCallStartLevel, 1:enteredLift, 2:carCallEndLevel, 3:exitedLift
##### 8:beacon, 9:inUse, 10:okay2Enter, 11:okay2Exit
##### 20:inUse, 21:doorOpen, 22:doorClose, 23:goingUp, 24:goingDown, 25:stationary 
##### 40:carLevel, 100:serviceAvailable, 101: emergencyPower, 102:fireServiceAlarm
##################################################

")



(provide :htbot/lift "e5880338d440c98c58ce479e42b36bed")


