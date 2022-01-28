;; Auto-generated. Do not edit!


(when (boundp 'htbot::lumstatus)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'lumstatus (find-package "HTBOT")))
(unless (find-package "HTBOT::LUMSTATUS")
  (make-package "HTBOT::LUMSTATUS"))

(in-package "ROS")
;;//! \htmlinclude lumstatus.msg.html


(defclass htbot::lumstatus
  :super ros::object
  :slots (_stn0 _stn1 _stn2 _stn3 _stn4 _stn5 _stn6 _stn7 _stn8 _stn9 _batlevel _laserstatus _errorlog ))

(defmethod htbot::lumstatus
  (:init
   (&key
    ((:stn0 __stn0) 0)
    ((:stn1 __stn1) 0)
    ((:stn2 __stn2) 0)
    ((:stn3 __stn3) 0)
    ((:stn4 __stn4) 0)
    ((:stn5 __stn5) 0)
    ((:stn6 __stn6) 0)
    ((:stn7 __stn7) 0)
    ((:stn8 __stn8) 0)
    ((:stn9 __stn9) 0)
    ((:batlevel __batlevel) 0.0)
    ((:laserstatus __laserstatus) "")
    ((:errorlog __errorlog) "")
    )
   (send-super :init)
   (setq _stn0 (round __stn0))
   (setq _stn1 (round __stn1))
   (setq _stn2 (round __stn2))
   (setq _stn3 (round __stn3))
   (setq _stn4 (round __stn4))
   (setq _stn5 (round __stn5))
   (setq _stn6 (round __stn6))
   (setq _stn7 (round __stn7))
   (setq _stn8 (round __stn8))
   (setq _stn9 (round __stn9))
   (setq _batlevel (float __batlevel))
   (setq _laserstatus (string __laserstatus))
   (setq _errorlog (string __errorlog))
   self)
  (:stn0
   (&optional __stn0)
   (if __stn0 (setq _stn0 __stn0)) _stn0)
  (:stn1
   (&optional __stn1)
   (if __stn1 (setq _stn1 __stn1)) _stn1)
  (:stn2
   (&optional __stn2)
   (if __stn2 (setq _stn2 __stn2)) _stn2)
  (:stn3
   (&optional __stn3)
   (if __stn3 (setq _stn3 __stn3)) _stn3)
  (:stn4
   (&optional __stn4)
   (if __stn4 (setq _stn4 __stn4)) _stn4)
  (:stn5
   (&optional __stn5)
   (if __stn5 (setq _stn5 __stn5)) _stn5)
  (:stn6
   (&optional __stn6)
   (if __stn6 (setq _stn6 __stn6)) _stn6)
  (:stn7
   (&optional __stn7)
   (if __stn7 (setq _stn7 __stn7)) _stn7)
  (:stn8
   (&optional __stn8)
   (if __stn8 (setq _stn8 __stn8)) _stn8)
  (:stn9
   (&optional __stn9)
   (if __stn9 (setq _stn9 __stn9)) _stn9)
  (:batlevel
   (&optional __batlevel)
   (if __batlevel (setq _batlevel __batlevel)) _batlevel)
  (:laserstatus
   (&optional __laserstatus)
   (if __laserstatus (setq _laserstatus __laserstatus)) _laserstatus)
  (:errorlog
   (&optional __errorlog)
   (if __errorlog (setq _errorlog __errorlog)) _errorlog)
  (:serialization-length
   ()
   (+
    ;; int8 _stn0
    1
    ;; int8 _stn1
    1
    ;; int8 _stn2
    1
    ;; int8 _stn3
    1
    ;; int8 _stn4
    1
    ;; int8 _stn5
    1
    ;; int8 _stn6
    1
    ;; int8 _stn7
    1
    ;; int8 _stn8
    1
    ;; int8 _stn9
    1
    ;; float32 _batlevel
    4
    ;; string _laserstatus
    4 (length _laserstatus)
    ;; string _errorlog
    4 (length _errorlog)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _stn0
       (write-byte _stn0 s)
     ;; int8 _stn1
       (write-byte _stn1 s)
     ;; int8 _stn2
       (write-byte _stn2 s)
     ;; int8 _stn3
       (write-byte _stn3 s)
     ;; int8 _stn4
       (write-byte _stn4 s)
     ;; int8 _stn5
       (write-byte _stn5 s)
     ;; int8 _stn6
       (write-byte _stn6 s)
     ;; int8 _stn7
       (write-byte _stn7 s)
     ;; int8 _stn8
       (write-byte _stn8 s)
     ;; int8 _stn9
       (write-byte _stn9 s)
     ;; float32 _batlevel
       (sys::poke _batlevel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _laserstatus
       (write-long (length _laserstatus) s) (princ _laserstatus s)
     ;; string _errorlog
       (write-long (length _errorlog) s) (princ _errorlog s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _stn0
     (setq _stn0 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stn0 127) (setq _stn0 (- _stn0 256)))
   ;; int8 _stn1
     (setq _stn1 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stn1 127) (setq _stn1 (- _stn1 256)))
   ;; int8 _stn2
     (setq _stn2 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stn2 127) (setq _stn2 (- _stn2 256)))
   ;; int8 _stn3
     (setq _stn3 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stn3 127) (setq _stn3 (- _stn3 256)))
   ;; int8 _stn4
     (setq _stn4 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stn4 127) (setq _stn4 (- _stn4 256)))
   ;; int8 _stn5
     (setq _stn5 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stn5 127) (setq _stn5 (- _stn5 256)))
   ;; int8 _stn6
     (setq _stn6 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stn6 127) (setq _stn6 (- _stn6 256)))
   ;; int8 _stn7
     (setq _stn7 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stn7 127) (setq _stn7 (- _stn7 256)))
   ;; int8 _stn8
     (setq _stn8 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stn8 127) (setq _stn8 (- _stn8 256)))
   ;; int8 _stn9
     (setq _stn9 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stn9 127) (setq _stn9 (- _stn9 256)))
   ;; float32 _batlevel
     (setq _batlevel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _laserstatus
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _laserstatus (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _errorlog
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _errorlog (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get htbot::lumstatus :md5sum-) "f8b3c1afcfd978c9fc81e921d6f4109c")
(setf (get htbot::lumstatus :datatype-) "htbot/lumstatus")
(setf (get htbot::lumstatus :definition-)
      "int8 stn0
int8 stn1
int8 stn2
int8 stn3
int8 stn4
int8 stn5
int8 stn6
int8 stn7
int8 stn8
int8 stn9
float32 batlevel
string laserstatus
string errorlog

")



(provide :htbot/lumstatus "f8b3c1afcfd978c9fc81e921d6f4109c")


