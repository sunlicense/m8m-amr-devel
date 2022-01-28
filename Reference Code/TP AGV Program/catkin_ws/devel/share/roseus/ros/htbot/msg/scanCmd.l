;; Auto-generated. Do not edit!


(when (boundp 'htbot::scanCmd)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'scanCmd (find-package "HTBOT")))
(unless (find-package "HTBOT::SCANCMD")
  (make-package "HTBOT::SCANCMD"))

(in-package "ROS")
;;//! \htmlinclude scanCmd.msg.html


(defclass htbot::scanCmd
  :super ros::object
  :slots (_cmd _file _f1 _f2 _f3 _lp _gp _opt ))

(defmethod htbot::scanCmd
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:file __file) "")
    ((:f1 __f1) 0.0)
    ((:f2 __f2) 0.0)
    ((:f3 __f3) 0.0)
    ((:lp __lp) 0)
    ((:gp __gp) 0)
    ((:opt __opt) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _file (string __file))
   (setq _f1 (float __f1))
   (setq _f2 (float __f2))
   (setq _f3 (float __f3))
   (setq _lp (round __lp))
   (setq _gp (round __gp))
   (setq _opt (round __opt))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:file
   (&optional __file)
   (if __file (setq _file __file)) _file)
  (:f1
   (&optional __f1)
   (if __f1 (setq _f1 __f1)) _f1)
  (:f2
   (&optional __f2)
   (if __f2 (setq _f2 __f2)) _f2)
  (:f3
   (&optional __f3)
   (if __f3 (setq _f3 __f3)) _f3)
  (:lp
   (&optional __lp)
   (if __lp (setq _lp __lp)) _lp)
  (:gp
   (&optional __gp)
   (if __gp (setq _gp __gp)) _gp)
  (:opt
   (&optional __opt)
   (if __opt (setq _opt __opt)) _opt)
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ;; string _file
    4 (length _file)
    ;; float32 _f1
    4
    ;; float32 _f2
    4
    ;; float32 _f3
    4
    ;; int8 _lp
    1
    ;; int8 _gp
    1
    ;; int8 _opt
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _cmd
       (write-byte _cmd s)
     ;; string _file
       (write-long (length _file) s) (princ _file s)
     ;; float32 _f1
       (sys::poke _f1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _f2
       (sys::poke _f2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _f3
       (sys::poke _f3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _lp
       (write-byte _lp s)
     ;; int8 _gp
       (write-byte _gp s)
     ;; int8 _opt
       (write-byte _opt s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;; string _file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _f1
     (setq _f1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _f2
     (setq _f2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _f3
     (setq _f3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _lp
     (setq _lp (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _lp 127) (setq _lp (- _lp 256)))
   ;; int8 _gp
     (setq _gp (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _gp 127) (setq _gp (- _gp 256)))
   ;; int8 _opt
     (setq _opt (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _opt 127) (setq _opt (- _opt 256)))
   ;;
   self)
  )

(setf (get htbot::scanCmd :md5sum-) "009679f12681386d87811bbbbfbf00db")
(setf (get htbot::scanCmd :datatype-) "htbot/scanCmd")
(setf (get htbot::scanCmd :definition-)
      "int8 cmd
string file
float32 f1
float32 f2
float32 f3
int8 lp
int8 gp
int8 opt

")



(provide :htbot/scanCmd "009679f12681386d87811bbbbfbf00db")


