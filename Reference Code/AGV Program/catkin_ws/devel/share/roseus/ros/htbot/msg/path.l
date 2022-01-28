;; Auto-generated. Do not edit!


(when (boundp 'htbot::path)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'path (find-package "HTBOT")))
(unless (find-package "HTBOT::PATH")
  (make-package "HTBOT::PATH"))

(in-package "ROS")
;;//! \htmlinclude path.msg.html


(defclass htbot::path
  :super ros::object
  :slots (_cmd _px _py _pz _prx _pry _prz _prw _gx _gy _gz _grx _gry _grz _grw _tol ))

(defmethod htbot::path
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:px __px) 0.0)
    ((:py __py) 0.0)
    ((:pz __pz) 0.0)
    ((:prx __prx) 0.0)
    ((:pry __pry) 0.0)
    ((:prz __prz) 0.0)
    ((:prw __prw) 0.0)
    ((:gx __gx) 0.0)
    ((:gy __gy) 0.0)
    ((:gz __gz) 0.0)
    ((:grx __grx) 0.0)
    ((:gry __gry) 0.0)
    ((:grz __grz) 0.0)
    ((:grw __grw) 0.0)
    ((:tol __tol) 0.0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _px (float __px))
   (setq _py (float __py))
   (setq _pz (float __pz))
   (setq _prx (float __prx))
   (setq _pry (float __pry))
   (setq _prz (float __prz))
   (setq _prw (float __prw))
   (setq _gx (float __gx))
   (setq _gy (float __gy))
   (setq _gz (float __gz))
   (setq _grx (float __grx))
   (setq _gry (float __gry))
   (setq _grz (float __grz))
   (setq _grw (float __grw))
   (setq _tol (float __tol))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:px
   (&optional __px)
   (if __px (setq _px __px)) _px)
  (:py
   (&optional __py)
   (if __py (setq _py __py)) _py)
  (:pz
   (&optional __pz)
   (if __pz (setq _pz __pz)) _pz)
  (:prx
   (&optional __prx)
   (if __prx (setq _prx __prx)) _prx)
  (:pry
   (&optional __pry)
   (if __pry (setq _pry __pry)) _pry)
  (:prz
   (&optional __prz)
   (if __prz (setq _prz __prz)) _prz)
  (:prw
   (&optional __prw)
   (if __prw (setq _prw __prw)) _prw)
  (:gx
   (&optional __gx)
   (if __gx (setq _gx __gx)) _gx)
  (:gy
   (&optional __gy)
   (if __gy (setq _gy __gy)) _gy)
  (:gz
   (&optional __gz)
   (if __gz (setq _gz __gz)) _gz)
  (:grx
   (&optional __grx)
   (if __grx (setq _grx __grx)) _grx)
  (:gry
   (&optional __gry)
   (if __gry (setq _gry __gry)) _gry)
  (:grz
   (&optional __grz)
   (if __grz (setq _grz __grz)) _grz)
  (:grw
   (&optional __grw)
   (if __grw (setq _grw __grw)) _grw)
  (:tol
   (&optional __tol)
   (if __tol (setq _tol __tol)) _tol)
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ;; float32 _px
    4
    ;; float32 _py
    4
    ;; float32 _pz
    4
    ;; float32 _prx
    4
    ;; float32 _pry
    4
    ;; float32 _prz
    4
    ;; float32 _prw
    4
    ;; float32 _gx
    4
    ;; float32 _gy
    4
    ;; float32 _gz
    4
    ;; float32 _grx
    4
    ;; float32 _gry
    4
    ;; float32 _grz
    4
    ;; float32 _grw
    4
    ;; float32 _tol
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _cmd
       (write-byte _cmd s)
     ;; float32 _px
       (sys::poke _px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _py
       (sys::poke _py (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pz
       (sys::poke _pz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _prx
       (sys::poke _prx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pry
       (sys::poke _pry (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _prz
       (sys::poke _prz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _prw
       (sys::poke _prw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gx
       (sys::poke _gx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gy
       (sys::poke _gy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gz
       (sys::poke _gz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _grx
       (sys::poke _grx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gry
       (sys::poke _gry (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _grz
       (sys::poke _grz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _grw
       (sys::poke _grw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tol
       (sys::poke _tol (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;; float32 _px
     (setq _px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _py
     (setq _py (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pz
     (setq _pz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _prx
     (setq _prx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pry
     (setq _pry (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _prz
     (setq _prz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _prw
     (setq _prw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gx
     (setq _gx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gy
     (setq _gy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gz
     (setq _gz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _grx
     (setq _grx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gry
     (setq _gry (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _grz
     (setq _grz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _grw
     (setq _grw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tol
     (setq _tol (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get htbot::path :md5sum-) "0d2fce4d2355f03d0ce87b3bd4babb5e")
(setf (get htbot::path :datatype-) "htbot/path")
(setf (get htbot::path :definition-)
      "int8 cmd
float32 px
float32 py
float32 pz
float32 prx
float32 pry
float32 prz
float32 prw
float32 gx
float32 gy
float32 gz
float32 grx
float32 gry
float32 grz
float32 grw
float32 tol

")



(provide :htbot/path "0d2fce4d2355f03d0ce87b3bd4babb5e")


