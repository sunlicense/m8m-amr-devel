;; Auto-generated. Do not edit!


(when (boundp 'htbot::mqueue)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'mqueue (find-package "HTBOT")))
(unless (find-package "HTBOT::MQUEUE")
  (make-package "HTBOT::MQUEUE"))
(unless (find-package "HTBOT::MQUEUEREQUEST")
  (make-package "HTBOT::MQUEUEREQUEST"))
(unless (find-package "HTBOT::MQUEUERESPONSE")
  (make-package "HTBOT::MQUEUERESPONSE"))

(in-package "ROS")





(defclass htbot::mqueueRequest
  :super ros::object
  :slots (_cmd _LP _GN _LPT _gps _lps _pw _tx _ty _tz _rx _ry _rz _rw _prd _pra _psd _psa _prd1 _pra1 _psd1 _psa1 _align _func _trolley _trolleydist ))

(defmethod htbot::mqueueRequest
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:LP __LP) 0)
    ((:GN __GN) 0)
    ((:LPT __LPT) 0)
    ((:gps __gps) "")
    ((:lps __lps) "")
    ((:pw __pw) "")
    ((:tx __tx) 0.0)
    ((:ty __ty) 0.0)
    ((:tz __tz) 0.0)
    ((:rx __rx) 0.0)
    ((:ry __ry) 0.0)
    ((:rz __rz) 0.0)
    ((:rw __rw) 0.0)
    ((:prd __prd) 0.0)
    ((:pra __pra) 0.0)
    ((:psd __psd) 0.0)
    ((:psa __psa) 0.0)
    ((:prd1 __prd1) 0.0)
    ((:pra1 __pra1) 0.0)
    ((:psd1 __psd1) 0.0)
    ((:psa1 __psa1) 0.0)
    ((:align __align) 0.0)
    ((:func __func) 0.0)
    ((:trolley __trolley) 0.0)
    ((:trolleydist __trolleydist) 0.0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _LP (round __LP))
   (setq _GN (round __GN))
   (setq _LPT (round __LPT))
   (setq _gps (string __gps))
   (setq _lps (string __lps))
   (setq _pw (string __pw))
   (setq _tx (float __tx))
   (setq _ty (float __ty))
   (setq _tz (float __tz))
   (setq _rx (float __rx))
   (setq _ry (float __ry))
   (setq _rz (float __rz))
   (setq _rw (float __rw))
   (setq _prd (float __prd))
   (setq _pra (float __pra))
   (setq _psd (float __psd))
   (setq _psa (float __psa))
   (setq _prd1 (float __prd1))
   (setq _pra1 (float __pra1))
   (setq _psd1 (float __psd1))
   (setq _psa1 (float __psa1))
   (setq _align (float __align))
   (setq _func (float __func))
   (setq _trolley (float __trolley))
   (setq _trolleydist (float __trolleydist))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:LP
   (&optional __LP)
   (if __LP (setq _LP __LP)) _LP)
  (:GN
   (&optional __GN)
   (if __GN (setq _GN __GN)) _GN)
  (:LPT
   (&optional __LPT)
   (if __LPT (setq _LPT __LPT)) _LPT)
  (:gps
   (&optional __gps)
   (if __gps (setq _gps __gps)) _gps)
  (:lps
   (&optional __lps)
   (if __lps (setq _lps __lps)) _lps)
  (:pw
   (&optional __pw)
   (if __pw (setq _pw __pw)) _pw)
  (:tx
   (&optional __tx)
   (if __tx (setq _tx __tx)) _tx)
  (:ty
   (&optional __ty)
   (if __ty (setq _ty __ty)) _ty)
  (:tz
   (&optional __tz)
   (if __tz (setq _tz __tz)) _tz)
  (:rx
   (&optional __rx)
   (if __rx (setq _rx __rx)) _rx)
  (:ry
   (&optional __ry)
   (if __ry (setq _ry __ry)) _ry)
  (:rz
   (&optional __rz)
   (if __rz (setq _rz __rz)) _rz)
  (:rw
   (&optional __rw)
   (if __rw (setq _rw __rw)) _rw)
  (:prd
   (&optional __prd)
   (if __prd (setq _prd __prd)) _prd)
  (:pra
   (&optional __pra)
   (if __pra (setq _pra __pra)) _pra)
  (:psd
   (&optional __psd)
   (if __psd (setq _psd __psd)) _psd)
  (:psa
   (&optional __psa)
   (if __psa (setq _psa __psa)) _psa)
  (:prd1
   (&optional __prd1)
   (if __prd1 (setq _prd1 __prd1)) _prd1)
  (:pra1
   (&optional __pra1)
   (if __pra1 (setq _pra1 __pra1)) _pra1)
  (:psd1
   (&optional __psd1)
   (if __psd1 (setq _psd1 __psd1)) _psd1)
  (:psa1
   (&optional __psa1)
   (if __psa1 (setq _psa1 __psa1)) _psa1)
  (:align
   (&optional __align)
   (if __align (setq _align __align)) _align)
  (:func
   (&optional __func)
   (if __func (setq _func __func)) _func)
  (:trolley
   (&optional __trolley)
   (if __trolley (setq _trolley __trolley)) _trolley)
  (:trolleydist
   (&optional __trolleydist)
   (if __trolleydist (setq _trolleydist __trolleydist)) _trolleydist)
  (:serialization-length
   ()
   (+
    ;; int16 _cmd
    2
    ;; int8 _LP
    1
    ;; int8 _GN
    1
    ;; int8 _LPT
    1
    ;; string _gps
    4 (length _gps)
    ;; string _lps
    4 (length _lps)
    ;; string _pw
    4 (length _pw)
    ;; float32 _tx
    4
    ;; float32 _ty
    4
    ;; float32 _tz
    4
    ;; float32 _rx
    4
    ;; float32 _ry
    4
    ;; float32 _rz
    4
    ;; float32 _rw
    4
    ;; float32 _prd
    4
    ;; float32 _pra
    4
    ;; float32 _psd
    4
    ;; float32 _psa
    4
    ;; float32 _prd1
    4
    ;; float32 _pra1
    4
    ;; float32 _psd1
    4
    ;; float32 _psa1
    4
    ;; float32 _align
    4
    ;; float32 _func
    4
    ;; float32 _trolley
    4
    ;; float32 _trolleydist
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _cmd
       (write-word _cmd s)
     ;; int8 _LP
       (write-byte _LP s)
     ;; int8 _GN
       (write-byte _GN s)
     ;; int8 _LPT
       (write-byte _LPT s)
     ;; string _gps
       (write-long (length _gps) s) (princ _gps s)
     ;; string _lps
       (write-long (length _lps) s) (princ _lps s)
     ;; string _pw
       (write-long (length _pw) s) (princ _pw s)
     ;; float32 _tx
       (sys::poke _tx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ty
       (sys::poke _ty (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tz
       (sys::poke _tz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rx
       (sys::poke _rx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ry
       (sys::poke _ry (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rz
       (sys::poke _rz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rw
       (sys::poke _rw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _prd
       (sys::poke _prd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pra
       (sys::poke _pra (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psd
       (sys::poke _psd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psa
       (sys::poke _psa (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _prd1
       (sys::poke _prd1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pra1
       (sys::poke _pra1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psd1
       (sys::poke _psd1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psa1
       (sys::poke _psa1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _align
       (sys::poke _align (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _func
       (sys::poke _func (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _trolley
       (sys::poke _trolley (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _trolleydist
       (sys::poke _trolleydist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _cmd
     (setq _cmd (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int8 _LP
     (setq _LP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _LP 127) (setq _LP (- _LP 256)))
   ;; int8 _GN
     (setq _GN (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _GN 127) (setq _GN (- _GN 256)))
   ;; int8 _LPT
     (setq _LPT (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _LPT 127) (setq _LPT (- _LPT 256)))
   ;; string _gps
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gps (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lps
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lps (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _pw
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _pw (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _tx
     (setq _tx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ty
     (setq _ty (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tz
     (setq _tz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rx
     (setq _rx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ry
     (setq _ry (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rz
     (setq _rz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rw
     (setq _rw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _prd
     (setq _prd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pra
     (setq _pra (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psd
     (setq _psd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psa
     (setq _psa (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _prd1
     (setq _prd1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pra1
     (setq _pra1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psd1
     (setq _psd1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psa1
     (setq _psa1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _align
     (setq _align (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _func
     (setq _func (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _trolley
     (setq _trolley (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _trolleydist
     (setq _trolleydist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass htbot::mqueueResponse
  :super ros::object
  :slots (_status _tx _ty _tz _rx _ry _rz _rw _prd _pra _psd _psa _prd1 _pra1 _psd1 _psa1 _LP _cLP _nGP _cGN _nIQ _marked _gps _lps _lps1 _lps2 _lps3 _lps4 _lps5 _lps6 _lps7 _lps8 _marked1 _marked2 _marked3 _marked4 _marked5 _marked6 _marked7 _marked8 _align _func _trolley _trolleydist ))

(defmethod htbot::mqueueResponse
  (:init
   (&key
    ((:status __status) 0)
    ((:tx __tx) 0.0)
    ((:ty __ty) 0.0)
    ((:tz __tz) 0.0)
    ((:rx __rx) 0.0)
    ((:ry __ry) 0.0)
    ((:rz __rz) 0.0)
    ((:rw __rw) 0.0)
    ((:prd __prd) 0.0)
    ((:pra __pra) 0.0)
    ((:psd __psd) 0.0)
    ((:psa __psa) 0.0)
    ((:prd1 __prd1) 0.0)
    ((:pra1 __pra1) 0.0)
    ((:psd1 __psd1) 0.0)
    ((:psa1 __psa1) 0.0)
    ((:LP __LP) 0)
    ((:cLP __cLP) 0)
    ((:nGP __nGP) 0)
    ((:cGN __cGN) 0)
    ((:nIQ __nIQ) 0)
    ((:marked __marked) 0)
    ((:gps __gps) "")
    ((:lps __lps) "")
    ((:lps1 __lps1) "")
    ((:lps2 __lps2) "")
    ((:lps3 __lps3) "")
    ((:lps4 __lps4) "")
    ((:lps5 __lps5) "")
    ((:lps6 __lps6) "")
    ((:lps7 __lps7) "")
    ((:lps8 __lps8) "")
    ((:marked1 __marked1) 0)
    ((:marked2 __marked2) 0)
    ((:marked3 __marked3) 0)
    ((:marked4 __marked4) 0)
    ((:marked5 __marked5) 0)
    ((:marked6 __marked6) 0)
    ((:marked7 __marked7) 0)
    ((:marked8 __marked8) 0)
    ((:align __align) 0.0)
    ((:func __func) 0.0)
    ((:trolley __trolley) 0.0)
    ((:trolleydist __trolleydist) 0.0)
    )
   (send-super :init)
   (setq _status (round __status))
   (setq _tx (float __tx))
   (setq _ty (float __ty))
   (setq _tz (float __tz))
   (setq _rx (float __rx))
   (setq _ry (float __ry))
   (setq _rz (float __rz))
   (setq _rw (float __rw))
   (setq _prd (float __prd))
   (setq _pra (float __pra))
   (setq _psd (float __psd))
   (setq _psa (float __psa))
   (setq _prd1 (float __prd1))
   (setq _pra1 (float __pra1))
   (setq _psd1 (float __psd1))
   (setq _psa1 (float __psa1))
   (setq _LP (round __LP))
   (setq _cLP (round __cLP))
   (setq _nGP (round __nGP))
   (setq _cGN (round __cGN))
   (setq _nIQ (round __nIQ))
   (setq _marked (round __marked))
   (setq _gps (string __gps))
   (setq _lps (string __lps))
   (setq _lps1 (string __lps1))
   (setq _lps2 (string __lps2))
   (setq _lps3 (string __lps3))
   (setq _lps4 (string __lps4))
   (setq _lps5 (string __lps5))
   (setq _lps6 (string __lps6))
   (setq _lps7 (string __lps7))
   (setq _lps8 (string __lps8))
   (setq _marked1 (round __marked1))
   (setq _marked2 (round __marked2))
   (setq _marked3 (round __marked3))
   (setq _marked4 (round __marked4))
   (setq _marked5 (round __marked5))
   (setq _marked6 (round __marked6))
   (setq _marked7 (round __marked7))
   (setq _marked8 (round __marked8))
   (setq _align (float __align))
   (setq _func (float __func))
   (setq _trolley (float __trolley))
   (setq _trolleydist (float __trolleydist))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:tx
   (&optional __tx)
   (if __tx (setq _tx __tx)) _tx)
  (:ty
   (&optional __ty)
   (if __ty (setq _ty __ty)) _ty)
  (:tz
   (&optional __tz)
   (if __tz (setq _tz __tz)) _tz)
  (:rx
   (&optional __rx)
   (if __rx (setq _rx __rx)) _rx)
  (:ry
   (&optional __ry)
   (if __ry (setq _ry __ry)) _ry)
  (:rz
   (&optional __rz)
   (if __rz (setq _rz __rz)) _rz)
  (:rw
   (&optional __rw)
   (if __rw (setq _rw __rw)) _rw)
  (:prd
   (&optional __prd)
   (if __prd (setq _prd __prd)) _prd)
  (:pra
   (&optional __pra)
   (if __pra (setq _pra __pra)) _pra)
  (:psd
   (&optional __psd)
   (if __psd (setq _psd __psd)) _psd)
  (:psa
   (&optional __psa)
   (if __psa (setq _psa __psa)) _psa)
  (:prd1
   (&optional __prd1)
   (if __prd1 (setq _prd1 __prd1)) _prd1)
  (:pra1
   (&optional __pra1)
   (if __pra1 (setq _pra1 __pra1)) _pra1)
  (:psd1
   (&optional __psd1)
   (if __psd1 (setq _psd1 __psd1)) _psd1)
  (:psa1
   (&optional __psa1)
   (if __psa1 (setq _psa1 __psa1)) _psa1)
  (:LP
   (&optional __LP)
   (if __LP (setq _LP __LP)) _LP)
  (:cLP
   (&optional __cLP)
   (if __cLP (setq _cLP __cLP)) _cLP)
  (:nGP
   (&optional __nGP)
   (if __nGP (setq _nGP __nGP)) _nGP)
  (:cGN
   (&optional __cGN)
   (if __cGN (setq _cGN __cGN)) _cGN)
  (:nIQ
   (&optional __nIQ)
   (if __nIQ (setq _nIQ __nIQ)) _nIQ)
  (:marked
   (&optional __marked)
   (if __marked (setq _marked __marked)) _marked)
  (:gps
   (&optional __gps)
   (if __gps (setq _gps __gps)) _gps)
  (:lps
   (&optional __lps)
   (if __lps (setq _lps __lps)) _lps)
  (:lps1
   (&optional __lps1)
   (if __lps1 (setq _lps1 __lps1)) _lps1)
  (:lps2
   (&optional __lps2)
   (if __lps2 (setq _lps2 __lps2)) _lps2)
  (:lps3
   (&optional __lps3)
   (if __lps3 (setq _lps3 __lps3)) _lps3)
  (:lps4
   (&optional __lps4)
   (if __lps4 (setq _lps4 __lps4)) _lps4)
  (:lps5
   (&optional __lps5)
   (if __lps5 (setq _lps5 __lps5)) _lps5)
  (:lps6
   (&optional __lps6)
   (if __lps6 (setq _lps6 __lps6)) _lps6)
  (:lps7
   (&optional __lps7)
   (if __lps7 (setq _lps7 __lps7)) _lps7)
  (:lps8
   (&optional __lps8)
   (if __lps8 (setq _lps8 __lps8)) _lps8)
  (:marked1
   (&optional __marked1)
   (if __marked1 (setq _marked1 __marked1)) _marked1)
  (:marked2
   (&optional __marked2)
   (if __marked2 (setq _marked2 __marked2)) _marked2)
  (:marked3
   (&optional __marked3)
   (if __marked3 (setq _marked3 __marked3)) _marked3)
  (:marked4
   (&optional __marked4)
   (if __marked4 (setq _marked4 __marked4)) _marked4)
  (:marked5
   (&optional __marked5)
   (if __marked5 (setq _marked5 __marked5)) _marked5)
  (:marked6
   (&optional __marked6)
   (if __marked6 (setq _marked6 __marked6)) _marked6)
  (:marked7
   (&optional __marked7)
   (if __marked7 (setq _marked7 __marked7)) _marked7)
  (:marked8
   (&optional __marked8)
   (if __marked8 (setq _marked8 __marked8)) _marked8)
  (:align
   (&optional __align)
   (if __align (setq _align __align)) _align)
  (:func
   (&optional __func)
   (if __func (setq _func __func)) _func)
  (:trolley
   (&optional __trolley)
   (if __trolley (setq _trolley __trolley)) _trolley)
  (:trolleydist
   (&optional __trolleydist)
   (if __trolleydist (setq _trolleydist __trolleydist)) _trolleydist)
  (:serialization-length
   ()
   (+
    ;; int16 _status
    2
    ;; float32 _tx
    4
    ;; float32 _ty
    4
    ;; float32 _tz
    4
    ;; float32 _rx
    4
    ;; float32 _ry
    4
    ;; float32 _rz
    4
    ;; float32 _rw
    4
    ;; float32 _prd
    4
    ;; float32 _pra
    4
    ;; float32 _psd
    4
    ;; float32 _psa
    4
    ;; float32 _prd1
    4
    ;; float32 _pra1
    4
    ;; float32 _psd1
    4
    ;; float32 _psa1
    4
    ;; int8 _LP
    1
    ;; int8 _cLP
    1
    ;; int8 _nGP
    1
    ;; int8 _cGN
    1
    ;; int8 _nIQ
    1
    ;; int8 _marked
    1
    ;; string _gps
    4 (length _gps)
    ;; string _lps
    4 (length _lps)
    ;; string _lps1
    4 (length _lps1)
    ;; string _lps2
    4 (length _lps2)
    ;; string _lps3
    4 (length _lps3)
    ;; string _lps4
    4 (length _lps4)
    ;; string _lps5
    4 (length _lps5)
    ;; string _lps6
    4 (length _lps6)
    ;; string _lps7
    4 (length _lps7)
    ;; string _lps8
    4 (length _lps8)
    ;; int8 _marked1
    1
    ;; int8 _marked2
    1
    ;; int8 _marked3
    1
    ;; int8 _marked4
    1
    ;; int8 _marked5
    1
    ;; int8 _marked6
    1
    ;; int8 _marked7
    1
    ;; int8 _marked8
    1
    ;; float32 _align
    4
    ;; float32 _func
    4
    ;; float32 _trolley
    4
    ;; float32 _trolleydist
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _status
       (write-word _status s)
     ;; float32 _tx
       (sys::poke _tx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ty
       (sys::poke _ty (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tz
       (sys::poke _tz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rx
       (sys::poke _rx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ry
       (sys::poke _ry (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rz
       (sys::poke _rz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rw
       (sys::poke _rw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _prd
       (sys::poke _prd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pra
       (sys::poke _pra (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psd
       (sys::poke _psd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psa
       (sys::poke _psa (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _prd1
       (sys::poke _prd1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pra1
       (sys::poke _pra1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psd1
       (sys::poke _psd1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psa1
       (sys::poke _psa1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _LP
       (write-byte _LP s)
     ;; int8 _cLP
       (write-byte _cLP s)
     ;; int8 _nGP
       (write-byte _nGP s)
     ;; int8 _cGN
       (write-byte _cGN s)
     ;; int8 _nIQ
       (write-byte _nIQ s)
     ;; int8 _marked
       (write-byte _marked s)
     ;; string _gps
       (write-long (length _gps) s) (princ _gps s)
     ;; string _lps
       (write-long (length _lps) s) (princ _lps s)
     ;; string _lps1
       (write-long (length _lps1) s) (princ _lps1 s)
     ;; string _lps2
       (write-long (length _lps2) s) (princ _lps2 s)
     ;; string _lps3
       (write-long (length _lps3) s) (princ _lps3 s)
     ;; string _lps4
       (write-long (length _lps4) s) (princ _lps4 s)
     ;; string _lps5
       (write-long (length _lps5) s) (princ _lps5 s)
     ;; string _lps6
       (write-long (length _lps6) s) (princ _lps6 s)
     ;; string _lps7
       (write-long (length _lps7) s) (princ _lps7 s)
     ;; string _lps8
       (write-long (length _lps8) s) (princ _lps8 s)
     ;; int8 _marked1
       (write-byte _marked1 s)
     ;; int8 _marked2
       (write-byte _marked2 s)
     ;; int8 _marked3
       (write-byte _marked3 s)
     ;; int8 _marked4
       (write-byte _marked4 s)
     ;; int8 _marked5
       (write-byte _marked5 s)
     ;; int8 _marked6
       (write-byte _marked6 s)
     ;; int8 _marked7
       (write-byte _marked7 s)
     ;; int8 _marked8
       (write-byte _marked8 s)
     ;; float32 _align
       (sys::poke _align (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _func
       (sys::poke _func (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _trolley
       (sys::poke _trolley (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _trolleydist
       (sys::poke _trolleydist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _status
     (setq _status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _tx
     (setq _tx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ty
     (setq _ty (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tz
     (setq _tz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rx
     (setq _rx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ry
     (setq _ry (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rz
     (setq _rz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rw
     (setq _rw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _prd
     (setq _prd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pra
     (setq _pra (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psd
     (setq _psd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psa
     (setq _psa (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _prd1
     (setq _prd1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pra1
     (setq _pra1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psd1
     (setq _psd1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psa1
     (setq _psa1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _LP
     (setq _LP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _LP 127) (setq _LP (- _LP 256)))
   ;; int8 _cLP
     (setq _cLP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cLP 127) (setq _cLP (- _cLP 256)))
   ;; int8 _nGP
     (setq _nGP (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _nGP 127) (setq _nGP (- _nGP 256)))
   ;; int8 _cGN
     (setq _cGN (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cGN 127) (setq _cGN (- _cGN 256)))
   ;; int8 _nIQ
     (setq _nIQ (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _nIQ 127) (setq _nIQ (- _nIQ 256)))
   ;; int8 _marked
     (setq _marked (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _marked 127) (setq _marked (- _marked 256)))
   ;; string _gps
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gps (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lps
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lps (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lps1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lps1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lps2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lps2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lps3
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lps3 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lps4
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lps4 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lps5
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lps5 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lps6
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lps6 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lps7
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lps7 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lps8
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lps8 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _marked1
     (setq _marked1 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _marked1 127) (setq _marked1 (- _marked1 256)))
   ;; int8 _marked2
     (setq _marked2 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _marked2 127) (setq _marked2 (- _marked2 256)))
   ;; int8 _marked3
     (setq _marked3 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _marked3 127) (setq _marked3 (- _marked3 256)))
   ;; int8 _marked4
     (setq _marked4 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _marked4 127) (setq _marked4 (- _marked4 256)))
   ;; int8 _marked5
     (setq _marked5 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _marked5 127) (setq _marked5 (- _marked5 256)))
   ;; int8 _marked6
     (setq _marked6 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _marked6 127) (setq _marked6 (- _marked6 256)))
   ;; int8 _marked7
     (setq _marked7 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _marked7 127) (setq _marked7 (- _marked7 256)))
   ;; int8 _marked8
     (setq _marked8 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _marked8 127) (setq _marked8 (- _marked8 256)))
   ;; float32 _align
     (setq _align (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _func
     (setq _func (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _trolley
     (setq _trolley (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _trolleydist
     (setq _trolleydist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass htbot::mqueue
  :super ros::object
  :slots ())

(setf (get htbot::mqueue :md5sum-) "688bcf05bb0a3ea18504032392975cc6")
(setf (get htbot::mqueue :datatype-) "htbot/mqueue")
(setf (get htbot::mqueue :request) htbot::mqueueRequest)
(setf (get htbot::mqueue :response) htbot::mqueueResponse)

(defmethod htbot::mqueueRequest
  (:response () (instance htbot::mqueueResponse :init)))

(setf (get htbot::mqueueRequest :md5sum-) "688bcf05bb0a3ea18504032392975cc6")
(setf (get htbot::mqueueRequest :datatype-) "htbot/mqueueRequest")
(setf (get htbot::mqueueRequest :definition-)
      "int16 cmd
int8 LP
int8 GN
int8 LPT
string gps
string lps
string pw
float32 tx
float32 ty
float32 tz
float32 rx
float32 ry
float32 rz
float32 rw
float32 prd
float32 pra
float32 psd
float32 psa
float32 prd1
float32 pra1
float32 psd1
float32 psa1
float32 align
float32 func
float32 trolley
float32 trolleydist
---
int16 status
float32 tx
float32 ty
float32 tz
float32 rx
float32 ry
float32 rz
float32 rw
float32 prd
float32 pra
float32 psd
float32 psa
float32 prd1
float32 pra1
float32 psd1
float32 psa1
int8 LP
int8 cLP
int8 nGP
int8 cGN
int8 nIQ
int8 marked
string gps
string lps
string lps1
string lps2
string lps3
string lps4
string lps5
string lps6
string lps7
string lps8
int8 marked1
int8 marked2
int8 marked3
int8 marked4
int8 marked5
int8 marked6
int8 marked7
int8 marked8
float32 align
float32 func
float32 trolley
float32 trolleydist

")

(setf (get htbot::mqueueResponse :md5sum-) "688bcf05bb0a3ea18504032392975cc6")
(setf (get htbot::mqueueResponse :datatype-) "htbot/mqueueResponse")
(setf (get htbot::mqueueResponse :definition-)
      "int16 cmd
int8 LP
int8 GN
int8 LPT
string gps
string lps
string pw
float32 tx
float32 ty
float32 tz
float32 rx
float32 ry
float32 rz
float32 rw
float32 prd
float32 pra
float32 psd
float32 psa
float32 prd1
float32 pra1
float32 psd1
float32 psa1
float32 align
float32 func
float32 trolley
float32 trolleydist
---
int16 status
float32 tx
float32 ty
float32 tz
float32 rx
float32 ry
float32 rz
float32 rw
float32 prd
float32 pra
float32 psd
float32 psa
float32 prd1
float32 pra1
float32 psd1
float32 psa1
int8 LP
int8 cLP
int8 nGP
int8 cGN
int8 nIQ
int8 marked
string gps
string lps
string lps1
string lps2
string lps3
string lps4
string lps5
string lps6
string lps7
string lps8
int8 marked1
int8 marked2
int8 marked3
int8 marked4
int8 marked5
int8 marked6
int8 marked7
int8 marked8
float32 align
float32 func
float32 trolley
float32 trolleydist

")



(provide :htbot/mqueue "688bcf05bb0a3ea18504032392975cc6")


