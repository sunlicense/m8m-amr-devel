;; Auto-generated. Do not edit!


(when (boundp 'htbot::queue)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'queue (find-package "HTBOT")))
(unless (find-package "HTBOT::QUEUE")
  (make-package "HTBOT::QUEUE"))

(in-package "ROS")
;;//! \htmlinclude queue.msg.html


(defclass htbot::queue
  :super ros::object
  :slots (_noQ _fLP1 _tLP1 _fLP2 _tLP2 _fLP3 _tLP3 _fLP4 _tLP4 _LPName _LPInfo ))

(defmethod htbot::queue
  (:init
   (&key
    ((:noQ __noQ) 0)
    ((:fLP1 __fLP1) "")
    ((:tLP1 __tLP1) "")
    ((:fLP2 __fLP2) "")
    ((:tLP2 __tLP2) "")
    ((:fLP3 __fLP3) "")
    ((:tLP3 __tLP3) "")
    ((:fLP4 __fLP4) "")
    ((:tLP4 __tLP4) "")
    ((:LPName __LPName) "")
    ((:LPInfo __LPInfo) "")
    )
   (send-super :init)
   (setq _noQ (round __noQ))
   (setq _fLP1 (string __fLP1))
   (setq _tLP1 (string __tLP1))
   (setq _fLP2 (string __fLP2))
   (setq _tLP2 (string __tLP2))
   (setq _fLP3 (string __fLP3))
   (setq _tLP3 (string __tLP3))
   (setq _fLP4 (string __fLP4))
   (setq _tLP4 (string __tLP4))
   (setq _LPName (string __LPName))
   (setq _LPInfo (string __LPInfo))
   self)
  (:noQ
   (&optional __noQ)
   (if __noQ (setq _noQ __noQ)) _noQ)
  (:fLP1
   (&optional __fLP1)
   (if __fLP1 (setq _fLP1 __fLP1)) _fLP1)
  (:tLP1
   (&optional __tLP1)
   (if __tLP1 (setq _tLP1 __tLP1)) _tLP1)
  (:fLP2
   (&optional __fLP2)
   (if __fLP2 (setq _fLP2 __fLP2)) _fLP2)
  (:tLP2
   (&optional __tLP2)
   (if __tLP2 (setq _tLP2 __tLP2)) _tLP2)
  (:fLP3
   (&optional __fLP3)
   (if __fLP3 (setq _fLP3 __fLP3)) _fLP3)
  (:tLP3
   (&optional __tLP3)
   (if __tLP3 (setq _tLP3 __tLP3)) _tLP3)
  (:fLP4
   (&optional __fLP4)
   (if __fLP4 (setq _fLP4 __fLP4)) _fLP4)
  (:tLP4
   (&optional __tLP4)
   (if __tLP4 (setq _tLP4 __tLP4)) _tLP4)
  (:LPName
   (&optional __LPName)
   (if __LPName (setq _LPName __LPName)) _LPName)
  (:LPInfo
   (&optional __LPInfo)
   (if __LPInfo (setq _LPInfo __LPInfo)) _LPInfo)
  (:serialization-length
   ()
   (+
    ;; int8 _noQ
    1
    ;; string _fLP1
    4 (length _fLP1)
    ;; string _tLP1
    4 (length _tLP1)
    ;; string _fLP2
    4 (length _fLP2)
    ;; string _tLP2
    4 (length _tLP2)
    ;; string _fLP3
    4 (length _fLP3)
    ;; string _tLP3
    4 (length _tLP3)
    ;; string _fLP4
    4 (length _fLP4)
    ;; string _tLP4
    4 (length _tLP4)
    ;; string _LPName
    4 (length _LPName)
    ;; string _LPInfo
    4 (length _LPInfo)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _noQ
       (write-byte _noQ s)
     ;; string _fLP1
       (write-long (length _fLP1) s) (princ _fLP1 s)
     ;; string _tLP1
       (write-long (length _tLP1) s) (princ _tLP1 s)
     ;; string _fLP2
       (write-long (length _fLP2) s) (princ _fLP2 s)
     ;; string _tLP2
       (write-long (length _tLP2) s) (princ _tLP2 s)
     ;; string _fLP3
       (write-long (length _fLP3) s) (princ _fLP3 s)
     ;; string _tLP3
       (write-long (length _tLP3) s) (princ _tLP3 s)
     ;; string _fLP4
       (write-long (length _fLP4) s) (princ _fLP4 s)
     ;; string _tLP4
       (write-long (length _tLP4) s) (princ _tLP4 s)
     ;; string _LPName
       (write-long (length _LPName) s) (princ _LPName s)
     ;; string _LPInfo
       (write-long (length _LPInfo) s) (princ _LPInfo s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _noQ
     (setq _noQ (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _noQ 127) (setq _noQ (- _noQ 256)))
   ;; string _fLP1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fLP1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _tLP1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tLP1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _fLP2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fLP2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _tLP2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tLP2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _fLP3
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fLP3 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _tLP3
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tLP3 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _fLP4
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fLP4 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _tLP4
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tLP4 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _LPName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _LPName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _LPInfo
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _LPInfo (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get htbot::queue :md5sum-) "1ad64500a08450047cb508b267d3d903")
(setf (get htbot::queue :datatype-) "htbot/queue")
(setf (get htbot::queue :definition-)
      "int8 noQ
string fLP1
string tLP1
string fLP2
string tLP2
string fLP3
string tLP3
string fLP4
string tLP4
string LPName
string LPInfo

")



(provide :htbot/queue "1ad64500a08450047cb508b267d3d903")


