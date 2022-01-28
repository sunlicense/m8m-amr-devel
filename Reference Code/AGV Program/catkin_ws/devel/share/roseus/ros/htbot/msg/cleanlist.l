;; Auto-generated. Do not edit!


(when (boundp 'htbot::cleanlist)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'cleanlist (find-package "HTBOT")))
(unless (find-package "HTBOT::CLEANLIST")
  (make-package "HTBOT::CLEANLIST"))

(in-package "ROS")
;;//! \htmlinclude cleanlist.msg.html


(defclass htbot::cleanlist
  :super ros::object
  :slots (_info _mapno _cplanno _mapdir _mapname _cplandir _cleanplan ))

(defmethod htbot::cleanlist
  (:init
   (&key
    ((:info __info) 0)
    ((:mapno __mapno) 0)
    ((:cplanno __cplanno) 0)
    ((:mapdir __mapdir) "")
    ((:mapname __mapname) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:cplandir __cplandir) "")
    ((:cleanplan __cleanplan) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _info (round __info))
   (setq _mapno (round __mapno))
   (setq _cplanno (round __cplanno))
   (setq _mapdir (string __mapdir))
   (setq _mapname __mapname)
   (setq _cplandir (string __cplandir))
   (setq _cleanplan __cleanplan)
   self)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:mapno
   (&optional __mapno)
   (if __mapno (setq _mapno __mapno)) _mapno)
  (:cplanno
   (&optional __cplanno)
   (if __cplanno (setq _cplanno __cplanno)) _cplanno)
  (:mapdir
   (&optional __mapdir)
   (if __mapdir (setq _mapdir __mapdir)) _mapdir)
  (:mapname
   (&optional __mapname)
   (if __mapname (setq _mapname __mapname)) _mapname)
  (:cplandir
   (&optional __cplandir)
   (if __cplandir (setq _cplandir __cplandir)) _cplandir)
  (:cleanplan
   (&optional __cleanplan)
   (if __cleanplan (setq _cleanplan __cleanplan)) _cleanplan)
  (:serialization-length
   ()
   (+
    ;; int8 _info
    1
    ;; int32 _mapno
    4
    ;; int32 _cplanno
    4
    ;; string _mapdir
    4 (length _mapdir)
    ;; string[] _mapname
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _mapname)) 4
    ;; string _cplandir
    4 (length _cplandir)
    ;; string[] _cleanplan
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _cleanplan)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _info
       (write-byte _info s)
     ;; int32 _mapno
       (write-long _mapno s)
     ;; int32 _cplanno
       (write-long _cplanno s)
     ;; string _mapdir
       (write-long (length _mapdir) s) (princ _mapdir s)
     ;; string[] _mapname
     (write-long (length _mapname) s)
     (dolist (elem _mapname)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _cplandir
       (write-long (length _cplandir) s) (princ _cplandir s)
     ;; string[] _cleanplan
     (write-long (length _cleanplan) s)
     (dolist (elem _cleanplan)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _info
     (setq _info (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _info 127) (setq _info (- _info 256)))
   ;; int32 _mapno
     (setq _mapno (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _cplanno
     (setq _cplanno (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _mapdir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mapdir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _mapname
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mapname (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _mapname i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _cplandir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cplandir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _cleanplan
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cleanplan (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _cleanplan i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get htbot::cleanlist :md5sum-) "975117791827cbfacd76c6c1c424923b")
(setf (get htbot::cleanlist :datatype-) "htbot/cleanlist")
(setf (get htbot::cleanlist :definition-)
      "int8 info
int32 mapno
int32 cplanno
string mapdir
string[] mapname
string cplandir
string[] cleanplan

")



(provide :htbot/cleanlist "975117791827cbfacd76c6c1c424923b")


