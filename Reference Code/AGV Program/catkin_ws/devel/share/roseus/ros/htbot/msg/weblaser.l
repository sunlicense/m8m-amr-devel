;; Auto-generated. Do not edit!


(when (boundp 'htbot::weblaser)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'weblaser (find-package "HTBOT")))
(unless (find-package "HTBOT::WEBLASER")
  (make-package "HTBOT::WEBLASER"))

(in-package "ROS")
;;//! \htmlinclude weblaser.msg.html


(defclass htbot::weblaser
  :super ros::object
  :slots (_info _size _px _py ))

(defmethod htbot::weblaser
  (:init
   (&key
    ((:info __info) 0)
    ((:size __size) 0)
    ((:px __px) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:py __py) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _info (round __info))
   (setq _size (round __size))
   (setq _px __px)
   (setq _py __py)
   self)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:px
   (&optional __px)
   (if __px (setq _px __px)) _px)
  (:py
   (&optional __py)
   (if __py (setq _py __py)) _py)
  (:serialization-length
   ()
   (+
    ;; int8 _info
    1
    ;; int32 _size
    4
    ;; float32[] _px
    (* 4    (length _px)) 4
    ;; float32[] _py
    (* 4    (length _py)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _info
       (write-byte _info s)
     ;; int32 _size
       (write-long _size s)
     ;; float32[] _px
     (write-long (length _px) s)
     (dotimes (i (length _px))
       (sys::poke (elt _px i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _py
     (write-long (length _py) s)
     (dotimes (i (length _py))
       (sys::poke (elt _py i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _info
     (setq _info (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _info 127) (setq _info (- _info 256)))
   ;; int32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _px
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _px (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _px i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _py
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _py (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _py i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get htbot::weblaser :md5sum-) "f7e0272dce619f1a82242dcdb545a8bc")
(setf (get htbot::weblaser :datatype-) "htbot/weblaser")
(setf (get htbot::weblaser :definition-)
      "int8 info
int32 size
float32[] px
float32[] py

")



(provide :htbot/weblaser "f7e0272dce619f1a82242dcdb545a8bc")


