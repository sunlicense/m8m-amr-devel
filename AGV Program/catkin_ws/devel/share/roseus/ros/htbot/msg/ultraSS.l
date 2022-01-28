;; Auto-generated. Do not edit!


(when (boundp 'htbot::ultraSS)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'ultraSS (find-package "HTBOT")))
(unless (find-package "HTBOT::ULTRASS")
  (make-package "HTBOT::ULTRASS"))

(in-package "ROS")
;;//! \htmlinclude ultraSS.msg.html


(defclass htbot::ultraSS
  :super ros::object
  :slots (_uFR _uFL _uRF _uRR _uLF _uLR ))

(defmethod htbot::ultraSS
  (:init
   (&key
    ((:uFR __uFR) 0.0)
    ((:uFL __uFL) 0.0)
    ((:uRF __uRF) 0.0)
    ((:uRR __uRR) 0.0)
    ((:uLF __uLF) 0.0)
    ((:uLR __uLR) 0.0)
    )
   (send-super :init)
   (setq _uFR (float __uFR))
   (setq _uFL (float __uFL))
   (setq _uRF (float __uRF))
   (setq _uRR (float __uRR))
   (setq _uLF (float __uLF))
   (setq _uLR (float __uLR))
   self)
  (:uFR
   (&optional __uFR)
   (if __uFR (setq _uFR __uFR)) _uFR)
  (:uFL
   (&optional __uFL)
   (if __uFL (setq _uFL __uFL)) _uFL)
  (:uRF
   (&optional __uRF)
   (if __uRF (setq _uRF __uRF)) _uRF)
  (:uRR
   (&optional __uRR)
   (if __uRR (setq _uRR __uRR)) _uRR)
  (:uLF
   (&optional __uLF)
   (if __uLF (setq _uLF __uLF)) _uLF)
  (:uLR
   (&optional __uLR)
   (if __uLR (setq _uLR __uLR)) _uLR)
  (:serialization-length
   ()
   (+
    ;; float32 _uFR
    4
    ;; float32 _uFL
    4
    ;; float32 _uRF
    4
    ;; float32 _uRR
    4
    ;; float32 _uLF
    4
    ;; float32 _uLR
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _uFR
       (sys::poke _uFR (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _uFL
       (sys::poke _uFL (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _uRF
       (sys::poke _uRF (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _uRR
       (sys::poke _uRR (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _uLF
       (sys::poke _uLF (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _uLR
       (sys::poke _uLR (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _uFR
     (setq _uFR (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _uFL
     (setq _uFL (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _uRF
     (setq _uRF (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _uRR
     (setq _uRR (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _uLF
     (setq _uLF (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _uLR
     (setq _uLR (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get htbot::ultraSS :md5sum-) "9863d3f832b0ad0ccba6621fbb52a2e5")
(setf (get htbot::ultraSS :datatype-) "htbot/ultraSS")
(setf (get htbot::ultraSS :definition-)
      "float32 uFR
float32 uFL
float32 uRF
float32 uRR
float32 uLF
float32 uLR

")



(provide :htbot/ultraSS "9863d3f832b0ad0ccba6621fbb52a2e5")


