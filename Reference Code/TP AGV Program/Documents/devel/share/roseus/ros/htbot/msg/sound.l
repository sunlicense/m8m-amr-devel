;; Auto-generated. Do not edit!


(when (boundp 'htbot::sound)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'sound (find-package "HTBOT")))
(unless (find-package "HTBOT::SOUND")
  (make-package "HTBOT::SOUND"))

(in-package "ROS")
;;//! \htmlinclude sound.msg.html


(defclass htbot::sound
  :super ros::object
  :slots (_id _startdelay _restartdelay ))

(defmethod htbot::sound
  (:init
   (&key
    ((:id __id) 0)
    ((:startdelay __startdelay) 0)
    ((:restartdelay __restartdelay) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _startdelay (round __startdelay))
   (setq _restartdelay (round __restartdelay))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:startdelay
   (&optional __startdelay)
   (if __startdelay (setq _startdelay __startdelay)) _startdelay)
  (:restartdelay
   (&optional __restartdelay)
   (if __restartdelay (setq _restartdelay __restartdelay)) _restartdelay)
  (:serialization-length
   ()
   (+
    ;; int8 _id
    1
    ;; int8 _startdelay
    1
    ;; int8 _restartdelay
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _id
       (write-byte _id s)
     ;; int8 _startdelay
       (write-byte _startdelay s)
     ;; int8 _restartdelay
       (write-byte _restartdelay s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id 127) (setq _id (- _id 256)))
   ;; int8 _startdelay
     (setq _startdelay (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _startdelay 127) (setq _startdelay (- _startdelay 256)))
   ;; int8 _restartdelay
     (setq _restartdelay (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _restartdelay 127) (setq _restartdelay (- _restartdelay 256)))
   ;;
   self)
  )

(setf (get htbot::sound :md5sum-) "42b51a54eeade2ca22faa10c3766c366")
(setf (get htbot::sound :datatype-) "htbot/sound")
(setf (get htbot::sound :definition-)
      "int8 id
int8 startdelay
int8 restartdelay

")



(provide :htbot/sound "42b51a54eeade2ca22faa10c3766c366")


