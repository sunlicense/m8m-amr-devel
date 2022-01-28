;; Auto-generated. Do not edit!


(when (boundp 'htbot::Empty)
  (if (not (find-package "HTBOT"))
    (make-package "HTBOT"))
  (shadow 'Empty (find-package "HTBOT")))
(unless (find-package "HTBOT::EMPTY")
  (make-package "HTBOT::EMPTY"))
(unless (find-package "HTBOT::EMPTYREQUEST")
  (make-package "HTBOT::EMPTYREQUEST"))
(unless (find-package "HTBOT::EMPTYRESPONSE")
  (make-package "HTBOT::EMPTYRESPONSE"))

(in-package "ROS")





(defclass htbot::EmptyRequest
  :super ros::object
  :slots ())

(defmethod htbot::EmptyRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass htbot::EmptyResponse
  :super ros::object
  :slots ())

(defmethod htbot::EmptyResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass htbot::Empty
  :super ros::object
  :slots ())

(setf (get htbot::Empty :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get htbot::Empty :datatype-) "htbot/Empty")
(setf (get htbot::Empty :request) htbot::EmptyRequest)
(setf (get htbot::Empty :response) htbot::EmptyResponse)

(defmethod htbot::EmptyRequest
  (:response () (instance htbot::EmptyResponse :init)))

(setf (get htbot::EmptyRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get htbot::EmptyRequest :datatype-) "htbot/EmptyRequest")
(setf (get htbot::EmptyRequest :definition-)
      "---

")

(setf (get htbot::EmptyResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get htbot::EmptyResponse :datatype-) "htbot/EmptyResponse")
(setf (get htbot::EmptyResponse :definition-)
      "---

")



(provide :htbot/Empty "d41d8cd98f00b204e9800998ecf8427e")


