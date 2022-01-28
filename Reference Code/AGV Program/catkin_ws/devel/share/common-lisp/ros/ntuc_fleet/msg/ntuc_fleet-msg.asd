
(cl:in-package :asdf)

(defsystem "ntuc_fleet-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "agv_status" :depends-on ("_package_agv_status"))
    (:file "_package_agv_status" :depends-on ("_package"))
    (:file "jobs" :depends-on ("_package_jobs"))
    (:file "_package_jobs" :depends-on ("_package"))
    (:file "task" :depends-on ("_package_task"))
    (:file "_package_task" :depends-on ("_package"))
  ))