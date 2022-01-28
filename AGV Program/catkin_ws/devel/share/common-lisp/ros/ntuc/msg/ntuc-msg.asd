
(cl:in-package :asdf)

(defsystem "ntuc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "lift" :depends-on ("_package_lift"))
    (:file "_package_lift" :depends-on ("_package"))
  ))