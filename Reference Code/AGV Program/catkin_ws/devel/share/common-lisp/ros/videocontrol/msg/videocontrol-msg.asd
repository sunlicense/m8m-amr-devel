
(cl:in-package :asdf)

(defsystem "videocontrol-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "debug" :depends-on ("_package_debug"))
    (:file "_package_debug" :depends-on ("_package"))
    (:file "dyna" :depends-on ("_package_dyna"))
    (:file "_package_dyna" :depends-on ("_package"))
  ))