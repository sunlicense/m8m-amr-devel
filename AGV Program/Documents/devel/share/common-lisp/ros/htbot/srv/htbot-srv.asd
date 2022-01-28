
(cl:in-package :asdf)

(defsystem "htbot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Empty" :depends-on ("_package_Empty"))
    (:file "_package_Empty" :depends-on ("_package"))
    (:file "mqueue" :depends-on ("_package_mqueue"))
    (:file "_package_mqueue" :depends-on ("_package"))
    (:file "scanMcmd" :depends-on ("_package_scanMcmd"))
    (:file "_package_scanMcmd" :depends-on ("_package"))
    (:file "sendgoal" :depends-on ("_package_sendgoal"))
    (:file "_package_sendgoal" :depends-on ("_package"))
    (:file "srvcmd" :depends-on ("_package_srvcmd"))
    (:file "_package_srvcmd" :depends-on ("_package"))
  ))