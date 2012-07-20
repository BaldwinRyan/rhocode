
(cl:in-package :asdf)

(defsystem "simple-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "setGoal" :depends-on ("_package_setGoal"))
    (:file "_package_setGoal" :depends-on ("_package"))
    (:file "set_goal" :depends-on ("_package_set_goal"))
    (:file "_package_set_goal" :depends-on ("_package"))
  ))