
(cl:in-package :asdf)

(defsystem "uav_simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CustomPoint" :depends-on ("_package_CustomPoint"))
    (:file "_package_CustomPoint" :depends-on ("_package"))
    (:file "LivoxCustomMsg" :depends-on ("_package_LivoxCustomMsg"))
    (:file "_package_LivoxCustomMsg" :depends-on ("_package"))
  ))