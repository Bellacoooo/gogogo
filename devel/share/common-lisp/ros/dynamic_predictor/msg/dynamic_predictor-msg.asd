
(cl:in-package :asdf)

(defsystem "dynamic_predictor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PredictedObstacle" :depends-on ("_package_PredictedObstacle"))
    (:file "_package_PredictedObstacle" :depends-on ("_package"))
    (:file "PredictedObstacles" :depends-on ("_package_PredictedObstacles"))
    (:file "_package_PredictedObstacles" :depends-on ("_package"))
    (:file "PredictedTrajectory" :depends-on ("_package_PredictedTrajectory"))
    (:file "_package_PredictedTrajectory" :depends-on ("_package"))
  ))