;; Auto-generated. Do not edit!


(when (boundp 'dynamic_predictor::PredictedObstacle)
  (if (not (find-package "DYNAMIC_PREDICTOR"))
    (make-package "DYNAMIC_PREDICTOR"))
  (shadow 'PredictedObstacle (find-package "DYNAMIC_PREDICTOR")))
(unless (find-package "DYNAMIC_PREDICTOR::PREDICTEDOBSTACLE")
  (make-package "DYNAMIC_PREDICTOR::PREDICTEDOBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude PredictedObstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass dynamic_predictor::PredictedObstacle
  :super ros::object
  :slots (_id _current_position _current_velocity _modes ))

(defmethod dynamic_predictor::PredictedObstacle
  (:init
   (&key
    ((:id __id) 0)
    ((:current_position __current_position) (instance geometry_msgs::Point :init))
    ((:current_velocity __current_velocity) (instance geometry_msgs::Vector3 :init))
    ((:modes __modes) ())
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _current_position __current_position)
   (setq _current_velocity __current_velocity)
   (setq _modes __modes)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:current_position
   (&rest __current_position)
   (if (keywordp (car __current_position))
       (send* _current_position __current_position)
     (progn
       (if __current_position (setq _current_position (car __current_position)))
       _current_position)))
  (:current_velocity
   (&rest __current_velocity)
   (if (keywordp (car __current_velocity))
       (send* _current_velocity __current_velocity)
     (progn
       (if __current_velocity (setq _current_velocity (car __current_velocity)))
       _current_velocity)))
  (:modes
   (&rest __modes)
   (if (keywordp (car __modes))
       (send* _modes __modes)
     (progn
       (if __modes (setq _modes (car __modes)))
       _modes)))
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; geometry_msgs/Point _current_position
    (send _current_position :serialization-length)
    ;; geometry_msgs/Vector3 _current_velocity
    (send _current_velocity :serialization-length)
    ;; dynamic_predictor/PredictedTrajectory[] _modes
    (apply #'+ (send-all _modes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; geometry_msgs/Point _current_position
       (send _current_position :serialize s)
     ;; geometry_msgs/Vector3 _current_velocity
       (send _current_velocity :serialize s)
     ;; dynamic_predictor/PredictedTrajectory[] _modes
     (write-long (length _modes) s)
     (dolist (elem _modes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point _current_position
     (send _current_position :deserialize buf ptr-) (incf ptr- (send _current_position :serialization-length))
   ;; geometry_msgs/Vector3 _current_velocity
     (send _current_velocity :deserialize buf ptr-) (incf ptr- (send _current_velocity :serialization-length))
   ;; dynamic_predictor/PredictedTrajectory[] _modes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _modes (let (r) (dotimes (i n) (push (instance dynamic_predictor::PredictedTrajectory :init) r)) r))
     (dolist (elem- _modes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dynamic_predictor::PredictedObstacle :md5sum-) "6b1bfccd711ab582d4c8a979685459e6")
(setf (get dynamic_predictor::PredictedObstacle :datatype-) "dynamic_predictor/PredictedObstacle")
(setf (get dynamic_predictor::PredictedObstacle :definition-)
      "# Obstacle identifier from tracker
int32 id

geometry_msgs/Point current_position
geometry_msgs/Vector3 current_velocity

# All modal trajectories for this obstacle
dynamic_predictor/PredictedTrajectory[] modes


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: dynamic_predictor/PredictedTrajectory
# Probability weight of this mode
float64 probability

# Mean trajectory for this mode (length = horizon)
geometry_msgs/Point[] mean

# Variance per time step (same length as mean)
# Variance is ordered as var(x), var(y), var(z)
geometry_msgs/Vector3[] variance


")



(provide :dynamic_predictor/PredictedObstacle "6b1bfccd711ab582d4c8a979685459e6")


