;; Auto-generated. Do not edit!


(when (boundp 'dynamic_predictor::PredictedObstacles)
  (if (not (find-package "DYNAMIC_PREDICTOR"))
    (make-package "DYNAMIC_PREDICTOR"))
  (shadow 'PredictedObstacles (find-package "DYNAMIC_PREDICTOR")))
(unless (find-package "DYNAMIC_PREDICTOR::PREDICTEDOBSTACLES")
  (make-package "DYNAMIC_PREDICTOR::PREDICTEDOBSTACLES"))

(in-package "ROS")
;;//! \htmlinclude PredictedObstacles.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dynamic_predictor::PredictedObstacles
  :super ros::object
  :slots (_header _obstacles ))

(defmethod dynamic_predictor::PredictedObstacles
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacles __obstacles) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacles __obstacles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; dynamic_predictor/PredictedObstacle[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; dynamic_predictor/PredictedObstacle[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; dynamic_predictor/PredictedObstacle[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance dynamic_predictor::PredictedObstacle :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dynamic_predictor::PredictedObstacles :md5sum-) "de883972cf9be65d742bb9007ac50081")
(setf (get dynamic_predictor::PredictedObstacles :datatype-) "dynamic_predictor/PredictedObstacles")
(setf (get dynamic_predictor::PredictedObstacles :definition-)
      "std_msgs/Header header

dynamic_predictor/PredictedObstacle[] obstacles


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: dynamic_predictor/PredictedObstacle
# Obstacle identifier from tracker
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



(provide :dynamic_predictor/PredictedObstacles "de883972cf9be65d742bb9007ac50081")


