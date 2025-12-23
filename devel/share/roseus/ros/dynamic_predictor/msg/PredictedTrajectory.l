;; Auto-generated. Do not edit!


(when (boundp 'dynamic_predictor::PredictedTrajectory)
  (if (not (find-package "DYNAMIC_PREDICTOR"))
    (make-package "DYNAMIC_PREDICTOR"))
  (shadow 'PredictedTrajectory (find-package "DYNAMIC_PREDICTOR")))
(unless (find-package "DYNAMIC_PREDICTOR::PREDICTEDTRAJECTORY")
  (make-package "DYNAMIC_PREDICTOR::PREDICTEDTRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude PredictedTrajectory.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass dynamic_predictor::PredictedTrajectory
  :super ros::object
  :slots (_probability _mean _variance ))

(defmethod dynamic_predictor::PredictedTrajectory
  (:init
   (&key
    ((:probability __probability) 0.0)
    ((:mean __mean) ())
    ((:variance __variance) ())
    )
   (send-super :init)
   (setq _probability (float __probability))
   (setq _mean __mean)
   (setq _variance __variance)
   self)
  (:probability
   (&optional __probability)
   (if __probability (setq _probability __probability)) _probability)
  (:mean
   (&rest __mean)
   (if (keywordp (car __mean))
       (send* _mean __mean)
     (progn
       (if __mean (setq _mean (car __mean)))
       _mean)))
  (:variance
   (&rest __variance)
   (if (keywordp (car __variance))
       (send* _variance __variance)
     (progn
       (if __variance (setq _variance (car __variance)))
       _variance)))
  (:serialization-length
   ()
   (+
    ;; float64 _probability
    8
    ;; geometry_msgs/Point[] _mean
    (apply #'+ (send-all _mean :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _variance
    (apply #'+ (send-all _variance :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _probability
       (sys::poke _probability (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point[] _mean
     (write-long (length _mean) s)
     (dolist (elem _mean)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _variance
     (write-long (length _variance) s)
     (dolist (elem _variance)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _probability
     (setq _probability (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point[] _mean
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mean (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _mean)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _variance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _variance (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _variance)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dynamic_predictor::PredictedTrajectory :md5sum-) "48bdf4c0064d9378b7fbef39e2f807e0")
(setf (get dynamic_predictor::PredictedTrajectory :datatype-) "dynamic_predictor/PredictedTrajectory")
(setf (get dynamic_predictor::PredictedTrajectory :definition-)
      "# Probability weight of this mode
float64 probability

# Mean trajectory for this mode (length = horizon)
geometry_msgs/Point[] mean

# Variance per time step (same length as mean)
# Variance is ordered as var(x), var(y), var(z)
geometry_msgs/Vector3[] variance


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
")



(provide :dynamic_predictor/PredictedTrajectory "48bdf4c0064d9378b7fbef39e2f807e0")


