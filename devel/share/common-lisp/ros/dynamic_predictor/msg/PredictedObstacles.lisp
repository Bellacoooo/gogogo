; Auto-generated. Do not edit!


(cl:in-package dynamic_predictor-msg)


;//! \htmlinclude PredictedObstacles.msg.html

(cl:defclass <PredictedObstacles> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector dynamic_predictor-msg:PredictedObstacle)
   :initform (cl:make-array 0 :element-type 'dynamic_predictor-msg:PredictedObstacle :initial-element (cl:make-instance 'dynamic_predictor-msg:PredictedObstacle))))
)

(cl:defclass PredictedObstacles (<PredictedObstacles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PredictedObstacles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PredictedObstacles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamic_predictor-msg:<PredictedObstacles> is deprecated: use dynamic_predictor-msg:PredictedObstacles instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PredictedObstacles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_predictor-msg:header-val is deprecated.  Use dynamic_predictor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <PredictedObstacles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_predictor-msg:obstacles-val is deprecated.  Use dynamic_predictor-msg:obstacles instead.")
  (obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PredictedObstacles>) ostream)
  "Serializes a message object of type '<PredictedObstacles>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PredictedObstacles>) istream)
  "Deserializes a message object of type '<PredictedObstacles>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'dynamic_predictor-msg:PredictedObstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PredictedObstacles>)))
  "Returns string type for a message object of type '<PredictedObstacles>"
  "dynamic_predictor/PredictedObstacles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PredictedObstacles)))
  "Returns string type for a message object of type 'PredictedObstacles"
  "dynamic_predictor/PredictedObstacles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PredictedObstacles>)))
  "Returns md5sum for a message object of type '<PredictedObstacles>"
  "de883972cf9be65d742bb9007ac50081")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PredictedObstacles)))
  "Returns md5sum for a message object of type 'PredictedObstacles"
  "de883972cf9be65d742bb9007ac50081")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PredictedObstacles>)))
  "Returns full string definition for message of type '<PredictedObstacles>"
  (cl:format cl:nil "std_msgs/Header header~%~%dynamic_predictor/PredictedObstacle[] obstacles~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: dynamic_predictor/PredictedObstacle~%# Obstacle identifier from tracker~%int32 id~%~%geometry_msgs/Point current_position~%geometry_msgs/Vector3 current_velocity~%~%# All modal trajectories for this obstacle~%dynamic_predictor/PredictedTrajectory[] modes~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: dynamic_predictor/PredictedTrajectory~%# Probability weight of this mode~%float64 probability~%~%# Mean trajectory for this mode (length = horizon)~%geometry_msgs/Point[] mean~%~%# Variance per time step (same length as mean)~%# Variance is ordered as var(x), var(y), var(z)~%geometry_msgs/Vector3[] variance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PredictedObstacles)))
  "Returns full string definition for message of type 'PredictedObstacles"
  (cl:format cl:nil "std_msgs/Header header~%~%dynamic_predictor/PredictedObstacle[] obstacles~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: dynamic_predictor/PredictedObstacle~%# Obstacle identifier from tracker~%int32 id~%~%geometry_msgs/Point current_position~%geometry_msgs/Vector3 current_velocity~%~%# All modal trajectories for this obstacle~%dynamic_predictor/PredictedTrajectory[] modes~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: dynamic_predictor/PredictedTrajectory~%# Probability weight of this mode~%float64 probability~%~%# Mean trajectory for this mode (length = horizon)~%geometry_msgs/Point[] mean~%~%# Variance per time step (same length as mean)~%# Variance is ordered as var(x), var(y), var(z)~%geometry_msgs/Vector3[] variance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PredictedObstacles>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PredictedObstacles>))
  "Converts a ROS message object to a list"
  (cl:list 'PredictedObstacles
    (cl:cons ':header (header msg))
    (cl:cons ':obstacles (obstacles msg))
))
