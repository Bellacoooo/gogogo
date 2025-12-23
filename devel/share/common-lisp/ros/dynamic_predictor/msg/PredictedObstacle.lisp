; Auto-generated. Do not edit!


(cl:in-package dynamic_predictor-msg)


;//! \htmlinclude PredictedObstacle.msg.html

(cl:defclass <PredictedObstacle> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (current_position
    :reader current_position
    :initarg :current_position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (current_velocity
    :reader current_velocity
    :initarg :current_velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (modes
    :reader modes
    :initarg :modes
    :type (cl:vector dynamic_predictor-msg:PredictedTrajectory)
   :initform (cl:make-array 0 :element-type 'dynamic_predictor-msg:PredictedTrajectory :initial-element (cl:make-instance 'dynamic_predictor-msg:PredictedTrajectory))))
)

(cl:defclass PredictedObstacle (<PredictedObstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PredictedObstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PredictedObstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamic_predictor-msg:<PredictedObstacle> is deprecated: use dynamic_predictor-msg:PredictedObstacle instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <PredictedObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_predictor-msg:id-val is deprecated.  Use dynamic_predictor-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'current_position-val :lambda-list '(m))
(cl:defmethod current_position-val ((m <PredictedObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_predictor-msg:current_position-val is deprecated.  Use dynamic_predictor-msg:current_position instead.")
  (current_position m))

(cl:ensure-generic-function 'current_velocity-val :lambda-list '(m))
(cl:defmethod current_velocity-val ((m <PredictedObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_predictor-msg:current_velocity-val is deprecated.  Use dynamic_predictor-msg:current_velocity instead.")
  (current_velocity m))

(cl:ensure-generic-function 'modes-val :lambda-list '(m))
(cl:defmethod modes-val ((m <PredictedObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_predictor-msg:modes-val is deprecated.  Use dynamic_predictor-msg:modes instead.")
  (modes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PredictedObstacle>) ostream)
  "Serializes a message object of type '<PredictedObstacle>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'current_position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'current_velocity) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'modes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'modes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PredictedObstacle>) istream)
  "Deserializes a message object of type '<PredictedObstacle>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'current_position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'current_velocity) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'modes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'modes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'dynamic_predictor-msg:PredictedTrajectory))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PredictedObstacle>)))
  "Returns string type for a message object of type '<PredictedObstacle>"
  "dynamic_predictor/PredictedObstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PredictedObstacle)))
  "Returns string type for a message object of type 'PredictedObstacle"
  "dynamic_predictor/PredictedObstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PredictedObstacle>)))
  "Returns md5sum for a message object of type '<PredictedObstacle>"
  "6b1bfccd711ab582d4c8a979685459e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PredictedObstacle)))
  "Returns md5sum for a message object of type 'PredictedObstacle"
  "6b1bfccd711ab582d4c8a979685459e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PredictedObstacle>)))
  "Returns full string definition for message of type '<PredictedObstacle>"
  (cl:format cl:nil "# Obstacle identifier from tracker~%int32 id~%~%geometry_msgs/Point current_position~%geometry_msgs/Vector3 current_velocity~%~%# All modal trajectories for this obstacle~%dynamic_predictor/PredictedTrajectory[] modes~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: dynamic_predictor/PredictedTrajectory~%# Probability weight of this mode~%float64 probability~%~%# Mean trajectory for this mode (length = horizon)~%geometry_msgs/Point[] mean~%~%# Variance per time step (same length as mean)~%# Variance is ordered as var(x), var(y), var(z)~%geometry_msgs/Vector3[] variance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PredictedObstacle)))
  "Returns full string definition for message of type 'PredictedObstacle"
  (cl:format cl:nil "# Obstacle identifier from tracker~%int32 id~%~%geometry_msgs/Point current_position~%geometry_msgs/Vector3 current_velocity~%~%# All modal trajectories for this obstacle~%dynamic_predictor/PredictedTrajectory[] modes~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: dynamic_predictor/PredictedTrajectory~%# Probability weight of this mode~%float64 probability~%~%# Mean trajectory for this mode (length = horizon)~%geometry_msgs/Point[] mean~%~%# Variance per time step (same length as mean)~%# Variance is ordered as var(x), var(y), var(z)~%geometry_msgs/Vector3[] variance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PredictedObstacle>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'current_position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'current_velocity))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'modes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PredictedObstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'PredictedObstacle
    (cl:cons ':id (id msg))
    (cl:cons ':current_position (current_position msg))
    (cl:cons ':current_velocity (current_velocity msg))
    (cl:cons ':modes (modes msg))
))
