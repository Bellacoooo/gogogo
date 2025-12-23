; Auto-generated. Do not edit!


(cl:in-package dynamic_predictor-msg)


;//! \htmlinclude PredictedTrajectory.msg.html

(cl:defclass <PredictedTrajectory> (roslisp-msg-protocol:ros-message)
  ((probability
    :reader probability
    :initarg :probability
    :type cl:float
    :initform 0.0)
   (mean
    :reader mean
    :initarg :mean
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (variance
    :reader variance
    :initarg :variance
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3))))
)

(cl:defclass PredictedTrajectory (<PredictedTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PredictedTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PredictedTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamic_predictor-msg:<PredictedTrajectory> is deprecated: use dynamic_predictor-msg:PredictedTrajectory instead.")))

(cl:ensure-generic-function 'probability-val :lambda-list '(m))
(cl:defmethod probability-val ((m <PredictedTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_predictor-msg:probability-val is deprecated.  Use dynamic_predictor-msg:probability instead.")
  (probability m))

(cl:ensure-generic-function 'mean-val :lambda-list '(m))
(cl:defmethod mean-val ((m <PredictedTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_predictor-msg:mean-val is deprecated.  Use dynamic_predictor-msg:mean instead.")
  (mean m))

(cl:ensure-generic-function 'variance-val :lambda-list '(m))
(cl:defmethod variance-val ((m <PredictedTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_predictor-msg:variance-val is deprecated.  Use dynamic_predictor-msg:variance instead.")
  (variance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PredictedTrajectory>) ostream)
  "Serializes a message object of type '<PredictedTrajectory>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'probability))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'mean))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'variance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'variance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PredictedTrajectory>) istream)
  "Deserializes a message object of type '<PredictedTrajectory>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'probability) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mean) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mean)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'variance) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'variance)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PredictedTrajectory>)))
  "Returns string type for a message object of type '<PredictedTrajectory>"
  "dynamic_predictor/PredictedTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PredictedTrajectory)))
  "Returns string type for a message object of type 'PredictedTrajectory"
  "dynamic_predictor/PredictedTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PredictedTrajectory>)))
  "Returns md5sum for a message object of type '<PredictedTrajectory>"
  "48bdf4c0064d9378b7fbef39e2f807e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PredictedTrajectory)))
  "Returns md5sum for a message object of type 'PredictedTrajectory"
  "48bdf4c0064d9378b7fbef39e2f807e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PredictedTrajectory>)))
  "Returns full string definition for message of type '<PredictedTrajectory>"
  (cl:format cl:nil "# Probability weight of this mode~%float64 probability~%~%# Mean trajectory for this mode (length = horizon)~%geometry_msgs/Point[] mean~%~%# Variance per time step (same length as mean)~%# Variance is ordered as var(x), var(y), var(z)~%geometry_msgs/Vector3[] variance~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PredictedTrajectory)))
  "Returns full string definition for message of type 'PredictedTrajectory"
  (cl:format cl:nil "# Probability weight of this mode~%float64 probability~%~%# Mean trajectory for this mode (length = horizon)~%geometry_msgs/Point[] mean~%~%# Variance per time step (same length as mean)~%# Variance is ordered as var(x), var(y), var(z)~%geometry_msgs/Vector3[] variance~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PredictedTrajectory>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mean) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'variance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PredictedTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'PredictedTrajectory
    (cl:cons ':probability (probability msg))
    (cl:cons ':mean (mean msg))
    (cl:cons ':variance (variance msg))
))
