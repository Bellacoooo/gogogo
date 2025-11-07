; Auto-generated. Do not edit!


(cl:in-package tracking_controller-msg)


;//! \htmlinclude Target.msg.html

(cl:defclass <Target> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (type_mask
    :reader type_mask
    :initarg :type_mask
    :type cl:fixnum
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass Target (<Target>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Target>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Target)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tracking_controller-msg:<Target> is deprecated: use tracking_controller-msg:Target instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tracking_controller-msg:header-val is deprecated.  Use tracking_controller-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'type_mask-val :lambda-list '(m))
(cl:defmethod type_mask-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tracking_controller-msg:type_mask-val is deprecated.  Use tracking_controller-msg:type_mask instead.")
  (type_mask m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tracking_controller-msg:position-val is deprecated.  Use tracking_controller-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tracking_controller-msg:velocity-val is deprecated.  Use tracking_controller-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tracking_controller-msg:acceleration-val is deprecated.  Use tracking_controller-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <Target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tracking_controller-msg:yaw-val is deprecated.  Use tracking_controller-msg:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Target>)))
    "Constants for message type '<Target>"
  '((:IGNORE_ACC . 1)
    (:IGNORE_ACC_VEL . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Target)))
    "Constants for message type 'Target"
  '((:IGNORE_ACC . 1)
    (:IGNORE_ACC_VEL . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Target>) ostream)
  "Serializes a message object of type '<Target>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type_mask)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Target>) istream)
  "Deserializes a message object of type '<Target>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type_mask)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Target>)))
  "Returns string type for a message object of type '<Target>"
  "tracking_controller/Target")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Target)))
  "Returns string type for a message object of type 'Target"
  "tracking_controller/Target")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Target>)))
  "Returns md5sum for a message object of type '<Target>"
  "260f087a436e669355e95cd12c00e98a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Target)))
  "Returns md5sum for a message object of type 'Target"
  "260f087a436e669355e95cd12c00e98a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Target>)))
  "Returns full string definition for message of type '<Target>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 type_mask~%uint8 IGNORE_ACC = 1	# Position Reference~%uint8 IGNORE_ACC_VEL = 2	# Position Reference~%~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%float32 yaw~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Target)))
  "Returns full string definition for message of type 'Target"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 type_mask~%uint8 IGNORE_ACC = 1	# Position Reference~%uint8 IGNORE_ACC_VEL = 2	# Position Reference~%~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%float32 yaw~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Target>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Target>))
  "Converts a ROS message object to a list"
  (cl:list 'Target
    (cl:cons ':header (header msg))
    (cl:cons ':type_mask (type_mask msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':yaw (yaw msg))
))
