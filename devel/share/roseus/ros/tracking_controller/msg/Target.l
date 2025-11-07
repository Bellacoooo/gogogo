;; Auto-generated. Do not edit!


(when (boundp 'tracking_controller::Target)
  (if (not (find-package "TRACKING_CONTROLLER"))
    (make-package "TRACKING_CONTROLLER"))
  (shadow 'Target (find-package "TRACKING_CONTROLLER")))
(unless (find-package "TRACKING_CONTROLLER::TARGET")
  (make-package "TRACKING_CONTROLLER::TARGET"))

(in-package "ROS")
;;//! \htmlinclude Target.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*IGNORE_ACC*" (find-package "TRACKING_CONTROLLER::TARGET"))
(shadow '*IGNORE_ACC* (find-package "TRACKING_CONTROLLER::TARGET"))
(defconstant tracking_controller::Target::*IGNORE_ACC* 1)
(intern "*IGNORE_ACC_VEL*" (find-package "TRACKING_CONTROLLER::TARGET"))
(shadow '*IGNORE_ACC_VEL* (find-package "TRACKING_CONTROLLER::TARGET"))
(defconstant tracking_controller::Target::*IGNORE_ACC_VEL* 2)

(defun tracking_controller::Target-to-symbol (const)
  (cond
        ((= const 1) 'tracking_controller::Target::*IGNORE_ACC*)
        ((= const 2) 'tracking_controller::Target::*IGNORE_ACC_VEL*)
        (t nil)))

(defclass tracking_controller::Target
  :super ros::object
  :slots (_header _type_mask _position _velocity _acceleration _yaw ))

(defmethod tracking_controller::Target
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:type_mask __type_mask) 0)
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:acceleration __acceleration) (instance geometry_msgs::Vector3 :init))
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _type_mask (round __type_mask))
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _acceleration __acceleration)
   (setq _yaw (float __yaw))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:type_mask
   (&optional __type_mask)
   (if __type_mask (setq _type_mask __type_mask)) _type_mask)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _type_mask
    1
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; geometry_msgs/Vector3 _acceleration
    (send _acceleration :serialization-length)
    ;; float32 _yaw
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _type_mask
       (write-byte _type_mask s)
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Vector3 _acceleration
       (send _acceleration :serialize s)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _type_mask
     (setq _type_mask (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Vector3 _acceleration
     (send _acceleration :deserialize buf ptr-) (incf ptr- (send _acceleration :serialization-length))
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tracking_controller::Target :md5sum-) "260f087a436e669355e95cd12c00e98a")
(setf (get tracking_controller::Target :datatype-) "tracking_controller/Target")
(setf (get tracking_controller::Target :definition-)
      "std_msgs/Header header

uint8 type_mask
uint8 IGNORE_ACC = 1	# Position Reference
uint8 IGNORE_ACC_VEL = 2	# Position Reference

geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float32 yaw
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



(provide :tracking_controller/Target "260f087a436e669355e95cd12c00e98a")


