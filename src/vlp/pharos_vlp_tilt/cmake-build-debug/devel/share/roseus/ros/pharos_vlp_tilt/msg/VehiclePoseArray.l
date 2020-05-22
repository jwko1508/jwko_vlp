;; Auto-generated. Do not edit!


(when (boundp 'pharos_vlp_tilt::VehiclePoseArray)
  (if (not (find-package "PHAROS_VLP_TILT"))
    (make-package "PHAROS_VLP_TILT"))
  (shadow 'VehiclePoseArray (find-package "PHAROS_VLP_TILT")))
(unless (find-package "PHAROS_VLP_TILT::VEHICLEPOSEARRAY")
  (make-package "PHAROS_VLP_TILT::VEHICLEPOSEARRAY"))

(in-package "ROS")
;;//! \htmlinclude VehiclePoseArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pharos_vlp_tilt::VehiclePoseArray
  :super ros::object
  :slots (_header _vehicles ))

(defmethod pharos_vlp_tilt::VehiclePoseArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:vehicles __vehicles) (let (r) (dotimes (i 0) (push (instance pharos_vlp_tilt::VehiclePose :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _vehicles __vehicles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:vehicles
   (&rest __vehicles)
   (if (keywordp (car __vehicles))
       (send* _vehicles __vehicles)
     (progn
       (if __vehicles (setq _vehicles (car __vehicles)))
       _vehicles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pharos_vlp_tilt/VehiclePose[] _vehicles
    (apply #'+ (send-all _vehicles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pharos_vlp_tilt/VehiclePose[] _vehicles
     (write-long (length _vehicles) s)
     (dolist (elem _vehicles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pharos_vlp_tilt/VehiclePose[] _vehicles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vehicles (let (r) (dotimes (i n) (push (instance pharos_vlp_tilt::VehiclePose :init) r)) r))
     (dolist (elem- _vehicles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pharos_vlp_tilt::VehiclePoseArray :md5sum-) "75b3ae63766b70da2ab55a64ee96561f")
(setf (get pharos_vlp_tilt::VehiclePoseArray :datatype-) "pharos_vlp_tilt/VehiclePoseArray")
(setf (get pharos_vlp_tilt::VehiclePoseArray :definition-)
      "std_msgs/Header header
VehiclePose[] vehicles

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: pharos_vlp_tilt/VehiclePose
float64 x
float64 y
float64 theta
std_msgs/Time stamp
================================================================================
MSG: std_msgs/Time
time data

")



(provide :pharos_vlp_tilt/VehiclePoseArray "75b3ae63766b70da2ab55a64ee96561f")


