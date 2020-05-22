;; Auto-generated. Do not edit!


(when (boundp 'pharos_vlp_tilt::vector_perfect_array)
  (if (not (find-package "PHAROS_VLP_TILT"))
    (make-package "PHAROS_VLP_TILT"))
  (shadow 'vector_perfect_array (find-package "PHAROS_VLP_TILT")))
(unless (find-package "PHAROS_VLP_TILT::VECTOR_PERFECT_ARRAY")
  (make-package "PHAROS_VLP_TILT::VECTOR_PERFECT_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude vector_perfect_array.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pharos_vlp_tilt::vector_perfect_array
  :super ros::object
  :slots (_header _one ))

(defmethod pharos_vlp_tilt::vector_perfect_array
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:one __one) (let (r) (dotimes (i 0) (push (instance pharos_vlp_tilt::perfectarray :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _one __one)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:one
   (&rest __one)
   (if (keywordp (car __one))
       (send* _one __one)
     (progn
       (if __one (setq _one (car __one)))
       _one)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pharos_vlp_tilt/perfectarray[] _one
    (apply #'+ (send-all _one :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pharos_vlp_tilt/perfectarray[] _one
     (write-long (length _one) s)
     (dolist (elem _one)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pharos_vlp_tilt/perfectarray[] _one
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _one (let (r) (dotimes (i n) (push (instance pharos_vlp_tilt::perfectarray :init) r)) r))
     (dolist (elem- _one)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pharos_vlp_tilt::vector_perfect_array :md5sum-) "57e47adbe1b2b913a3385ad439b88169")
(setf (get pharos_vlp_tilt::vector_perfect_array :datatype-) "pharos_vlp_tilt/vector_perfect_array")
(setf (get pharos_vlp_tilt::vector_perfect_array :definition-)
      "std_msgs/Header header
perfectarray[] one

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
MSG: pharos_vlp_tilt/perfectarray
std_msgs/Header header
perfect[] objects
min_seq[16] min_seq
max_seq[16] max_seq
center_position center
center_position min_center
center_position max_center
point min_object
point max_object
int32 min_hori
int32 max_hori
================================================================================
MSG: pharos_vlp_tilt/perfect
info info
point point
state state

================================================================================
MSG: pharos_vlp_tilt/info
int32 laser
int32 hori

================================================================================
MSG: pharos_vlp_tilt/point
float64 x
float64 y
float64 z
float64 intensity

================================================================================
MSG: pharos_vlp_tilt/state
int32 is_ground
int32 is_del
int32 is_infect

================================================================================
MSG: pharos_vlp_tilt/min_seq
int16 i
int16 hori

================================================================================
MSG: pharos_vlp_tilt/max_seq
int16 i
int16 hori

================================================================================
MSG: pharos_vlp_tilt/center_position
float64 x
float64 y
float64 z

")



(provide :pharos_vlp_tilt/vector_perfect_array "57e47adbe1b2b913a3385ad439b88169")


