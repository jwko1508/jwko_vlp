;; Auto-generated. Do not edit!


(when (boundp 'pharos_vlp_tilt::perfectarray)
  (if (not (find-package "PHAROS_VLP_TILT"))
    (make-package "PHAROS_VLP_TILT"))
  (shadow 'perfectarray (find-package "PHAROS_VLP_TILT")))
(unless (find-package "PHAROS_VLP_TILT::PERFECTARRAY")
  (make-package "PHAROS_VLP_TILT::PERFECTARRAY"))

(in-package "ROS")
;;//! \htmlinclude perfectarray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pharos_vlp_tilt::perfectarray
  :super ros::object
  :slots (_header _objects _min_seq _max_seq _center _min_center _max_center _min_object _max_object _min_hori _max_hori ))

(defmethod pharos_vlp_tilt::perfectarray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:objects __objects) (let (r) (dotimes (i 0) (push (instance pharos_vlp_tilt::perfect :init) r)) r))
    ((:min_seq __min_seq) (let (r) (dotimes (i 16) (push (instance pharos_vlp_tilt::min_seq :init) r)) r))
    ((:max_seq __max_seq) (let (r) (dotimes (i 16) (push (instance pharos_vlp_tilt::max_seq :init) r)) r))
    ((:center __center) (instance pharos_vlp_tilt::center_position :init))
    ((:min_center __min_center) (instance pharos_vlp_tilt::center_position :init))
    ((:max_center __max_center) (instance pharos_vlp_tilt::center_position :init))
    ((:min_object __min_object) (instance pharos_vlp_tilt::point :init))
    ((:max_object __max_object) (instance pharos_vlp_tilt::point :init))
    ((:min_hori __min_hori) 0)
    ((:max_hori __max_hori) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _objects __objects)
   (setq _min_seq __min_seq)
   (setq _max_seq __max_seq)
   (setq _center __center)
   (setq _min_center __min_center)
   (setq _max_center __max_center)
   (setq _min_object __min_object)
   (setq _max_object __max_object)
   (setq _min_hori (round __min_hori))
   (setq _max_hori (round __max_hori))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:min_seq
   (&rest __min_seq)
   (if (keywordp (car __min_seq))
       (send* _min_seq __min_seq)
     (progn
       (if __min_seq (setq _min_seq (car __min_seq)))
       _min_seq)))
  (:max_seq
   (&rest __max_seq)
   (if (keywordp (car __max_seq))
       (send* _max_seq __max_seq)
     (progn
       (if __max_seq (setq _max_seq (car __max_seq)))
       _max_seq)))
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:min_center
   (&rest __min_center)
   (if (keywordp (car __min_center))
       (send* _min_center __min_center)
     (progn
       (if __min_center (setq _min_center (car __min_center)))
       _min_center)))
  (:max_center
   (&rest __max_center)
   (if (keywordp (car __max_center))
       (send* _max_center __max_center)
     (progn
       (if __max_center (setq _max_center (car __max_center)))
       _max_center)))
  (:min_object
   (&rest __min_object)
   (if (keywordp (car __min_object))
       (send* _min_object __min_object)
     (progn
       (if __min_object (setq _min_object (car __min_object)))
       _min_object)))
  (:max_object
   (&rest __max_object)
   (if (keywordp (car __max_object))
       (send* _max_object __max_object)
     (progn
       (if __max_object (setq _max_object (car __max_object)))
       _max_object)))
  (:min_hori
   (&optional __min_hori)
   (if __min_hori (setq _min_hori __min_hori)) _min_hori)
  (:max_hori
   (&optional __max_hori)
   (if __max_hori (setq _max_hori __max_hori)) _max_hori)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pharos_vlp_tilt/perfect[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ;; pharos_vlp_tilt/min_seq[16] _min_seq
    (apply #'+ (send-all _min_seq :serialization-length))
    ;; pharos_vlp_tilt/max_seq[16] _max_seq
    (apply #'+ (send-all _max_seq :serialization-length))
    ;; pharos_vlp_tilt/center_position _center
    (send _center :serialization-length)
    ;; pharos_vlp_tilt/center_position _min_center
    (send _min_center :serialization-length)
    ;; pharos_vlp_tilt/center_position _max_center
    (send _max_center :serialization-length)
    ;; pharos_vlp_tilt/point _min_object
    (send _min_object :serialization-length)
    ;; pharos_vlp_tilt/point _max_object
    (send _max_object :serialization-length)
    ;; int32 _min_hori
    4
    ;; int32 _max_hori
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pharos_vlp_tilt/perfect[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;; pharos_vlp_tilt/min_seq[16] _min_seq
     (dolist (elem _min_seq)
       (send elem :serialize s)
       )
     ;; pharos_vlp_tilt/max_seq[16] _max_seq
     (dolist (elem _max_seq)
       (send elem :serialize s)
       )
     ;; pharos_vlp_tilt/center_position _center
       (send _center :serialize s)
     ;; pharos_vlp_tilt/center_position _min_center
       (send _min_center :serialize s)
     ;; pharos_vlp_tilt/center_position _max_center
       (send _max_center :serialize s)
     ;; pharos_vlp_tilt/point _min_object
       (send _min_object :serialize s)
     ;; pharos_vlp_tilt/point _max_object
       (send _max_object :serialize s)
     ;; int32 _min_hori
       (write-long _min_hori s)
     ;; int32 _max_hori
       (write-long _max_hori s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pharos_vlp_tilt/perfect[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance pharos_vlp_tilt::perfect :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; pharos_vlp_tilt/min_seq[16] _min_seq
   (dotimes (i 16)
     (send (elt _min_seq i) :deserialize buf ptr-) (incf ptr- (send (elt _min_seq i) :serialization-length))
     )
   ;; pharos_vlp_tilt/max_seq[16] _max_seq
   (dotimes (i 16)
     (send (elt _max_seq i) :deserialize buf ptr-) (incf ptr- (send (elt _max_seq i) :serialization-length))
     )
   ;; pharos_vlp_tilt/center_position _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; pharos_vlp_tilt/center_position _min_center
     (send _min_center :deserialize buf ptr-) (incf ptr- (send _min_center :serialization-length))
   ;; pharos_vlp_tilt/center_position _max_center
     (send _max_center :deserialize buf ptr-) (incf ptr- (send _max_center :serialization-length))
   ;; pharos_vlp_tilt/point _min_object
     (send _min_object :deserialize buf ptr-) (incf ptr- (send _min_object :serialization-length))
   ;; pharos_vlp_tilt/point _max_object
     (send _max_object :deserialize buf ptr-) (incf ptr- (send _max_object :serialization-length))
   ;; int32 _min_hori
     (setq _min_hori (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_hori
     (setq _max_hori (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pharos_vlp_tilt::perfectarray :md5sum-) "393d188248af978f922d9cd672182dbf")
(setf (get pharos_vlp_tilt::perfectarray :datatype-) "pharos_vlp_tilt/perfectarray")
(setf (get pharos_vlp_tilt::perfectarray :definition-)
      "std_msgs/Header header
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



(provide :pharos_vlp_tilt/perfectarray "393d188248af978f922d9cd672182dbf")


