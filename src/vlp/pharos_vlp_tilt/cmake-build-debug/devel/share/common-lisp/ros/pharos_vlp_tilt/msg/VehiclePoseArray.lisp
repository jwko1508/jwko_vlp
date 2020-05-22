; Auto-generated. Do not edit!


(cl:in-package pharos_vlp_tilt-msg)


;//! \htmlinclude VehiclePoseArray.msg.html

(cl:defclass <VehiclePoseArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (vehicles
    :reader vehicles
    :initarg :vehicles
    :type (cl:vector pharos_vlp_tilt-msg:VehiclePose)
   :initform (cl:make-array 0 :element-type 'pharos_vlp_tilt-msg:VehiclePose :initial-element (cl:make-instance 'pharos_vlp_tilt-msg:VehiclePose))))
)

(cl:defclass VehiclePoseArray (<VehiclePoseArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehiclePoseArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehiclePoseArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pharos_vlp_tilt-msg:<VehiclePoseArray> is deprecated: use pharos_vlp_tilt-msg:VehiclePoseArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VehiclePoseArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:header-val is deprecated.  Use pharos_vlp_tilt-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'vehicles-val :lambda-list '(m))
(cl:defmethod vehicles-val ((m <VehiclePoseArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:vehicles-val is deprecated.  Use pharos_vlp_tilt-msg:vehicles instead.")
  (vehicles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehiclePoseArray>) ostream)
  "Serializes a message object of type '<VehiclePoseArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vehicles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'vehicles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehiclePoseArray>) istream)
  "Deserializes a message object of type '<VehiclePoseArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vehicles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vehicles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pharos_vlp_tilt-msg:VehiclePose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehiclePoseArray>)))
  "Returns string type for a message object of type '<VehiclePoseArray>"
  "pharos_vlp_tilt/VehiclePoseArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehiclePoseArray)))
  "Returns string type for a message object of type 'VehiclePoseArray"
  "pharos_vlp_tilt/VehiclePoseArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehiclePoseArray>)))
  "Returns md5sum for a message object of type '<VehiclePoseArray>"
  "75b3ae63766b70da2ab55a64ee96561f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehiclePoseArray)))
  "Returns md5sum for a message object of type 'VehiclePoseArray"
  "75b3ae63766b70da2ab55a64ee96561f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehiclePoseArray>)))
  "Returns full string definition for message of type '<VehiclePoseArray>"
  (cl:format cl:nil "std_msgs/Header header~%VehiclePose[] vehicles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pharos_vlp_tilt/VehiclePose~%float64 x~%float64 y~%float64 theta~%std_msgs/Time stamp~%================================================================================~%MSG: std_msgs/Time~%time data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehiclePoseArray)))
  "Returns full string definition for message of type 'VehiclePoseArray"
  (cl:format cl:nil "std_msgs/Header header~%VehiclePose[] vehicles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pharos_vlp_tilt/VehiclePose~%float64 x~%float64 y~%float64 theta~%std_msgs/Time stamp~%================================================================================~%MSG: std_msgs/Time~%time data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehiclePoseArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vehicles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehiclePoseArray>))
  "Converts a ROS message object to a list"
  (cl:list 'VehiclePoseArray
    (cl:cons ':header (header msg))
    (cl:cons ':vehicles (vehicles msg))
))
