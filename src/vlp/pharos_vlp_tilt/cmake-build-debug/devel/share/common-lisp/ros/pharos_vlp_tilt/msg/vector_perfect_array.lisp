; Auto-generated. Do not edit!


(cl:in-package pharos_vlp_tilt-msg)


;//! \htmlinclude vector_perfect_array.msg.html

(cl:defclass <vector_perfect_array> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (one
    :reader one
    :initarg :one
    :type (cl:vector pharos_vlp_tilt-msg:perfectarray)
   :initform (cl:make-array 0 :element-type 'pharos_vlp_tilt-msg:perfectarray :initial-element (cl:make-instance 'pharos_vlp_tilt-msg:perfectarray))))
)

(cl:defclass vector_perfect_array (<vector_perfect_array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vector_perfect_array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vector_perfect_array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pharos_vlp_tilt-msg:<vector_perfect_array> is deprecated: use pharos_vlp_tilt-msg:vector_perfect_array instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <vector_perfect_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:header-val is deprecated.  Use pharos_vlp_tilt-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'one-val :lambda-list '(m))
(cl:defmethod one-val ((m <vector_perfect_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:one-val is deprecated.  Use pharos_vlp_tilt-msg:one instead.")
  (one m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vector_perfect_array>) ostream)
  "Serializes a message object of type '<vector_perfect_array>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'one))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'one))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vector_perfect_array>) istream)
  "Deserializes a message object of type '<vector_perfect_array>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'one) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'one)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pharos_vlp_tilt-msg:perfectarray))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vector_perfect_array>)))
  "Returns string type for a message object of type '<vector_perfect_array>"
  "pharos_vlp_tilt/vector_perfect_array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vector_perfect_array)))
  "Returns string type for a message object of type 'vector_perfect_array"
  "pharos_vlp_tilt/vector_perfect_array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vector_perfect_array>)))
  "Returns md5sum for a message object of type '<vector_perfect_array>"
  "57e47adbe1b2b913a3385ad439b88169")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vector_perfect_array)))
  "Returns md5sum for a message object of type 'vector_perfect_array"
  "57e47adbe1b2b913a3385ad439b88169")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vector_perfect_array>)))
  "Returns full string definition for message of type '<vector_perfect_array>"
  (cl:format cl:nil "std_msgs/Header header~%perfectarray[] one~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pharos_vlp_tilt/perfectarray~%std_msgs/Header header~%perfect[] objects~%min_seq[16] min_seq~%max_seq[16] max_seq~%center_position center~%center_position min_center~%center_position max_center~%point min_object~%point max_object~%int32 min_hori~%int32 max_hori~%================================================================================~%MSG: pharos_vlp_tilt/perfect~%info info~%point point~%state state~%~%================================================================================~%MSG: pharos_vlp_tilt/info~%int32 laser~%int32 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/point~%float64 x~%float64 y~%float64 z~%float64 intensity~%~%================================================================================~%MSG: pharos_vlp_tilt/state~%int32 is_ground~%int32 is_del~%int32 is_infect~%~%================================================================================~%MSG: pharos_vlp_tilt/min_seq~%int16 i~%int16 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/max_seq~%int16 i~%int16 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/center_position~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vector_perfect_array)))
  "Returns full string definition for message of type 'vector_perfect_array"
  (cl:format cl:nil "std_msgs/Header header~%perfectarray[] one~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pharos_vlp_tilt/perfectarray~%std_msgs/Header header~%perfect[] objects~%min_seq[16] min_seq~%max_seq[16] max_seq~%center_position center~%center_position min_center~%center_position max_center~%point min_object~%point max_object~%int32 min_hori~%int32 max_hori~%================================================================================~%MSG: pharos_vlp_tilt/perfect~%info info~%point point~%state state~%~%================================================================================~%MSG: pharos_vlp_tilt/info~%int32 laser~%int32 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/point~%float64 x~%float64 y~%float64 z~%float64 intensity~%~%================================================================================~%MSG: pharos_vlp_tilt/state~%int32 is_ground~%int32 is_del~%int32 is_infect~%~%================================================================================~%MSG: pharos_vlp_tilt/min_seq~%int16 i~%int16 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/max_seq~%int16 i~%int16 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/center_position~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vector_perfect_array>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'one) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vector_perfect_array>))
  "Converts a ROS message object to a list"
  (cl:list 'vector_perfect_array
    (cl:cons ':header (header msg))
    (cl:cons ':one (one msg))
))
