; Auto-generated. Do not edit!


(cl:in-package pharos_vlp_tilt-msg)


;//! \htmlinclude perfectarray.msg.html

(cl:defclass <perfectarray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (objects
    :reader objects
    :initarg :objects
    :type (cl:vector pharos_vlp_tilt-msg:perfect)
   :initform (cl:make-array 0 :element-type 'pharos_vlp_tilt-msg:perfect :initial-element (cl:make-instance 'pharos_vlp_tilt-msg:perfect)))
   (min_seq
    :reader min_seq
    :initarg :min_seq
    :type (cl:vector pharos_vlp_tilt-msg:min_seq)
   :initform (cl:make-array 16 :element-type 'pharos_vlp_tilt-msg:min_seq :initial-element (cl:make-instance 'pharos_vlp_tilt-msg:min_seq)))
   (max_seq
    :reader max_seq
    :initarg :max_seq
    :type (cl:vector pharos_vlp_tilt-msg:max_seq)
   :initform (cl:make-array 16 :element-type 'pharos_vlp_tilt-msg:max_seq :initial-element (cl:make-instance 'pharos_vlp_tilt-msg:max_seq)))
   (center
    :reader center
    :initarg :center
    :type pharos_vlp_tilt-msg:center_position
    :initform (cl:make-instance 'pharos_vlp_tilt-msg:center_position))
   (min_center
    :reader min_center
    :initarg :min_center
    :type pharos_vlp_tilt-msg:center_position
    :initform (cl:make-instance 'pharos_vlp_tilt-msg:center_position))
   (max_center
    :reader max_center
    :initarg :max_center
    :type pharos_vlp_tilt-msg:center_position
    :initform (cl:make-instance 'pharos_vlp_tilt-msg:center_position))
   (min_object
    :reader min_object
    :initarg :min_object
    :type pharos_vlp_tilt-msg:point
    :initform (cl:make-instance 'pharos_vlp_tilt-msg:point))
   (max_object
    :reader max_object
    :initarg :max_object
    :type pharos_vlp_tilt-msg:point
    :initform (cl:make-instance 'pharos_vlp_tilt-msg:point))
   (min_hori
    :reader min_hori
    :initarg :min_hori
    :type cl:integer
    :initform 0)
   (max_hori
    :reader max_hori
    :initarg :max_hori
    :type cl:integer
    :initform 0))
)

(cl:defclass perfectarray (<perfectarray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <perfectarray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'perfectarray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pharos_vlp_tilt-msg:<perfectarray> is deprecated: use pharos_vlp_tilt-msg:perfectarray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:header-val is deprecated.  Use pharos_vlp_tilt-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:objects-val is deprecated.  Use pharos_vlp_tilt-msg:objects instead.")
  (objects m))

(cl:ensure-generic-function 'min_seq-val :lambda-list '(m))
(cl:defmethod min_seq-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:min_seq-val is deprecated.  Use pharos_vlp_tilt-msg:min_seq instead.")
  (min_seq m))

(cl:ensure-generic-function 'max_seq-val :lambda-list '(m))
(cl:defmethod max_seq-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:max_seq-val is deprecated.  Use pharos_vlp_tilt-msg:max_seq instead.")
  (max_seq m))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:center-val is deprecated.  Use pharos_vlp_tilt-msg:center instead.")
  (center m))

(cl:ensure-generic-function 'min_center-val :lambda-list '(m))
(cl:defmethod min_center-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:min_center-val is deprecated.  Use pharos_vlp_tilt-msg:min_center instead.")
  (min_center m))

(cl:ensure-generic-function 'max_center-val :lambda-list '(m))
(cl:defmethod max_center-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:max_center-val is deprecated.  Use pharos_vlp_tilt-msg:max_center instead.")
  (max_center m))

(cl:ensure-generic-function 'min_object-val :lambda-list '(m))
(cl:defmethod min_object-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:min_object-val is deprecated.  Use pharos_vlp_tilt-msg:min_object instead.")
  (min_object m))

(cl:ensure-generic-function 'max_object-val :lambda-list '(m))
(cl:defmethod max_object-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:max_object-val is deprecated.  Use pharos_vlp_tilt-msg:max_object instead.")
  (max_object m))

(cl:ensure-generic-function 'min_hori-val :lambda-list '(m))
(cl:defmethod min_hori-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:min_hori-val is deprecated.  Use pharos_vlp_tilt-msg:min_hori instead.")
  (min_hori m))

(cl:ensure-generic-function 'max_hori-val :lambda-list '(m))
(cl:defmethod max_hori-val ((m <perfectarray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:max_hori-val is deprecated.  Use pharos_vlp_tilt-msg:max_hori instead.")
  (max_hori m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <perfectarray>) ostream)
  "Serializes a message object of type '<perfectarray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'min_seq))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'max_seq))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min_center) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max_center) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min_object) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max_object) ostream)
  (cl:let* ((signed (cl:slot-value msg 'min_hori)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'max_hori)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <perfectarray>) istream)
  "Deserializes a message object of type '<perfectarray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pharos_vlp_tilt-msg:perfect))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:setf (cl:slot-value msg 'min_seq) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'min_seq)))
    (cl:dotimes (i 16)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pharos_vlp_tilt-msg:min_seq))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'max_seq) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'max_seq)))
    (cl:dotimes (i 16)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pharos_vlp_tilt-msg:max_seq))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min_center) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max_center) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min_object) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max_object) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'min_hori) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'max_hori) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<perfectarray>)))
  "Returns string type for a message object of type '<perfectarray>"
  "pharos_vlp_tilt/perfectarray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'perfectarray)))
  "Returns string type for a message object of type 'perfectarray"
  "pharos_vlp_tilt/perfectarray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<perfectarray>)))
  "Returns md5sum for a message object of type '<perfectarray>"
  "393d188248af978f922d9cd672182dbf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'perfectarray)))
  "Returns md5sum for a message object of type 'perfectarray"
  "393d188248af978f922d9cd672182dbf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<perfectarray>)))
  "Returns full string definition for message of type '<perfectarray>"
  (cl:format cl:nil "std_msgs/Header header~%perfect[] objects~%min_seq[16] min_seq~%max_seq[16] max_seq~%center_position center~%center_position min_center~%center_position max_center~%point min_object~%point max_object~%int32 min_hori~%int32 max_hori~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pharos_vlp_tilt/perfect~%info info~%point point~%state state~%~%================================================================================~%MSG: pharos_vlp_tilt/info~%int32 laser~%int32 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/point~%float64 x~%float64 y~%float64 z~%float64 intensity~%~%================================================================================~%MSG: pharos_vlp_tilt/state~%int32 is_ground~%int32 is_del~%int32 is_infect~%~%================================================================================~%MSG: pharos_vlp_tilt/min_seq~%int16 i~%int16 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/max_seq~%int16 i~%int16 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/center_position~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'perfectarray)))
  "Returns full string definition for message of type 'perfectarray"
  (cl:format cl:nil "std_msgs/Header header~%perfect[] objects~%min_seq[16] min_seq~%max_seq[16] max_seq~%center_position center~%center_position min_center~%center_position max_center~%point min_object~%point max_object~%int32 min_hori~%int32 max_hori~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pharos_vlp_tilt/perfect~%info info~%point point~%state state~%~%================================================================================~%MSG: pharos_vlp_tilt/info~%int32 laser~%int32 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/point~%float64 x~%float64 y~%float64 z~%float64 intensity~%~%================================================================================~%MSG: pharos_vlp_tilt/state~%int32 is_ground~%int32 is_del~%int32 is_infect~%~%================================================================================~%MSG: pharos_vlp_tilt/min_seq~%int16 i~%int16 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/max_seq~%int16 i~%int16 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/center_position~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <perfectarray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'min_seq) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'max_seq) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min_center))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max_center))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min_object))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max_object))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <perfectarray>))
  "Converts a ROS message object to a list"
  (cl:list 'perfectarray
    (cl:cons ':header (header msg))
    (cl:cons ':objects (objects msg))
    (cl:cons ':min_seq (min_seq msg))
    (cl:cons ':max_seq (max_seq msg))
    (cl:cons ':center (center msg))
    (cl:cons ':min_center (min_center msg))
    (cl:cons ':max_center (max_center msg))
    (cl:cons ':min_object (min_object msg))
    (cl:cons ':max_object (max_object msg))
    (cl:cons ':min_hori (min_hori msg))
    (cl:cons ':max_hori (max_hori msg))
))
