; Auto-generated. Do not edit!


(cl:in-package pharos_vlp_tilt-msg)


;//! \htmlinclude perfect.msg.html

(cl:defclass <perfect> (roslisp-msg-protocol:ros-message)
  ((info
    :reader info
    :initarg :info
    :type pharos_vlp_tilt-msg:info
    :initform (cl:make-instance 'pharos_vlp_tilt-msg:info))
   (point
    :reader point
    :initarg :point
    :type pharos_vlp_tilt-msg:point
    :initform (cl:make-instance 'pharos_vlp_tilt-msg:point))
   (state
    :reader state
    :initarg :state
    :type pharos_vlp_tilt-msg:state
    :initform (cl:make-instance 'pharos_vlp_tilt-msg:state)))
)

(cl:defclass perfect (<perfect>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <perfect>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'perfect)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pharos_vlp_tilt-msg:<perfect> is deprecated: use pharos_vlp_tilt-msg:perfect instead.")))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <perfect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:info-val is deprecated.  Use pharos_vlp_tilt-msg:info instead.")
  (info m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <perfect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:point-val is deprecated.  Use pharos_vlp_tilt-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <perfect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:state-val is deprecated.  Use pharos_vlp_tilt-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <perfect>) ostream)
  "Serializes a message object of type '<perfect>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'info) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <perfect>) istream)
  "Deserializes a message object of type '<perfect>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'info) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<perfect>)))
  "Returns string type for a message object of type '<perfect>"
  "pharos_vlp_tilt/perfect")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'perfect)))
  "Returns string type for a message object of type 'perfect"
  "pharos_vlp_tilt/perfect")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<perfect>)))
  "Returns md5sum for a message object of type '<perfect>"
  "90a56c4e8308c1352b958efc8367b00b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'perfect)))
  "Returns md5sum for a message object of type 'perfect"
  "90a56c4e8308c1352b958efc8367b00b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<perfect>)))
  "Returns full string definition for message of type '<perfect>"
  (cl:format cl:nil "info info~%point point~%state state~%~%================================================================================~%MSG: pharos_vlp_tilt/info~%int32 laser~%int32 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/point~%float64 x~%float64 y~%float64 z~%float64 intensity~%~%================================================================================~%MSG: pharos_vlp_tilt/state~%int32 is_ground~%int32 is_del~%int32 is_infect~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'perfect)))
  "Returns full string definition for message of type 'perfect"
  (cl:format cl:nil "info info~%point point~%state state~%~%================================================================================~%MSG: pharos_vlp_tilt/info~%int32 laser~%int32 hori~%~%================================================================================~%MSG: pharos_vlp_tilt/point~%float64 x~%float64 y~%float64 z~%float64 intensity~%~%================================================================================~%MSG: pharos_vlp_tilt/state~%int32 is_ground~%int32 is_del~%int32 is_infect~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <perfect>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'info))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <perfect>))
  "Converts a ROS message object to a list"
  (cl:list 'perfect
    (cl:cons ':info (info msg))
    (cl:cons ':point (point msg))
    (cl:cons ':state (state msg))
))
