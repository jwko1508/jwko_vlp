; Auto-generated. Do not edit!


(cl:in-package pharos_vlp_tilt-msg)


;//! \htmlinclude state.msg.html

(cl:defclass <state> (roslisp-msg-protocol:ros-message)
  ((is_ground
    :reader is_ground
    :initarg :is_ground
    :type cl:integer
    :initform 0)
   (is_del
    :reader is_del
    :initarg :is_del
    :type cl:integer
    :initform 0)
   (is_infect
    :reader is_infect
    :initarg :is_infect
    :type cl:integer
    :initform 0))
)

(cl:defclass state (<state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pharos_vlp_tilt-msg:<state> is deprecated: use pharos_vlp_tilt-msg:state instead.")))

(cl:ensure-generic-function 'is_ground-val :lambda-list '(m))
(cl:defmethod is_ground-val ((m <state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:is_ground-val is deprecated.  Use pharos_vlp_tilt-msg:is_ground instead.")
  (is_ground m))

(cl:ensure-generic-function 'is_del-val :lambda-list '(m))
(cl:defmethod is_del-val ((m <state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:is_del-val is deprecated.  Use pharos_vlp_tilt-msg:is_del instead.")
  (is_del m))

(cl:ensure-generic-function 'is_infect-val :lambda-list '(m))
(cl:defmethod is_infect-val ((m <state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:is_infect-val is deprecated.  Use pharos_vlp_tilt-msg:is_infect instead.")
  (is_infect m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <state>) ostream)
  "Serializes a message object of type '<state>"
  (cl:let* ((signed (cl:slot-value msg 'is_ground)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'is_del)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'is_infect)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <state>) istream)
  "Deserializes a message object of type '<state>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'is_ground) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'is_del) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'is_infect) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<state>)))
  "Returns string type for a message object of type '<state>"
  "pharos_vlp_tilt/state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'state)))
  "Returns string type for a message object of type 'state"
  "pharos_vlp_tilt/state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<state>)))
  "Returns md5sum for a message object of type '<state>"
  "4dde323cb233595cceb5a9451b77b1b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'state)))
  "Returns md5sum for a message object of type 'state"
  "4dde323cb233595cceb5a9451b77b1b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<state>)))
  "Returns full string definition for message of type '<state>"
  (cl:format cl:nil "int32 is_ground~%int32 is_del~%int32 is_infect~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'state)))
  "Returns full string definition for message of type 'state"
  (cl:format cl:nil "int32 is_ground~%int32 is_del~%int32 is_infect~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <state>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <state>))
  "Converts a ROS message object to a list"
  (cl:list 'state
    (cl:cons ':is_ground (is_ground msg))
    (cl:cons ':is_del (is_del msg))
    (cl:cons ':is_infect (is_infect msg))
))
