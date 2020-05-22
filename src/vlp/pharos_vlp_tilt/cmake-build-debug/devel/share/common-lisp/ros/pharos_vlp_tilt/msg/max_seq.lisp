; Auto-generated. Do not edit!


(cl:in-package pharos_vlp_tilt-msg)


;//! \htmlinclude max_seq.msg.html

(cl:defclass <max_seq> (roslisp-msg-protocol:ros-message)
  ((i
    :reader i
    :initarg :i
    :type cl:fixnum
    :initform 0)
   (hori
    :reader hori
    :initarg :hori
    :type cl:fixnum
    :initform 0))
)

(cl:defclass max_seq (<max_seq>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <max_seq>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'max_seq)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pharos_vlp_tilt-msg:<max_seq> is deprecated: use pharos_vlp_tilt-msg:max_seq instead.")))

(cl:ensure-generic-function 'i-val :lambda-list '(m))
(cl:defmethod i-val ((m <max_seq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:i-val is deprecated.  Use pharos_vlp_tilt-msg:i instead.")
  (i m))

(cl:ensure-generic-function 'hori-val :lambda-list '(m))
(cl:defmethod hori-val ((m <max_seq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pharos_vlp_tilt-msg:hori-val is deprecated.  Use pharos_vlp_tilt-msg:hori instead.")
  (hori m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <max_seq>) ostream)
  "Serializes a message object of type '<max_seq>"
  (cl:let* ((signed (cl:slot-value msg 'i)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'hori)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <max_seq>) istream)
  "Deserializes a message object of type '<max_seq>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hori) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<max_seq>)))
  "Returns string type for a message object of type '<max_seq>"
  "pharos_vlp_tilt/max_seq")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'max_seq)))
  "Returns string type for a message object of type 'max_seq"
  "pharos_vlp_tilt/max_seq")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<max_seq>)))
  "Returns md5sum for a message object of type '<max_seq>"
  "22324cdcfc8df707c9b750202d426c68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'max_seq)))
  "Returns md5sum for a message object of type 'max_seq"
  "22324cdcfc8df707c9b750202d426c68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<max_seq>)))
  "Returns full string definition for message of type '<max_seq>"
  (cl:format cl:nil "int16 i~%int16 hori~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'max_seq)))
  "Returns full string definition for message of type 'max_seq"
  (cl:format cl:nil "int16 i~%int16 hori~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <max_seq>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <max_seq>))
  "Converts a ROS message object to a list"
  (cl:list 'max_seq
    (cl:cons ':i (i msg))
    (cl:cons ':hori (hori msg))
))
