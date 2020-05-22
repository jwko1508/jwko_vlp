;; Auto-generated. Do not edit!


(when (boundp 'pharos_vlp_tilt::state)
  (if (not (find-package "PHAROS_VLP_TILT"))
    (make-package "PHAROS_VLP_TILT"))
  (shadow 'state (find-package "PHAROS_VLP_TILT")))
(unless (find-package "PHAROS_VLP_TILT::STATE")
  (make-package "PHAROS_VLP_TILT::STATE"))

(in-package "ROS")
;;//! \htmlinclude state.msg.html


(defclass pharos_vlp_tilt::state
  :super ros::object
  :slots (_is_ground _is_del _is_infect ))

(defmethod pharos_vlp_tilt::state
  (:init
   (&key
    ((:is_ground __is_ground) 0)
    ((:is_del __is_del) 0)
    ((:is_infect __is_infect) 0)
    )
   (send-super :init)
   (setq _is_ground (round __is_ground))
   (setq _is_del (round __is_del))
   (setq _is_infect (round __is_infect))
   self)
  (:is_ground
   (&optional __is_ground)
   (if __is_ground (setq _is_ground __is_ground)) _is_ground)
  (:is_del
   (&optional __is_del)
   (if __is_del (setq _is_del __is_del)) _is_del)
  (:is_infect
   (&optional __is_infect)
   (if __is_infect (setq _is_infect __is_infect)) _is_infect)
  (:serialization-length
   ()
   (+
    ;; int32 _is_ground
    4
    ;; int32 _is_del
    4
    ;; int32 _is_infect
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _is_ground
       (write-long _is_ground s)
     ;; int32 _is_del
       (write-long _is_del s)
     ;; int32 _is_infect
       (write-long _is_infect s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _is_ground
     (setq _is_ground (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _is_del
     (setq _is_del (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _is_infect
     (setq _is_infect (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pharos_vlp_tilt::state :md5sum-) "4dde323cb233595cceb5a9451b77b1b4")
(setf (get pharos_vlp_tilt::state :datatype-) "pharos_vlp_tilt/state")
(setf (get pharos_vlp_tilt::state :definition-)
      "int32 is_ground
int32 is_del
int32 is_infect

")



(provide :pharos_vlp_tilt/state "4dde323cb233595cceb5a9451b77b1b4")


