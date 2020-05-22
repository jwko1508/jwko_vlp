;; Auto-generated. Do not edit!


(when (boundp 'pharos_vlp_tilt::perfect)
  (if (not (find-package "PHAROS_VLP_TILT"))
    (make-package "PHAROS_VLP_TILT"))
  (shadow 'perfect (find-package "PHAROS_VLP_TILT")))
(unless (find-package "PHAROS_VLP_TILT::PERFECT")
  (make-package "PHAROS_VLP_TILT::PERFECT"))

(in-package "ROS")
;;//! \htmlinclude perfect.msg.html


(defclass pharos_vlp_tilt::perfect
  :super ros::object
  :slots (_info _point _state ))

(defmethod pharos_vlp_tilt::perfect
  (:init
   (&key
    ((:info __info) (instance pharos_vlp_tilt::info :init))
    ((:point __point) (instance pharos_vlp_tilt::point :init))
    ((:state __state) (instance pharos_vlp_tilt::state :init))
    )
   (send-super :init)
   (setq _info __info)
   (setq _point __point)
   (setq _state __state)
   self)
  (:info
   (&rest __info)
   (if (keywordp (car __info))
       (send* _info __info)
     (progn
       (if __info (setq _info (car __info)))
       _info)))
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:state
   (&rest __state)
   (if (keywordp (car __state))
       (send* _state __state)
     (progn
       (if __state (setq _state (car __state)))
       _state)))
  (:serialization-length
   ()
   (+
    ;; pharos_vlp_tilt/info _info
    (send _info :serialization-length)
    ;; pharos_vlp_tilt/point _point
    (send _point :serialization-length)
    ;; pharos_vlp_tilt/state _state
    (send _state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; pharos_vlp_tilt/info _info
       (send _info :serialize s)
     ;; pharos_vlp_tilt/point _point
       (send _point :serialize s)
     ;; pharos_vlp_tilt/state _state
       (send _state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; pharos_vlp_tilt/info _info
     (send _info :deserialize buf ptr-) (incf ptr- (send _info :serialization-length))
   ;; pharos_vlp_tilt/point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; pharos_vlp_tilt/state _state
     (send _state :deserialize buf ptr-) (incf ptr- (send _state :serialization-length))
   ;;
   self)
  )

(setf (get pharos_vlp_tilt::perfect :md5sum-) "90a56c4e8308c1352b958efc8367b00b")
(setf (get pharos_vlp_tilt::perfect :datatype-) "pharos_vlp_tilt/perfect")
(setf (get pharos_vlp_tilt::perfect :definition-)
      "info info
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

")



(provide :pharos_vlp_tilt/perfect "90a56c4e8308c1352b958efc8367b00b")


