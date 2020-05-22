;; Auto-generated. Do not edit!


(when (boundp 'pharos_vlp_tilt::info)
  (if (not (find-package "PHAROS_VLP_TILT"))
    (make-package "PHAROS_VLP_TILT"))
  (shadow 'info (find-package "PHAROS_VLP_TILT")))
(unless (find-package "PHAROS_VLP_TILT::INFO")
  (make-package "PHAROS_VLP_TILT::INFO"))

(in-package "ROS")
;;//! \htmlinclude info.msg.html


(defclass pharos_vlp_tilt::info
  :super ros::object
  :slots (_laser _hori ))

(defmethod pharos_vlp_tilt::info
  (:init
   (&key
    ((:laser __laser) 0)
    ((:hori __hori) 0)
    )
   (send-super :init)
   (setq _laser (round __laser))
   (setq _hori (round __hori))
   self)
  (:laser
   (&optional __laser)
   (if __laser (setq _laser __laser)) _laser)
  (:hori
   (&optional __hori)
   (if __hori (setq _hori __hori)) _hori)
  (:serialization-length
   ()
   (+
    ;; int32 _laser
    4
    ;; int32 _hori
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _laser
       (write-long _laser s)
     ;; int32 _hori
       (write-long _hori s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _laser
     (setq _laser (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _hori
     (setq _hori (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pharos_vlp_tilt::info :md5sum-) "e2ba7373c9fec63a00e8b05a56280e35")
(setf (get pharos_vlp_tilt::info :datatype-) "pharos_vlp_tilt/info")
(setf (get pharos_vlp_tilt::info :definition-)
      "int32 laser
int32 hori

")



(provide :pharos_vlp_tilt/info "e2ba7373c9fec63a00e8b05a56280e35")


