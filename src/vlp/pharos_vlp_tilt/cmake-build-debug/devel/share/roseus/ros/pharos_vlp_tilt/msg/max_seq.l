;; Auto-generated. Do not edit!


(when (boundp 'pharos_vlp_tilt::max_seq)
  (if (not (find-package "PHAROS_VLP_TILT"))
    (make-package "PHAROS_VLP_TILT"))
  (shadow 'max_seq (find-package "PHAROS_VLP_TILT")))
(unless (find-package "PHAROS_VLP_TILT::MAX_SEQ")
  (make-package "PHAROS_VLP_TILT::MAX_SEQ"))

(in-package "ROS")
;;//! \htmlinclude max_seq.msg.html


(defclass pharos_vlp_tilt::max_seq
  :super ros::object
  :slots (_i _hori ))

(defmethod pharos_vlp_tilt::max_seq
  (:init
   (&key
    ((:i __i) 0)
    ((:hori __hori) 0)
    )
   (send-super :init)
   (setq _i (round __i))
   (setq _hori (round __hori))
   self)
  (:i
   (&optional __i)
   (if __i (setq _i __i)) _i)
  (:hori
   (&optional __hori)
   (if __hori (setq _hori __hori)) _hori)
  (:serialization-length
   ()
   (+
    ;; int16 _i
    2
    ;; int16 _hori
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _i
       (write-word _i s)
     ;; int16 _hori
       (write-word _hori s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _i
     (setq _i (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _hori
     (setq _hori (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get pharos_vlp_tilt::max_seq :md5sum-) "22324cdcfc8df707c9b750202d426c68")
(setf (get pharos_vlp_tilt::max_seq :datatype-) "pharos_vlp_tilt/max_seq")
(setf (get pharos_vlp_tilt::max_seq :definition-)
      "int16 i
int16 hori

")



(provide :pharos_vlp_tilt/max_seq "22324cdcfc8df707c9b750202d426c68")


