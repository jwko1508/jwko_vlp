;; Auto-generated. Do not edit!


(when (boundp 'pharos_vlp_tilt::point)
  (if (not (find-package "PHAROS_VLP_TILT"))
    (make-package "PHAROS_VLP_TILT"))
  (shadow 'point (find-package "PHAROS_VLP_TILT")))
(unless (find-package "PHAROS_VLP_TILT::POINT")
  (make-package "PHAROS_VLP_TILT::POINT"))

(in-package "ROS")
;;//! \htmlinclude point.msg.html


(defclass pharos_vlp_tilt::point
  :super ros::object
  :slots (_x _y _z _intensity ))

(defmethod pharos_vlp_tilt::point
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:intensity __intensity) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _intensity (float __intensity))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:intensity
   (&optional __intensity)
   (if __intensity (setq _intensity __intensity)) _intensity)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _intensity
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _intensity
       (sys::poke _intensity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _intensity
     (setq _intensity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get pharos_vlp_tilt::point :md5sum-) "00ee07b5674ef45ef5f2f55dbd93bbe7")
(setf (get pharos_vlp_tilt::point :datatype-) "pharos_vlp_tilt/point")
(setf (get pharos_vlp_tilt::point :definition-)
      "float64 x
float64 y
float64 z
float64 intensity

")



(provide :pharos_vlp_tilt/point "00ee07b5674ef45ef5f2f55dbd93bbe7")


