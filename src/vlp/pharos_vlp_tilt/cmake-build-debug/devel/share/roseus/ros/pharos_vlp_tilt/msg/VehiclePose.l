;; Auto-generated. Do not edit!


(when (boundp 'pharos_vlp_tilt::VehiclePose)
  (if (not (find-package "PHAROS_VLP_TILT"))
    (make-package "PHAROS_VLP_TILT"))
  (shadow 'VehiclePose (find-package "PHAROS_VLP_TILT")))
(unless (find-package "PHAROS_VLP_TILT::VEHICLEPOSE")
  (make-package "PHAROS_VLP_TILT::VEHICLEPOSE"))

(in-package "ROS")
;;//! \htmlinclude VehiclePose.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pharos_vlp_tilt::VehiclePose
  :super ros::object
  :slots (_x _y _theta _stamp ))

(defmethod pharos_vlp_tilt::VehiclePose
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    ((:stamp __stamp) (instance std_msgs::Time :init))
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   (setq _stamp __stamp)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:stamp
   (&rest __stamp)
   (if (keywordp (car __stamp))
       (send* _stamp __stamp)
     (progn
       (if __stamp (setq _stamp (car __stamp)))
       _stamp)))
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _theta
    8
    ;; std_msgs/Time _stamp
    (send _stamp :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; std_msgs/Time _stamp
       (send _stamp :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; std_msgs/Time _stamp
     (send _stamp :deserialize buf ptr-) (incf ptr- (send _stamp :serialization-length))
   ;;
   self)
  )

(setf (get pharos_vlp_tilt::VehiclePose :md5sum-) "a4cf57efffc41543b58e200275651435")
(setf (get pharos_vlp_tilt::VehiclePose :datatype-) "pharos_vlp_tilt/VehiclePose")
(setf (get pharos_vlp_tilt::VehiclePose :definition-)
      "float64 x
float64 y
float64 theta
std_msgs/Time stamp
================================================================================
MSG: std_msgs/Time
time data

")



(provide :pharos_vlp_tilt/VehiclePose "a4cf57efffc41543b58e200275651435")


