
(cl:in-package :asdf)

(defsystem "pharos_vlp_tilt-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "VehiclePose" :depends-on ("_package_VehiclePose"))
    (:file "_package_VehiclePose" :depends-on ("_package"))
    (:file "VehiclePoseArray" :depends-on ("_package_VehiclePoseArray"))
    (:file "_package_VehiclePoseArray" :depends-on ("_package"))
    (:file "center_position" :depends-on ("_package_center_position"))
    (:file "_package_center_position" :depends-on ("_package"))
    (:file "info" :depends-on ("_package_info"))
    (:file "_package_info" :depends-on ("_package"))
    (:file "max_seq" :depends-on ("_package_max_seq"))
    (:file "_package_max_seq" :depends-on ("_package"))
    (:file "min_seq" :depends-on ("_package_min_seq"))
    (:file "_package_min_seq" :depends-on ("_package"))
    (:file "perfect" :depends-on ("_package_perfect"))
    (:file "_package_perfect" :depends-on ("_package"))
    (:file "perfectarray" :depends-on ("_package_perfectarray"))
    (:file "_package_perfectarray" :depends-on ("_package"))
    (:file "point" :depends-on ("_package_point"))
    (:file "_package_point" :depends-on ("_package"))
    (:file "state" :depends-on ("_package_state"))
    (:file "_package_state" :depends-on ("_package"))
    (:file "vector_perfect_array" :depends-on ("_package_vector_perfect_array"))
    (:file "_package_vector_perfect_array" :depends-on ("_package"))
  ))