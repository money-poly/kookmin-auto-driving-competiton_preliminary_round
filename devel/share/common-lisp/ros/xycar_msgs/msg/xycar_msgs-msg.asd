
(cl:in-package :asdf)

(defsystem "xycar_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "xycar_motor" :depends-on ("_package_xycar_motor"))
    (:file "_package_xycar_motor" :depends-on ("_package"))
    (:file "xycar_ultrasounds" :depends-on ("_package_xycar_ultrasounds"))
    (:file "_package_xycar_ultrasounds" :depends-on ("_package"))
  ))