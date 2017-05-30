
(cl:in-package :asdf)

(defsystem "im_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Battery" :depends-on ("_package_Battery"))
    (:file "_package_Battery" :depends-on ("_package"))
    (:file "Bumper" :depends-on ("_package_Bumper"))
    (:file "_package_Bumper" :depends-on ("_package"))
    (:file "WheelVel" :depends-on ("_package_WheelVel"))
    (:file "_package_WheelVel" :depends-on ("_package"))
    (:file "Voltage" :depends-on ("_package_Voltage"))
    (:file "_package_Voltage" :depends-on ("_package"))
    (:file "BumperState" :depends-on ("_package_BumperState"))
    (:file "_package_BumperState" :depends-on ("_package"))
  ))