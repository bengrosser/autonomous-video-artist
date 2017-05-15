
(cl:in-package :asdf)

(defsystem "im_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetRGB" :depends-on ("_package_SetRGB"))
    (:file "_package_SetRGB" :depends-on ("_package"))
  ))