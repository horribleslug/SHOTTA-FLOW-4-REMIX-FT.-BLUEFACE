
(cl:in-package :asdf)

(defsystem "adeept_awr_ros_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ArrayIR" :depends-on ("_package_ArrayIR"))
    (:file "_package_ArrayIR" :depends-on ("_package"))
  ))