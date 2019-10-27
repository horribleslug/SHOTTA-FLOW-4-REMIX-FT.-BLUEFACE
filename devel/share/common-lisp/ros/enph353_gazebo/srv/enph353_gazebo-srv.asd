
(cl:in-package :asdf)

(defsystem "enph353_gazebo-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetLegalPlates" :depends-on ("_package_GetLegalPlates"))
    (:file "_package_GetLegalPlates" :depends-on ("_package"))
    (:file "SubmitPlate" :depends-on ("_package_SubmitPlate"))
    (:file "_package_SubmitPlate" :depends-on ("_package"))
  ))