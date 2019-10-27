;; Auto-generated. Do not edit!


(when (boundp 'enph353_gazebo::GetLegalPlates)
  (if (not (find-package "ENPH353_GAZEBO"))
    (make-package "ENPH353_GAZEBO"))
  (shadow 'GetLegalPlates (find-package "ENPH353_GAZEBO")))
(unless (find-package "ENPH353_GAZEBO::GETLEGALPLATES")
  (make-package "ENPH353_GAZEBO::GETLEGALPLATES"))
(unless (find-package "ENPH353_GAZEBO::GETLEGALPLATESREQUEST")
  (make-package "ENPH353_GAZEBO::GETLEGALPLATESREQUEST"))
(unless (find-package "ENPH353_GAZEBO::GETLEGALPLATESRESPONSE")
  (make-package "ENPH353_GAZEBO::GETLEGALPLATESRESPONSE"))

(in-package "ROS")





(defclass enph353_gazebo::GetLegalPlatesRequest
  :super ros::object
  :slots (_id ))

(defmethod enph353_gazebo::GetLegalPlatesRequest
  (:init
   (&key
    ((:id __id) "")
    )
   (send-super :init)
   (setq _id (string __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass enph353_gazebo::GetLegalPlatesResponse
  :super ros::object
  :slots (_plates ))

(defmethod enph353_gazebo::GetLegalPlatesResponse
  (:init
   (&key
    ((:plates __plates) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _plates __plates)
   self)
  (:plates
   (&optional __plates)
   (if __plates (setq _plates __plates)) _plates)
  (:serialization-length
   ()
   (+
    ;; string[] _plates
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _plates)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _plates
     (write-long (length _plates) s)
     (dolist (elem _plates)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _plates
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _plates (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _plates i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass enph353_gazebo::GetLegalPlates
  :super ros::object
  :slots ())

(setf (get enph353_gazebo::GetLegalPlates :md5sum-) "6ea3d197527ea87828940965246c0b06")
(setf (get enph353_gazebo::GetLegalPlates :datatype-) "enph353_gazebo/GetLegalPlates")
(setf (get enph353_gazebo::GetLegalPlates :request) enph353_gazebo::GetLegalPlatesRequest)
(setf (get enph353_gazebo::GetLegalPlates :response) enph353_gazebo::GetLegalPlatesResponse)

(defmethod enph353_gazebo::GetLegalPlatesRequest
  (:response () (instance enph353_gazebo::GetLegalPlatesResponse :init)))

(setf (get enph353_gazebo::GetLegalPlatesRequest :md5sum-) "6ea3d197527ea87828940965246c0b06")
(setf (get enph353_gazebo::GetLegalPlatesRequest :datatype-) "enph353_gazebo/GetLegalPlatesRequest")
(setf (get enph353_gazebo::GetLegalPlatesRequest :definition-)
      "string id
---
string[] plates
")

(setf (get enph353_gazebo::GetLegalPlatesResponse :md5sum-) "6ea3d197527ea87828940965246c0b06")
(setf (get enph353_gazebo::GetLegalPlatesResponse :datatype-) "enph353_gazebo/GetLegalPlatesResponse")
(setf (get enph353_gazebo::GetLegalPlatesResponse :definition-)
      "string id
---
string[] plates
")



(provide :enph353_gazebo/GetLegalPlates "6ea3d197527ea87828940965246c0b06")


