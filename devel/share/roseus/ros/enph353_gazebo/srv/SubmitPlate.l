;; Auto-generated. Do not edit!


(when (boundp 'enph353_gazebo::SubmitPlate)
  (if (not (find-package "ENPH353_GAZEBO"))
    (make-package "ENPH353_GAZEBO"))
  (shadow 'SubmitPlate (find-package "ENPH353_GAZEBO")))
(unless (find-package "ENPH353_GAZEBO::SUBMITPLATE")
  (make-package "ENPH353_GAZEBO::SUBMITPLATE"))
(unless (find-package "ENPH353_GAZEBO::SUBMITPLATEREQUEST")
  (make-package "ENPH353_GAZEBO::SUBMITPLATEREQUEST"))
(unless (find-package "ENPH353_GAZEBO::SUBMITPLATERESPONSE")
  (make-package "ENPH353_GAZEBO::SUBMITPLATERESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass enph353_gazebo::SubmitPlateRequest
  :super ros::object
  :slots (_id _plate _location _legal _evidence ))

(defmethod enph353_gazebo::SubmitPlateRequest
  (:init
   (&key
    ((:id __id) "")
    ((:plate __plate) "")
    ((:location __location) 0)
    ((:legal __legal) nil)
    ((:evidence __evidence) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _plate (string __plate))
   (setq _location (round __location))
   (setq _legal __legal)
   (setq _evidence __evidence)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:plate
   (&optional __plate)
   (if __plate (setq _plate __plate)) _plate)
  (:location
   (&optional __location)
   (if __location (setq _location __location)) _location)
  (:legal
   (&optional __legal)
   (if __legal (setq _legal __legal)) _legal)
  (:evidence
   (&rest __evidence)
   (if (keywordp (car __evidence))
       (send* _evidence __evidence)
     (progn
       (if __evidence (setq _evidence (car __evidence)))
       _evidence)))
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; string _plate
    4 (length _plate)
    ;; int8 _location
    1
    ;; bool _legal
    1
    ;; sensor_msgs/Image _evidence
    (send _evidence :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; string _plate
       (write-long (length _plate) s) (princ _plate s)
     ;; int8 _location
       (write-byte _location s)
     ;; bool _legal
       (if _legal (write-byte -1 s) (write-byte 0 s))
     ;; sensor_msgs/Image _evidence
       (send _evidence :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _plate
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _plate (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _location
     (setq _location (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _location 127) (setq _location (- _location 256)))
   ;; bool _legal
     (setq _legal (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; sensor_msgs/Image _evidence
     (send _evidence :deserialize buf ptr-) (incf ptr- (send _evidence :serialization-length))
   ;;
   self)
  )

(defclass enph353_gazebo::SubmitPlateResponse
  :super ros::object
  :slots (_correct ))

(defmethod enph353_gazebo::SubmitPlateResponse
  (:init
   (&key
    ((:correct __correct) nil)
    )
   (send-super :init)
   (setq _correct __correct)
   self)
  (:correct
   (&optional __correct)
   (if __correct (setq _correct __correct)) _correct)
  (:serialization-length
   ()
   (+
    ;; bool _correct
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _correct
       (if _correct (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _correct
     (setq _correct (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass enph353_gazebo::SubmitPlate
  :super ros::object
  :slots ())

(setf (get enph353_gazebo::SubmitPlate :md5sum-) "d7a5f82904bfc83df9f5ede6fec81ad1")
(setf (get enph353_gazebo::SubmitPlate :datatype-) "enph353_gazebo/SubmitPlate")
(setf (get enph353_gazebo::SubmitPlate :request) enph353_gazebo::SubmitPlateRequest)
(setf (get enph353_gazebo::SubmitPlate :response) enph353_gazebo::SubmitPlateResponse)

(defmethod enph353_gazebo::SubmitPlateRequest
  (:response () (instance enph353_gazebo::SubmitPlateResponse :init)))

(setf (get enph353_gazebo::SubmitPlateRequest :md5sum-) "d7a5f82904bfc83df9f5ede6fec81ad1")
(setf (get enph353_gazebo::SubmitPlateRequest :datatype-) "enph353_gazebo/SubmitPlateRequest")
(setf (get enph353_gazebo::SubmitPlateRequest :definition-)
      "string id
string plate
int8 location
bool legal
sensor_msgs/Image evidence

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
---
bool correct
")

(setf (get enph353_gazebo::SubmitPlateResponse :md5sum-) "d7a5f82904bfc83df9f5ede6fec81ad1")
(setf (get enph353_gazebo::SubmitPlateResponse :datatype-) "enph353_gazebo/SubmitPlateResponse")
(setf (get enph353_gazebo::SubmitPlateResponse :definition-)
      "string id
string plate
int8 location
bool legal
sensor_msgs/Image evidence

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
---
bool correct
")



(provide :enph353_gazebo/SubmitPlate "d7a5f82904bfc83df9f5ede6fec81ad1")


