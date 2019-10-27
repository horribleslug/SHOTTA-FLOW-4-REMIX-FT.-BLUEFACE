;; Auto-generated. Do not edit!


(when (boundp 'adeept_awr_ros_driver::ArrayIR)
  (if (not (find-package "ADEEPT_AWR_ROS_DRIVER"))
    (make-package "ADEEPT_AWR_ROS_DRIVER"))
  (shadow 'ArrayIR (find-package "ADEEPT_AWR_ROS_DRIVER")))
(unless (find-package "ADEEPT_AWR_ROS_DRIVER::ARRAYIR")
  (make-package "ADEEPT_AWR_ROS_DRIVER::ARRAYIR"))

(in-package "ROS")
;;//! \htmlinclude ArrayIR.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass adeept_awr_ros_driver::ArrayIR
  :super ros::object
  :slots (_header _left _middle _right ))

(defmethod adeept_awr_ros_driver::ArrayIR
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left __left) nil)
    ((:middle __middle) nil)
    ((:right __right) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left __left)
   (setq _middle __middle)
   (setq _right __right)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:middle
   (&optional __middle)
   (if __middle (setq _middle __middle)) _middle)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _left
    1
    ;; bool _middle
    1
    ;; bool _right
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _left
       (if _left (write-byte -1 s) (write-byte 0 s))
     ;; bool _middle
       (if _middle (write-byte -1 s) (write-byte 0 s))
     ;; bool _right
       (if _right (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _left
     (setq _left (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _middle
     (setq _middle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right
     (setq _right (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get adeept_awr_ros_driver::ArrayIR :md5sum-) "232d72af632e962eb66f18154418a761")
(setf (get adeept_awr_ros_driver::ArrayIR :datatype-) "adeept_awr_ros_driver/ArrayIR")
(setf (get adeept_awr_ros_driver::ArrayIR :definition-)
      "Header header
bool left
bool middle
bool right

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

")



(provide :adeept_awr_ros_driver/ArrayIR "232d72af632e962eb66f18154418a761")


