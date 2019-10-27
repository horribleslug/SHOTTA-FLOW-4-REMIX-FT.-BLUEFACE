; Auto-generated. Do not edit!


(cl:in-package adeept_awr_ros_driver-msg)


;//! \htmlinclude ArrayIR.msg.html

(cl:defclass <ArrayIR> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left
    :reader left
    :initarg :left
    :type cl:boolean
    :initform cl:nil)
   (middle
    :reader middle
    :initarg :middle
    :type cl:boolean
    :initform cl:nil)
   (right
    :reader right
    :initarg :right
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ArrayIR (<ArrayIR>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArrayIR>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArrayIR)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name adeept_awr_ros_driver-msg:<ArrayIR> is deprecated: use adeept_awr_ros_driver-msg:ArrayIR instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ArrayIR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adeept_awr_ros_driver-msg:header-val is deprecated.  Use adeept_awr_ros_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <ArrayIR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adeept_awr_ros_driver-msg:left-val is deprecated.  Use adeept_awr_ros_driver-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'middle-val :lambda-list '(m))
(cl:defmethod middle-val ((m <ArrayIR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adeept_awr_ros_driver-msg:middle-val is deprecated.  Use adeept_awr_ros_driver-msg:middle instead.")
  (middle m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <ArrayIR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adeept_awr_ros_driver-msg:right-val is deprecated.  Use adeept_awr_ros_driver-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArrayIR>) ostream)
  "Serializes a message object of type '<ArrayIR>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'middle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArrayIR>) istream)
  "Deserializes a message object of type '<ArrayIR>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'left) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'middle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArrayIR>)))
  "Returns string type for a message object of type '<ArrayIR>"
  "adeept_awr_ros_driver/ArrayIR")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArrayIR)))
  "Returns string type for a message object of type 'ArrayIR"
  "adeept_awr_ros_driver/ArrayIR")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArrayIR>)))
  "Returns md5sum for a message object of type '<ArrayIR>"
  "232d72af632e962eb66f18154418a761")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArrayIR)))
  "Returns md5sum for a message object of type 'ArrayIR"
  "232d72af632e962eb66f18154418a761")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArrayIR>)))
  "Returns full string definition for message of type '<ArrayIR>"
  (cl:format cl:nil "Header header~%bool left~%bool middle~%bool right~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArrayIR)))
  "Returns full string definition for message of type 'ArrayIR"
  (cl:format cl:nil "Header header~%bool left~%bool middle~%bool right~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArrayIR>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArrayIR>))
  "Converts a ROS message object to a list"
  (cl:list 'ArrayIR
    (cl:cons ':header (header msg))
    (cl:cons ':left (left msg))
    (cl:cons ':middle (middle msg))
    (cl:cons ':right (right msg))
))
