; Auto-generated. Do not edit!


(cl:in-package enph353_gazebo-srv)


;//! \htmlinclude SubmitPlate-request.msg.html

(cl:defclass <SubmitPlate-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform "")
   (plate
    :reader plate
    :initarg :plate
    :type cl:string
    :initform "")
   (location
    :reader location
    :initarg :location
    :type cl:fixnum
    :initform 0)
   (legal
    :reader legal
    :initarg :legal
    :type cl:boolean
    :initform cl:nil)
   (evidence
    :reader evidence
    :initarg :evidence
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass SubmitPlate-request (<SubmitPlate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SubmitPlate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SubmitPlate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name enph353_gazebo-srv:<SubmitPlate-request> is deprecated: use enph353_gazebo-srv:SubmitPlate-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SubmitPlate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader enph353_gazebo-srv:id-val is deprecated.  Use enph353_gazebo-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'plate-val :lambda-list '(m))
(cl:defmethod plate-val ((m <SubmitPlate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader enph353_gazebo-srv:plate-val is deprecated.  Use enph353_gazebo-srv:plate instead.")
  (plate m))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <SubmitPlate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader enph353_gazebo-srv:location-val is deprecated.  Use enph353_gazebo-srv:location instead.")
  (location m))

(cl:ensure-generic-function 'legal-val :lambda-list '(m))
(cl:defmethod legal-val ((m <SubmitPlate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader enph353_gazebo-srv:legal-val is deprecated.  Use enph353_gazebo-srv:legal instead.")
  (legal m))

(cl:ensure-generic-function 'evidence-val :lambda-list '(m))
(cl:defmethod evidence-val ((m <SubmitPlate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader enph353_gazebo-srv:evidence-val is deprecated.  Use enph353_gazebo-srv:evidence instead.")
  (evidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SubmitPlate-request>) ostream)
  "Serializes a message object of type '<SubmitPlate-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'plate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'plate))
  (cl:let* ((signed (cl:slot-value msg 'location)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'legal) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'evidence) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SubmitPlate-request>) istream)
  "Deserializes a message object of type '<SubmitPlate-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'plate) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'plate) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'legal) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'evidence) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SubmitPlate-request>)))
  "Returns string type for a service object of type '<SubmitPlate-request>"
  "enph353_gazebo/SubmitPlateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SubmitPlate-request)))
  "Returns string type for a service object of type 'SubmitPlate-request"
  "enph353_gazebo/SubmitPlateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SubmitPlate-request>)))
  "Returns md5sum for a message object of type '<SubmitPlate-request>"
  "d7a5f82904bfc83df9f5ede6fec81ad1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SubmitPlate-request)))
  "Returns md5sum for a message object of type 'SubmitPlate-request"
  "d7a5f82904bfc83df9f5ede6fec81ad1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SubmitPlate-request>)))
  "Returns full string definition for message of type '<SubmitPlate-request>"
  (cl:format cl:nil "string id~%string plate~%int8 location~%bool legal~%sensor_msgs/Image evidence~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SubmitPlate-request)))
  "Returns full string definition for message of type 'SubmitPlate-request"
  (cl:format cl:nil "string id~%string plate~%int8 location~%bool legal~%sensor_msgs/Image evidence~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SubmitPlate-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
     4 (cl:length (cl:slot-value msg 'plate))
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'evidence))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SubmitPlate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SubmitPlate-request
    (cl:cons ':id (id msg))
    (cl:cons ':plate (plate msg))
    (cl:cons ':location (location msg))
    (cl:cons ':legal (legal msg))
    (cl:cons ':evidence (evidence msg))
))
;//! \htmlinclude SubmitPlate-response.msg.html

(cl:defclass <SubmitPlate-response> (roslisp-msg-protocol:ros-message)
  ((correct
    :reader correct
    :initarg :correct
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SubmitPlate-response (<SubmitPlate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SubmitPlate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SubmitPlate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name enph353_gazebo-srv:<SubmitPlate-response> is deprecated: use enph353_gazebo-srv:SubmitPlate-response instead.")))

(cl:ensure-generic-function 'correct-val :lambda-list '(m))
(cl:defmethod correct-val ((m <SubmitPlate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader enph353_gazebo-srv:correct-val is deprecated.  Use enph353_gazebo-srv:correct instead.")
  (correct m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SubmitPlate-response>) ostream)
  "Serializes a message object of type '<SubmitPlate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'correct) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SubmitPlate-response>) istream)
  "Deserializes a message object of type '<SubmitPlate-response>"
    (cl:setf (cl:slot-value msg 'correct) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SubmitPlate-response>)))
  "Returns string type for a service object of type '<SubmitPlate-response>"
  "enph353_gazebo/SubmitPlateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SubmitPlate-response)))
  "Returns string type for a service object of type 'SubmitPlate-response"
  "enph353_gazebo/SubmitPlateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SubmitPlate-response>)))
  "Returns md5sum for a message object of type '<SubmitPlate-response>"
  "d7a5f82904bfc83df9f5ede6fec81ad1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SubmitPlate-response)))
  "Returns md5sum for a message object of type 'SubmitPlate-response"
  "d7a5f82904bfc83df9f5ede6fec81ad1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SubmitPlate-response>)))
  "Returns full string definition for message of type '<SubmitPlate-response>"
  (cl:format cl:nil "bool correct~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SubmitPlate-response)))
  "Returns full string definition for message of type 'SubmitPlate-response"
  (cl:format cl:nil "bool correct~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SubmitPlate-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SubmitPlate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SubmitPlate-response
    (cl:cons ':correct (correct msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SubmitPlate)))
  'SubmitPlate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SubmitPlate)))
  'SubmitPlate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SubmitPlate)))
  "Returns string type for a service object of type '<SubmitPlate>"
  "enph353_gazebo/SubmitPlate")