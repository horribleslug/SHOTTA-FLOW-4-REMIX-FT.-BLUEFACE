; Auto-generated. Do not edit!


(cl:in-package enph353_gazebo-srv)


;//! \htmlinclude GetLegalPlates-request.msg.html

(cl:defclass <GetLegalPlates-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform ""))
)

(cl:defclass GetLegalPlates-request (<GetLegalPlates-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLegalPlates-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLegalPlates-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name enph353_gazebo-srv:<GetLegalPlates-request> is deprecated: use enph353_gazebo-srv:GetLegalPlates-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <GetLegalPlates-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader enph353_gazebo-srv:id-val is deprecated.  Use enph353_gazebo-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLegalPlates-request>) ostream)
  "Serializes a message object of type '<GetLegalPlates-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLegalPlates-request>) istream)
  "Deserializes a message object of type '<GetLegalPlates-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLegalPlates-request>)))
  "Returns string type for a service object of type '<GetLegalPlates-request>"
  "enph353_gazebo/GetLegalPlatesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLegalPlates-request)))
  "Returns string type for a service object of type 'GetLegalPlates-request"
  "enph353_gazebo/GetLegalPlatesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLegalPlates-request>)))
  "Returns md5sum for a message object of type '<GetLegalPlates-request>"
  "6ea3d197527ea87828940965246c0b06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLegalPlates-request)))
  "Returns md5sum for a message object of type 'GetLegalPlates-request"
  "6ea3d197527ea87828940965246c0b06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLegalPlates-request>)))
  "Returns full string definition for message of type '<GetLegalPlates-request>"
  (cl:format cl:nil "string id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLegalPlates-request)))
  "Returns full string definition for message of type 'GetLegalPlates-request"
  (cl:format cl:nil "string id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLegalPlates-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLegalPlates-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLegalPlates-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude GetLegalPlates-response.msg.html

(cl:defclass <GetLegalPlates-response> (roslisp-msg-protocol:ros-message)
  ((plates
    :reader plates
    :initarg :plates
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass GetLegalPlates-response (<GetLegalPlates-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLegalPlates-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLegalPlates-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name enph353_gazebo-srv:<GetLegalPlates-response> is deprecated: use enph353_gazebo-srv:GetLegalPlates-response instead.")))

(cl:ensure-generic-function 'plates-val :lambda-list '(m))
(cl:defmethod plates-val ((m <GetLegalPlates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader enph353_gazebo-srv:plates-val is deprecated.  Use enph353_gazebo-srv:plates instead.")
  (plates m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLegalPlates-response>) ostream)
  "Serializes a message object of type '<GetLegalPlates-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'plates))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'plates))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLegalPlates-response>) istream)
  "Deserializes a message object of type '<GetLegalPlates-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'plates) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'plates)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLegalPlates-response>)))
  "Returns string type for a service object of type '<GetLegalPlates-response>"
  "enph353_gazebo/GetLegalPlatesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLegalPlates-response)))
  "Returns string type for a service object of type 'GetLegalPlates-response"
  "enph353_gazebo/GetLegalPlatesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLegalPlates-response>)))
  "Returns md5sum for a message object of type '<GetLegalPlates-response>"
  "6ea3d197527ea87828940965246c0b06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLegalPlates-response)))
  "Returns md5sum for a message object of type 'GetLegalPlates-response"
  "6ea3d197527ea87828940965246c0b06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLegalPlates-response>)))
  "Returns full string definition for message of type '<GetLegalPlates-response>"
  (cl:format cl:nil "string[] plates~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLegalPlates-response)))
  "Returns full string definition for message of type 'GetLegalPlates-response"
  (cl:format cl:nil "string[] plates~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLegalPlates-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'plates) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLegalPlates-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLegalPlates-response
    (cl:cons ':plates (plates msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetLegalPlates)))
  'GetLegalPlates-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetLegalPlates)))
  'GetLegalPlates-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLegalPlates)))
  "Returns string type for a service object of type '<GetLegalPlates>"
  "enph353_gazebo/GetLegalPlates")