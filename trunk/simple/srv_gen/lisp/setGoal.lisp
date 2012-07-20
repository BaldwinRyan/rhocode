; Auto-generated. Do not edit!


(cl:in-package simple-srv)


;//! \htmlinclude setGoal-request.msg.html

(cl:defclass <setGoal-request> (roslisp-msg-protocol:ros-message)
  ((latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0))
)

(cl:defclass setGoal-request (<setGoal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setGoal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setGoal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple-srv:<setGoal-request> is deprecated: use simple-srv:setGoal-request instead.")))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <setGoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple-srv:latitude-val is deprecated.  Use simple-srv:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <setGoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple-srv:longitude-val is deprecated.  Use simple-srv:longitude instead.")
  (longitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setGoal-request>) ostream)
  "Serializes a message object of type '<setGoal-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setGoal-request>) istream)
  "Deserializes a message object of type '<setGoal-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setGoal-request>)))
  "Returns string type for a service object of type '<setGoal-request>"
  "simple/setGoalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setGoal-request)))
  "Returns string type for a service object of type 'setGoal-request"
  "simple/setGoalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setGoal-request>)))
  "Returns md5sum for a message object of type '<setGoal-request>"
  "b32bd4bd892938aeb7df6ad5978034bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setGoal-request)))
  "Returns md5sum for a message object of type 'setGoal-request"
  "b32bd4bd892938aeb7df6ad5978034bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setGoal-request>)))
  "Returns full string definition for message of type '<setGoal-request>"
  (cl:format cl:nil "~%float64 latitude~%float64 longitude~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setGoal-request)))
  "Returns full string definition for message of type 'setGoal-request"
  (cl:format cl:nil "~%float64 latitude~%float64 longitude~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setGoal-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setGoal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setGoal-request
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
))
;//! \htmlinclude setGoal-response.msg.html

(cl:defclass <setGoal-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setGoal-response (<setGoal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setGoal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setGoal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple-srv:<setGoal-response> is deprecated: use simple-srv:setGoal-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setGoal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple-srv:success-val is deprecated.  Use simple-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setGoal-response>) ostream)
  "Serializes a message object of type '<setGoal-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setGoal-response>) istream)
  "Deserializes a message object of type '<setGoal-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setGoal-response>)))
  "Returns string type for a service object of type '<setGoal-response>"
  "simple/setGoalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setGoal-response)))
  "Returns string type for a service object of type 'setGoal-response"
  "simple/setGoalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setGoal-response>)))
  "Returns md5sum for a message object of type '<setGoal-response>"
  "b32bd4bd892938aeb7df6ad5978034bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setGoal-response)))
  "Returns md5sum for a message object of type 'setGoal-response"
  "b32bd4bd892938aeb7df6ad5978034bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setGoal-response>)))
  "Returns full string definition for message of type '<setGoal-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setGoal-response)))
  "Returns full string definition for message of type 'setGoal-response"
  (cl:format cl:nil "bool success~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setGoal-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setGoal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setGoal-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setGoal)))
  'setGoal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setGoal)))
  'setGoal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setGoal)))
  "Returns string type for a service object of type '<setGoal>"
  "simple/setGoal")