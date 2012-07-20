; Auto-generated. Do not edit!


(cl:in-package simple-srv)


;//! \htmlinclude set_goal-request.msg.html

(cl:defclass <set_goal-request> (roslisp-msg-protocol:ros-message)
  ((destination_reached
    :reader destination_reached
    :initarg :destination_reached
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass set_goal-request (<set_goal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_goal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_goal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple-srv:<set_goal-request> is deprecated: use simple-srv:set_goal-request instead.")))

(cl:ensure-generic-function 'destination_reached-val :lambda-list '(m))
(cl:defmethod destination_reached-val ((m <set_goal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple-srv:destination_reached-val is deprecated.  Use simple-srv:destination_reached instead.")
  (destination_reached m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_goal-request>) ostream)
  "Serializes a message object of type '<set_goal-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'destination_reached) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_goal-request>) istream)
  "Deserializes a message object of type '<set_goal-request>"
    (cl:setf (cl:slot-value msg 'destination_reached) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_goal-request>)))
  "Returns string type for a service object of type '<set_goal-request>"
  "simple/set_goalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_goal-request)))
  "Returns string type for a service object of type 'set_goal-request"
  "simple/set_goalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_goal-request>)))
  "Returns md5sum for a message object of type '<set_goal-request>"
  "bdb49742d07e21edd728ab9a496b6f93")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_goal-request)))
  "Returns md5sum for a message object of type 'set_goal-request"
  "bdb49742d07e21edd728ab9a496b6f93")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_goal-request>)))
  "Returns full string definition for message of type '<set_goal-request>"
  (cl:format cl:nil "~%~%~%bool destination_reached~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_goal-request)))
  "Returns full string definition for message of type 'set_goal-request"
  (cl:format cl:nil "~%~%~%bool destination_reached~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_goal-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_goal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'set_goal-request
    (cl:cons ':destination_reached (destination_reached msg))
))
;//! \htmlinclude set_goal-response.msg.html

(cl:defclass <set_goal-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass set_goal-response (<set_goal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_goal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_goal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple-srv:<set_goal-response> is deprecated: use simple-srv:set_goal-response instead.")))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <set_goal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple-srv:latitude-val is deprecated.  Use simple-srv:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <set_goal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple-srv:longitude-val is deprecated.  Use simple-srv:longitude instead.")
  (longitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_goal-response>) ostream)
  "Serializes a message object of type '<set_goal-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_goal-response>) istream)
  "Deserializes a message object of type '<set_goal-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_goal-response>)))
  "Returns string type for a service object of type '<set_goal-response>"
  "simple/set_goalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_goal-response)))
  "Returns string type for a service object of type 'set_goal-response"
  "simple/set_goalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_goal-response>)))
  "Returns md5sum for a message object of type '<set_goal-response>"
  "bdb49742d07e21edd728ab9a496b6f93")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_goal-response)))
  "Returns md5sum for a message object of type 'set_goal-response"
  "bdb49742d07e21edd728ab9a496b6f93")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_goal-response>)))
  "Returns full string definition for message of type '<set_goal-response>"
  (cl:format cl:nil "~%float64 latitude~%float64 longitude~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_goal-response)))
  "Returns full string definition for message of type 'set_goal-response"
  (cl:format cl:nil "~%float64 latitude~%float64 longitude~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_goal-response>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_goal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'set_goal-response
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'set_goal)))
  'set_goal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'set_goal)))
  'set_goal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_goal)))
  "Returns string type for a service object of type '<set_goal>"
  "simple/set_goal")