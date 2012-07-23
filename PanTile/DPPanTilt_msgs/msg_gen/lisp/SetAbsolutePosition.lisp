; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude SetAbsolutePosition.msg.html

(cl:defclass <SetAbsolutePosition> (roslisp-msg-protocol:ros-message)
  ((panPosition
    :reader panPosition
    :initarg :panPosition
    :type cl:float
    :initform 0.0)
   (tiltPosition
    :reader tiltPosition
    :initarg :tiltPosition
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetAbsolutePosition (<SetAbsolutePosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAbsolutePosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAbsolutePosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<SetAbsolutePosition> is deprecated: use DPPanTilt_msgs-msg:SetAbsolutePosition instead.")))

(cl:ensure-generic-function 'panPosition-val :lambda-list '(m))
(cl:defmethod panPosition-val ((m <SetAbsolutePosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panPosition-val is deprecated.  Use DPPanTilt_msgs-msg:panPosition instead.")
  (panPosition m))

(cl:ensure-generic-function 'tiltPosition-val :lambda-list '(m))
(cl:defmethod tiltPosition-val ((m <SetAbsolutePosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltPosition-val is deprecated.  Use DPPanTilt_msgs-msg:tiltPosition instead.")
  (tiltPosition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAbsolutePosition>) ostream)
  "Serializes a message object of type '<SetAbsolutePosition>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAbsolutePosition>) istream)
  "Deserializes a message object of type '<SetAbsolutePosition>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panPosition) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltPosition) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAbsolutePosition>)))
  "Returns string type for a message object of type '<SetAbsolutePosition>"
  "DPPanTilt_msgs/SetAbsolutePosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAbsolutePosition)))
  "Returns string type for a message object of type 'SetAbsolutePosition"
  "DPPanTilt_msgs/SetAbsolutePosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAbsolutePosition>)))
  "Returns md5sum for a message object of type '<SetAbsolutePosition>"
  "06c39833eb74fde9834f06e8b2303d18")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAbsolutePosition)))
  "Returns md5sum for a message object of type 'SetAbsolutePosition"
  "06c39833eb74fde9834f06e8b2303d18")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAbsolutePosition>)))
  "Returns full string definition for message of type '<SetAbsolutePosition>"
  (cl:format cl:nil "float32 panPosition~%float32 tiltPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAbsolutePosition)))
  "Returns full string definition for message of type 'SetAbsolutePosition"
  (cl:format cl:nil "float32 panPosition~%float32 tiltPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAbsolutePosition>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAbsolutePosition>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAbsolutePosition
    (cl:cons ':panPosition (panPosition msg))
    (cl:cons ':tiltPosition (tiltPosition msg))
))
