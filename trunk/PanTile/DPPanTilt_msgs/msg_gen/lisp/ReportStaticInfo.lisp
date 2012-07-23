; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude ReportStaticInfo.msg.html

(cl:defclass <ReportStaticInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (panMinimumValidPositionInRadians
    :reader panMinimumValidPositionInRadians
    :initarg :panMinimumValidPositionInRadians
    :type cl:float
    :initform 0.0)
   (tiltMinimumValidPositionInRadians
    :reader tiltMinimumValidPositionInRadians
    :initarg :tiltMinimumValidPositionInRadians
    :type cl:float
    :initform 0.0)
   (panMaximumValidPositionInRadians
    :reader panMaximumValidPositionInRadians
    :initarg :panMaximumValidPositionInRadians
    :type cl:float
    :initform 0.0)
   (tiltMaximumValidPositionInRadians
    :reader tiltMaximumValidPositionInRadians
    :initarg :tiltMaximumValidPositionInRadians
    :type cl:float
    :initform 0.0)
   (panLowerSpeedBoundInRadiansPerSec
    :reader panLowerSpeedBoundInRadiansPerSec
    :initarg :panLowerSpeedBoundInRadiansPerSec
    :type cl:float
    :initform 0.0)
   (tiltLowerSpeedBoundInRadiansPerSec
    :reader tiltLowerSpeedBoundInRadiansPerSec
    :initarg :tiltLowerSpeedBoundInRadiansPerSec
    :type cl:float
    :initform 0.0)
   (panUpperSpeedBoundInRadiansPerSec
    :reader panUpperSpeedBoundInRadiansPerSec
    :initarg :panUpperSpeedBoundInRadiansPerSec
    :type cl:float
    :initform 0.0)
   (tiltUpperSpeedBoundInRadiansPerSec
    :reader tiltUpperSpeedBoundInRadiansPerSec
    :initarg :tiltUpperSpeedBoundInRadiansPerSec
    :type cl:float
    :initform 0.0))
)

(cl:defclass ReportStaticInfo (<ReportStaticInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReportStaticInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReportStaticInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<ReportStaticInfo> is deprecated: use DPPanTilt_msgs-msg:ReportStaticInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ReportStaticInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:header-val is deprecated.  Use DPPanTilt_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'panMinimumValidPositionInRadians-val :lambda-list '(m))
(cl:defmethod panMinimumValidPositionInRadians-val ((m <ReportStaticInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panMinimumValidPositionInRadians-val is deprecated.  Use DPPanTilt_msgs-msg:panMinimumValidPositionInRadians instead.")
  (panMinimumValidPositionInRadians m))

(cl:ensure-generic-function 'tiltMinimumValidPositionInRadians-val :lambda-list '(m))
(cl:defmethod tiltMinimumValidPositionInRadians-val ((m <ReportStaticInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltMinimumValidPositionInRadians-val is deprecated.  Use DPPanTilt_msgs-msg:tiltMinimumValidPositionInRadians instead.")
  (tiltMinimumValidPositionInRadians m))

(cl:ensure-generic-function 'panMaximumValidPositionInRadians-val :lambda-list '(m))
(cl:defmethod panMaximumValidPositionInRadians-val ((m <ReportStaticInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panMaximumValidPositionInRadians-val is deprecated.  Use DPPanTilt_msgs-msg:panMaximumValidPositionInRadians instead.")
  (panMaximumValidPositionInRadians m))

(cl:ensure-generic-function 'tiltMaximumValidPositionInRadians-val :lambda-list '(m))
(cl:defmethod tiltMaximumValidPositionInRadians-val ((m <ReportStaticInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltMaximumValidPositionInRadians-val is deprecated.  Use DPPanTilt_msgs-msg:tiltMaximumValidPositionInRadians instead.")
  (tiltMaximumValidPositionInRadians m))

(cl:ensure-generic-function 'panLowerSpeedBoundInRadiansPerSec-val :lambda-list '(m))
(cl:defmethod panLowerSpeedBoundInRadiansPerSec-val ((m <ReportStaticInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panLowerSpeedBoundInRadiansPerSec-val is deprecated.  Use DPPanTilt_msgs-msg:panLowerSpeedBoundInRadiansPerSec instead.")
  (panLowerSpeedBoundInRadiansPerSec m))

(cl:ensure-generic-function 'tiltLowerSpeedBoundInRadiansPerSec-val :lambda-list '(m))
(cl:defmethod tiltLowerSpeedBoundInRadiansPerSec-val ((m <ReportStaticInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltLowerSpeedBoundInRadiansPerSec-val is deprecated.  Use DPPanTilt_msgs-msg:tiltLowerSpeedBoundInRadiansPerSec instead.")
  (tiltLowerSpeedBoundInRadiansPerSec m))

(cl:ensure-generic-function 'panUpperSpeedBoundInRadiansPerSec-val :lambda-list '(m))
(cl:defmethod panUpperSpeedBoundInRadiansPerSec-val ((m <ReportStaticInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panUpperSpeedBoundInRadiansPerSec-val is deprecated.  Use DPPanTilt_msgs-msg:panUpperSpeedBoundInRadiansPerSec instead.")
  (panUpperSpeedBoundInRadiansPerSec m))

(cl:ensure-generic-function 'tiltUpperSpeedBoundInRadiansPerSec-val :lambda-list '(m))
(cl:defmethod tiltUpperSpeedBoundInRadiansPerSec-val ((m <ReportStaticInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltUpperSpeedBoundInRadiansPerSec-val is deprecated.  Use DPPanTilt_msgs-msg:tiltUpperSpeedBoundInRadiansPerSec instead.")
  (tiltUpperSpeedBoundInRadiansPerSec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReportStaticInfo>) ostream)
  "Serializes a message object of type '<ReportStaticInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panMinimumValidPositionInRadians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltMinimumValidPositionInRadians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panMaximumValidPositionInRadians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltMaximumValidPositionInRadians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panLowerSpeedBoundInRadiansPerSec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltLowerSpeedBoundInRadiansPerSec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panUpperSpeedBoundInRadiansPerSec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltUpperSpeedBoundInRadiansPerSec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReportStaticInfo>) istream)
  "Deserializes a message object of type '<ReportStaticInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panMinimumValidPositionInRadians) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltMinimumValidPositionInRadians) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panMaximumValidPositionInRadians) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltMaximumValidPositionInRadians) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panLowerSpeedBoundInRadiansPerSec) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltLowerSpeedBoundInRadiansPerSec) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panUpperSpeedBoundInRadiansPerSec) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltUpperSpeedBoundInRadiansPerSec) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReportStaticInfo>)))
  "Returns string type for a message object of type '<ReportStaticInfo>"
  "DPPanTilt_msgs/ReportStaticInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReportStaticInfo)))
  "Returns string type for a message object of type 'ReportStaticInfo"
  "DPPanTilt_msgs/ReportStaticInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReportStaticInfo>)))
  "Returns md5sum for a message object of type '<ReportStaticInfo>"
  "949294eda1f9ef4201055f8d681ac295")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReportStaticInfo)))
  "Returns md5sum for a message object of type 'ReportStaticInfo"
  "949294eda1f9ef4201055f8d681ac295")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReportStaticInfo>)))
  "Returns full string definition for message of type '<ReportStaticInfo>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to report the static information about the~%# Pan/Tilt Unit.  It is not likely that much of this information will change~%# during the course of operation.~%~%Header header~%~%~%float32 panMinimumValidPositionInRadians~%float32 tiltMinimumValidPositionInRadians~%~%float32 panMaximumValidPositionInRadians~%float32 tiltMaximumValidPositionInRadians~%~%float32 panLowerSpeedBoundInRadiansPerSec~%float32 tiltLowerSpeedBoundInRadiansPerSec~%~%float32 panUpperSpeedBoundInRadiansPerSec~%float32 tiltUpperSpeedBoundInRadiansPerSec~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReportStaticInfo)))
  "Returns full string definition for message of type 'ReportStaticInfo"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to report the static information about the~%# Pan/Tilt Unit.  It is not likely that much of this information will change~%# during the course of operation.~%~%Header header~%~%~%float32 panMinimumValidPositionInRadians~%float32 tiltMinimumValidPositionInRadians~%~%float32 panMaximumValidPositionInRadians~%float32 tiltMaximumValidPositionInRadians~%~%float32 panLowerSpeedBoundInRadiansPerSec~%float32 tiltLowerSpeedBoundInRadiansPerSec~%~%float32 panUpperSpeedBoundInRadiansPerSec~%float32 tiltUpperSpeedBoundInRadiansPerSec~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReportStaticInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReportStaticInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'ReportStaticInfo
    (cl:cons ':header (header msg))
    (cl:cons ':panMinimumValidPositionInRadians (panMinimumValidPositionInRadians msg))
    (cl:cons ':tiltMinimumValidPositionInRadians (tiltMinimumValidPositionInRadians msg))
    (cl:cons ':panMaximumValidPositionInRadians (panMaximumValidPositionInRadians msg))
    (cl:cons ':tiltMaximumValidPositionInRadians (tiltMaximumValidPositionInRadians msg))
    (cl:cons ':panLowerSpeedBoundInRadiansPerSec (panLowerSpeedBoundInRadiansPerSec msg))
    (cl:cons ':tiltLowerSpeedBoundInRadiansPerSec (tiltLowerSpeedBoundInRadiansPerSec msg))
    (cl:cons ':panUpperSpeedBoundInRadiansPerSec (panUpperSpeedBoundInRadiansPerSec msg))
    (cl:cons ':tiltUpperSpeedBoundInRadiansPerSec (tiltUpperSpeedBoundInRadiansPerSec msg))
))
