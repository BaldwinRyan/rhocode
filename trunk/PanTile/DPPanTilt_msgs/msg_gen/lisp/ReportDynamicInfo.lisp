; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude ReportDynamicInfo.msg.html

(cl:defclass <ReportDynamicInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (currentlyInPositionControl
    :reader currentlyInPositionControl
    :initarg :currentlyInPositionControl
    :type cl:boolean
    :initform cl:nil)
   (panPositionInRadians
    :reader panPositionInRadians
    :initarg :panPositionInRadians
    :type cl:float
    :initform 0.0)
   (tiltPositionInRadians
    :reader tiltPositionInRadians
    :initarg :tiltPositionInRadians
    :type cl:float
    :initform 0.0)
   (panVelocityInRadiansPerSec
    :reader panVelocityInRadiansPerSec
    :initarg :panVelocityInRadiansPerSec
    :type cl:float
    :initform 0.0)
   (tiltVelocityInRadiansPerSec
    :reader tiltVelocityInRadiansPerSec
    :initarg :tiltVelocityInRadiansPerSec
    :type cl:float
    :initform 0.0))
)

(cl:defclass ReportDynamicInfo (<ReportDynamicInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReportDynamicInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReportDynamicInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<ReportDynamicInfo> is deprecated: use DPPanTilt_msgs-msg:ReportDynamicInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ReportDynamicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:header-val is deprecated.  Use DPPanTilt_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'currentlyInPositionControl-val :lambda-list '(m))
(cl:defmethod currentlyInPositionControl-val ((m <ReportDynamicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:currentlyInPositionControl-val is deprecated.  Use DPPanTilt_msgs-msg:currentlyInPositionControl instead.")
  (currentlyInPositionControl m))

(cl:ensure-generic-function 'panPositionInRadians-val :lambda-list '(m))
(cl:defmethod panPositionInRadians-val ((m <ReportDynamicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panPositionInRadians-val is deprecated.  Use DPPanTilt_msgs-msg:panPositionInRadians instead.")
  (panPositionInRadians m))

(cl:ensure-generic-function 'tiltPositionInRadians-val :lambda-list '(m))
(cl:defmethod tiltPositionInRadians-val ((m <ReportDynamicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltPositionInRadians-val is deprecated.  Use DPPanTilt_msgs-msg:tiltPositionInRadians instead.")
  (tiltPositionInRadians m))

(cl:ensure-generic-function 'panVelocityInRadiansPerSec-val :lambda-list '(m))
(cl:defmethod panVelocityInRadiansPerSec-val ((m <ReportDynamicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panVelocityInRadiansPerSec-val is deprecated.  Use DPPanTilt_msgs-msg:panVelocityInRadiansPerSec instead.")
  (panVelocityInRadiansPerSec m))

(cl:ensure-generic-function 'tiltVelocityInRadiansPerSec-val :lambda-list '(m))
(cl:defmethod tiltVelocityInRadiansPerSec-val ((m <ReportDynamicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltVelocityInRadiansPerSec-val is deprecated.  Use DPPanTilt_msgs-msg:tiltVelocityInRadiansPerSec instead.")
  (tiltVelocityInRadiansPerSec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReportDynamicInfo>) ostream)
  "Serializes a message object of type '<ReportDynamicInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'currentlyInPositionControl) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panPositionInRadians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltPositionInRadians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panVelocityInRadiansPerSec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltVelocityInRadiansPerSec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReportDynamicInfo>) istream)
  "Deserializes a message object of type '<ReportDynamicInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'currentlyInPositionControl) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panPositionInRadians) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltPositionInRadians) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panVelocityInRadiansPerSec) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltVelocityInRadiansPerSec) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReportDynamicInfo>)))
  "Returns string type for a message object of type '<ReportDynamicInfo>"
  "DPPanTilt_msgs/ReportDynamicInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReportDynamicInfo)))
  "Returns string type for a message object of type 'ReportDynamicInfo"
  "DPPanTilt_msgs/ReportDynamicInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReportDynamicInfo>)))
  "Returns md5sum for a message object of type '<ReportDynamicInfo>"
  "2b3f54532ce42444329f3bb657130369")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReportDynamicInfo)))
  "Returns md5sum for a message object of type 'ReportDynamicInfo"
  "2b3f54532ce42444329f3bb657130369")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReportDynamicInfo>)))
  "Returns full string definition for message of type '<ReportDynamicInfo>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to report the information about the Pan/Tilt unit~%# that will be changing as the unit is used.~%~%Header header~%~%# This bool marks whether or not the unit is in Position Control.  If it~%# is not, this means the unit is in Velocity Control.~%bool currentlyInPositionControl~%~%# These are the last recorded positions of the axes~%float32 panPositionInRadians~%float32 tiltPositionInRadians~%~%# These are the last recorded velocities of the axes~%float32 panVelocityInRadiansPerSec~%float32 tiltVelocityInRadiansPerSec~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReportDynamicInfo)))
  "Returns full string definition for message of type 'ReportDynamicInfo"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to report the information about the Pan/Tilt unit~%# that will be changing as the unit is used.~%~%Header header~%~%# This bool marks whether or not the unit is in Position Control.  If it~%# is not, this means the unit is in Velocity Control.~%bool currentlyInPositionControl~%~%# These are the last recorded positions of the axes~%float32 panPositionInRadians~%float32 tiltPositionInRadians~%~%# These are the last recorded velocities of the axes~%float32 panVelocityInRadiansPerSec~%float32 tiltVelocityInRadiansPerSec~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReportDynamicInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReportDynamicInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'ReportDynamicInfo
    (cl:cons ':header (header msg))
    (cl:cons ':currentlyInPositionControl (currentlyInPositionControl msg))
    (cl:cons ':panPositionInRadians (panPositionInRadians msg))
    (cl:cons ':tiltPositionInRadians (tiltPositionInRadians msg))
    (cl:cons ':panVelocityInRadiansPerSec (panVelocityInRadiansPerSec msg))
    (cl:cons ':tiltVelocityInRadiansPerSec (tiltVelocityInRadiansPerSec msg))
))
