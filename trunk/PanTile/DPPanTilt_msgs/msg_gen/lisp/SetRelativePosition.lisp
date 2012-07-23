; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude SetRelativePosition.msg.html

(cl:defclass <SetRelativePosition> (roslisp-msg-protocol:ros-message)
  ((panCommandInRadians
    :reader panCommandInRadians
    :initarg :panCommandInRadians
    :type cl:float
    :initform 0.0)
   (tiltCommandInRadians
    :reader tiltCommandInRadians
    :initarg :tiltCommandInRadians
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetRelativePosition (<SetRelativePosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRelativePosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRelativePosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<SetRelativePosition> is deprecated: use DPPanTilt_msgs-msg:SetRelativePosition instead.")))

(cl:ensure-generic-function 'panCommandInRadians-val :lambda-list '(m))
(cl:defmethod panCommandInRadians-val ((m <SetRelativePosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panCommandInRadians-val is deprecated.  Use DPPanTilt_msgs-msg:panCommandInRadians instead.")
  (panCommandInRadians m))

(cl:ensure-generic-function 'tiltCommandInRadians-val :lambda-list '(m))
(cl:defmethod tiltCommandInRadians-val ((m <SetRelativePosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltCommandInRadians-val is deprecated.  Use DPPanTilt_msgs-msg:tiltCommandInRadians instead.")
  (tiltCommandInRadians m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRelativePosition>) ostream)
  "Serializes a message object of type '<SetRelativePosition>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panCommandInRadians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltCommandInRadians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRelativePosition>) istream)
  "Deserializes a message object of type '<SetRelativePosition>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panCommandInRadians) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltCommandInRadians) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRelativePosition>)))
  "Returns string type for a message object of type '<SetRelativePosition>"
  "DPPanTilt_msgs/SetRelativePosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRelativePosition)))
  "Returns string type for a message object of type 'SetRelativePosition"
  "DPPanTilt_msgs/SetRelativePosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRelativePosition>)))
  "Returns md5sum for a message object of type '<SetRelativePosition>"
  "41b9a6d5574ea2c52357f80f6ee38f0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRelativePosition)))
  "Returns md5sum for a message object of type 'SetRelativePosition"
  "41b9a6d5574ea2c52357f80f6ee38f0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRelativePosition>)))
  "Returns full string definition for message of type '<SetRelativePosition>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the relative position of both the Pan and~%# Tilt axes. This message is only valid if the unit is in Position control.~%~%float32 panCommandInRadians~%float32 tiltCommandInRadians~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRelativePosition)))
  "Returns full string definition for message of type 'SetRelativePosition"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the relative position of both the Pan and~%# Tilt axes. This message is only valid if the unit is in Position control.~%~%float32 panCommandInRadians~%float32 tiltCommandInRadians~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRelativePosition>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRelativePosition>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRelativePosition
    (cl:cons ':panCommandInRadians (panCommandInRadians msg))
    (cl:cons ':tiltCommandInRadians (tiltCommandInRadians msg))
))
