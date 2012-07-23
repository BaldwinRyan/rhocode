; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude SetVelocityPan.msg.html

(cl:defclass <SetVelocityPan> (roslisp-msg-protocol:ros-message)
  ((panVelocityInRadiansPerSec
    :reader panVelocityInRadiansPerSec
    :initarg :panVelocityInRadiansPerSec
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetVelocityPan (<SetVelocityPan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVelocityPan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVelocityPan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<SetVelocityPan> is deprecated: use DPPanTilt_msgs-msg:SetVelocityPan instead.")))

(cl:ensure-generic-function 'panVelocityInRadiansPerSec-val :lambda-list '(m))
(cl:defmethod panVelocityInRadiansPerSec-val ((m <SetVelocityPan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panVelocityInRadiansPerSec-val is deprecated.  Use DPPanTilt_msgs-msg:panVelocityInRadiansPerSec instead.")
  (panVelocityInRadiansPerSec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVelocityPan>) ostream)
  "Serializes a message object of type '<SetVelocityPan>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panVelocityInRadiansPerSec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVelocityPan>) istream)
  "Deserializes a message object of type '<SetVelocityPan>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panVelocityInRadiansPerSec) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVelocityPan>)))
  "Returns string type for a message object of type '<SetVelocityPan>"
  "DPPanTilt_msgs/SetVelocityPan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelocityPan)))
  "Returns string type for a message object of type 'SetVelocityPan"
  "DPPanTilt_msgs/SetVelocityPan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVelocityPan>)))
  "Returns md5sum for a message object of type '<SetVelocityPan>"
  "e892766b6e81cfc1a2703711ab4dcbd7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVelocityPan)))
  "Returns md5sum for a message object of type 'SetVelocityPan"
  "e892766b6e81cfc1a2703711ab4dcbd7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVelocityPan>)))
  "Returns full string definition for message of type '<SetVelocityPan>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the velocity of the Pan axis.~%# This message is only valid if the unit is in Velocity control.~%~%float32 panVelocityInRadiansPerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVelocityPan)))
  "Returns full string definition for message of type 'SetVelocityPan"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the velocity of the Pan axis.~%# This message is only valid if the unit is in Velocity control.~%~%float32 panVelocityInRadiansPerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVelocityPan>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVelocityPan>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVelocityPan
    (cl:cons ':panVelocityInRadiansPerSec (panVelocityInRadiansPerSec msg))
))
