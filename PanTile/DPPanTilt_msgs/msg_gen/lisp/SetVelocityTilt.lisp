; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude SetVelocityTilt.msg.html

(cl:defclass <SetVelocityTilt> (roslisp-msg-protocol:ros-message)
  ((tiltVelocityInRadiansPerSec
    :reader tiltVelocityInRadiansPerSec
    :initarg :tiltVelocityInRadiansPerSec
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetVelocityTilt (<SetVelocityTilt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVelocityTilt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVelocityTilt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<SetVelocityTilt> is deprecated: use DPPanTilt_msgs-msg:SetVelocityTilt instead.")))

(cl:ensure-generic-function 'tiltVelocityInRadiansPerSec-val :lambda-list '(m))
(cl:defmethod tiltVelocityInRadiansPerSec-val ((m <SetVelocityTilt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltVelocityInRadiansPerSec-val is deprecated.  Use DPPanTilt_msgs-msg:tiltVelocityInRadiansPerSec instead.")
  (tiltVelocityInRadiansPerSec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVelocityTilt>) ostream)
  "Serializes a message object of type '<SetVelocityTilt>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltVelocityInRadiansPerSec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVelocityTilt>) istream)
  "Deserializes a message object of type '<SetVelocityTilt>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltVelocityInRadiansPerSec) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVelocityTilt>)))
  "Returns string type for a message object of type '<SetVelocityTilt>"
  "DPPanTilt_msgs/SetVelocityTilt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelocityTilt)))
  "Returns string type for a message object of type 'SetVelocityTilt"
  "DPPanTilt_msgs/SetVelocityTilt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVelocityTilt>)))
  "Returns md5sum for a message object of type '<SetVelocityTilt>"
  "a74c80254409a57dbafc7f3c0af016f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVelocityTilt)))
  "Returns md5sum for a message object of type 'SetVelocityTilt"
  "a74c80254409a57dbafc7f3c0af016f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVelocityTilt>)))
  "Returns full string definition for message of type '<SetVelocityTilt>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the velocity of the Tilt axis.~%# This message is only valid if the unit is in Velocity control.~%~%float32 tiltVelocityInRadiansPerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVelocityTilt)))
  "Returns full string definition for message of type 'SetVelocityTilt"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the velocity of the Tilt axis.~%# This message is only valid if the unit is in Velocity control.~%~%float32 tiltVelocityInRadiansPerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVelocityTilt>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVelocityTilt>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVelocityTilt
    (cl:cons ':tiltVelocityInRadiansPerSec (tiltVelocityInRadiansPerSec msg))
))
