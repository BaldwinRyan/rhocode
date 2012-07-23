; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude SetSpeedForPositionMovesPan.msg.html

(cl:defclass <SetSpeedForPositionMovesPan> (roslisp-msg-protocol:ros-message)
  ((panSpeedInRadiansPerSec
    :reader panSpeedInRadiansPerSec
    :initarg :panSpeedInRadiansPerSec
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetSpeedForPositionMovesPan (<SetSpeedForPositionMovesPan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSpeedForPositionMovesPan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSpeedForPositionMovesPan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<SetSpeedForPositionMovesPan> is deprecated: use DPPanTilt_msgs-msg:SetSpeedForPositionMovesPan instead.")))

(cl:ensure-generic-function 'panSpeedInRadiansPerSec-val :lambda-list '(m))
(cl:defmethod panSpeedInRadiansPerSec-val ((m <SetSpeedForPositionMovesPan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panSpeedInRadiansPerSec-val is deprecated.  Use DPPanTilt_msgs-msg:panSpeedInRadiansPerSec instead.")
  (panSpeedInRadiansPerSec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSpeedForPositionMovesPan>) ostream)
  "Serializes a message object of type '<SetSpeedForPositionMovesPan>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panSpeedInRadiansPerSec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSpeedForPositionMovesPan>) istream)
  "Deserializes a message object of type '<SetSpeedForPositionMovesPan>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panSpeedInRadiansPerSec) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSpeedForPositionMovesPan>)))
  "Returns string type for a message object of type '<SetSpeedForPositionMovesPan>"
  "DPPanTilt_msgs/SetSpeedForPositionMovesPan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeedForPositionMovesPan)))
  "Returns string type for a message object of type 'SetSpeedForPositionMovesPan"
  "DPPanTilt_msgs/SetSpeedForPositionMovesPan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSpeedForPositionMovesPan>)))
  "Returns md5sum for a message object of type '<SetSpeedForPositionMovesPan>"
  "fd70471fc974badb4d31c1118a4742cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSpeedForPositionMovesPan)))
  "Returns md5sum for a message object of type 'SetSpeedForPositionMovesPan"
  "fd70471fc974badb4d31c1118a4742cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSpeedForPositionMovesPan>)))
  "Returns full string definition for message of type '<SetSpeedForPositionMovesPan>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the maximum speed that a position move will~%# be able to obtain.~%~%float32 panSpeedInRadiansPerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSpeedForPositionMovesPan)))
  "Returns full string definition for message of type 'SetSpeedForPositionMovesPan"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the maximum speed that a position move will~%# be able to obtain.~%~%float32 panSpeedInRadiansPerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSpeedForPositionMovesPan>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSpeedForPositionMovesPan>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSpeedForPositionMovesPan
    (cl:cons ':panSpeedInRadiansPerSec (panSpeedInRadiansPerSec msg))
))
