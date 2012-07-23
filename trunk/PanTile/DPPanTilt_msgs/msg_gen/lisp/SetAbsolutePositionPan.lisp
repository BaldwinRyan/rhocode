; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude SetAbsolutePositionPan.msg.html

(cl:defclass <SetAbsolutePositionPan> (roslisp-msg-protocol:ros-message)
  ((panPositionInRadians
    :reader panPositionInRadians
    :initarg :panPositionInRadians
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetAbsolutePositionPan (<SetAbsolutePositionPan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAbsolutePositionPan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAbsolutePositionPan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<SetAbsolutePositionPan> is deprecated: use DPPanTilt_msgs-msg:SetAbsolutePositionPan instead.")))

(cl:ensure-generic-function 'panPositionInRadians-val :lambda-list '(m))
(cl:defmethod panPositionInRadians-val ((m <SetAbsolutePositionPan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:panPositionInRadians-val is deprecated.  Use DPPanTilt_msgs-msg:panPositionInRadians instead.")
  (panPositionInRadians m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAbsolutePositionPan>) ostream)
  "Serializes a message object of type '<SetAbsolutePositionPan>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'panPositionInRadians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAbsolutePositionPan>) istream)
  "Deserializes a message object of type '<SetAbsolutePositionPan>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'panPositionInRadians) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAbsolutePositionPan>)))
  "Returns string type for a message object of type '<SetAbsolutePositionPan>"
  "DPPanTilt_msgs/SetAbsolutePositionPan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAbsolutePositionPan)))
  "Returns string type for a message object of type 'SetAbsolutePositionPan"
  "DPPanTilt_msgs/SetAbsolutePositionPan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAbsolutePositionPan>)))
  "Returns md5sum for a message object of type '<SetAbsolutePositionPan>"
  "a40e742fd49bd81d2afb807c5ae11850")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAbsolutePositionPan)))
  "Returns md5sum for a message object of type 'SetAbsolutePositionPan"
  "a40e742fd49bd81d2afb807c5ae11850")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAbsolutePositionPan>)))
  "Returns full string definition for message of type '<SetAbsolutePositionPan>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the absolute position of the Pan axis.~%# This message is only valid if the unit is in Position control.~%~%float32 panPositionInRadians~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAbsolutePositionPan)))
  "Returns full string definition for message of type 'SetAbsolutePositionPan"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the absolute position of the Pan axis.~%# This message is only valid if the unit is in Position control.~%~%float32 panPositionInRadians~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAbsolutePositionPan>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAbsolutePositionPan>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAbsolutePositionPan
    (cl:cons ':panPositionInRadians (panPositionInRadians msg))
))
