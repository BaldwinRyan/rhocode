; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude SetAbsolutePositionTilt.msg.html

(cl:defclass <SetAbsolutePositionTilt> (roslisp-msg-protocol:ros-message)
  ((tiltPositionInRadians
    :reader tiltPositionInRadians
    :initarg :tiltPositionInRadians
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetAbsolutePositionTilt (<SetAbsolutePositionTilt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAbsolutePositionTilt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAbsolutePositionTilt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<SetAbsolutePositionTilt> is deprecated: use DPPanTilt_msgs-msg:SetAbsolutePositionTilt instead.")))

(cl:ensure-generic-function 'tiltPositionInRadians-val :lambda-list '(m))
(cl:defmethod tiltPositionInRadians-val ((m <SetAbsolutePositionTilt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltPositionInRadians-val is deprecated.  Use DPPanTilt_msgs-msg:tiltPositionInRadians instead.")
  (tiltPositionInRadians m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAbsolutePositionTilt>) ostream)
  "Serializes a message object of type '<SetAbsolutePositionTilt>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltPositionInRadians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAbsolutePositionTilt>) istream)
  "Deserializes a message object of type '<SetAbsolutePositionTilt>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltPositionInRadians) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAbsolutePositionTilt>)))
  "Returns string type for a message object of type '<SetAbsolutePositionTilt>"
  "DPPanTilt_msgs/SetAbsolutePositionTilt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAbsolutePositionTilt)))
  "Returns string type for a message object of type 'SetAbsolutePositionTilt"
  "DPPanTilt_msgs/SetAbsolutePositionTilt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAbsolutePositionTilt>)))
  "Returns md5sum for a message object of type '<SetAbsolutePositionTilt>"
  "e188591d689122d81b1dc7b2f58035e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAbsolutePositionTilt)))
  "Returns md5sum for a message object of type 'SetAbsolutePositionTilt"
  "e188591d689122d81b1dc7b2f58035e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAbsolutePositionTilt>)))
  "Returns full string definition for message of type '<SetAbsolutePositionTilt>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the absolute position of the Tilt axis.~%# This message is only valid if the unit is in Position control.~%~%float32 tiltPositionInRadians~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAbsolutePositionTilt)))
  "Returns full string definition for message of type 'SetAbsolutePositionTilt"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the absolute position of the Tilt axis.~%# This message is only valid if the unit is in Position control.~%~%float32 tiltPositionInRadians~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAbsolutePositionTilt>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAbsolutePositionTilt>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAbsolutePositionTilt
    (cl:cons ':tiltPositionInRadians (tiltPositionInRadians msg))
))
