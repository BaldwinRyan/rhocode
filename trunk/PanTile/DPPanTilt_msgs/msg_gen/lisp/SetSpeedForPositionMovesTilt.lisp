; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude SetSpeedForPositionMovesTilt.msg.html

(cl:defclass <SetSpeedForPositionMovesTilt> (roslisp-msg-protocol:ros-message)
  ((tiltSpeedInRadiansPerSec
    :reader tiltSpeedInRadiansPerSec
    :initarg :tiltSpeedInRadiansPerSec
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetSpeedForPositionMovesTilt (<SetSpeedForPositionMovesTilt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSpeedForPositionMovesTilt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSpeedForPositionMovesTilt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<SetSpeedForPositionMovesTilt> is deprecated: use DPPanTilt_msgs-msg:SetSpeedForPositionMovesTilt instead.")))

(cl:ensure-generic-function 'tiltSpeedInRadiansPerSec-val :lambda-list '(m))
(cl:defmethod tiltSpeedInRadiansPerSec-val ((m <SetSpeedForPositionMovesTilt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader DPPanTilt_msgs-msg:tiltSpeedInRadiansPerSec-val is deprecated.  Use DPPanTilt_msgs-msg:tiltSpeedInRadiansPerSec instead.")
  (tiltSpeedInRadiansPerSec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSpeedForPositionMovesTilt>) ostream)
  "Serializes a message object of type '<SetSpeedForPositionMovesTilt>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tiltSpeedInRadiansPerSec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSpeedForPositionMovesTilt>) istream)
  "Deserializes a message object of type '<SetSpeedForPositionMovesTilt>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tiltSpeedInRadiansPerSec) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSpeedForPositionMovesTilt>)))
  "Returns string type for a message object of type '<SetSpeedForPositionMovesTilt>"
  "DPPanTilt_msgs/SetSpeedForPositionMovesTilt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeedForPositionMovesTilt)))
  "Returns string type for a message object of type 'SetSpeedForPositionMovesTilt"
  "DPPanTilt_msgs/SetSpeedForPositionMovesTilt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSpeedForPositionMovesTilt>)))
  "Returns md5sum for a message object of type '<SetSpeedForPositionMovesTilt>"
  "747efaceb02af3c079c7615b4c868ad9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSpeedForPositionMovesTilt)))
  "Returns md5sum for a message object of type 'SetSpeedForPositionMovesTilt"
  "747efaceb02af3c079c7615b4c868ad9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSpeedForPositionMovesTilt>)))
  "Returns full string definition for message of type '<SetSpeedForPositionMovesTilt>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the maximum speed that a position move will~%# be able to obtain.~%~%float32 tiltSpeedInRadiansPerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSpeedForPositionMovesTilt)))
  "Returns full string definition for message of type 'SetSpeedForPositionMovesTilt"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to set the maximum speed that a position move will~%# be able to obtain.~%~%float32 tiltSpeedInRadiansPerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSpeedForPositionMovesTilt>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSpeedForPositionMovesTilt>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSpeedForPositionMovesTilt
    (cl:cons ':tiltSpeedInRadiansPerSec (tiltSpeedInRadiansPerSec msg))
))
