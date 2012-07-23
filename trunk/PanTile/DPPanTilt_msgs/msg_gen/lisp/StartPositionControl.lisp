; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude StartPositionControl.msg.html

(cl:defclass <StartPositionControl> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StartPositionControl (<StartPositionControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartPositionControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartPositionControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<StartPositionControl> is deprecated: use DPPanTilt_msgs-msg:StartPositionControl instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartPositionControl>) ostream)
  "Serializes a message object of type '<StartPositionControl>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartPositionControl>) istream)
  "Deserializes a message object of type '<StartPositionControl>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartPositionControl>)))
  "Returns string type for a message object of type '<StartPositionControl>"
  "DPPanTilt_msgs/StartPositionControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartPositionControl)))
  "Returns string type for a message object of type 'StartPositionControl"
  "DPPanTilt_msgs/StartPositionControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartPositionControl>)))
  "Returns md5sum for a message object of type '<StartPositionControl>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartPositionControl)))
  "Returns md5sum for a message object of type 'StartPositionControl"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartPositionControl>)))
  "Returns full string definition for message of type '<StartPositionControl>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to put the unit into Position Control.~%# This means that position commands will be valid, and velocity commands~%# will be ignored.~%~%# The body of this message is intentionally blank.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartPositionControl)))
  "Returns full string definition for message of type 'StartPositionControl"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to put the unit into Position Control.~%# This means that position commands will be valid, and velocity commands~%# will be ignored.~%~%# The body of this message is intentionally blank.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartPositionControl>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartPositionControl>))
  "Converts a ROS message object to a list"
  (cl:list 'StartPositionControl
))
