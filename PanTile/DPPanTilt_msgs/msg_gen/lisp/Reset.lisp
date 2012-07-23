; Auto-generated. Do not edit!


(cl:in-package DPPanTilt_msgs-msg)


;//! \htmlinclude Reset.msg.html

(cl:defclass <Reset> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Reset (<Reset>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Reset>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Reset)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name DPPanTilt_msgs-msg:<Reset> is deprecated: use DPPanTilt_msgs-msg:Reset instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Reset>) ostream)
  "Serializes a message object of type '<Reset>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Reset>) istream)
  "Deserializes a message object of type '<Reset>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Reset>)))
  "Returns string type for a message object of type '<Reset>"
  "DPPanTilt_msgs/Reset")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Reset)))
  "Returns string type for a message object of type 'Reset"
  "DPPanTilt_msgs/Reset")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Reset>)))
  "Returns md5sum for a message object of type '<Reset>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Reset)))
  "Returns md5sum for a message object of type 'Reset"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Reset>)))
  "Returns full string definition for message of type '<Reset>"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to reset the Pan/Tilt unit.  This will send~%# the unit through its homing sequence.  Please note that the unit will~%# not provide any feedback while it is going through the homing sequence.~%~%# This message intentionally has no body.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Reset)))
  "Returns full string definition for message of type 'Reset"
  (cl:format cl:nil "#~%# COPYRIGHT (C) 2005-2010~%# RE2, INC.~%# ALL RIGHTS RESERVED~%#~%# This is free software:~%# you can redistribute it and/or modify it under the terms of the GNU General~%# Public License as published by the Free Software Foundation, either version~%# 3 of the License, or (at your option) any later version.~%#~%# You should have received a copy of the GNU General Public License along~%# with this package.  If not, see <http://www.gnu.org/licenses/>.~%#~%# RE2, INC. disclaims all warranties with regard to this software, including~%# all implied warranties of merchantability and fitness, in no event shall~%# RE2, INC. be liable for any special, indirect or consequential damages or~%# any damages whatsoever resulting from loss of use, data or profits, whether~%# in an action of contract, negligence or other tortious action, arising out~%# of or in connection with the use or performance of this software.~%#~%#~%#~%~%# This message is used to reset the Pan/Tilt unit.  This will send~%# the unit through its homing sequence.  Please note that the unit will~%# not provide any feedback while it is going through the homing sequence.~%~%# This message intentionally has no body.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Reset>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Reset>))
  "Converts a ROS message object to a list"
  (cl:list 'Reset
))
