"""autogenerated by genmsg_py from SetRelativePosition.msg. Do not edit."""
import roslib.message
import struct


class SetRelativePosition(roslib.message.Message):
  _md5sum = "41b9a6d5574ea2c52357f80f6ee38f0e"
  _type = "DPPanTilt_msgs/SetRelativePosition"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#
# COPYRIGHT (C) 2005-2010
# RE2, INC.
# ALL RIGHTS RESERVED
#
# This is free software:
# you can redistribute it and/or modify it under the terms of the GNU General
# Public License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# You should have received a copy of the GNU General Public License along
# with this package.  If not, see <http://www.gnu.org/licenses/>.
#
# RE2, INC. disclaims all warranties with regard to this software, including
# all implied warranties of merchantability and fitness, in no event shall
# RE2, INC. be liable for any special, indirect or consequential damages or
# any damages whatsoever resulting from loss of use, data or profits, whether
# in an action of contract, negligence or other tortious action, arising out
# of or in connection with the use or performance of this software.
#
#
#

# This message is used to set the relative position of both the Pan and
# Tilt axes. This message is only valid if the unit is in Position control.

float32 panCommandInRadians
float32 tiltCommandInRadians

"""
  __slots__ = ['panCommandInRadians','tiltCommandInRadians']
  _slot_types = ['float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       panCommandInRadians,tiltCommandInRadians
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(SetRelativePosition, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.panCommandInRadians is None:
        self.panCommandInRadians = 0.
      if self.tiltCommandInRadians is None:
        self.tiltCommandInRadians = 0.
    else:
      self.panCommandInRadians = 0.
      self.tiltCommandInRadians = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_2f.pack(_x.panCommandInRadians, _x.tiltCommandInRadians))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.panCommandInRadians, _x.tiltCommandInRadians,) = _struct_2f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_2f.pack(_x.panCommandInRadians, _x.tiltCommandInRadians))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.panCommandInRadians, _x.tiltCommandInRadians,) = _struct_2f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2f = struct.Struct("<2f")
