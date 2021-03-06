# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from pharos_vlp_tilt/vector_perfect_array.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import pharos_vlp_tilt.msg
import std_msgs.msg

class vector_perfect_array(genpy.Message):
  _md5sum = "57e47adbe1b2b913a3385ad439b88169"
  _type = "pharos_vlp_tilt/vector_perfect_array"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
perfectarray[] one

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: pharos_vlp_tilt/perfectarray
std_msgs/Header header
perfect[] objects
min_seq[16] min_seq
max_seq[16] max_seq
center_position center
center_position min_center
center_position max_center
point min_object
point max_object
int32 min_hori
int32 max_hori
================================================================================
MSG: pharos_vlp_tilt/perfect
info info
point point
state state

================================================================================
MSG: pharos_vlp_tilt/info
int32 laser
int32 hori

================================================================================
MSG: pharos_vlp_tilt/point
float64 x
float64 y
float64 z
float64 intensity

================================================================================
MSG: pharos_vlp_tilt/state
int32 is_ground
int32 is_del
int32 is_infect

================================================================================
MSG: pharos_vlp_tilt/min_seq
int16 i
int16 hori

================================================================================
MSG: pharos_vlp_tilt/max_seq
int16 i
int16 hori

================================================================================
MSG: pharos_vlp_tilt/center_position
float64 x
float64 y
float64 z
"""
  __slots__ = ['header','one']
  _slot_types = ['std_msgs/Header','pharos_vlp_tilt/perfectarray[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,one

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(vector_perfect_array, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.one is None:
        self.one = []
    else:
      self.header = std_msgs.msg.Header()
      self.one = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.one)
      buff.write(_struct_I.pack(length))
      for val1 in self.one:
        _v1 = val1.header
        buff.write(_get_struct_I().pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        length = len(val1.objects)
        buff.write(_struct_I.pack(length))
        for val2 in val1.objects:
          _v3 = val2.info
          _x = _v3
          buff.write(_get_struct_2i().pack(_x.laser, _x.hori))
          _v4 = val2.point
          _x = _v4
          buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.intensity))
          _v5 = val2.state
          _x = _v5
          buff.write(_get_struct_3i().pack(_x.is_ground, _x.is_del, _x.is_infect))
        for val2 in val1.min_seq:
          _x = val2
          buff.write(_get_struct_2h().pack(_x.i, _x.hori))
        for val2 in val1.max_seq:
          _x = val2
          buff.write(_get_struct_2h().pack(_x.i, _x.hori))
        _v6 = val1.center
        _x = _v6
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v7 = val1.min_center
        _x = _v7
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v8 = val1.max_center
        _x = _v8
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v9 = val1.min_object
        _x = _v9
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.intensity))
        _v10 = val1.max_object
        _x = _v10
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.intensity))
        _x = val1
        buff.write(_get_struct_2i().pack(_x.min_hori, _x.max_hori))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.one is None:
        self.one = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.one = []
      for i in range(0, length):
        val1 = pharos_vlp_tilt.msg.perfectarray()
        _v11 = val1.header
        start = end
        end += 4
        (_v11.seq,) = _get_struct_I().unpack(str[start:end])
        _v12 = _v11.stamp
        _x = _v12
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v11.frame_id = str[start:end].decode('utf-8')
        else:
          _v11.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.objects = []
        for i in range(0, length):
          val2 = pharos_vlp_tilt.msg.perfect()
          _v13 = val2.info
          _x = _v13
          start = end
          end += 8
          (_x.laser, _x.hori,) = _get_struct_2i().unpack(str[start:end])
          _v14 = val2.point
          _x = _v14
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.intensity,) = _get_struct_4d().unpack(str[start:end])
          _v15 = val2.state
          _x = _v15
          start = end
          end += 12
          (_x.is_ground, _x.is_del, _x.is_infect,) = _get_struct_3i().unpack(str[start:end])
          val1.objects.append(val2)
        val1.min_seq = []
        for i in range(0, 16):
          val2 = pharos_vlp_tilt.msg.min_seq()
          _x = val2
          start = end
          end += 4
          (_x.i, _x.hori,) = _get_struct_2h().unpack(str[start:end])
          val1.min_seq.append(val2)
        val1.max_seq = []
        for i in range(0, 16):
          val2 = pharos_vlp_tilt.msg.max_seq()
          _x = val2
          start = end
          end += 4
          (_x.i, _x.hori,) = _get_struct_2h().unpack(str[start:end])
          val1.max_seq.append(val2)
        _v16 = val1.center
        _x = _v16
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v17 = val1.min_center
        _x = _v17
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v18 = val1.max_center
        _x = _v18
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v19 = val1.min_object
        _x = _v19
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.intensity,) = _get_struct_4d().unpack(str[start:end])
        _v20 = val1.max_object
        _x = _v20
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.intensity,) = _get_struct_4d().unpack(str[start:end])
        _x = val1
        start = end
        end += 8
        (_x.min_hori, _x.max_hori,) = _get_struct_2i().unpack(str[start:end])
        self.one.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.one)
      buff.write(_struct_I.pack(length))
      for val1 in self.one:
        _v21 = val1.header
        buff.write(_get_struct_I().pack(_v21.seq))
        _v22 = _v21.stamp
        _x = _v22
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v21.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        length = len(val1.objects)
        buff.write(_struct_I.pack(length))
        for val2 in val1.objects:
          _v23 = val2.info
          _x = _v23
          buff.write(_get_struct_2i().pack(_x.laser, _x.hori))
          _v24 = val2.point
          _x = _v24
          buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.intensity))
          _v25 = val2.state
          _x = _v25
          buff.write(_get_struct_3i().pack(_x.is_ground, _x.is_del, _x.is_infect))
        for val2 in val1.min_seq:
          _x = val2
          buff.write(_get_struct_2h().pack(_x.i, _x.hori))
        for val2 in val1.max_seq:
          _x = val2
          buff.write(_get_struct_2h().pack(_x.i, _x.hori))
        _v26 = val1.center
        _x = _v26
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v27 = val1.min_center
        _x = _v27
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v28 = val1.max_center
        _x = _v28
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v29 = val1.min_object
        _x = _v29
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.intensity))
        _v30 = val1.max_object
        _x = _v30
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.intensity))
        _x = val1
        buff.write(_get_struct_2i().pack(_x.min_hori, _x.max_hori))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.one is None:
        self.one = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.one = []
      for i in range(0, length):
        val1 = pharos_vlp_tilt.msg.perfectarray()
        _v31 = val1.header
        start = end
        end += 4
        (_v31.seq,) = _get_struct_I().unpack(str[start:end])
        _v32 = _v31.stamp
        _x = _v32
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v31.frame_id = str[start:end].decode('utf-8')
        else:
          _v31.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.objects = []
        for i in range(0, length):
          val2 = pharos_vlp_tilt.msg.perfect()
          _v33 = val2.info
          _x = _v33
          start = end
          end += 8
          (_x.laser, _x.hori,) = _get_struct_2i().unpack(str[start:end])
          _v34 = val2.point
          _x = _v34
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.intensity,) = _get_struct_4d().unpack(str[start:end])
          _v35 = val2.state
          _x = _v35
          start = end
          end += 12
          (_x.is_ground, _x.is_del, _x.is_infect,) = _get_struct_3i().unpack(str[start:end])
          val1.objects.append(val2)
        val1.min_seq = []
        for i in range(0, 16):
          val2 = pharos_vlp_tilt.msg.min_seq()
          _x = val2
          start = end
          end += 4
          (_x.i, _x.hori,) = _get_struct_2h().unpack(str[start:end])
          val1.min_seq.append(val2)
        val1.max_seq = []
        for i in range(0, 16):
          val2 = pharos_vlp_tilt.msg.max_seq()
          _x = val2
          start = end
          end += 4
          (_x.i, _x.hori,) = _get_struct_2h().unpack(str[start:end])
          val1.max_seq.append(val2)
        _v36 = val1.center
        _x = _v36
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v37 = val1.min_center
        _x = _v37
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v38 = val1.max_center
        _x = _v38
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v39 = val1.min_object
        _x = _v39
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.intensity,) = _get_struct_4d().unpack(str[start:end])
        _v40 = val1.max_object
        _x = _v40
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.intensity,) = _get_struct_4d().unpack(str[start:end])
        _x = val1
        start = end
        end += 8
        (_x.min_hori, _x.max_hori,) = _get_struct_2i().unpack(str[start:end])
        self.one.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3i = None
def _get_struct_3i():
    global _struct_3i
    if _struct_3i is None:
        _struct_3i = struct.Struct("<3i")
    return _struct_3i
_struct_2i = None
def _get_struct_2i():
    global _struct_2i
    if _struct_2i is None:
        _struct_2i = struct.Struct("<2i")
    return _struct_2i
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_2h = None
def _get_struct_2h():
    global _struct_2h
    if _struct_2h is None:
        _struct_2h = struct.Struct("<2h")
    return _struct_2h
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
