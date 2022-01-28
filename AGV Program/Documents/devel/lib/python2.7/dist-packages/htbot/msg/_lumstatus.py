# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from htbot/lumstatus.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class lumstatus(genpy.Message):
  _md5sum = "f8b3c1afcfd978c9fc81e921d6f4109c"
  _type = "htbot/lumstatus"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 stn0
int8 stn1
int8 stn2
int8 stn3
int8 stn4
int8 stn5
int8 stn6
int8 stn7
int8 stn8
int8 stn9
float32 batlevel
string laserstatus
string errorlog
"""
  __slots__ = ['stn0','stn1','stn2','stn3','stn4','stn5','stn6','stn7','stn8','stn9','batlevel','laserstatus','errorlog']
  _slot_types = ['int8','int8','int8','int8','int8','int8','int8','int8','int8','int8','float32','string','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       stn0,stn1,stn2,stn3,stn4,stn5,stn6,stn7,stn8,stn9,batlevel,laserstatus,errorlog

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(lumstatus, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.stn0 is None:
        self.stn0 = 0
      if self.stn1 is None:
        self.stn1 = 0
      if self.stn2 is None:
        self.stn2 = 0
      if self.stn3 is None:
        self.stn3 = 0
      if self.stn4 is None:
        self.stn4 = 0
      if self.stn5 is None:
        self.stn5 = 0
      if self.stn6 is None:
        self.stn6 = 0
      if self.stn7 is None:
        self.stn7 = 0
      if self.stn8 is None:
        self.stn8 = 0
      if self.stn9 is None:
        self.stn9 = 0
      if self.batlevel is None:
        self.batlevel = 0.
      if self.laserstatus is None:
        self.laserstatus = ''
      if self.errorlog is None:
        self.errorlog = ''
    else:
      self.stn0 = 0
      self.stn1 = 0
      self.stn2 = 0
      self.stn3 = 0
      self.stn4 = 0
      self.stn5 = 0
      self.stn6 = 0
      self.stn7 = 0
      self.stn8 = 0
      self.stn9 = 0
      self.batlevel = 0.
      self.laserstatus = ''
      self.errorlog = ''

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
      buff.write(_get_struct_10bf().pack(_x.stn0, _x.stn1, _x.stn2, _x.stn3, _x.stn4, _x.stn5, _x.stn6, _x.stn7, _x.stn8, _x.stn9, _x.batlevel))
      _x = self.laserstatus
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.errorlog
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 14
      (_x.stn0, _x.stn1, _x.stn2, _x.stn3, _x.stn4, _x.stn5, _x.stn6, _x.stn7, _x.stn8, _x.stn9, _x.batlevel,) = _get_struct_10bf().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.laserstatus = str[start:end].decode('utf-8')
      else:
        self.laserstatus = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.errorlog = str[start:end].decode('utf-8')
      else:
        self.errorlog = str[start:end]
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
      buff.write(_get_struct_10bf().pack(_x.stn0, _x.stn1, _x.stn2, _x.stn3, _x.stn4, _x.stn5, _x.stn6, _x.stn7, _x.stn8, _x.stn9, _x.batlevel))
      _x = self.laserstatus
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.errorlog
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 14
      (_x.stn0, _x.stn1, _x.stn2, _x.stn3, _x.stn4, _x.stn5, _x.stn6, _x.stn7, _x.stn8, _x.stn9, _x.batlevel,) = _get_struct_10bf().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.laserstatus = str[start:end].decode('utf-8')
      else:
        self.laserstatus = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.errorlog = str[start:end].decode('utf-8')
      else:
        self.errorlog = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_10bf = None
def _get_struct_10bf():
    global _struct_10bf
    if _struct_10bf is None:
        _struct_10bf = struct.Struct("<10bf")
    return _struct_10bf