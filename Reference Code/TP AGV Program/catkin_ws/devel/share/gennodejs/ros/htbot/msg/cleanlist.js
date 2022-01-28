// Auto-generated. Do not edit!

// (in-package htbot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class cleanlist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.info = null;
      this.mapno = null;
      this.cplanno = null;
      this.mapdir = null;
      this.mapname = null;
      this.cplandir = null;
      this.cleanplan = null;
    }
    else {
      if (initObj.hasOwnProperty('info')) {
        this.info = initObj.info
      }
      else {
        this.info = 0;
      }
      if (initObj.hasOwnProperty('mapno')) {
        this.mapno = initObj.mapno
      }
      else {
        this.mapno = 0;
      }
      if (initObj.hasOwnProperty('cplanno')) {
        this.cplanno = initObj.cplanno
      }
      else {
        this.cplanno = 0;
      }
      if (initObj.hasOwnProperty('mapdir')) {
        this.mapdir = initObj.mapdir
      }
      else {
        this.mapdir = '';
      }
      if (initObj.hasOwnProperty('mapname')) {
        this.mapname = initObj.mapname
      }
      else {
        this.mapname = [];
      }
      if (initObj.hasOwnProperty('cplandir')) {
        this.cplandir = initObj.cplandir
      }
      else {
        this.cplandir = '';
      }
      if (initObj.hasOwnProperty('cleanplan')) {
        this.cleanplan = initObj.cleanplan
      }
      else {
        this.cleanplan = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cleanlist
    // Serialize message field [info]
    bufferOffset = _serializer.int8(obj.info, buffer, bufferOffset);
    // Serialize message field [mapno]
    bufferOffset = _serializer.int32(obj.mapno, buffer, bufferOffset);
    // Serialize message field [cplanno]
    bufferOffset = _serializer.int32(obj.cplanno, buffer, bufferOffset);
    // Serialize message field [mapdir]
    bufferOffset = _serializer.string(obj.mapdir, buffer, bufferOffset);
    // Serialize message field [mapname]
    bufferOffset = _arraySerializer.string(obj.mapname, buffer, bufferOffset, null);
    // Serialize message field [cplandir]
    bufferOffset = _serializer.string(obj.cplandir, buffer, bufferOffset);
    // Serialize message field [cleanplan]
    bufferOffset = _arraySerializer.string(obj.cleanplan, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cleanlist
    let len;
    let data = new cleanlist(null);
    // Deserialize message field [info]
    data.info = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [mapno]
    data.mapno = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [cplanno]
    data.cplanno = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mapdir]
    data.mapdir = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mapname]
    data.mapname = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [cplandir]
    data.cplandir = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cleanplan]
    data.cleanplan = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.mapdir.length;
    object.mapname.forEach((val) => {
      length += 4 + val.length;
    });
    length += object.cplandir.length;
    object.cleanplan.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/cleanlist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '975117791827cbfacd76c6c1c424923b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 info
    int32 mapno
    int32 cplanno
    string mapdir
    string[] mapname
    string cplandir
    string[] cleanplan
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cleanlist(null);
    if (msg.info !== undefined) {
      resolved.info = msg.info;
    }
    else {
      resolved.info = 0
    }

    if (msg.mapno !== undefined) {
      resolved.mapno = msg.mapno;
    }
    else {
      resolved.mapno = 0
    }

    if (msg.cplanno !== undefined) {
      resolved.cplanno = msg.cplanno;
    }
    else {
      resolved.cplanno = 0
    }

    if (msg.mapdir !== undefined) {
      resolved.mapdir = msg.mapdir;
    }
    else {
      resolved.mapdir = ''
    }

    if (msg.mapname !== undefined) {
      resolved.mapname = msg.mapname;
    }
    else {
      resolved.mapname = []
    }

    if (msg.cplandir !== undefined) {
      resolved.cplandir = msg.cplandir;
    }
    else {
      resolved.cplandir = ''
    }

    if (msg.cleanplan !== undefined) {
      resolved.cleanplan = msg.cleanplan;
    }
    else {
      resolved.cleanplan = []
    }

    return resolved;
    }
};

module.exports = cleanlist;
