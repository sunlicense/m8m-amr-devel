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

class move_status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stat = null;
    }
    else {
      if (initObj.hasOwnProperty('stat')) {
        this.stat = initObj.stat
      }
      else {
        this.stat = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type move_status
    // Serialize message field [stat]
    bufferOffset = _serializer.int8(obj.stat, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type move_status
    let len;
    let data = new move_status(null);
    // Deserialize message field [stat]
    data.stat = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/move_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '85998e8afa5502f501182cfd6840bd64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 stat
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new move_status(null);
    if (msg.stat !== undefined) {
      resolved.stat = msg.stat;
    }
    else {
      resolved.stat = 0
    }

    return resolved;
    }
};

module.exports = move_status;
