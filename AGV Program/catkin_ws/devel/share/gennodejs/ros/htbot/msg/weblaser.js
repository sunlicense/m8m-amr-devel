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

class weblaser {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.info = null;
      this.size = null;
      this.px = null;
      this.py = null;
    }
    else {
      if (initObj.hasOwnProperty('info')) {
        this.info = initObj.info
      }
      else {
        this.info = 0;
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = 0;
      }
      if (initObj.hasOwnProperty('px')) {
        this.px = initObj.px
      }
      else {
        this.px = [];
      }
      if (initObj.hasOwnProperty('py')) {
        this.py = initObj.py
      }
      else {
        this.py = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type weblaser
    // Serialize message field [info]
    bufferOffset = _serializer.int8(obj.info, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.int32(obj.size, buffer, bufferOffset);
    // Serialize message field [px]
    bufferOffset = _arraySerializer.float32(obj.px, buffer, bufferOffset, null);
    // Serialize message field [py]
    bufferOffset = _arraySerializer.float32(obj.py, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type weblaser
    let len;
    let data = new weblaser(null);
    // Deserialize message field [info]
    data.info = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [px]
    data.px = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [py]
    data.py = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.px.length;
    length += 4 * object.py.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/weblaser';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f7e0272dce619f1a82242dcdb545a8bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 info
    int32 size
    float32[] px
    float32[] py
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new weblaser(null);
    if (msg.info !== undefined) {
      resolved.info = msg.info;
    }
    else {
      resolved.info = 0
    }

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = 0
    }

    if (msg.px !== undefined) {
      resolved.px = msg.px;
    }
    else {
      resolved.px = []
    }

    if (msg.py !== undefined) {
      resolved.py = msg.py;
    }
    else {
      resolved.py = []
    }

    return resolved;
    }
};

module.exports = weblaser;
