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

class ultraSS {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.uFR = null;
      this.uFL = null;
      this.uRF = null;
      this.uRR = null;
      this.uLF = null;
      this.uLR = null;
    }
    else {
      if (initObj.hasOwnProperty('uFR')) {
        this.uFR = initObj.uFR
      }
      else {
        this.uFR = 0.0;
      }
      if (initObj.hasOwnProperty('uFL')) {
        this.uFL = initObj.uFL
      }
      else {
        this.uFL = 0.0;
      }
      if (initObj.hasOwnProperty('uRF')) {
        this.uRF = initObj.uRF
      }
      else {
        this.uRF = 0.0;
      }
      if (initObj.hasOwnProperty('uRR')) {
        this.uRR = initObj.uRR
      }
      else {
        this.uRR = 0.0;
      }
      if (initObj.hasOwnProperty('uLF')) {
        this.uLF = initObj.uLF
      }
      else {
        this.uLF = 0.0;
      }
      if (initObj.hasOwnProperty('uLR')) {
        this.uLR = initObj.uLR
      }
      else {
        this.uLR = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ultraSS
    // Serialize message field [uFR]
    bufferOffset = _serializer.float32(obj.uFR, buffer, bufferOffset);
    // Serialize message field [uFL]
    bufferOffset = _serializer.float32(obj.uFL, buffer, bufferOffset);
    // Serialize message field [uRF]
    bufferOffset = _serializer.float32(obj.uRF, buffer, bufferOffset);
    // Serialize message field [uRR]
    bufferOffset = _serializer.float32(obj.uRR, buffer, bufferOffset);
    // Serialize message field [uLF]
    bufferOffset = _serializer.float32(obj.uLF, buffer, bufferOffset);
    // Serialize message field [uLR]
    bufferOffset = _serializer.float32(obj.uLR, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ultraSS
    let len;
    let data = new ultraSS(null);
    // Deserialize message field [uFR]
    data.uFR = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [uFL]
    data.uFL = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [uRF]
    data.uRF = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [uRR]
    data.uRR = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [uLF]
    data.uLF = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [uLR]
    data.uLR = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/ultraSS';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9863d3f832b0ad0ccba6621fbb52a2e5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 uFR
    float32 uFL
    float32 uRF
    float32 uRR
    float32 uLF
    float32 uLR
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ultraSS(null);
    if (msg.uFR !== undefined) {
      resolved.uFR = msg.uFR;
    }
    else {
      resolved.uFR = 0.0
    }

    if (msg.uFL !== undefined) {
      resolved.uFL = msg.uFL;
    }
    else {
      resolved.uFL = 0.0
    }

    if (msg.uRF !== undefined) {
      resolved.uRF = msg.uRF;
    }
    else {
      resolved.uRF = 0.0
    }

    if (msg.uRR !== undefined) {
      resolved.uRR = msg.uRR;
    }
    else {
      resolved.uRR = 0.0
    }

    if (msg.uLF !== undefined) {
      resolved.uLF = msg.uLF;
    }
    else {
      resolved.uLF = 0.0
    }

    if (msg.uLR !== undefined) {
      resolved.uLR = msg.uLR;
    }
    else {
      resolved.uLR = 0.0
    }

    return resolved;
    }
};

module.exports = ultraSS;
