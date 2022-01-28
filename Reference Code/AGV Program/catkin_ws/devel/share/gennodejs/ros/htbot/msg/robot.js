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

class robot {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.amclx = null;
      this.amcly = null;
      this.amcla = null;
      this.robotposediff = null;
      this.localisationstatus = null;
    }
    else {
      if (initObj.hasOwnProperty('amclx')) {
        this.amclx = initObj.amclx
      }
      else {
        this.amclx = 0.0;
      }
      if (initObj.hasOwnProperty('amcly')) {
        this.amcly = initObj.amcly
      }
      else {
        this.amcly = 0.0;
      }
      if (initObj.hasOwnProperty('amcla')) {
        this.amcla = initObj.amcla
      }
      else {
        this.amcla = 0.0;
      }
      if (initObj.hasOwnProperty('robotposediff')) {
        this.robotposediff = initObj.robotposediff
      }
      else {
        this.robotposediff = 0.0;
      }
      if (initObj.hasOwnProperty('localisationstatus')) {
        this.localisationstatus = initObj.localisationstatus
      }
      else {
        this.localisationstatus = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type robot
    // Serialize message field [amclx]
    bufferOffset = _serializer.float32(obj.amclx, buffer, bufferOffset);
    // Serialize message field [amcly]
    bufferOffset = _serializer.float32(obj.amcly, buffer, bufferOffset);
    // Serialize message field [amcla]
    bufferOffset = _serializer.float32(obj.amcla, buffer, bufferOffset);
    // Serialize message field [robotposediff]
    bufferOffset = _serializer.float32(obj.robotposediff, buffer, bufferOffset);
    // Serialize message field [localisationstatus]
    bufferOffset = _serializer.string(obj.localisationstatus, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robot
    let len;
    let data = new robot(null);
    // Deserialize message field [amclx]
    data.amclx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [amcly]
    data.amcly = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [amcla]
    data.amcla = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [robotposediff]
    data.robotposediff = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [localisationstatus]
    data.localisationstatus = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.localisationstatus.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/robot';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9472043a148e005ed77686c11e7067c6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 amclx
    float32 amcly
    float32 amcla
    float32 robotposediff
    string localisationstatus
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new robot(null);
    if (msg.amclx !== undefined) {
      resolved.amclx = msg.amclx;
    }
    else {
      resolved.amclx = 0.0
    }

    if (msg.amcly !== undefined) {
      resolved.amcly = msg.amcly;
    }
    else {
      resolved.amcly = 0.0
    }

    if (msg.amcla !== undefined) {
      resolved.amcla = msg.amcla;
    }
    else {
      resolved.amcla = 0.0
    }

    if (msg.robotposediff !== undefined) {
      resolved.robotposediff = msg.robotposediff;
    }
    else {
      resolved.robotposediff = 0.0
    }

    if (msg.localisationstatus !== undefined) {
      resolved.localisationstatus = msg.localisationstatus;
    }
    else {
      resolved.localisationstatus = ''
    }

    return resolved;
    }
};

module.exports = robot;
