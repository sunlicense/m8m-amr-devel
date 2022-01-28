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

class path {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.px = null;
      this.py = null;
      this.pz = null;
      this.prx = null;
      this.pry = null;
      this.prz = null;
      this.prw = null;
      this.gx = null;
      this.gy = null;
      this.gz = null;
      this.grx = null;
      this.gry = null;
      this.grz = null;
      this.grw = null;
      this.tol = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = 0;
      }
      if (initObj.hasOwnProperty('px')) {
        this.px = initObj.px
      }
      else {
        this.px = 0.0;
      }
      if (initObj.hasOwnProperty('py')) {
        this.py = initObj.py
      }
      else {
        this.py = 0.0;
      }
      if (initObj.hasOwnProperty('pz')) {
        this.pz = initObj.pz
      }
      else {
        this.pz = 0.0;
      }
      if (initObj.hasOwnProperty('prx')) {
        this.prx = initObj.prx
      }
      else {
        this.prx = 0.0;
      }
      if (initObj.hasOwnProperty('pry')) {
        this.pry = initObj.pry
      }
      else {
        this.pry = 0.0;
      }
      if (initObj.hasOwnProperty('prz')) {
        this.prz = initObj.prz
      }
      else {
        this.prz = 0.0;
      }
      if (initObj.hasOwnProperty('prw')) {
        this.prw = initObj.prw
      }
      else {
        this.prw = 0.0;
      }
      if (initObj.hasOwnProperty('gx')) {
        this.gx = initObj.gx
      }
      else {
        this.gx = 0.0;
      }
      if (initObj.hasOwnProperty('gy')) {
        this.gy = initObj.gy
      }
      else {
        this.gy = 0.0;
      }
      if (initObj.hasOwnProperty('gz')) {
        this.gz = initObj.gz
      }
      else {
        this.gz = 0.0;
      }
      if (initObj.hasOwnProperty('grx')) {
        this.grx = initObj.grx
      }
      else {
        this.grx = 0.0;
      }
      if (initObj.hasOwnProperty('gry')) {
        this.gry = initObj.gry
      }
      else {
        this.gry = 0.0;
      }
      if (initObj.hasOwnProperty('grz')) {
        this.grz = initObj.grz
      }
      else {
        this.grz = 0.0;
      }
      if (initObj.hasOwnProperty('grw')) {
        this.grw = initObj.grw
      }
      else {
        this.grw = 0.0;
      }
      if (initObj.hasOwnProperty('tol')) {
        this.tol = initObj.tol
      }
      else {
        this.tol = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type path
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [px]
    bufferOffset = _serializer.float32(obj.px, buffer, bufferOffset);
    // Serialize message field [py]
    bufferOffset = _serializer.float32(obj.py, buffer, bufferOffset);
    // Serialize message field [pz]
    bufferOffset = _serializer.float32(obj.pz, buffer, bufferOffset);
    // Serialize message field [prx]
    bufferOffset = _serializer.float32(obj.prx, buffer, bufferOffset);
    // Serialize message field [pry]
    bufferOffset = _serializer.float32(obj.pry, buffer, bufferOffset);
    // Serialize message field [prz]
    bufferOffset = _serializer.float32(obj.prz, buffer, bufferOffset);
    // Serialize message field [prw]
    bufferOffset = _serializer.float32(obj.prw, buffer, bufferOffset);
    // Serialize message field [gx]
    bufferOffset = _serializer.float32(obj.gx, buffer, bufferOffset);
    // Serialize message field [gy]
    bufferOffset = _serializer.float32(obj.gy, buffer, bufferOffset);
    // Serialize message field [gz]
    bufferOffset = _serializer.float32(obj.gz, buffer, bufferOffset);
    // Serialize message field [grx]
    bufferOffset = _serializer.float32(obj.grx, buffer, bufferOffset);
    // Serialize message field [gry]
    bufferOffset = _serializer.float32(obj.gry, buffer, bufferOffset);
    // Serialize message field [grz]
    bufferOffset = _serializer.float32(obj.grz, buffer, bufferOffset);
    // Serialize message field [grw]
    bufferOffset = _serializer.float32(obj.grw, buffer, bufferOffset);
    // Serialize message field [tol]
    bufferOffset = _serializer.float32(obj.tol, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type path
    let len;
    let data = new path(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [px]
    data.px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [py]
    data.py = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pz]
    data.pz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [prx]
    data.prx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pry]
    data.pry = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [prz]
    data.prz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [prw]
    data.prw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gx]
    data.gx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gy]
    data.gy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gz]
    data.gz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [grx]
    data.grx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gry]
    data.gry = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [grz]
    data.grz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [grw]
    data.grw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tol]
    data.tol = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 61;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/path';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d2fce4d2355f03d0ce87b3bd4babb5e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 cmd
    float32 px
    float32 py
    float32 pz
    float32 prx
    float32 pry
    float32 prz
    float32 prw
    float32 gx
    float32 gy
    float32 gz
    float32 grx
    float32 gry
    float32 grz
    float32 grw
    float32 tol
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new path(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = 0
    }

    if (msg.px !== undefined) {
      resolved.px = msg.px;
    }
    else {
      resolved.px = 0.0
    }

    if (msg.py !== undefined) {
      resolved.py = msg.py;
    }
    else {
      resolved.py = 0.0
    }

    if (msg.pz !== undefined) {
      resolved.pz = msg.pz;
    }
    else {
      resolved.pz = 0.0
    }

    if (msg.prx !== undefined) {
      resolved.prx = msg.prx;
    }
    else {
      resolved.prx = 0.0
    }

    if (msg.pry !== undefined) {
      resolved.pry = msg.pry;
    }
    else {
      resolved.pry = 0.0
    }

    if (msg.prz !== undefined) {
      resolved.prz = msg.prz;
    }
    else {
      resolved.prz = 0.0
    }

    if (msg.prw !== undefined) {
      resolved.prw = msg.prw;
    }
    else {
      resolved.prw = 0.0
    }

    if (msg.gx !== undefined) {
      resolved.gx = msg.gx;
    }
    else {
      resolved.gx = 0.0
    }

    if (msg.gy !== undefined) {
      resolved.gy = msg.gy;
    }
    else {
      resolved.gy = 0.0
    }

    if (msg.gz !== undefined) {
      resolved.gz = msg.gz;
    }
    else {
      resolved.gz = 0.0
    }

    if (msg.grx !== undefined) {
      resolved.grx = msg.grx;
    }
    else {
      resolved.grx = 0.0
    }

    if (msg.gry !== undefined) {
      resolved.gry = msg.gry;
    }
    else {
      resolved.gry = 0.0
    }

    if (msg.grz !== undefined) {
      resolved.grz = msg.grz;
    }
    else {
      resolved.grz = 0.0
    }

    if (msg.grw !== undefined) {
      resolved.grw = msg.grw;
    }
    else {
      resolved.grw = 0.0
    }

    if (msg.tol !== undefined) {
      resolved.tol = msg.tol;
    }
    else {
      resolved.tol = 0.0
    }

    return resolved;
    }
};

module.exports = path;
