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

class move {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.rx = null;
      this.ry = null;
      this.rz = null;
      this.rw = null;
      this.pd = null;
      this.pa = null;
      this.gap = null;
      this.slp = null;
      this.elp = null;
      this.opt = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('rx')) {
        this.rx = initObj.rx
      }
      else {
        this.rx = 0.0;
      }
      if (initObj.hasOwnProperty('ry')) {
        this.ry = initObj.ry
      }
      else {
        this.ry = 0.0;
      }
      if (initObj.hasOwnProperty('rz')) {
        this.rz = initObj.rz
      }
      else {
        this.rz = 0.0;
      }
      if (initObj.hasOwnProperty('rw')) {
        this.rw = initObj.rw
      }
      else {
        this.rw = 0.0;
      }
      if (initObj.hasOwnProperty('pd')) {
        this.pd = initObj.pd
      }
      else {
        this.pd = 0.0;
      }
      if (initObj.hasOwnProperty('pa')) {
        this.pa = initObj.pa
      }
      else {
        this.pa = 0.0;
      }
      if (initObj.hasOwnProperty('gap')) {
        this.gap = initObj.gap
      }
      else {
        this.gap = 0.0;
      }
      if (initObj.hasOwnProperty('slp')) {
        this.slp = initObj.slp
      }
      else {
        this.slp = 0;
      }
      if (initObj.hasOwnProperty('elp')) {
        this.elp = initObj.elp
      }
      else {
        this.elp = 0;
      }
      if (initObj.hasOwnProperty('opt')) {
        this.opt = initObj.opt
      }
      else {
        this.opt = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type move
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [rx]
    bufferOffset = _serializer.float32(obj.rx, buffer, bufferOffset);
    // Serialize message field [ry]
    bufferOffset = _serializer.float32(obj.ry, buffer, bufferOffset);
    // Serialize message field [rz]
    bufferOffset = _serializer.float32(obj.rz, buffer, bufferOffset);
    // Serialize message field [rw]
    bufferOffset = _serializer.float32(obj.rw, buffer, bufferOffset);
    // Serialize message field [pd]
    bufferOffset = _serializer.float32(obj.pd, buffer, bufferOffset);
    // Serialize message field [pa]
    bufferOffset = _serializer.float32(obj.pa, buffer, bufferOffset);
    // Serialize message field [gap]
    bufferOffset = _serializer.float32(obj.gap, buffer, bufferOffset);
    // Serialize message field [slp]
    bufferOffset = _serializer.int8(obj.slp, buffer, bufferOffset);
    // Serialize message field [elp]
    bufferOffset = _serializer.int8(obj.elp, buffer, bufferOffset);
    // Serialize message field [opt]
    bufferOffset = _serializer.int8(obj.opt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type move
    let len;
    let data = new move(null);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rx]
    data.rx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ry]
    data.ry = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rz]
    data.rz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rw]
    data.rw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pd]
    data.pd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pa]
    data.pa = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gap]
    data.gap = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [slp]
    data.slp = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [elp]
    data.elp = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [opt]
    data.opt = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 43;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/move';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '05f4bf5bce5c30ffe01fb09c18cbbd86';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 x
    float32 y
    float32 z
    float32 rx
    float32 ry
    float32 rz
    float32 rw
    float32 pd
    float32 pa
    float32 gap
    int8 slp
    int8 elp
    int8 opt
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new move(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.rx !== undefined) {
      resolved.rx = msg.rx;
    }
    else {
      resolved.rx = 0.0
    }

    if (msg.ry !== undefined) {
      resolved.ry = msg.ry;
    }
    else {
      resolved.ry = 0.0
    }

    if (msg.rz !== undefined) {
      resolved.rz = msg.rz;
    }
    else {
      resolved.rz = 0.0
    }

    if (msg.rw !== undefined) {
      resolved.rw = msg.rw;
    }
    else {
      resolved.rw = 0.0
    }

    if (msg.pd !== undefined) {
      resolved.pd = msg.pd;
    }
    else {
      resolved.pd = 0.0
    }

    if (msg.pa !== undefined) {
      resolved.pa = msg.pa;
    }
    else {
      resolved.pa = 0.0
    }

    if (msg.gap !== undefined) {
      resolved.gap = msg.gap;
    }
    else {
      resolved.gap = 0.0
    }

    if (msg.slp !== undefined) {
      resolved.slp = msg.slp;
    }
    else {
      resolved.slp = 0
    }

    if (msg.elp !== undefined) {
      resolved.elp = msg.elp;
    }
    else {
      resolved.elp = 0
    }

    if (msg.opt !== undefined) {
      resolved.opt = msg.opt;
    }
    else {
      resolved.opt = 0
    }

    return resolved;
    }
};

module.exports = move;
