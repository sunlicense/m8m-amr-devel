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

class goal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.startidx = null;
      this.lastidx = null;
      this.numgoal = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.rx = null;
      this.ry = null;
      this.rz = null;
      this.rw = null;
      this.pd = null;
      this.pa = null;
      this.opt = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = 0;
      }
      if (initObj.hasOwnProperty('startidx')) {
        this.startidx = initObj.startidx
      }
      else {
        this.startidx = 0;
      }
      if (initObj.hasOwnProperty('lastidx')) {
        this.lastidx = initObj.lastidx
      }
      else {
        this.lastidx = 0;
      }
      if (initObj.hasOwnProperty('numgoal')) {
        this.numgoal = initObj.numgoal
      }
      else {
        this.numgoal = 0;
      }
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
      if (initObj.hasOwnProperty('opt')) {
        this.opt = initObj.opt
      }
      else {
        this.opt = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type goal
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [startidx]
    bufferOffset = _serializer.int16(obj.startidx, buffer, bufferOffset);
    // Serialize message field [lastidx]
    bufferOffset = _serializer.int16(obj.lastidx, buffer, bufferOffset);
    // Serialize message field [numgoal]
    bufferOffset = _serializer.int16(obj.numgoal, buffer, bufferOffset);
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
    // Serialize message field [opt]
    bufferOffset = _serializer.int8(obj.opt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type goal
    let len;
    let data = new goal(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [startidx]
    data.startidx = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [lastidx]
    data.lastidx = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [numgoal]
    data.numgoal = _deserializer.int16(buffer, bufferOffset);
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
    // Deserialize message field [opt]
    data.opt = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/goal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d9db86a8255735881de2fc33b96e154';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 cmd
    int16 startidx
    int16 lastidx
    int16 numgoal
    float32 x
    float32 y
    float32 z
    float32 rx
    float32 ry
    float32 rz
    float32 rw
    float32 pd
    float32 pa
    int8 opt
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new goal(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = 0
    }

    if (msg.startidx !== undefined) {
      resolved.startidx = msg.startidx;
    }
    else {
      resolved.startidx = 0
    }

    if (msg.lastidx !== undefined) {
      resolved.lastidx = msg.lastidx;
    }
    else {
      resolved.lastidx = 0
    }

    if (msg.numgoal !== undefined) {
      resolved.numgoal = msg.numgoal;
    }
    else {
      resolved.numgoal = 0
    }

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

    if (msg.opt !== undefined) {
      resolved.opt = msg.opt;
    }
    else {
      resolved.opt = 0
    }

    return resolved;
    }
};

module.exports = goal;
