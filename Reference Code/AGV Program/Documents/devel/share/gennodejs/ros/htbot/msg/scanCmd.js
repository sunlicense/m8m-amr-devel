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

class scanCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.file = null;
      this.f1 = null;
      this.f2 = null;
      this.f3 = null;
      this.lp = null;
      this.gp = null;
      this.opt = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = 0;
      }
      if (initObj.hasOwnProperty('file')) {
        this.file = initObj.file
      }
      else {
        this.file = '';
      }
      if (initObj.hasOwnProperty('f1')) {
        this.f1 = initObj.f1
      }
      else {
        this.f1 = 0.0;
      }
      if (initObj.hasOwnProperty('f2')) {
        this.f2 = initObj.f2
      }
      else {
        this.f2 = 0.0;
      }
      if (initObj.hasOwnProperty('f3')) {
        this.f3 = initObj.f3
      }
      else {
        this.f3 = 0.0;
      }
      if (initObj.hasOwnProperty('lp')) {
        this.lp = initObj.lp
      }
      else {
        this.lp = 0;
      }
      if (initObj.hasOwnProperty('gp')) {
        this.gp = initObj.gp
      }
      else {
        this.gp = 0;
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
    // Serializes a message object of type scanCmd
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [file]
    bufferOffset = _serializer.string(obj.file, buffer, bufferOffset);
    // Serialize message field [f1]
    bufferOffset = _serializer.float32(obj.f1, buffer, bufferOffset);
    // Serialize message field [f2]
    bufferOffset = _serializer.float32(obj.f2, buffer, bufferOffset);
    // Serialize message field [f3]
    bufferOffset = _serializer.float32(obj.f3, buffer, bufferOffset);
    // Serialize message field [lp]
    bufferOffset = _serializer.int8(obj.lp, buffer, bufferOffset);
    // Serialize message field [gp]
    bufferOffset = _serializer.int8(obj.gp, buffer, bufferOffset);
    // Serialize message field [opt]
    bufferOffset = _serializer.int8(obj.opt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type scanCmd
    let len;
    let data = new scanCmd(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [file]
    data.file = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [f1]
    data.f1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [f2]
    data.f2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [f3]
    data.f3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lp]
    data.lp = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [gp]
    data.gp = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [opt]
    data.opt = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.file.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/scanCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '009679f12681386d87811bbbbfbf00db';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 cmd
    string file
    float32 f1
    float32 f2
    float32 f3
    int8 lp
    int8 gp
    int8 opt
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new scanCmd(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = 0
    }

    if (msg.file !== undefined) {
      resolved.file = msg.file;
    }
    else {
      resolved.file = ''
    }

    if (msg.f1 !== undefined) {
      resolved.f1 = msg.f1;
    }
    else {
      resolved.f1 = 0.0
    }

    if (msg.f2 !== undefined) {
      resolved.f2 = msg.f2;
    }
    else {
      resolved.f2 = 0.0
    }

    if (msg.f3 !== undefined) {
      resolved.f3 = msg.f3;
    }
    else {
      resolved.f3 = 0.0
    }

    if (msg.lp !== undefined) {
      resolved.lp = msg.lp;
    }
    else {
      resolved.lp = 0
    }

    if (msg.gp !== undefined) {
      resolved.gp = msg.gp;
    }
    else {
      resolved.gp = 0
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

module.exports = scanCmd;
