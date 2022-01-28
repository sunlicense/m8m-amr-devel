// Auto-generated. Do not edit!

// (in-package htbot.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class scanMcmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.file = null;
      this.lp = null;
      this.gp = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type scanMcmdRequest
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [file]
    bufferOffset = _serializer.string(obj.file, buffer, bufferOffset);
    // Serialize message field [lp]
    bufferOffset = _serializer.int8(obj.lp, buffer, bufferOffset);
    // Serialize message field [gp]
    bufferOffset = _serializer.int8(obj.gp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type scanMcmdRequest
    let len;
    let data = new scanMcmdRequest(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [file]
    data.file = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lp]
    data.lp = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [gp]
    data.gp = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.file.length;
    return length + 7;
  }

  static datatype() {
    // Returns string type for a service object
    return 'htbot/scanMcmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '024dc0671c4169424ec159005dc1f9d0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 cmd
    string file
    int8 lp
    int8 gp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new scanMcmdRequest(null);
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

    return resolved;
    }
};

class scanMcmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
      this.s1 = null;
      this.s2 = null;
      this.x = null;
      this.y = null;
      this.an = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('s1')) {
        this.s1 = initObj.s1
      }
      else {
        this.s1 = '';
      }
      if (initObj.hasOwnProperty('s2')) {
        this.s2 = initObj.s2
      }
      else {
        this.s2 = '';
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
      if (initObj.hasOwnProperty('an')) {
        this.an = initObj.an
      }
      else {
        this.an = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type scanMcmdResponse
    // Serialize message field [status]
    bufferOffset = _serializer.int8(obj.status, buffer, bufferOffset);
    // Serialize message field [s1]
    bufferOffset = _serializer.string(obj.s1, buffer, bufferOffset);
    // Serialize message field [s2]
    bufferOffset = _serializer.string(obj.s2, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [an]
    bufferOffset = _serializer.float32(obj.an, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type scanMcmdResponse
    let len;
    let data = new scanMcmdResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [s1]
    data.s1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [s2]
    data.s2 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [an]
    data.an = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.s1.length;
    length += object.s2.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a service object
    return 'htbot/scanMcmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5106dd5514f949ed1f84cad57c27a682';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 status
    string s1
    string s2
    float32 x
    float32 y
    float32 an
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new scanMcmdResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.s1 !== undefined) {
      resolved.s1 = msg.s1;
    }
    else {
      resolved.s1 = ''
    }

    if (msg.s2 !== undefined) {
      resolved.s2 = msg.s2;
    }
    else {
      resolved.s2 = ''
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

    if (msg.an !== undefined) {
      resolved.an = msg.an;
    }
    else {
      resolved.an = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: scanMcmdRequest,
  Response: scanMcmdResponse,
  md5sum() { return 'fe363af63be51788db46af922590fcac'; },
  datatype() { return 'htbot/scanMcmd'; }
};
