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

class srvcmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.fromLP = null;
      this.toLP = null;
      this.cGP = null;
      this.cLP = null;
      this.fLP = null;
      this.tLP = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = 0;
      }
      if (initObj.hasOwnProperty('fromLP')) {
        this.fromLP = initObj.fromLP
      }
      else {
        this.fromLP = '';
      }
      if (initObj.hasOwnProperty('toLP')) {
        this.toLP = initObj.toLP
      }
      else {
        this.toLP = '';
      }
      if (initObj.hasOwnProperty('cGP')) {
        this.cGP = initObj.cGP
      }
      else {
        this.cGP = 0;
      }
      if (initObj.hasOwnProperty('cLP')) {
        this.cLP = initObj.cLP
      }
      else {
        this.cLP = 0;
      }
      if (initObj.hasOwnProperty('fLP')) {
        this.fLP = initObj.fLP
      }
      else {
        this.fLP = 0;
      }
      if (initObj.hasOwnProperty('tLP')) {
        this.tLP = initObj.tLP
      }
      else {
        this.tLP = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type srvcmdRequest
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [fromLP]
    bufferOffset = _serializer.string(obj.fromLP, buffer, bufferOffset);
    // Serialize message field [toLP]
    bufferOffset = _serializer.string(obj.toLP, buffer, bufferOffset);
    // Serialize message field [cGP]
    bufferOffset = _serializer.int8(obj.cGP, buffer, bufferOffset);
    // Serialize message field [cLP]
    bufferOffset = _serializer.int8(obj.cLP, buffer, bufferOffset);
    // Serialize message field [fLP]
    bufferOffset = _serializer.int8(obj.fLP, buffer, bufferOffset);
    // Serialize message field [tLP]
    bufferOffset = _serializer.int8(obj.tLP, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type srvcmdRequest
    let len;
    let data = new srvcmdRequest(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [fromLP]
    data.fromLP = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [toLP]
    data.toLP = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cGP]
    data.cGP = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [cLP]
    data.cLP = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [fLP]
    data.fLP = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [tLP]
    data.tLP = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.fromLP.length;
    length += object.toLP.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'htbot/srvcmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '09bc6b52683ec407d73185b7e17c7067';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 cmd
    string fromLP
    string toLP
    int8 cGP
    int8 cLP
    int8 fLP
    int8 tLP
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new srvcmdRequest(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = 0
    }

    if (msg.fromLP !== undefined) {
      resolved.fromLP = msg.fromLP;
    }
    else {
      resolved.fromLP = ''
    }

    if (msg.toLP !== undefined) {
      resolved.toLP = msg.toLP;
    }
    else {
      resolved.toLP = ''
    }

    if (msg.cGP !== undefined) {
      resolved.cGP = msg.cGP;
    }
    else {
      resolved.cGP = 0
    }

    if (msg.cLP !== undefined) {
      resolved.cLP = msg.cLP;
    }
    else {
      resolved.cLP = 0
    }

    if (msg.fLP !== undefined) {
      resolved.fLP = msg.fLP;
    }
    else {
      resolved.fLP = 0
    }

    if (msg.tLP !== undefined) {
      resolved.tLP = msg.tLP;
    }
    else {
      resolved.tLP = 0
    }

    return resolved;
    }
};

class srvcmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
      this.s1 = null;
      this.s2 = null;
      this.linear = null;
      this.angular = null;
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
      if (initObj.hasOwnProperty('linear')) {
        this.linear = initObj.linear
      }
      else {
        this.linear = 0.0;
      }
      if (initObj.hasOwnProperty('angular')) {
        this.angular = initObj.angular
      }
      else {
        this.angular = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type srvcmdResponse
    // Serialize message field [status]
    bufferOffset = _serializer.int8(obj.status, buffer, bufferOffset);
    // Serialize message field [s1]
    bufferOffset = _serializer.string(obj.s1, buffer, bufferOffset);
    // Serialize message field [s2]
    bufferOffset = _serializer.string(obj.s2, buffer, bufferOffset);
    // Serialize message field [linear]
    bufferOffset = _serializer.float32(obj.linear, buffer, bufferOffset);
    // Serialize message field [angular]
    bufferOffset = _serializer.float32(obj.angular, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type srvcmdResponse
    let len;
    let data = new srvcmdResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [s1]
    data.s1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [s2]
    data.s2 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [linear]
    data.linear = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular]
    data.angular = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.s1.length;
    length += object.s2.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'htbot/srvcmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bf41a1e7b1d81f6e588d96220bbc26a9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 status
    string s1
    string s2
    float32 linear
    float32 angular
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new srvcmdResponse(null);
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

    if (msg.linear !== undefined) {
      resolved.linear = msg.linear;
    }
    else {
      resolved.linear = 0.0
    }

    if (msg.angular !== undefined) {
      resolved.angular = msg.angular;
    }
    else {
      resolved.angular = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: srvcmdRequest,
  Response: srvcmdResponse,
  md5sum() { return '30eadfa966628646a567b1a8c11f77b8'; },
  datatype() { return 'htbot/srvcmd'; }
};
