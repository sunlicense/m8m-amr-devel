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

class task {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.time = null;
      this.type = null;
      this.fromLP = null;
      this.toLP = null;
      this.alloc = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = 0;
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('fromLP')) {
        this.fromLP = initObj.fromLP
      }
      else {
        this.fromLP = 0;
      }
      if (initObj.hasOwnProperty('toLP')) {
        this.toLP = initObj.toLP
      }
      else {
        this.toLP = 0;
      }
      if (initObj.hasOwnProperty('alloc')) {
        this.alloc = initObj.alloc
      }
      else {
        this.alloc = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type task
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.time(obj.time, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.int8(obj.type, buffer, bufferOffset);
    // Serialize message field [fromLP]
    bufferOffset = _serializer.int8(obj.fromLP, buffer, bufferOffset);
    // Serialize message field [toLP]
    bufferOffset = _serializer.int8(obj.toLP, buffer, bufferOffset);
    // Serialize message field [alloc]
    bufferOffset = _serializer.int8(obj.alloc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type task
    let len;
    let data = new task(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [fromLP]
    data.fromLP = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [toLP]
    data.toLP = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [alloc]
    data.alloc = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/task';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd9335104b860e9530a386a51f33ebb59';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 cmd
    time time
    int8 type
    int8 fromLP
    int8 toLP
    int8 alloc
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new task(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = 0
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = {secs: 0, nsecs: 0}
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.fromLP !== undefined) {
      resolved.fromLP = msg.fromLP;
    }
    else {
      resolved.fromLP = 0
    }

    if (msg.toLP !== undefined) {
      resolved.toLP = msg.toLP;
    }
    else {
      resolved.toLP = 0
    }

    if (msg.alloc !== undefined) {
      resolved.alloc = msg.alloc;
    }
    else {
      resolved.alloc = 0
    }

    return resolved;
    }
};

module.exports = task;
