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

class ntuc_status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.time = null;
      this.status = null;
      this.location = null;
      this.arrival = null;
      this.b_level = null;
      this.lift_status = null;
      this.complete = null;
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
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('location')) {
        this.location = initObj.location
      }
      else {
        this.location = 0;
      }
      if (initObj.hasOwnProperty('arrival')) {
        this.arrival = initObj.arrival
      }
      else {
        this.arrival = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('b_level')) {
        this.b_level = initObj.b_level
      }
      else {
        this.b_level = 0;
      }
      if (initObj.hasOwnProperty('lift_status')) {
        this.lift_status = initObj.lift_status
      }
      else {
        this.lift_status = 0;
      }
      if (initObj.hasOwnProperty('complete')) {
        this.complete = initObj.complete
      }
      else {
        this.complete = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ntuc_status
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.time(obj.time, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.int8(obj.status, buffer, bufferOffset);
    // Serialize message field [location]
    bufferOffset = _serializer.int8(obj.location, buffer, bufferOffset);
    // Serialize message field [arrival]
    bufferOffset = _serializer.time(obj.arrival, buffer, bufferOffset);
    // Serialize message field [b_level]
    bufferOffset = _serializer.int8(obj.b_level, buffer, bufferOffset);
    // Serialize message field [lift_status]
    bufferOffset = _serializer.int8(obj.lift_status, buffer, bufferOffset);
    // Serialize message field [complete]
    bufferOffset = _serializer.int8(obj.complete, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ntuc_status
    let len;
    let data = new ntuc_status(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [location]
    data.location = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [arrival]
    data.arrival = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [b_level]
    data.b_level = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [lift_status]
    data.lift_status = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [complete]
    data.complete = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/ntuc_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ad6efd2f44b02d2eda310e45f4e16a44';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 cmd
    time time
    int8 status
    int8 location
    time arrival
    int8 b_level
    int8 lift_status
    int8 complete
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ntuc_status(null);
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

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.location !== undefined) {
      resolved.location = msg.location;
    }
    else {
      resolved.location = 0
    }

    if (msg.arrival !== undefined) {
      resolved.arrival = msg.arrival;
    }
    else {
      resolved.arrival = {secs: 0, nsecs: 0}
    }

    if (msg.b_level !== undefined) {
      resolved.b_level = msg.b_level;
    }
    else {
      resolved.b_level = 0
    }

    if (msg.lift_status !== undefined) {
      resolved.lift_status = msg.lift_status;
    }
    else {
      resolved.lift_status = 0
    }

    if (msg.complete !== undefined) {
      resolved.complete = msg.complete;
    }
    else {
      resolved.complete = 0
    }

    return resolved;
    }
};

module.exports = ntuc_status;
