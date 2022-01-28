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

class navstatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stn0 = null;
      this.stn1 = null;
      this.stn2 = null;
      this.stn3 = null;
      this.stn4 = null;
      this.stn5 = null;
      this.stn6 = null;
      this.stn7 = null;
      this.stn8 = null;
      this.stn9 = null;
    }
    else {
      if (initObj.hasOwnProperty('stn0')) {
        this.stn0 = initObj.stn0
      }
      else {
        this.stn0 = 0;
      }
      if (initObj.hasOwnProperty('stn1')) {
        this.stn1 = initObj.stn1
      }
      else {
        this.stn1 = 0;
      }
      if (initObj.hasOwnProperty('stn2')) {
        this.stn2 = initObj.stn2
      }
      else {
        this.stn2 = 0;
      }
      if (initObj.hasOwnProperty('stn3')) {
        this.stn3 = initObj.stn3
      }
      else {
        this.stn3 = 0;
      }
      if (initObj.hasOwnProperty('stn4')) {
        this.stn4 = initObj.stn4
      }
      else {
        this.stn4 = 0;
      }
      if (initObj.hasOwnProperty('stn5')) {
        this.stn5 = initObj.stn5
      }
      else {
        this.stn5 = 0;
      }
      if (initObj.hasOwnProperty('stn6')) {
        this.stn6 = initObj.stn6
      }
      else {
        this.stn6 = 0;
      }
      if (initObj.hasOwnProperty('stn7')) {
        this.stn7 = initObj.stn7
      }
      else {
        this.stn7 = 0;
      }
      if (initObj.hasOwnProperty('stn8')) {
        this.stn8 = initObj.stn8
      }
      else {
        this.stn8 = 0;
      }
      if (initObj.hasOwnProperty('stn9')) {
        this.stn9 = initObj.stn9
      }
      else {
        this.stn9 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navstatus
    // Serialize message field [stn0]
    bufferOffset = _serializer.int8(obj.stn0, buffer, bufferOffset);
    // Serialize message field [stn1]
    bufferOffset = _serializer.int8(obj.stn1, buffer, bufferOffset);
    // Serialize message field [stn2]
    bufferOffset = _serializer.int8(obj.stn2, buffer, bufferOffset);
    // Serialize message field [stn3]
    bufferOffset = _serializer.int8(obj.stn3, buffer, bufferOffset);
    // Serialize message field [stn4]
    bufferOffset = _serializer.int8(obj.stn4, buffer, bufferOffset);
    // Serialize message field [stn5]
    bufferOffset = _serializer.int8(obj.stn5, buffer, bufferOffset);
    // Serialize message field [stn6]
    bufferOffset = _serializer.int8(obj.stn6, buffer, bufferOffset);
    // Serialize message field [stn7]
    bufferOffset = _serializer.int8(obj.stn7, buffer, bufferOffset);
    // Serialize message field [stn8]
    bufferOffset = _serializer.int8(obj.stn8, buffer, bufferOffset);
    // Serialize message field [stn9]
    bufferOffset = _serializer.int8(obj.stn9, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navstatus
    let len;
    let data = new navstatus(null);
    // Deserialize message field [stn0]
    data.stn0 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stn1]
    data.stn1 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stn2]
    data.stn2 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stn3]
    data.stn3 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stn4]
    data.stn4 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stn5]
    data.stn5 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stn6]
    data.stn6 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stn7]
    data.stn7 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stn8]
    data.stn8 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stn9]
    data.stn9 = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/navstatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc229716d82952abdbbbe7b0e8dff271';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 stn0
    int8 stn1
    int8 stn2
    int8 stn3
    int8 stn4
    int8 stn5
    int8 stn6
    int8 stn7
    int8 stn8
    int8 stn9
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new navstatus(null);
    if (msg.stn0 !== undefined) {
      resolved.stn0 = msg.stn0;
    }
    else {
      resolved.stn0 = 0
    }

    if (msg.stn1 !== undefined) {
      resolved.stn1 = msg.stn1;
    }
    else {
      resolved.stn1 = 0
    }

    if (msg.stn2 !== undefined) {
      resolved.stn2 = msg.stn2;
    }
    else {
      resolved.stn2 = 0
    }

    if (msg.stn3 !== undefined) {
      resolved.stn3 = msg.stn3;
    }
    else {
      resolved.stn3 = 0
    }

    if (msg.stn4 !== undefined) {
      resolved.stn4 = msg.stn4;
    }
    else {
      resolved.stn4 = 0
    }

    if (msg.stn5 !== undefined) {
      resolved.stn5 = msg.stn5;
    }
    else {
      resolved.stn5 = 0
    }

    if (msg.stn6 !== undefined) {
      resolved.stn6 = msg.stn6;
    }
    else {
      resolved.stn6 = 0
    }

    if (msg.stn7 !== undefined) {
      resolved.stn7 = msg.stn7;
    }
    else {
      resolved.stn7 = 0
    }

    if (msg.stn8 !== undefined) {
      resolved.stn8 = msg.stn8;
    }
    else {
      resolved.stn8 = 0
    }

    if (msg.stn9 !== undefined) {
      resolved.stn9 = msg.stn9;
    }
    else {
      resolved.stn9 = 0
    }

    return resolved;
    }
};

module.exports = navstatus;
