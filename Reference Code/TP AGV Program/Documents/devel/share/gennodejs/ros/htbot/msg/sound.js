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

class sound {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.startdelay = null;
      this.restartdelay = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('startdelay')) {
        this.startdelay = initObj.startdelay
      }
      else {
        this.startdelay = 0;
      }
      if (initObj.hasOwnProperty('restartdelay')) {
        this.restartdelay = initObj.restartdelay
      }
      else {
        this.restartdelay = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sound
    // Serialize message field [id]
    bufferOffset = _serializer.int8(obj.id, buffer, bufferOffset);
    // Serialize message field [startdelay]
    bufferOffset = _serializer.int8(obj.startdelay, buffer, bufferOffset);
    // Serialize message field [restartdelay]
    bufferOffset = _serializer.int8(obj.restartdelay, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sound
    let len;
    let data = new sound(null);
    // Deserialize message field [id]
    data.id = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [startdelay]
    data.startdelay = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [restartdelay]
    data.restartdelay = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/sound';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '42b51a54eeade2ca22faa10c3766c366';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 id
    int8 startdelay
    int8 restartdelay
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sound(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.startdelay !== undefined) {
      resolved.startdelay = msg.startdelay;
    }
    else {
      resolved.startdelay = 0
    }

    if (msg.restartdelay !== undefined) {
      resolved.restartdelay = msg.restartdelay;
    }
    else {
      resolved.restartdelay = 0
    }

    return resolved;
    }
};

module.exports = sound;
