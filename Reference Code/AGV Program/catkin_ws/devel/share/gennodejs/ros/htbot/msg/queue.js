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

class queue {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.noQ = null;
      this.fLP1 = null;
      this.tLP1 = null;
      this.fLP2 = null;
      this.tLP2 = null;
      this.fLP3 = null;
      this.tLP3 = null;
      this.fLP4 = null;
      this.tLP4 = null;
      this.LPName = null;
      this.LPInfo = null;
    }
    else {
      if (initObj.hasOwnProperty('noQ')) {
        this.noQ = initObj.noQ
      }
      else {
        this.noQ = 0;
      }
      if (initObj.hasOwnProperty('fLP1')) {
        this.fLP1 = initObj.fLP1
      }
      else {
        this.fLP1 = '';
      }
      if (initObj.hasOwnProperty('tLP1')) {
        this.tLP1 = initObj.tLP1
      }
      else {
        this.tLP1 = '';
      }
      if (initObj.hasOwnProperty('fLP2')) {
        this.fLP2 = initObj.fLP2
      }
      else {
        this.fLP2 = '';
      }
      if (initObj.hasOwnProperty('tLP2')) {
        this.tLP2 = initObj.tLP2
      }
      else {
        this.tLP2 = '';
      }
      if (initObj.hasOwnProperty('fLP3')) {
        this.fLP3 = initObj.fLP3
      }
      else {
        this.fLP3 = '';
      }
      if (initObj.hasOwnProperty('tLP3')) {
        this.tLP3 = initObj.tLP3
      }
      else {
        this.tLP3 = '';
      }
      if (initObj.hasOwnProperty('fLP4')) {
        this.fLP4 = initObj.fLP4
      }
      else {
        this.fLP4 = '';
      }
      if (initObj.hasOwnProperty('tLP4')) {
        this.tLP4 = initObj.tLP4
      }
      else {
        this.tLP4 = '';
      }
      if (initObj.hasOwnProperty('LPName')) {
        this.LPName = initObj.LPName
      }
      else {
        this.LPName = '';
      }
      if (initObj.hasOwnProperty('LPInfo')) {
        this.LPInfo = initObj.LPInfo
      }
      else {
        this.LPInfo = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type queue
    // Serialize message field [noQ]
    bufferOffset = _serializer.int8(obj.noQ, buffer, bufferOffset);
    // Serialize message field [fLP1]
    bufferOffset = _serializer.string(obj.fLP1, buffer, bufferOffset);
    // Serialize message field [tLP1]
    bufferOffset = _serializer.string(obj.tLP1, buffer, bufferOffset);
    // Serialize message field [fLP2]
    bufferOffset = _serializer.string(obj.fLP2, buffer, bufferOffset);
    // Serialize message field [tLP2]
    bufferOffset = _serializer.string(obj.tLP2, buffer, bufferOffset);
    // Serialize message field [fLP3]
    bufferOffset = _serializer.string(obj.fLP3, buffer, bufferOffset);
    // Serialize message field [tLP3]
    bufferOffset = _serializer.string(obj.tLP3, buffer, bufferOffset);
    // Serialize message field [fLP4]
    bufferOffset = _serializer.string(obj.fLP4, buffer, bufferOffset);
    // Serialize message field [tLP4]
    bufferOffset = _serializer.string(obj.tLP4, buffer, bufferOffset);
    // Serialize message field [LPName]
    bufferOffset = _serializer.string(obj.LPName, buffer, bufferOffset);
    // Serialize message field [LPInfo]
    bufferOffset = _serializer.string(obj.LPInfo, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type queue
    let len;
    let data = new queue(null);
    // Deserialize message field [noQ]
    data.noQ = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [fLP1]
    data.fLP1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [tLP1]
    data.tLP1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [fLP2]
    data.fLP2 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [tLP2]
    data.tLP2 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [fLP3]
    data.fLP3 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [tLP3]
    data.tLP3 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [fLP4]
    data.fLP4 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [tLP4]
    data.tLP4 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [LPName]
    data.LPName = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [LPInfo]
    data.LPInfo = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.fLP1.length;
    length += object.tLP1.length;
    length += object.fLP2.length;
    length += object.tLP2.length;
    length += object.fLP3.length;
    length += object.tLP3.length;
    length += object.fLP4.length;
    length += object.tLP4.length;
    length += object.LPName.length;
    length += object.LPInfo.length;
    return length + 41;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/queue';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1ad64500a08450047cb508b267d3d903';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 noQ
    string fLP1
    string tLP1
    string fLP2
    string tLP2
    string fLP3
    string tLP3
    string fLP4
    string tLP4
    string LPName
    string LPInfo
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new queue(null);
    if (msg.noQ !== undefined) {
      resolved.noQ = msg.noQ;
    }
    else {
      resolved.noQ = 0
    }

    if (msg.fLP1 !== undefined) {
      resolved.fLP1 = msg.fLP1;
    }
    else {
      resolved.fLP1 = ''
    }

    if (msg.tLP1 !== undefined) {
      resolved.tLP1 = msg.tLP1;
    }
    else {
      resolved.tLP1 = ''
    }

    if (msg.fLP2 !== undefined) {
      resolved.fLP2 = msg.fLP2;
    }
    else {
      resolved.fLP2 = ''
    }

    if (msg.tLP2 !== undefined) {
      resolved.tLP2 = msg.tLP2;
    }
    else {
      resolved.tLP2 = ''
    }

    if (msg.fLP3 !== undefined) {
      resolved.fLP3 = msg.fLP3;
    }
    else {
      resolved.fLP3 = ''
    }

    if (msg.tLP3 !== undefined) {
      resolved.tLP3 = msg.tLP3;
    }
    else {
      resolved.tLP3 = ''
    }

    if (msg.fLP4 !== undefined) {
      resolved.fLP4 = msg.fLP4;
    }
    else {
      resolved.fLP4 = ''
    }

    if (msg.tLP4 !== undefined) {
      resolved.tLP4 = msg.tLP4;
    }
    else {
      resolved.tLP4 = ''
    }

    if (msg.LPName !== undefined) {
      resolved.LPName = msg.LPName;
    }
    else {
      resolved.LPName = ''
    }

    if (msg.LPInfo !== undefined) {
      resolved.LPInfo = msg.LPInfo;
    }
    else {
      resolved.LPInfo = ''
    }

    return resolved;
    }
};

module.exports = queue;
