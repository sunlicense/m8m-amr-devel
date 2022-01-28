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

class dyna {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.paramid = null;
      this.intValue = null;
      this.doubleValue = null;
      this.strValue = null;
      this.boolValue = null;
      this.ftprintno = null;
      this.footprintlist = null;
    }
    else {
      if (initObj.hasOwnProperty('paramid')) {
        this.paramid = initObj.paramid
      }
      else {
        this.paramid = 0;
      }
      if (initObj.hasOwnProperty('intValue')) {
        this.intValue = initObj.intValue
      }
      else {
        this.intValue = 0;
      }
      if (initObj.hasOwnProperty('doubleValue')) {
        this.doubleValue = initObj.doubleValue
      }
      else {
        this.doubleValue = 0.0;
      }
      if (initObj.hasOwnProperty('strValue')) {
        this.strValue = initObj.strValue
      }
      else {
        this.strValue = '';
      }
      if (initObj.hasOwnProperty('boolValue')) {
        this.boolValue = initObj.boolValue
      }
      else {
        this.boolValue = false;
      }
      if (initObj.hasOwnProperty('ftprintno')) {
        this.ftprintno = initObj.ftprintno
      }
      else {
        this.ftprintno = 0;
      }
      if (initObj.hasOwnProperty('footprintlist')) {
        this.footprintlist = initObj.footprintlist
      }
      else {
        this.footprintlist = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type dyna
    // Serialize message field [paramid]
    bufferOffset = _serializer.int8(obj.paramid, buffer, bufferOffset);
    // Serialize message field [intValue]
    bufferOffset = _serializer.int32(obj.intValue, buffer, bufferOffset);
    // Serialize message field [doubleValue]
    bufferOffset = _serializer.float64(obj.doubleValue, buffer, bufferOffset);
    // Serialize message field [strValue]
    bufferOffset = _serializer.string(obj.strValue, buffer, bufferOffset);
    // Serialize message field [boolValue]
    bufferOffset = _serializer.bool(obj.boolValue, buffer, bufferOffset);
    // Serialize message field [ftprintno]
    bufferOffset = _serializer.int8(obj.ftprintno, buffer, bufferOffset);
    // Serialize message field [footprintlist]
    bufferOffset = _arraySerializer.float64(obj.footprintlist, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type dyna
    let len;
    let data = new dyna(null);
    // Deserialize message field [paramid]
    data.paramid = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [intValue]
    data.intValue = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [doubleValue]
    data.doubleValue = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [strValue]
    data.strValue = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [boolValue]
    data.boolValue = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ftprintno]
    data.ftprintno = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [footprintlist]
    data.footprintlist = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.strValue.length;
    length += 8 * object.footprintlist.length;
    return length + 23;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/dyna';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aeca87cb1dcfc788fa047b80da0c718b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 paramid
    int32 intValue
    float64 doubleValue
    string strValue
    bool boolValue
    int8 ftprintno
    float64[] footprintlist
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new dyna(null);
    if (msg.paramid !== undefined) {
      resolved.paramid = msg.paramid;
    }
    else {
      resolved.paramid = 0
    }

    if (msg.intValue !== undefined) {
      resolved.intValue = msg.intValue;
    }
    else {
      resolved.intValue = 0
    }

    if (msg.doubleValue !== undefined) {
      resolved.doubleValue = msg.doubleValue;
    }
    else {
      resolved.doubleValue = 0.0
    }

    if (msg.strValue !== undefined) {
      resolved.strValue = msg.strValue;
    }
    else {
      resolved.strValue = ''
    }

    if (msg.boolValue !== undefined) {
      resolved.boolValue = msg.boolValue;
    }
    else {
      resolved.boolValue = false
    }

    if (msg.ftprintno !== undefined) {
      resolved.ftprintno = msg.ftprintno;
    }
    else {
      resolved.ftprintno = 0
    }

    if (msg.footprintlist !== undefined) {
      resolved.footprintlist = msg.footprintlist;
    }
    else {
      resolved.footprintlist = []
    }

    return resolved;
    }
};

module.exports = dyna;
