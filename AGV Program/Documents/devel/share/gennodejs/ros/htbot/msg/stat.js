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

class stat {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed = null;
      this.trip = null;
      this.time = null;
      this.maxspeed = null;
      this.minspeed = null;
      this.avgspeed = null;
      this.curspeed = null;
      this.totaltriptime = null;
      this.totaltripdist = null;
      this.avgvolt = null;
      this.avgcurr = null;
      this.maxcurr = null;
      this.mincurr = null;
      this.amphr = null;
      this.batlevel = null;
      this.clearops = null;
      this.estop = null;
      this.motordisable = null;
    }
    else {
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
      if (initObj.hasOwnProperty('trip')) {
        this.trip = initObj.trip
      }
      else {
        this.trip = 0;
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0;
      }
      if (initObj.hasOwnProperty('maxspeed')) {
        this.maxspeed = initObj.maxspeed
      }
      else {
        this.maxspeed = 0.0;
      }
      if (initObj.hasOwnProperty('minspeed')) {
        this.minspeed = initObj.minspeed
      }
      else {
        this.minspeed = 0.0;
      }
      if (initObj.hasOwnProperty('avgspeed')) {
        this.avgspeed = initObj.avgspeed
      }
      else {
        this.avgspeed = 0.0;
      }
      if (initObj.hasOwnProperty('curspeed')) {
        this.curspeed = initObj.curspeed
      }
      else {
        this.curspeed = 0.0;
      }
      if (initObj.hasOwnProperty('totaltriptime')) {
        this.totaltriptime = initObj.totaltriptime
      }
      else {
        this.totaltriptime = 0.0;
      }
      if (initObj.hasOwnProperty('totaltripdist')) {
        this.totaltripdist = initObj.totaltripdist
      }
      else {
        this.totaltripdist = 0.0;
      }
      if (initObj.hasOwnProperty('avgvolt')) {
        this.avgvolt = initObj.avgvolt
      }
      else {
        this.avgvolt = 0.0;
      }
      if (initObj.hasOwnProperty('avgcurr')) {
        this.avgcurr = initObj.avgcurr
      }
      else {
        this.avgcurr = 0.0;
      }
      if (initObj.hasOwnProperty('maxcurr')) {
        this.maxcurr = initObj.maxcurr
      }
      else {
        this.maxcurr = 0.0;
      }
      if (initObj.hasOwnProperty('mincurr')) {
        this.mincurr = initObj.mincurr
      }
      else {
        this.mincurr = 0.0;
      }
      if (initObj.hasOwnProperty('amphr')) {
        this.amphr = initObj.amphr
      }
      else {
        this.amphr = 0.0;
      }
      if (initObj.hasOwnProperty('batlevel')) {
        this.batlevel = initObj.batlevel
      }
      else {
        this.batlevel = 0.0;
      }
      if (initObj.hasOwnProperty('clearops')) {
        this.clearops = initObj.clearops
      }
      else {
        this.clearops = 0;
      }
      if (initObj.hasOwnProperty('estop')) {
        this.estop = initObj.estop
      }
      else {
        this.estop = 0;
      }
      if (initObj.hasOwnProperty('motordisable')) {
        this.motordisable = initObj.motordisable
      }
      else {
        this.motordisable = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type stat
    // Serialize message field [speed]
    bufferOffset = _serializer.int8(obj.speed, buffer, bufferOffset);
    // Serialize message field [trip]
    bufferOffset = _serializer.int8(obj.trip, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.int8(obj.time, buffer, bufferOffset);
    // Serialize message field [maxspeed]
    bufferOffset = _serializer.float32(obj.maxspeed, buffer, bufferOffset);
    // Serialize message field [minspeed]
    bufferOffset = _serializer.float32(obj.minspeed, buffer, bufferOffset);
    // Serialize message field [avgspeed]
    bufferOffset = _serializer.float32(obj.avgspeed, buffer, bufferOffset);
    // Serialize message field [curspeed]
    bufferOffset = _serializer.float32(obj.curspeed, buffer, bufferOffset);
    // Serialize message field [totaltriptime]
    bufferOffset = _serializer.float32(obj.totaltriptime, buffer, bufferOffset);
    // Serialize message field [totaltripdist]
    bufferOffset = _serializer.float32(obj.totaltripdist, buffer, bufferOffset);
    // Serialize message field [avgvolt]
    bufferOffset = _serializer.float32(obj.avgvolt, buffer, bufferOffset);
    // Serialize message field [avgcurr]
    bufferOffset = _serializer.float32(obj.avgcurr, buffer, bufferOffset);
    // Serialize message field [maxcurr]
    bufferOffset = _serializer.float32(obj.maxcurr, buffer, bufferOffset);
    // Serialize message field [mincurr]
    bufferOffset = _serializer.float32(obj.mincurr, buffer, bufferOffset);
    // Serialize message field [amphr]
    bufferOffset = _serializer.float32(obj.amphr, buffer, bufferOffset);
    // Serialize message field [batlevel]
    bufferOffset = _serializer.float32(obj.batlevel, buffer, bufferOffset);
    // Serialize message field [clearops]
    bufferOffset = _serializer.int16(obj.clearops, buffer, bufferOffset);
    // Serialize message field [estop]
    bufferOffset = _serializer.int16(obj.estop, buffer, bufferOffset);
    // Serialize message field [motordisable]
    bufferOffset = _serializer.int16(obj.motordisable, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type stat
    let len;
    let data = new stat(null);
    // Deserialize message field [speed]
    data.speed = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [trip]
    data.trip = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [maxspeed]
    data.maxspeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [minspeed]
    data.minspeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [avgspeed]
    data.avgspeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [curspeed]
    data.curspeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [totaltriptime]
    data.totaltriptime = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [totaltripdist]
    data.totaltripdist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [avgvolt]
    data.avgvolt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [avgcurr]
    data.avgcurr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [maxcurr]
    data.maxcurr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mincurr]
    data.mincurr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [amphr]
    data.amphr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [batlevel]
    data.batlevel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [clearops]
    data.clearops = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [estop]
    data.estop = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motordisable]
    data.motordisable = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/stat';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e06faac651c5c8ab03e4e96be2620bbd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 speed
    int8 trip
    int8 time
    float32 maxspeed
    float32 minspeed
    float32 avgspeed
    float32 curspeed
    float32 totaltriptime
    float32 totaltripdist
    float32 avgvolt
    float32 avgcurr
    float32 maxcurr
    float32 mincurr
    float32 amphr
    float32 batlevel
    int16 clearops
    int16 estop
    int16 motordisable
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new stat(null);
    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    if (msg.trip !== undefined) {
      resolved.trip = msg.trip;
    }
    else {
      resolved.trip = 0
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0
    }

    if (msg.maxspeed !== undefined) {
      resolved.maxspeed = msg.maxspeed;
    }
    else {
      resolved.maxspeed = 0.0
    }

    if (msg.minspeed !== undefined) {
      resolved.minspeed = msg.minspeed;
    }
    else {
      resolved.minspeed = 0.0
    }

    if (msg.avgspeed !== undefined) {
      resolved.avgspeed = msg.avgspeed;
    }
    else {
      resolved.avgspeed = 0.0
    }

    if (msg.curspeed !== undefined) {
      resolved.curspeed = msg.curspeed;
    }
    else {
      resolved.curspeed = 0.0
    }

    if (msg.totaltriptime !== undefined) {
      resolved.totaltriptime = msg.totaltriptime;
    }
    else {
      resolved.totaltriptime = 0.0
    }

    if (msg.totaltripdist !== undefined) {
      resolved.totaltripdist = msg.totaltripdist;
    }
    else {
      resolved.totaltripdist = 0.0
    }

    if (msg.avgvolt !== undefined) {
      resolved.avgvolt = msg.avgvolt;
    }
    else {
      resolved.avgvolt = 0.0
    }

    if (msg.avgcurr !== undefined) {
      resolved.avgcurr = msg.avgcurr;
    }
    else {
      resolved.avgcurr = 0.0
    }

    if (msg.maxcurr !== undefined) {
      resolved.maxcurr = msg.maxcurr;
    }
    else {
      resolved.maxcurr = 0.0
    }

    if (msg.mincurr !== undefined) {
      resolved.mincurr = msg.mincurr;
    }
    else {
      resolved.mincurr = 0.0
    }

    if (msg.amphr !== undefined) {
      resolved.amphr = msg.amphr;
    }
    else {
      resolved.amphr = 0.0
    }

    if (msg.batlevel !== undefined) {
      resolved.batlevel = msg.batlevel;
    }
    else {
      resolved.batlevel = 0.0
    }

    if (msg.clearops !== undefined) {
      resolved.clearops = msg.clearops;
    }
    else {
      resolved.clearops = 0
    }

    if (msg.estop !== undefined) {
      resolved.estop = msg.estop;
    }
    else {
      resolved.estop = 0
    }

    if (msg.motordisable !== undefined) {
      resolved.motordisable = msg.motordisable;
    }
    else {
      resolved.motordisable = 0
    }

    return resolved;
    }
};

module.exports = stat;
