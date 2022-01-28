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

class lift {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.cfloor = null;
      this.dfloor = null;
      this.inuse = null;
      this.dooropen = null;
      this.doorclose = null;
      this.goingup = null;
      this.goingdown = null;
      this.stationary = null;
      this.serviceavail = null;
      this.epower = null;
      this.fireservice = null;
      this.beacon = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = 0;
      }
      if (initObj.hasOwnProperty('cfloor')) {
        this.cfloor = initObj.cfloor
      }
      else {
        this.cfloor = 0;
      }
      if (initObj.hasOwnProperty('dfloor')) {
        this.dfloor = initObj.dfloor
      }
      else {
        this.dfloor = 0;
      }
      if (initObj.hasOwnProperty('inuse')) {
        this.inuse = initObj.inuse
      }
      else {
        this.inuse = 0;
      }
      if (initObj.hasOwnProperty('dooropen')) {
        this.dooropen = initObj.dooropen
      }
      else {
        this.dooropen = 0;
      }
      if (initObj.hasOwnProperty('doorclose')) {
        this.doorclose = initObj.doorclose
      }
      else {
        this.doorclose = 0;
      }
      if (initObj.hasOwnProperty('goingup')) {
        this.goingup = initObj.goingup
      }
      else {
        this.goingup = 0;
      }
      if (initObj.hasOwnProperty('goingdown')) {
        this.goingdown = initObj.goingdown
      }
      else {
        this.goingdown = 0;
      }
      if (initObj.hasOwnProperty('stationary')) {
        this.stationary = initObj.stationary
      }
      else {
        this.stationary = 0;
      }
      if (initObj.hasOwnProperty('serviceavail')) {
        this.serviceavail = initObj.serviceavail
      }
      else {
        this.serviceavail = 0;
      }
      if (initObj.hasOwnProperty('epower')) {
        this.epower = initObj.epower
      }
      else {
        this.epower = 0;
      }
      if (initObj.hasOwnProperty('fireservice')) {
        this.fireservice = initObj.fireservice
      }
      else {
        this.fireservice = 0;
      }
      if (initObj.hasOwnProperty('beacon')) {
        this.beacon = initObj.beacon
      }
      else {
        this.beacon = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lift
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [cfloor]
    bufferOffset = _serializer.int8(obj.cfloor, buffer, bufferOffset);
    // Serialize message field [dfloor]
    bufferOffset = _serializer.int8(obj.dfloor, buffer, bufferOffset);
    // Serialize message field [inuse]
    bufferOffset = _serializer.int8(obj.inuse, buffer, bufferOffset);
    // Serialize message field [dooropen]
    bufferOffset = _serializer.int8(obj.dooropen, buffer, bufferOffset);
    // Serialize message field [doorclose]
    bufferOffset = _serializer.int8(obj.doorclose, buffer, bufferOffset);
    // Serialize message field [goingup]
    bufferOffset = _serializer.int8(obj.goingup, buffer, bufferOffset);
    // Serialize message field [goingdown]
    bufferOffset = _serializer.int8(obj.goingdown, buffer, bufferOffset);
    // Serialize message field [stationary]
    bufferOffset = _serializer.int8(obj.stationary, buffer, bufferOffset);
    // Serialize message field [serviceavail]
    bufferOffset = _serializer.int8(obj.serviceavail, buffer, bufferOffset);
    // Serialize message field [epower]
    bufferOffset = _serializer.int8(obj.epower, buffer, bufferOffset);
    // Serialize message field [fireservice]
    bufferOffset = _serializer.int8(obj.fireservice, buffer, bufferOffset);
    // Serialize message field [beacon]
    bufferOffset = _serializer.int8(obj.beacon, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lift
    let len;
    let data = new lift(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [cfloor]
    data.cfloor = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [dfloor]
    data.dfloor = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [inuse]
    data.inuse = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [dooropen]
    data.dooropen = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [doorclose]
    data.doorclose = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [goingup]
    data.goingup = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [goingdown]
    data.goingdown = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stationary]
    data.stationary = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [serviceavail]
    data.serviceavail = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [epower]
    data.epower = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [fireservice]
    data.fireservice = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [beacon]
    data.beacon = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'htbot/lift';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e5880338d440c98c58ce479e42b36bed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ##################################################
    ##### Message type: lift.msg
    ##### Use in Topics: /to_lift, /fr_lift
    ##################################################
    int8 cmd			# command code			[0-102]
    int8 cfloor			# start of trip [1-11] or current car level [1-11]
    int8 dfloor			# end level of trip	[1-11]
    int8 inuse			# request for in_use activation [0/1] or in_use status [0/1]
    int8 dooropen		# door open status 	[0/1]
    int8 doorclose		# door close status 	[0/1]
    int8 goingup		# going up status 	[0/1]
    int8 goingdown		# going down status 	[0/1]
    int8 stationary		# stationary status 	[0/1]
    int8 serviceavail	# service available status	[0/1]
    int8 epower			# emergency power status 	[0/1]
    int8 fireservice	# fire service status 		[0/1]
    int8 beacon			# beacon light alarm 		[0/1], 1 to trigger, 0 to turn off
    
    ##################################################
    ##### cmd value usage:
    ##### 0:carCallStartLevel, 1:enteredLift, 2:carCallEndLevel, 3:exitedLift
    ##### 8:beacon, 9:inUse, 10:okay2Enter, 11:okay2Exit
    ##### 20:inUse, 21:doorOpen, 22:doorClose, 23:goingUp, 24:goingDown, 25:stationary 
    ##### 40:carLevel, 100:serviceAvailable, 101: emergencyPower, 102:fireServiceAlarm
    ##################################################
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lift(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = 0
    }

    if (msg.cfloor !== undefined) {
      resolved.cfloor = msg.cfloor;
    }
    else {
      resolved.cfloor = 0
    }

    if (msg.dfloor !== undefined) {
      resolved.dfloor = msg.dfloor;
    }
    else {
      resolved.dfloor = 0
    }

    if (msg.inuse !== undefined) {
      resolved.inuse = msg.inuse;
    }
    else {
      resolved.inuse = 0
    }

    if (msg.dooropen !== undefined) {
      resolved.dooropen = msg.dooropen;
    }
    else {
      resolved.dooropen = 0
    }

    if (msg.doorclose !== undefined) {
      resolved.doorclose = msg.doorclose;
    }
    else {
      resolved.doorclose = 0
    }

    if (msg.goingup !== undefined) {
      resolved.goingup = msg.goingup;
    }
    else {
      resolved.goingup = 0
    }

    if (msg.goingdown !== undefined) {
      resolved.goingdown = msg.goingdown;
    }
    else {
      resolved.goingdown = 0
    }

    if (msg.stationary !== undefined) {
      resolved.stationary = msg.stationary;
    }
    else {
      resolved.stationary = 0
    }

    if (msg.serviceavail !== undefined) {
      resolved.serviceavail = msg.serviceavail;
    }
    else {
      resolved.serviceavail = 0
    }

    if (msg.epower !== undefined) {
      resolved.epower = msg.epower;
    }
    else {
      resolved.epower = 0
    }

    if (msg.fireservice !== undefined) {
      resolved.fireservice = msg.fireservice;
    }
    else {
      resolved.fireservice = 0
    }

    if (msg.beacon !== undefined) {
      resolved.beacon = msg.beacon;
    }
    else {
      resolved.beacon = 0
    }

    return resolved;
    }
};

module.exports = lift;
