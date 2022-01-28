// Auto-generated. Do not edit!

// (in-package ntuc_fleet.msg)


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
    return 'ntuc_fleet/task';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd9335104b860e9530a386a51f33ebb59';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ##################################################
    ##### Message type: task.msg
    ##### Use in Topics: /fr_station, /fr_fleet, /fr_agv
    ##################################################
    int8 cmd		# command code 
    time time		# time task is queued
    int8 type		# different trolley type
    int8 fromLP		# starting LP
    int8 toLP		# ending LP
    int8 alloc		# 0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2
    
    ##################################################
    ## cmd		1:new task, 2:assigned task, 3:cancel task
    ## time		time.secs and time.nsecs (ROS time)
    ## type		1:Bian Marie, 2:Food Warmer, 3:Linen
    ## fromLP	70:L1LRm3-9, 73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,
    ##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward9, 
    ##				10:L10StaffDorm, 11:L11Ktchen,
    ##				51:L1MainStore, 52:L1NonHalalPrepRoom
    ## toLP		73:L1LRm3, 74:L1LRm4, 75:L1LRm5, 76:L1LRm6, 77:L1LRm7, 78:L1LRm8, 79:L1LRm9,
    ##				3:L3Ward3, 4:L4Ward4, 5:L5Ward5, 6:L6Ward6, 7:L7Ward7, 8:L8Ward8, 9:L9Ward,
    ##				10: L10StaffDorm, 11:L11Ktchen,
    ##				51:L1MainStore, 52:L1NonHalalPrepRoom
    ## alloc	0:not allocated, 1:allocated to AGV1, 2:allocated to AGV2
    ##################################################	
    ## LP Range:: 	2-49: Lift Area, 50-69: Lobby Area, 70-100: Laundry Area 
    ##################################################	
    
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
