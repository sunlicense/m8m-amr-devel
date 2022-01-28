// Auto-generated. Do not edit!

// (in-package ntuc_fleet.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let task = require('./task.js');

//-----------------------------------------------------------

class jobs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.time = null;
      this.tasks = null;
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
      if (initObj.hasOwnProperty('tasks')) {
        this.tasks = initObj.tasks
      }
      else {
        this.tasks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type jobs
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.time(obj.time, buffer, bufferOffset);
    // Serialize message field [tasks]
    // Serialize the length for message field [tasks]
    bufferOffset = _serializer.uint32(obj.tasks.length, buffer, bufferOffset);
    obj.tasks.forEach((val) => {
      bufferOffset = task.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type jobs
    let len;
    let data = new jobs(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [tasks]
    // Deserialize array length for message field [tasks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tasks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tasks[i] = task.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 13 * object.tasks.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ntuc_fleet/jobs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd7b1ebb1ce6d0c3bb74e63fcb9b9e5cb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ##################################################
    ##### Message type: jobs.msg
    ##### Use in Topics: /rq_jobs, /rv_jobs
    ##################################################
    int8 cmd				# command code	 
    time time			# time queue is updated
    task[] tasks		# list of task.msg (of type task.msg)
    
    ##################################################
    ## cmd		command code
    ## time		time in time.secs and and time.nsecs 
    ## task[]	list of pending tasks (of type task.msg)
    ##################################################
    
    ================================================================================
    MSG: ntuc_fleet/task
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
    const resolved = new jobs(null);
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

    if (msg.tasks !== undefined) {
      resolved.tasks = new Array(msg.tasks.length);
      for (let i = 0; i < resolved.tasks.length; ++i) {
        resolved.tasks[i] = task.Resolve(msg.tasks[i]);
      }
    }
    else {
      resolved.tasks = []
    }

    return resolved;
    }
};

module.exports = jobs;
