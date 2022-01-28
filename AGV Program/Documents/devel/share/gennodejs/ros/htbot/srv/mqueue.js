// Auto-generated. Do not edit!

// (in-package htbot.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class mqueueRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.LP = null;
      this.GN = null;
      this.gps = null;
      this.lps = null;
      this.pw = null;
      this.tx = null;
      this.ty = null;
      this.tz = null;
      this.rx = null;
      this.ry = null;
      this.rz = null;
      this.rw = null;
      this.prd = null;
      this.pra = null;
      this.psd = null;
      this.psa = null;
      this.prd1 = null;
      this.pra1 = null;
      this.psd1 = null;
      this.psa1 = null;
      this.align = null;
      this.autostart = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = 0;
      }
      if (initObj.hasOwnProperty('LP')) {
        this.LP = initObj.LP
      }
      else {
        this.LP = 0;
      }
      if (initObj.hasOwnProperty('GN')) {
        this.GN = initObj.GN
      }
      else {
        this.GN = 0;
      }
      if (initObj.hasOwnProperty('gps')) {
        this.gps = initObj.gps
      }
      else {
        this.gps = '';
      }
      if (initObj.hasOwnProperty('lps')) {
        this.lps = initObj.lps
      }
      else {
        this.lps = '';
      }
      if (initObj.hasOwnProperty('pw')) {
        this.pw = initObj.pw
      }
      else {
        this.pw = '';
      }
      if (initObj.hasOwnProperty('tx')) {
        this.tx = initObj.tx
      }
      else {
        this.tx = 0.0;
      }
      if (initObj.hasOwnProperty('ty')) {
        this.ty = initObj.ty
      }
      else {
        this.ty = 0.0;
      }
      if (initObj.hasOwnProperty('tz')) {
        this.tz = initObj.tz
      }
      else {
        this.tz = 0.0;
      }
      if (initObj.hasOwnProperty('rx')) {
        this.rx = initObj.rx
      }
      else {
        this.rx = 0.0;
      }
      if (initObj.hasOwnProperty('ry')) {
        this.ry = initObj.ry
      }
      else {
        this.ry = 0.0;
      }
      if (initObj.hasOwnProperty('rz')) {
        this.rz = initObj.rz
      }
      else {
        this.rz = 0.0;
      }
      if (initObj.hasOwnProperty('rw')) {
        this.rw = initObj.rw
      }
      else {
        this.rw = 0.0;
      }
      if (initObj.hasOwnProperty('prd')) {
        this.prd = initObj.prd
      }
      else {
        this.prd = 0.0;
      }
      if (initObj.hasOwnProperty('pra')) {
        this.pra = initObj.pra
      }
      else {
        this.pra = 0.0;
      }
      if (initObj.hasOwnProperty('psd')) {
        this.psd = initObj.psd
      }
      else {
        this.psd = 0.0;
      }
      if (initObj.hasOwnProperty('psa')) {
        this.psa = initObj.psa
      }
      else {
        this.psa = 0.0;
      }
      if (initObj.hasOwnProperty('prd1')) {
        this.prd1 = initObj.prd1
      }
      else {
        this.prd1 = 0.0;
      }
      if (initObj.hasOwnProperty('pra1')) {
        this.pra1 = initObj.pra1
      }
      else {
        this.pra1 = 0.0;
      }
      if (initObj.hasOwnProperty('psd1')) {
        this.psd1 = initObj.psd1
      }
      else {
        this.psd1 = 0.0;
      }
      if (initObj.hasOwnProperty('psa1')) {
        this.psa1 = initObj.psa1
      }
      else {
        this.psa1 = 0.0;
      }
      if (initObj.hasOwnProperty('align')) {
        this.align = initObj.align
      }
      else {
        this.align = 0.0;
      }
      if (initObj.hasOwnProperty('autostart')) {
        this.autostart = initObj.autostart
      }
      else {
        this.autostart = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mqueueRequest
    // Serialize message field [cmd]
    bufferOffset = _serializer.int8(obj.cmd, buffer, bufferOffset);
    // Serialize message field [LP]
    bufferOffset = _serializer.int8(obj.LP, buffer, bufferOffset);
    // Serialize message field [GN]
    bufferOffset = _serializer.int8(obj.GN, buffer, bufferOffset);
    // Serialize message field [gps]
    bufferOffset = _serializer.string(obj.gps, buffer, bufferOffset);
    // Serialize message field [lps]
    bufferOffset = _serializer.string(obj.lps, buffer, bufferOffset);
    // Serialize message field [pw]
    bufferOffset = _serializer.string(obj.pw, buffer, bufferOffset);
    // Serialize message field [tx]
    bufferOffset = _serializer.float32(obj.tx, buffer, bufferOffset);
    // Serialize message field [ty]
    bufferOffset = _serializer.float32(obj.ty, buffer, bufferOffset);
    // Serialize message field [tz]
    bufferOffset = _serializer.float32(obj.tz, buffer, bufferOffset);
    // Serialize message field [rx]
    bufferOffset = _serializer.float32(obj.rx, buffer, bufferOffset);
    // Serialize message field [ry]
    bufferOffset = _serializer.float32(obj.ry, buffer, bufferOffset);
    // Serialize message field [rz]
    bufferOffset = _serializer.float32(obj.rz, buffer, bufferOffset);
    // Serialize message field [rw]
    bufferOffset = _serializer.float32(obj.rw, buffer, bufferOffset);
    // Serialize message field [prd]
    bufferOffset = _serializer.float32(obj.prd, buffer, bufferOffset);
    // Serialize message field [pra]
    bufferOffset = _serializer.float32(obj.pra, buffer, bufferOffset);
    // Serialize message field [psd]
    bufferOffset = _serializer.float32(obj.psd, buffer, bufferOffset);
    // Serialize message field [psa]
    bufferOffset = _serializer.float32(obj.psa, buffer, bufferOffset);
    // Serialize message field [prd1]
    bufferOffset = _serializer.float32(obj.prd1, buffer, bufferOffset);
    // Serialize message field [pra1]
    bufferOffset = _serializer.float32(obj.pra1, buffer, bufferOffset);
    // Serialize message field [psd1]
    bufferOffset = _serializer.float32(obj.psd1, buffer, bufferOffset);
    // Serialize message field [psa1]
    bufferOffset = _serializer.float32(obj.psa1, buffer, bufferOffset);
    // Serialize message field [align]
    bufferOffset = _serializer.float32(obj.align, buffer, bufferOffset);
    // Serialize message field [autostart]
    bufferOffset = _serializer.float32(obj.autostart, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mqueueRequest
    let len;
    let data = new mqueueRequest(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [LP]
    data.LP = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [GN]
    data.GN = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [gps]
    data.gps = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lps]
    data.lps = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pw]
    data.pw = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [tx]
    data.tx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ty]
    data.ty = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tz]
    data.tz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rx]
    data.rx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ry]
    data.ry = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rz]
    data.rz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rw]
    data.rw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [prd]
    data.prd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pra]
    data.pra = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psd]
    data.psd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psa]
    data.psa = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [prd1]
    data.prd1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pra1]
    data.pra1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psd1]
    data.psd1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psa1]
    data.psa1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [align]
    data.align = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [autostart]
    data.autostart = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.gps.length;
    length += object.lps.length;
    length += object.pw.length;
    return length + 83;
  }

  static datatype() {
    // Returns string type for a service object
    return 'htbot/mqueueRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e4cb468e5ed01acf5ef7706e07bd3b67';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 cmd
    int8 LP
    int8 GN
    string gps
    string lps
    string pw
    float32 tx
    float32 ty
    float32 tz
    float32 rx
    float32 ry
    float32 rz
    float32 rw
    float32 prd
    float32 pra
    float32 psd
    float32 psa
    float32 prd1
    float32 pra1
    float32 psd1
    float32 psa1
    float32 align
    float32 autostart
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mqueueRequest(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = 0
    }

    if (msg.LP !== undefined) {
      resolved.LP = msg.LP;
    }
    else {
      resolved.LP = 0
    }

    if (msg.GN !== undefined) {
      resolved.GN = msg.GN;
    }
    else {
      resolved.GN = 0
    }

    if (msg.gps !== undefined) {
      resolved.gps = msg.gps;
    }
    else {
      resolved.gps = ''
    }

    if (msg.lps !== undefined) {
      resolved.lps = msg.lps;
    }
    else {
      resolved.lps = ''
    }

    if (msg.pw !== undefined) {
      resolved.pw = msg.pw;
    }
    else {
      resolved.pw = ''
    }

    if (msg.tx !== undefined) {
      resolved.tx = msg.tx;
    }
    else {
      resolved.tx = 0.0
    }

    if (msg.ty !== undefined) {
      resolved.ty = msg.ty;
    }
    else {
      resolved.ty = 0.0
    }

    if (msg.tz !== undefined) {
      resolved.tz = msg.tz;
    }
    else {
      resolved.tz = 0.0
    }

    if (msg.rx !== undefined) {
      resolved.rx = msg.rx;
    }
    else {
      resolved.rx = 0.0
    }

    if (msg.ry !== undefined) {
      resolved.ry = msg.ry;
    }
    else {
      resolved.ry = 0.0
    }

    if (msg.rz !== undefined) {
      resolved.rz = msg.rz;
    }
    else {
      resolved.rz = 0.0
    }

    if (msg.rw !== undefined) {
      resolved.rw = msg.rw;
    }
    else {
      resolved.rw = 0.0
    }

    if (msg.prd !== undefined) {
      resolved.prd = msg.prd;
    }
    else {
      resolved.prd = 0.0
    }

    if (msg.pra !== undefined) {
      resolved.pra = msg.pra;
    }
    else {
      resolved.pra = 0.0
    }

    if (msg.psd !== undefined) {
      resolved.psd = msg.psd;
    }
    else {
      resolved.psd = 0.0
    }

    if (msg.psa !== undefined) {
      resolved.psa = msg.psa;
    }
    else {
      resolved.psa = 0.0
    }

    if (msg.prd1 !== undefined) {
      resolved.prd1 = msg.prd1;
    }
    else {
      resolved.prd1 = 0.0
    }

    if (msg.pra1 !== undefined) {
      resolved.pra1 = msg.pra1;
    }
    else {
      resolved.pra1 = 0.0
    }

    if (msg.psd1 !== undefined) {
      resolved.psd1 = msg.psd1;
    }
    else {
      resolved.psd1 = 0.0
    }

    if (msg.psa1 !== undefined) {
      resolved.psa1 = msg.psa1;
    }
    else {
      resolved.psa1 = 0.0
    }

    if (msg.align !== undefined) {
      resolved.align = msg.align;
    }
    else {
      resolved.align = 0.0
    }

    if (msg.autostart !== undefined) {
      resolved.autostart = msg.autostart;
    }
    else {
      resolved.autostart = 0.0
    }

    return resolved;
    }
};

class mqueueResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
      this.tx = null;
      this.ty = null;
      this.tz = null;
      this.rx = null;
      this.ry = null;
      this.rz = null;
      this.rw = null;
      this.prd = null;
      this.pra = null;
      this.psd = null;
      this.psa = null;
      this.prd1 = null;
      this.pra1 = null;
      this.psd1 = null;
      this.psa1 = null;
      this.LP = null;
      this.cLP = null;
      this.nGP = null;
      this.cGN = null;
      this.nIQ = null;
      this.marked = null;
      this.gps = null;
      this.lps = null;
      this.lps1 = null;
      this.lps2 = null;
      this.lps3 = null;
      this.lps4 = null;
      this.lps5 = null;
      this.lps6 = null;
      this.lps7 = null;
      this.lps8 = null;
      this.marked1 = null;
      this.marked2 = null;
      this.marked3 = null;
      this.marked4 = null;
      this.marked5 = null;
      this.marked6 = null;
      this.marked7 = null;
      this.marked8 = null;
      this.align = null;
      this.autostart = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('tx')) {
        this.tx = initObj.tx
      }
      else {
        this.tx = 0.0;
      }
      if (initObj.hasOwnProperty('ty')) {
        this.ty = initObj.ty
      }
      else {
        this.ty = 0.0;
      }
      if (initObj.hasOwnProperty('tz')) {
        this.tz = initObj.tz
      }
      else {
        this.tz = 0.0;
      }
      if (initObj.hasOwnProperty('rx')) {
        this.rx = initObj.rx
      }
      else {
        this.rx = 0.0;
      }
      if (initObj.hasOwnProperty('ry')) {
        this.ry = initObj.ry
      }
      else {
        this.ry = 0.0;
      }
      if (initObj.hasOwnProperty('rz')) {
        this.rz = initObj.rz
      }
      else {
        this.rz = 0.0;
      }
      if (initObj.hasOwnProperty('rw')) {
        this.rw = initObj.rw
      }
      else {
        this.rw = 0.0;
      }
      if (initObj.hasOwnProperty('prd')) {
        this.prd = initObj.prd
      }
      else {
        this.prd = 0.0;
      }
      if (initObj.hasOwnProperty('pra')) {
        this.pra = initObj.pra
      }
      else {
        this.pra = 0.0;
      }
      if (initObj.hasOwnProperty('psd')) {
        this.psd = initObj.psd
      }
      else {
        this.psd = 0.0;
      }
      if (initObj.hasOwnProperty('psa')) {
        this.psa = initObj.psa
      }
      else {
        this.psa = 0.0;
      }
      if (initObj.hasOwnProperty('prd1')) {
        this.prd1 = initObj.prd1
      }
      else {
        this.prd1 = 0.0;
      }
      if (initObj.hasOwnProperty('pra1')) {
        this.pra1 = initObj.pra1
      }
      else {
        this.pra1 = 0.0;
      }
      if (initObj.hasOwnProperty('psd1')) {
        this.psd1 = initObj.psd1
      }
      else {
        this.psd1 = 0.0;
      }
      if (initObj.hasOwnProperty('psa1')) {
        this.psa1 = initObj.psa1
      }
      else {
        this.psa1 = 0.0;
      }
      if (initObj.hasOwnProperty('LP')) {
        this.LP = initObj.LP
      }
      else {
        this.LP = 0;
      }
      if (initObj.hasOwnProperty('cLP')) {
        this.cLP = initObj.cLP
      }
      else {
        this.cLP = 0;
      }
      if (initObj.hasOwnProperty('nGP')) {
        this.nGP = initObj.nGP
      }
      else {
        this.nGP = 0;
      }
      if (initObj.hasOwnProperty('cGN')) {
        this.cGN = initObj.cGN
      }
      else {
        this.cGN = 0;
      }
      if (initObj.hasOwnProperty('nIQ')) {
        this.nIQ = initObj.nIQ
      }
      else {
        this.nIQ = 0;
      }
      if (initObj.hasOwnProperty('marked')) {
        this.marked = initObj.marked
      }
      else {
        this.marked = 0;
      }
      if (initObj.hasOwnProperty('gps')) {
        this.gps = initObj.gps
      }
      else {
        this.gps = '';
      }
      if (initObj.hasOwnProperty('lps')) {
        this.lps = initObj.lps
      }
      else {
        this.lps = '';
      }
      if (initObj.hasOwnProperty('lps1')) {
        this.lps1 = initObj.lps1
      }
      else {
        this.lps1 = '';
      }
      if (initObj.hasOwnProperty('lps2')) {
        this.lps2 = initObj.lps2
      }
      else {
        this.lps2 = '';
      }
      if (initObj.hasOwnProperty('lps3')) {
        this.lps3 = initObj.lps3
      }
      else {
        this.lps3 = '';
      }
      if (initObj.hasOwnProperty('lps4')) {
        this.lps4 = initObj.lps4
      }
      else {
        this.lps4 = '';
      }
      if (initObj.hasOwnProperty('lps5')) {
        this.lps5 = initObj.lps5
      }
      else {
        this.lps5 = '';
      }
      if (initObj.hasOwnProperty('lps6')) {
        this.lps6 = initObj.lps6
      }
      else {
        this.lps6 = '';
      }
      if (initObj.hasOwnProperty('lps7')) {
        this.lps7 = initObj.lps7
      }
      else {
        this.lps7 = '';
      }
      if (initObj.hasOwnProperty('lps8')) {
        this.lps8 = initObj.lps8
      }
      else {
        this.lps8 = '';
      }
      if (initObj.hasOwnProperty('marked1')) {
        this.marked1 = initObj.marked1
      }
      else {
        this.marked1 = 0;
      }
      if (initObj.hasOwnProperty('marked2')) {
        this.marked2 = initObj.marked2
      }
      else {
        this.marked2 = 0;
      }
      if (initObj.hasOwnProperty('marked3')) {
        this.marked3 = initObj.marked3
      }
      else {
        this.marked3 = 0;
      }
      if (initObj.hasOwnProperty('marked4')) {
        this.marked4 = initObj.marked4
      }
      else {
        this.marked4 = 0;
      }
      if (initObj.hasOwnProperty('marked5')) {
        this.marked5 = initObj.marked5
      }
      else {
        this.marked5 = 0;
      }
      if (initObj.hasOwnProperty('marked6')) {
        this.marked6 = initObj.marked6
      }
      else {
        this.marked6 = 0;
      }
      if (initObj.hasOwnProperty('marked7')) {
        this.marked7 = initObj.marked7
      }
      else {
        this.marked7 = 0;
      }
      if (initObj.hasOwnProperty('marked8')) {
        this.marked8 = initObj.marked8
      }
      else {
        this.marked8 = 0;
      }
      if (initObj.hasOwnProperty('align')) {
        this.align = initObj.align
      }
      else {
        this.align = 0.0;
      }
      if (initObj.hasOwnProperty('autostart')) {
        this.autostart = initObj.autostart
      }
      else {
        this.autostart = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mqueueResponse
    // Serialize message field [status]
    bufferOffset = _serializer.int8(obj.status, buffer, bufferOffset);
    // Serialize message field [tx]
    bufferOffset = _serializer.float32(obj.tx, buffer, bufferOffset);
    // Serialize message field [ty]
    bufferOffset = _serializer.float32(obj.ty, buffer, bufferOffset);
    // Serialize message field [tz]
    bufferOffset = _serializer.float32(obj.tz, buffer, bufferOffset);
    // Serialize message field [rx]
    bufferOffset = _serializer.float32(obj.rx, buffer, bufferOffset);
    // Serialize message field [ry]
    bufferOffset = _serializer.float32(obj.ry, buffer, bufferOffset);
    // Serialize message field [rz]
    bufferOffset = _serializer.float32(obj.rz, buffer, bufferOffset);
    // Serialize message field [rw]
    bufferOffset = _serializer.float32(obj.rw, buffer, bufferOffset);
    // Serialize message field [prd]
    bufferOffset = _serializer.float32(obj.prd, buffer, bufferOffset);
    // Serialize message field [pra]
    bufferOffset = _serializer.float32(obj.pra, buffer, bufferOffset);
    // Serialize message field [psd]
    bufferOffset = _serializer.float32(obj.psd, buffer, bufferOffset);
    // Serialize message field [psa]
    bufferOffset = _serializer.float32(obj.psa, buffer, bufferOffset);
    // Serialize message field [prd1]
    bufferOffset = _serializer.float32(obj.prd1, buffer, bufferOffset);
    // Serialize message field [pra1]
    bufferOffset = _serializer.float32(obj.pra1, buffer, bufferOffset);
    // Serialize message field [psd1]
    bufferOffset = _serializer.float32(obj.psd1, buffer, bufferOffset);
    // Serialize message field [psa1]
    bufferOffset = _serializer.float32(obj.psa1, buffer, bufferOffset);
    // Serialize message field [LP]
    bufferOffset = _serializer.int8(obj.LP, buffer, bufferOffset);
    // Serialize message field [cLP]
    bufferOffset = _serializer.int8(obj.cLP, buffer, bufferOffset);
    // Serialize message field [nGP]
    bufferOffset = _serializer.int8(obj.nGP, buffer, bufferOffset);
    // Serialize message field [cGN]
    bufferOffset = _serializer.int8(obj.cGN, buffer, bufferOffset);
    // Serialize message field [nIQ]
    bufferOffset = _serializer.int8(obj.nIQ, buffer, bufferOffset);
    // Serialize message field [marked]
    bufferOffset = _serializer.int8(obj.marked, buffer, bufferOffset);
    // Serialize message field [gps]
    bufferOffset = _serializer.string(obj.gps, buffer, bufferOffset);
    // Serialize message field [lps]
    bufferOffset = _serializer.string(obj.lps, buffer, bufferOffset);
    // Serialize message field [lps1]
    bufferOffset = _serializer.string(obj.lps1, buffer, bufferOffset);
    // Serialize message field [lps2]
    bufferOffset = _serializer.string(obj.lps2, buffer, bufferOffset);
    // Serialize message field [lps3]
    bufferOffset = _serializer.string(obj.lps3, buffer, bufferOffset);
    // Serialize message field [lps4]
    bufferOffset = _serializer.string(obj.lps4, buffer, bufferOffset);
    // Serialize message field [lps5]
    bufferOffset = _serializer.string(obj.lps5, buffer, bufferOffset);
    // Serialize message field [lps6]
    bufferOffset = _serializer.string(obj.lps6, buffer, bufferOffset);
    // Serialize message field [lps7]
    bufferOffset = _serializer.string(obj.lps7, buffer, bufferOffset);
    // Serialize message field [lps8]
    bufferOffset = _serializer.string(obj.lps8, buffer, bufferOffset);
    // Serialize message field [marked1]
    bufferOffset = _serializer.int8(obj.marked1, buffer, bufferOffset);
    // Serialize message field [marked2]
    bufferOffset = _serializer.int8(obj.marked2, buffer, bufferOffset);
    // Serialize message field [marked3]
    bufferOffset = _serializer.int8(obj.marked3, buffer, bufferOffset);
    // Serialize message field [marked4]
    bufferOffset = _serializer.int8(obj.marked4, buffer, bufferOffset);
    // Serialize message field [marked5]
    bufferOffset = _serializer.int8(obj.marked5, buffer, bufferOffset);
    // Serialize message field [marked6]
    bufferOffset = _serializer.int8(obj.marked6, buffer, bufferOffset);
    // Serialize message field [marked7]
    bufferOffset = _serializer.int8(obj.marked7, buffer, bufferOffset);
    // Serialize message field [marked8]
    bufferOffset = _serializer.int8(obj.marked8, buffer, bufferOffset);
    // Serialize message field [align]
    bufferOffset = _serializer.float32(obj.align, buffer, bufferOffset);
    // Serialize message field [autostart]
    bufferOffset = _serializer.float32(obj.autostart, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mqueueResponse
    let len;
    let data = new mqueueResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [tx]
    data.tx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ty]
    data.ty = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tz]
    data.tz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rx]
    data.rx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ry]
    data.ry = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rz]
    data.rz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rw]
    data.rw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [prd]
    data.prd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pra]
    data.pra = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psd]
    data.psd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psa]
    data.psa = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [prd1]
    data.prd1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pra1]
    data.pra1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psd1]
    data.psd1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psa1]
    data.psa1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [LP]
    data.LP = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [cLP]
    data.cLP = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [nGP]
    data.nGP = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [cGN]
    data.cGN = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [nIQ]
    data.nIQ = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [marked]
    data.marked = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [gps]
    data.gps = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lps]
    data.lps = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lps1]
    data.lps1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lps2]
    data.lps2 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lps3]
    data.lps3 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lps4]
    data.lps4 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lps5]
    data.lps5 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lps6]
    data.lps6 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lps7]
    data.lps7 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lps8]
    data.lps8 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [marked1]
    data.marked1 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [marked2]
    data.marked2 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [marked3]
    data.marked3 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [marked4]
    data.marked4 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [marked5]
    data.marked5 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [marked6]
    data.marked6 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [marked7]
    data.marked7 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [marked8]
    data.marked8 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [align]
    data.align = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [autostart]
    data.autostart = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.gps.length;
    length += object.lps.length;
    length += object.lps1.length;
    length += object.lps2.length;
    length += object.lps3.length;
    length += object.lps4.length;
    length += object.lps5.length;
    length += object.lps6.length;
    length += object.lps7.length;
    length += object.lps8.length;
    return length + 123;
  }

  static datatype() {
    // Returns string type for a service object
    return 'htbot/mqueueResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a6a4649df65d8320dbceeee83af7bd77';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 status
    float32 tx
    float32 ty
    float32 tz
    float32 rx
    float32 ry
    float32 rz
    float32 rw
    float32 prd
    float32 pra
    float32 psd
    float32 psa
    float32 prd1
    float32 pra1
    float32 psd1
    float32 psa1
    int8 LP
    int8 cLP
    int8 nGP
    int8 cGN
    int8 nIQ
    int8 marked
    string gps
    string lps
    string lps1
    string lps2
    string lps3
    string lps4
    string lps5
    string lps6
    string lps7
    string lps8
    int8 marked1
    int8 marked2
    int8 marked3
    int8 marked4
    int8 marked5
    int8 marked6
    int8 marked7
    int8 marked8
    float32 align
    float32 autostart
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mqueueResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.tx !== undefined) {
      resolved.tx = msg.tx;
    }
    else {
      resolved.tx = 0.0
    }

    if (msg.ty !== undefined) {
      resolved.ty = msg.ty;
    }
    else {
      resolved.ty = 0.0
    }

    if (msg.tz !== undefined) {
      resolved.tz = msg.tz;
    }
    else {
      resolved.tz = 0.0
    }

    if (msg.rx !== undefined) {
      resolved.rx = msg.rx;
    }
    else {
      resolved.rx = 0.0
    }

    if (msg.ry !== undefined) {
      resolved.ry = msg.ry;
    }
    else {
      resolved.ry = 0.0
    }

    if (msg.rz !== undefined) {
      resolved.rz = msg.rz;
    }
    else {
      resolved.rz = 0.0
    }

    if (msg.rw !== undefined) {
      resolved.rw = msg.rw;
    }
    else {
      resolved.rw = 0.0
    }

    if (msg.prd !== undefined) {
      resolved.prd = msg.prd;
    }
    else {
      resolved.prd = 0.0
    }

    if (msg.pra !== undefined) {
      resolved.pra = msg.pra;
    }
    else {
      resolved.pra = 0.0
    }

    if (msg.psd !== undefined) {
      resolved.psd = msg.psd;
    }
    else {
      resolved.psd = 0.0
    }

    if (msg.psa !== undefined) {
      resolved.psa = msg.psa;
    }
    else {
      resolved.psa = 0.0
    }

    if (msg.prd1 !== undefined) {
      resolved.prd1 = msg.prd1;
    }
    else {
      resolved.prd1 = 0.0
    }

    if (msg.pra1 !== undefined) {
      resolved.pra1 = msg.pra1;
    }
    else {
      resolved.pra1 = 0.0
    }

    if (msg.psd1 !== undefined) {
      resolved.psd1 = msg.psd1;
    }
    else {
      resolved.psd1 = 0.0
    }

    if (msg.psa1 !== undefined) {
      resolved.psa1 = msg.psa1;
    }
    else {
      resolved.psa1 = 0.0
    }

    if (msg.LP !== undefined) {
      resolved.LP = msg.LP;
    }
    else {
      resolved.LP = 0
    }

    if (msg.cLP !== undefined) {
      resolved.cLP = msg.cLP;
    }
    else {
      resolved.cLP = 0
    }

    if (msg.nGP !== undefined) {
      resolved.nGP = msg.nGP;
    }
    else {
      resolved.nGP = 0
    }

    if (msg.cGN !== undefined) {
      resolved.cGN = msg.cGN;
    }
    else {
      resolved.cGN = 0
    }

    if (msg.nIQ !== undefined) {
      resolved.nIQ = msg.nIQ;
    }
    else {
      resolved.nIQ = 0
    }

    if (msg.marked !== undefined) {
      resolved.marked = msg.marked;
    }
    else {
      resolved.marked = 0
    }

    if (msg.gps !== undefined) {
      resolved.gps = msg.gps;
    }
    else {
      resolved.gps = ''
    }

    if (msg.lps !== undefined) {
      resolved.lps = msg.lps;
    }
    else {
      resolved.lps = ''
    }

    if (msg.lps1 !== undefined) {
      resolved.lps1 = msg.lps1;
    }
    else {
      resolved.lps1 = ''
    }

    if (msg.lps2 !== undefined) {
      resolved.lps2 = msg.lps2;
    }
    else {
      resolved.lps2 = ''
    }

    if (msg.lps3 !== undefined) {
      resolved.lps3 = msg.lps3;
    }
    else {
      resolved.lps3 = ''
    }

    if (msg.lps4 !== undefined) {
      resolved.lps4 = msg.lps4;
    }
    else {
      resolved.lps4 = ''
    }

    if (msg.lps5 !== undefined) {
      resolved.lps5 = msg.lps5;
    }
    else {
      resolved.lps5 = ''
    }

    if (msg.lps6 !== undefined) {
      resolved.lps6 = msg.lps6;
    }
    else {
      resolved.lps6 = ''
    }

    if (msg.lps7 !== undefined) {
      resolved.lps7 = msg.lps7;
    }
    else {
      resolved.lps7 = ''
    }

    if (msg.lps8 !== undefined) {
      resolved.lps8 = msg.lps8;
    }
    else {
      resolved.lps8 = ''
    }

    if (msg.marked1 !== undefined) {
      resolved.marked1 = msg.marked1;
    }
    else {
      resolved.marked1 = 0
    }

    if (msg.marked2 !== undefined) {
      resolved.marked2 = msg.marked2;
    }
    else {
      resolved.marked2 = 0
    }

    if (msg.marked3 !== undefined) {
      resolved.marked3 = msg.marked3;
    }
    else {
      resolved.marked3 = 0
    }

    if (msg.marked4 !== undefined) {
      resolved.marked4 = msg.marked4;
    }
    else {
      resolved.marked4 = 0
    }

    if (msg.marked5 !== undefined) {
      resolved.marked5 = msg.marked5;
    }
    else {
      resolved.marked5 = 0
    }

    if (msg.marked6 !== undefined) {
      resolved.marked6 = msg.marked6;
    }
    else {
      resolved.marked6 = 0
    }

    if (msg.marked7 !== undefined) {
      resolved.marked7 = msg.marked7;
    }
    else {
      resolved.marked7 = 0
    }

    if (msg.marked8 !== undefined) {
      resolved.marked8 = msg.marked8;
    }
    else {
      resolved.marked8 = 0
    }

    if (msg.align !== undefined) {
      resolved.align = msg.align;
    }
    else {
      resolved.align = 0.0
    }

    if (msg.autostart !== undefined) {
      resolved.autostart = msg.autostart;
    }
    else {
      resolved.autostart = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: mqueueRequest,
  Response: mqueueResponse,
  md5sum() { return '8063f205c633b3b5b9493f7d4b8b6575'; },
  datatype() { return 'htbot/mqueue'; }
};
