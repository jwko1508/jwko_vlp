// Auto-generated. Do not edit!

// (in-package pharos_vlp_tilt.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.laser = null;
      this.hori = null;
    }
    else {
      if (initObj.hasOwnProperty('laser')) {
        this.laser = initObj.laser
      }
      else {
        this.laser = 0;
      }
      if (initObj.hasOwnProperty('hori')) {
        this.hori = initObj.hori
      }
      else {
        this.hori = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type info
    // Serialize message field [laser]
    bufferOffset = _serializer.int32(obj.laser, buffer, bufferOffset);
    // Serialize message field [hori]
    bufferOffset = _serializer.int32(obj.hori, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type info
    let len;
    let data = new info(null);
    // Deserialize message field [laser]
    data.laser = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [hori]
    data.hori = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pharos_vlp_tilt/info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e2ba7373c9fec63a00e8b05a56280e35';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 laser
    int32 hori
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new info(null);
    if (msg.laser !== undefined) {
      resolved.laser = msg.laser;
    }
    else {
      resolved.laser = 0
    }

    if (msg.hori !== undefined) {
      resolved.hori = msg.hori;
    }
    else {
      resolved.hori = 0
    }

    return resolved;
    }
};

module.exports = info;
