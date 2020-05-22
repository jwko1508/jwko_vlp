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

class min_seq {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.i = null;
      this.hori = null;
    }
    else {
      if (initObj.hasOwnProperty('i')) {
        this.i = initObj.i
      }
      else {
        this.i = 0;
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
    // Serializes a message object of type min_seq
    // Serialize message field [i]
    bufferOffset = _serializer.int16(obj.i, buffer, bufferOffset);
    // Serialize message field [hori]
    bufferOffset = _serializer.int16(obj.hori, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type min_seq
    let len;
    let data = new min_seq(null);
    // Deserialize message field [i]
    data.i = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [hori]
    data.hori = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pharos_vlp_tilt/min_seq';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '22324cdcfc8df707c9b750202d426c68';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 i
    int16 hori
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new min_seq(null);
    if (msg.i !== undefined) {
      resolved.i = msg.i;
    }
    else {
      resolved.i = 0
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

module.exports = min_seq;
