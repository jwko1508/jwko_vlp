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

class state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_ground = null;
      this.is_del = null;
      this.is_infect = null;
    }
    else {
      if (initObj.hasOwnProperty('is_ground')) {
        this.is_ground = initObj.is_ground
      }
      else {
        this.is_ground = 0;
      }
      if (initObj.hasOwnProperty('is_del')) {
        this.is_del = initObj.is_del
      }
      else {
        this.is_del = 0;
      }
      if (initObj.hasOwnProperty('is_infect')) {
        this.is_infect = initObj.is_infect
      }
      else {
        this.is_infect = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type state
    // Serialize message field [is_ground]
    bufferOffset = _serializer.int32(obj.is_ground, buffer, bufferOffset);
    // Serialize message field [is_del]
    bufferOffset = _serializer.int32(obj.is_del, buffer, bufferOffset);
    // Serialize message field [is_infect]
    bufferOffset = _serializer.int32(obj.is_infect, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type state
    let len;
    let data = new state(null);
    // Deserialize message field [is_ground]
    data.is_ground = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [is_del]
    data.is_del = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [is_infect]
    data.is_infect = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pharos_vlp_tilt/state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4dde323cb233595cceb5a9451b77b1b4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 is_ground
    int32 is_del
    int32 is_infect
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new state(null);
    if (msg.is_ground !== undefined) {
      resolved.is_ground = msg.is_ground;
    }
    else {
      resolved.is_ground = 0
    }

    if (msg.is_del !== undefined) {
      resolved.is_del = msg.is_del;
    }
    else {
      resolved.is_del = 0
    }

    if (msg.is_infect !== undefined) {
      resolved.is_infect = msg.is_infect;
    }
    else {
      resolved.is_infect = 0
    }

    return resolved;
    }
};

module.exports = state;
