// Auto-generated. Do not edit!

// (in-package pharos_vlp_tilt.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let info = require('./info.js');
let point = require('./point.js');
let state = require('./state.js');

//-----------------------------------------------------------

class perfect {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.info = null;
      this.point = null;
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('info')) {
        this.info = initObj.info
      }
      else {
        this.info = new info();
      }
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new point();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = new state();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type perfect
    // Serialize message field [info]
    bufferOffset = info.serialize(obj.info, buffer, bufferOffset);
    // Serialize message field [point]
    bufferOffset = point.serialize(obj.point, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = state.serialize(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type perfect
    let len;
    let data = new perfect(null);
    // Deserialize message field [info]
    data.info = info.deserialize(buffer, bufferOffset);
    // Deserialize message field [point]
    data.point = point.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = state.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pharos_vlp_tilt/perfect';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '90a56c4e8308c1352b958efc8367b00b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    info info
    point point
    state state
    
    ================================================================================
    MSG: pharos_vlp_tilt/info
    int32 laser
    int32 hori
    
    ================================================================================
    MSG: pharos_vlp_tilt/point
    float64 x
    float64 y
    float64 z
    float64 intensity
    
    ================================================================================
    MSG: pharos_vlp_tilt/state
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
    const resolved = new perfect(null);
    if (msg.info !== undefined) {
      resolved.info = info.Resolve(msg.info)
    }
    else {
      resolved.info = new info()
    }

    if (msg.point !== undefined) {
      resolved.point = point.Resolve(msg.point)
    }
    else {
      resolved.point = new point()
    }

    if (msg.state !== undefined) {
      resolved.state = state.Resolve(msg.state)
    }
    else {
      resolved.state = new state()
    }

    return resolved;
    }
};

module.exports = perfect;
