// Auto-generated. Do not edit!

// (in-package pharos_vlp_tilt.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let VehiclePose = require('./VehiclePose.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class VehiclePoseArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.vehicles = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('vehicles')) {
        this.vehicles = initObj.vehicles
      }
      else {
        this.vehicles = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VehiclePoseArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [vehicles]
    // Serialize the length for message field [vehicles]
    bufferOffset = _serializer.uint32(obj.vehicles.length, buffer, bufferOffset);
    obj.vehicles.forEach((val) => {
      bufferOffset = VehiclePose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VehiclePoseArray
    let len;
    let data = new VehiclePoseArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [vehicles]
    // Deserialize array length for message field [vehicles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.vehicles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.vehicles[i] = VehiclePose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 32 * object.vehicles.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pharos_vlp_tilt/VehiclePoseArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '75b3ae63766b70da2ab55a64ee96561f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    VehiclePose[] vehicles
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: pharos_vlp_tilt/VehiclePose
    float64 x
    float64 y
    float64 theta
    std_msgs/Time stamp
    ================================================================================
    MSG: std_msgs/Time
    time data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VehiclePoseArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.vehicles !== undefined) {
      resolved.vehicles = new Array(msg.vehicles.length);
      for (let i = 0; i < resolved.vehicles.length; ++i) {
        resolved.vehicles[i] = VehiclePose.Resolve(msg.vehicles[i]);
      }
    }
    else {
      resolved.vehicles = []
    }

    return resolved;
    }
};

module.exports = VehiclePoseArray;
