// Auto-generated. Do not edit!

// (in-package pharos_vlp_tilt.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let perfectarray = require('./perfectarray.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class vector_perfect_array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.one = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('one')) {
        this.one = initObj.one
      }
      else {
        this.one = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vector_perfect_array
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [one]
    // Serialize the length for message field [one]
    bufferOffset = _serializer.uint32(obj.one.length, buffer, bufferOffset);
    obj.one.forEach((val) => {
      bufferOffset = perfectarray.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vector_perfect_array
    let len;
    let data = new vector_perfect_array(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [one]
    // Deserialize array length for message field [one]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.one = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.one[i] = perfectarray.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.one.forEach((val) => {
      length += perfectarray.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pharos_vlp_tilt/vector_perfect_array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57e47adbe1b2b913a3385ad439b88169';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    perfectarray[] one
    
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
    MSG: pharos_vlp_tilt/perfectarray
    std_msgs/Header header
    perfect[] objects
    min_seq[16] min_seq
    max_seq[16] max_seq
    center_position center
    center_position min_center
    center_position max_center
    point min_object
    point max_object
    int32 min_hori
    int32 max_hori
    ================================================================================
    MSG: pharos_vlp_tilt/perfect
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
    
    ================================================================================
    MSG: pharos_vlp_tilt/min_seq
    int16 i
    int16 hori
    
    ================================================================================
    MSG: pharos_vlp_tilt/max_seq
    int16 i
    int16 hori
    
    ================================================================================
    MSG: pharos_vlp_tilt/center_position
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vector_perfect_array(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.one !== undefined) {
      resolved.one = new Array(msg.one.length);
      for (let i = 0; i < resolved.one.length; ++i) {
        resolved.one[i] = perfectarray.Resolve(msg.one[i]);
      }
    }
    else {
      resolved.one = []
    }

    return resolved;
    }
};

module.exports = vector_perfect_array;
