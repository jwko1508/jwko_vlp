// Auto-generated. Do not edit!

// (in-package pharos_vlp_tilt.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let perfect = require('./perfect.js');
let min_seq = require('./min_seq.js');
let max_seq = require('./max_seq.js');
let center_position = require('./center_position.js');
let point = require('./point.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class perfectarray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.objects = null;
      this.min_seq = null;
      this.max_seq = null;
      this.center = null;
      this.min_center = null;
      this.max_center = null;
      this.min_object = null;
      this.max_object = null;
      this.min_hori = null;
      this.max_hori = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('objects')) {
        this.objects = initObj.objects
      }
      else {
        this.objects = [];
      }
      if (initObj.hasOwnProperty('min_seq')) {
        this.min_seq = initObj.min_seq
      }
      else {
        this.min_seq = new Array(16).fill(new min_seq());
      }
      if (initObj.hasOwnProperty('max_seq')) {
        this.max_seq = initObj.max_seq
      }
      else {
        this.max_seq = new Array(16).fill(new max_seq());
      }
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new center_position();
      }
      if (initObj.hasOwnProperty('min_center')) {
        this.min_center = initObj.min_center
      }
      else {
        this.min_center = new center_position();
      }
      if (initObj.hasOwnProperty('max_center')) {
        this.max_center = initObj.max_center
      }
      else {
        this.max_center = new center_position();
      }
      if (initObj.hasOwnProperty('min_object')) {
        this.min_object = initObj.min_object
      }
      else {
        this.min_object = new point();
      }
      if (initObj.hasOwnProperty('max_object')) {
        this.max_object = initObj.max_object
      }
      else {
        this.max_object = new point();
      }
      if (initObj.hasOwnProperty('min_hori')) {
        this.min_hori = initObj.min_hori
      }
      else {
        this.min_hori = 0;
      }
      if (initObj.hasOwnProperty('max_hori')) {
        this.max_hori = initObj.max_hori
      }
      else {
        this.max_hori = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type perfectarray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [objects]
    // Serialize the length for message field [objects]
    bufferOffset = _serializer.uint32(obj.objects.length, buffer, bufferOffset);
    obj.objects.forEach((val) => {
      bufferOffset = perfect.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [min_seq] has the right length
    if (obj.min_seq.length !== 16) {
      throw new Error('Unable to serialize array field min_seq - length must be 16')
    }
    // Serialize message field [min_seq]
    obj.min_seq.forEach((val) => {
      bufferOffset = min_seq.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [max_seq] has the right length
    if (obj.max_seq.length !== 16) {
      throw new Error('Unable to serialize array field max_seq - length must be 16')
    }
    // Serialize message field [max_seq]
    obj.max_seq.forEach((val) => {
      bufferOffset = max_seq.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [center]
    bufferOffset = center_position.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [min_center]
    bufferOffset = center_position.serialize(obj.min_center, buffer, bufferOffset);
    // Serialize message field [max_center]
    bufferOffset = center_position.serialize(obj.max_center, buffer, bufferOffset);
    // Serialize message field [min_object]
    bufferOffset = point.serialize(obj.min_object, buffer, bufferOffset);
    // Serialize message field [max_object]
    bufferOffset = point.serialize(obj.max_object, buffer, bufferOffset);
    // Serialize message field [min_hori]
    bufferOffset = _serializer.int32(obj.min_hori, buffer, bufferOffset);
    // Serialize message field [max_hori]
    bufferOffset = _serializer.int32(obj.max_hori, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type perfectarray
    let len;
    let data = new perfectarray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [objects]
    // Deserialize array length for message field [objects]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.objects = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.objects[i] = perfect.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [min_seq]
    len = 16;
    data.min_seq = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.min_seq[i] = min_seq.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [max_seq]
    len = 16;
    data.max_seq = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.max_seq[i] = max_seq.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [center]
    data.center = center_position.deserialize(buffer, bufferOffset);
    // Deserialize message field [min_center]
    data.min_center = center_position.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_center]
    data.max_center = center_position.deserialize(buffer, bufferOffset);
    // Deserialize message field [min_object]
    data.min_object = point.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_object]
    data.max_object = point.deserialize(buffer, bufferOffset);
    // Deserialize message field [min_hori]
    data.min_hori = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [max_hori]
    data.max_hori = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 52 * object.objects.length;
    return length + 276;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pharos_vlp_tilt/perfectarray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '393d188248af978f922d9cd672182dbf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new perfectarray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.objects !== undefined) {
      resolved.objects = new Array(msg.objects.length);
      for (let i = 0; i < resolved.objects.length; ++i) {
        resolved.objects[i] = perfect.Resolve(msg.objects[i]);
      }
    }
    else {
      resolved.objects = []
    }

    if (msg.min_seq !== undefined) {
      resolved.min_seq = new Array(16)
      for (let i = 0; i < resolved.min_seq.length; ++i) {
        if (msg.min_seq.length > i) {
          resolved.min_seq[i] = min_seq.Resolve(msg.min_seq[i]);
        }
        else {
          resolved.min_seq[i] = new min_seq();
        }
      }
    }
    else {
      resolved.min_seq = new Array(16).fill(new min_seq())
    }

    if (msg.max_seq !== undefined) {
      resolved.max_seq = new Array(16)
      for (let i = 0; i < resolved.max_seq.length; ++i) {
        if (msg.max_seq.length > i) {
          resolved.max_seq[i] = max_seq.Resolve(msg.max_seq[i]);
        }
        else {
          resolved.max_seq[i] = new max_seq();
        }
      }
    }
    else {
      resolved.max_seq = new Array(16).fill(new max_seq())
    }

    if (msg.center !== undefined) {
      resolved.center = center_position.Resolve(msg.center)
    }
    else {
      resolved.center = new center_position()
    }

    if (msg.min_center !== undefined) {
      resolved.min_center = center_position.Resolve(msg.min_center)
    }
    else {
      resolved.min_center = new center_position()
    }

    if (msg.max_center !== undefined) {
      resolved.max_center = center_position.Resolve(msg.max_center)
    }
    else {
      resolved.max_center = new center_position()
    }

    if (msg.min_object !== undefined) {
      resolved.min_object = point.Resolve(msg.min_object)
    }
    else {
      resolved.min_object = new point()
    }

    if (msg.max_object !== undefined) {
      resolved.max_object = point.Resolve(msg.max_object)
    }
    else {
      resolved.max_object = new point()
    }

    if (msg.min_hori !== undefined) {
      resolved.min_hori = msg.min_hori;
    }
    else {
      resolved.min_hori = 0
    }

    if (msg.max_hori !== undefined) {
      resolved.max_hori = msg.max_hori;
    }
    else {
      resolved.max_hori = 0
    }

    return resolved;
    }
};

module.exports = perfectarray;
