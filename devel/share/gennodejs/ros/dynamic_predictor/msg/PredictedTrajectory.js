// Auto-generated. Do not edit!

// (in-package dynamic_predictor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class PredictedTrajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.probability = null;
      this.mean = null;
      this.variance = null;
    }
    else {
      if (initObj.hasOwnProperty('probability')) {
        this.probability = initObj.probability
      }
      else {
        this.probability = 0.0;
      }
      if (initObj.hasOwnProperty('mean')) {
        this.mean = initObj.mean
      }
      else {
        this.mean = [];
      }
      if (initObj.hasOwnProperty('variance')) {
        this.variance = initObj.variance
      }
      else {
        this.variance = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PredictedTrajectory
    // Serialize message field [probability]
    bufferOffset = _serializer.float64(obj.probability, buffer, bufferOffset);
    // Serialize message field [mean]
    // Serialize the length for message field [mean]
    bufferOffset = _serializer.uint32(obj.mean.length, buffer, bufferOffset);
    obj.mean.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [variance]
    // Serialize the length for message field [variance]
    bufferOffset = _serializer.uint32(obj.variance.length, buffer, bufferOffset);
    obj.variance.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PredictedTrajectory
    let len;
    let data = new PredictedTrajectory(null);
    // Deserialize message field [probability]
    data.probability = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mean]
    // Deserialize array length for message field [mean]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.mean = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.mean[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [variance]
    // Deserialize array length for message field [variance]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.variance = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.variance[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.mean.length;
    length += 24 * object.variance.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dynamic_predictor/PredictedTrajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '48bdf4c0064d9378b7fbef39e2f807e0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Probability weight of this mode
    float64 probability
    
    # Mean trajectory for this mode (length = horizon)
    geometry_msgs/Point[] mean
    
    # Variance per time step (same length as mean)
    # Variance is ordered as var(x), var(y), var(z)
    geometry_msgs/Vector3[] variance
    
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new PredictedTrajectory(null);
    if (msg.probability !== undefined) {
      resolved.probability = msg.probability;
    }
    else {
      resolved.probability = 0.0
    }

    if (msg.mean !== undefined) {
      resolved.mean = new Array(msg.mean.length);
      for (let i = 0; i < resolved.mean.length; ++i) {
        resolved.mean[i] = geometry_msgs.msg.Point.Resolve(msg.mean[i]);
      }
    }
    else {
      resolved.mean = []
    }

    if (msg.variance !== undefined) {
      resolved.variance = new Array(msg.variance.length);
      for (let i = 0; i < resolved.variance.length; ++i) {
        resolved.variance[i] = geometry_msgs.msg.Vector3.Resolve(msg.variance[i]);
      }
    }
    else {
      resolved.variance = []
    }

    return resolved;
    }
};

module.exports = PredictedTrajectory;
