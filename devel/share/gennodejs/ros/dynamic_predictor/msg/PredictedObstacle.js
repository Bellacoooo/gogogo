// Auto-generated. Do not edit!

// (in-package dynamic_predictor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PredictedTrajectory = require('./PredictedTrajectory.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class PredictedObstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.current_position = null;
      this.current_velocity = null;
      this.modes = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('current_position')) {
        this.current_position = initObj.current_position
      }
      else {
        this.current_position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('current_velocity')) {
        this.current_velocity = initObj.current_velocity
      }
      else {
        this.current_velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('modes')) {
        this.modes = initObj.modes
      }
      else {
        this.modes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PredictedObstacle
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [current_position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.current_position, buffer, bufferOffset);
    // Serialize message field [current_velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.current_velocity, buffer, bufferOffset);
    // Serialize message field [modes]
    // Serialize the length for message field [modes]
    bufferOffset = _serializer.uint32(obj.modes.length, buffer, bufferOffset);
    obj.modes.forEach((val) => {
      bufferOffset = PredictedTrajectory.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PredictedObstacle
    let len;
    let data = new PredictedObstacle(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [current_position]
    data.current_position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [current_velocity]
    data.current_velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [modes]
    // Deserialize array length for message field [modes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.modes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.modes[i] = PredictedTrajectory.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.modes.forEach((val) => {
      length += PredictedTrajectory.getMessageSize(val);
    });
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dynamic_predictor/PredictedObstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6b1bfccd711ab582d4c8a979685459e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Obstacle identifier from tracker
    int32 id
    
    geometry_msgs/Point current_position
    geometry_msgs/Vector3 current_velocity
    
    # All modal trajectories for this obstacle
    dynamic_predictor/PredictedTrajectory[] modes
    
    
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
    ================================================================================
    MSG: dynamic_predictor/PredictedTrajectory
    # Probability weight of this mode
    float64 probability
    
    # Mean trajectory for this mode (length = horizon)
    geometry_msgs/Point[] mean
    
    # Variance per time step (same length as mean)
    # Variance is ordered as var(x), var(y), var(z)
    geometry_msgs/Vector3[] variance
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PredictedObstacle(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.current_position !== undefined) {
      resolved.current_position = geometry_msgs.msg.Point.Resolve(msg.current_position)
    }
    else {
      resolved.current_position = new geometry_msgs.msg.Point()
    }

    if (msg.current_velocity !== undefined) {
      resolved.current_velocity = geometry_msgs.msg.Vector3.Resolve(msg.current_velocity)
    }
    else {
      resolved.current_velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.modes !== undefined) {
      resolved.modes = new Array(msg.modes.length);
      for (let i = 0; i < resolved.modes.length; ++i) {
        resolved.modes[i] = PredictedTrajectory.Resolve(msg.modes[i]);
      }
    }
    else {
      resolved.modes = []
    }

    return resolved;
    }
};

module.exports = PredictedObstacle;
