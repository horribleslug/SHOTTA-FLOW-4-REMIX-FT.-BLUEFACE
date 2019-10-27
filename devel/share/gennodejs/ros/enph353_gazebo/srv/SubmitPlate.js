// Auto-generated. Do not edit!

// (in-package enph353_gazebo.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SubmitPlateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.plate = null;
      this.location = null;
      this.legal = null;
      this.evidence = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
      if (initObj.hasOwnProperty('plate')) {
        this.plate = initObj.plate
      }
      else {
        this.plate = '';
      }
      if (initObj.hasOwnProperty('location')) {
        this.location = initObj.location
      }
      else {
        this.location = 0;
      }
      if (initObj.hasOwnProperty('legal')) {
        this.legal = initObj.legal
      }
      else {
        this.legal = false;
      }
      if (initObj.hasOwnProperty('evidence')) {
        this.evidence = initObj.evidence
      }
      else {
        this.evidence = new sensor_msgs.msg.Image();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SubmitPlateRequest
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    // Serialize message field [plate]
    bufferOffset = _serializer.string(obj.plate, buffer, bufferOffset);
    // Serialize message field [location]
    bufferOffset = _serializer.int8(obj.location, buffer, bufferOffset);
    // Serialize message field [legal]
    bufferOffset = _serializer.bool(obj.legal, buffer, bufferOffset);
    // Serialize message field [evidence]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.evidence, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SubmitPlateRequest
    let len;
    let data = new SubmitPlateRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [plate]
    data.plate = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [location]
    data.location = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [legal]
    data.legal = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [evidence]
    data.evidence = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.id.length;
    length += object.plate.length;
    length += sensor_msgs.msg.Image.getMessageSize(object.evidence);
    return length + 10;
  }

  static datatype() {
    // Returns string type for a service object
    return 'enph353_gazebo/SubmitPlateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '18d1e9687ce9e858dca913bb7abb560e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string id
    string plate
    int8 location
    bool legal
    sensor_msgs/Image evidence
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SubmitPlateRequest(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    if (msg.plate !== undefined) {
      resolved.plate = msg.plate;
    }
    else {
      resolved.plate = ''
    }

    if (msg.location !== undefined) {
      resolved.location = msg.location;
    }
    else {
      resolved.location = 0
    }

    if (msg.legal !== undefined) {
      resolved.legal = msg.legal;
    }
    else {
      resolved.legal = false
    }

    if (msg.evidence !== undefined) {
      resolved.evidence = sensor_msgs.msg.Image.Resolve(msg.evidence)
    }
    else {
      resolved.evidence = new sensor_msgs.msg.Image()
    }

    return resolved;
    }
};

class SubmitPlateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.correct = null;
    }
    else {
      if (initObj.hasOwnProperty('correct')) {
        this.correct = initObj.correct
      }
      else {
        this.correct = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SubmitPlateResponse
    // Serialize message field [correct]
    bufferOffset = _serializer.bool(obj.correct, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SubmitPlateResponse
    let len;
    let data = new SubmitPlateResponse(null);
    // Deserialize message field [correct]
    data.correct = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'enph353_gazebo/SubmitPlateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d7b90c75811aaad705aac4e2b606238';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool correct
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SubmitPlateResponse(null);
    if (msg.correct !== undefined) {
      resolved.correct = msg.correct;
    }
    else {
      resolved.correct = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SubmitPlateRequest,
  Response: SubmitPlateResponse,
  md5sum() { return 'd7a5f82904bfc83df9f5ede6fec81ad1'; },
  datatype() { return 'enph353_gazebo/SubmitPlate'; }
};
