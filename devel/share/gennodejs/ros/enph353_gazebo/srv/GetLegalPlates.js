// Auto-generated. Do not edit!

// (in-package enph353_gazebo.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetLegalPlatesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetLegalPlatesRequest
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetLegalPlatesRequest
    let len;
    let data = new GetLegalPlatesRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.id.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'enph353_gazebo/GetLegalPlatesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bbfcda76036ebbe3d36caf7af80b260c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetLegalPlatesRequest(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    return resolved;
    }
};

class GetLegalPlatesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.plates = null;
    }
    else {
      if (initObj.hasOwnProperty('plates')) {
        this.plates = initObj.plates
      }
      else {
        this.plates = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetLegalPlatesResponse
    // Serialize message field [plates]
    bufferOffset = _arraySerializer.string(obj.plates, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetLegalPlatesResponse
    let len;
    let data = new GetLegalPlatesResponse(null);
    // Deserialize message field [plates]
    data.plates = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.plates.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'enph353_gazebo/GetLegalPlatesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d1915dd30f27b4e3f4a835e33870df3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] plates
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetLegalPlatesResponse(null);
    if (msg.plates !== undefined) {
      resolved.plates = msg.plates;
    }
    else {
      resolved.plates = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetLegalPlatesRequest,
  Response: GetLegalPlatesResponse,
  md5sum() { return '6ea3d197527ea87828940965246c0b06'; },
  datatype() { return 'enph353_gazebo/GetLegalPlates'; }
};
