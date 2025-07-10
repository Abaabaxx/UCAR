// Auto-generated. Do not edit!

// (in-package xf_mic_asr_offline.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class VoiceCmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.voice_cmd = null;
    }
    else {
      if (initObj.hasOwnProperty('voice_cmd')) {
        this.voice_cmd = initObj.voice_cmd
      }
      else {
        this.voice_cmd = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VoiceCmdRequest
    // Serialize message field [voice_cmd]
    bufferOffset = _serializer.string(obj.voice_cmd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VoiceCmdRequest
    let len;
    let data = new VoiceCmdRequest(null);
    // Deserialize message field [voice_cmd]
    data.voice_cmd = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.voice_cmd.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xf_mic_asr_offline/VoiceCmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b7806d69c897ffd9ad7c1697602fec40';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 请求部分 - 指定语音指令
    string voice_cmd
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VoiceCmdRequest(null);
    if (msg.voice_cmd !== undefined) {
      resolved.voice_cmd = msg.voice_cmd;
    }
    else {
      resolved.voice_cmd = ''
    }

    return resolved;
    }
};

class VoiceCmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VoiceCmdResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VoiceCmdResponse
    let len;
    let data = new VoiceCmdResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.message.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xf_mic_asr_offline/VoiceCmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 响应部分 - 返回播放结果
    bool success
    string message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VoiceCmdResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: VoiceCmdRequest,
  Response: VoiceCmdResponse,
  md5sum() { return 'db201c99548e0ca4c062a24b1bddb118'; },
  datatype() { return 'xf_mic_asr_offline/VoiceCmd'; }
};
