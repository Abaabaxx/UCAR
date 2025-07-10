; Auto-generated. Do not edit!


(cl:in-package xf_mic_asr_offline-srv)


;//! \htmlinclude VoiceCmd-request.msg.html

(cl:defclass <VoiceCmd-request> (roslisp-msg-protocol:ros-message)
  ((voice_cmd
    :reader voice_cmd
    :initarg :voice_cmd
    :type cl:string
    :initform ""))
)

(cl:defclass VoiceCmd-request (<VoiceCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VoiceCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VoiceCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xf_mic_asr_offline-srv:<VoiceCmd-request> is deprecated: use xf_mic_asr_offline-srv:VoiceCmd-request instead.")))

(cl:ensure-generic-function 'voice_cmd-val :lambda-list '(m))
(cl:defmethod voice_cmd-val ((m <VoiceCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xf_mic_asr_offline-srv:voice_cmd-val is deprecated.  Use xf_mic_asr_offline-srv:voice_cmd instead.")
  (voice_cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VoiceCmd-request>) ostream)
  "Serializes a message object of type '<VoiceCmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'voice_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'voice_cmd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VoiceCmd-request>) istream)
  "Deserializes a message object of type '<VoiceCmd-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'voice_cmd) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'voice_cmd) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VoiceCmd-request>)))
  "Returns string type for a service object of type '<VoiceCmd-request>"
  "xf_mic_asr_offline/VoiceCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VoiceCmd-request)))
  "Returns string type for a service object of type 'VoiceCmd-request"
  "xf_mic_asr_offline/VoiceCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VoiceCmd-request>)))
  "Returns md5sum for a message object of type '<VoiceCmd-request>"
  "db201c99548e0ca4c062a24b1bddb118")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VoiceCmd-request)))
  "Returns md5sum for a message object of type 'VoiceCmd-request"
  "db201c99548e0ca4c062a24b1bddb118")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VoiceCmd-request>)))
  "Returns full string definition for message of type '<VoiceCmd-request>"
  (cl:format cl:nil "# 请求部分 - 指定语音指令~%string voice_cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VoiceCmd-request)))
  "Returns full string definition for message of type 'VoiceCmd-request"
  (cl:format cl:nil "# 请求部分 - 指定语音指令~%string voice_cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VoiceCmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'voice_cmd))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VoiceCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'VoiceCmd-request
    (cl:cons ':voice_cmd (voice_cmd msg))
))
;//! \htmlinclude VoiceCmd-response.msg.html

(cl:defclass <VoiceCmd-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass VoiceCmd-response (<VoiceCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VoiceCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VoiceCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xf_mic_asr_offline-srv:<VoiceCmd-response> is deprecated: use xf_mic_asr_offline-srv:VoiceCmd-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <VoiceCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xf_mic_asr_offline-srv:success-val is deprecated.  Use xf_mic_asr_offline-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <VoiceCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xf_mic_asr_offline-srv:message-val is deprecated.  Use xf_mic_asr_offline-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VoiceCmd-response>) ostream)
  "Serializes a message object of type '<VoiceCmd-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VoiceCmd-response>) istream)
  "Deserializes a message object of type '<VoiceCmd-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VoiceCmd-response>)))
  "Returns string type for a service object of type '<VoiceCmd-response>"
  "xf_mic_asr_offline/VoiceCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VoiceCmd-response)))
  "Returns string type for a service object of type 'VoiceCmd-response"
  "xf_mic_asr_offline/VoiceCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VoiceCmd-response>)))
  "Returns md5sum for a message object of type '<VoiceCmd-response>"
  "db201c99548e0ca4c062a24b1bddb118")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VoiceCmd-response)))
  "Returns md5sum for a message object of type 'VoiceCmd-response"
  "db201c99548e0ca4c062a24b1bddb118")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VoiceCmd-response>)))
  "Returns full string definition for message of type '<VoiceCmd-response>"
  (cl:format cl:nil "# 响应部分 - 返回播放结果~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VoiceCmd-response)))
  "Returns full string definition for message of type 'VoiceCmd-response"
  (cl:format cl:nil "# 响应部分 - 返回播放结果~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VoiceCmd-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VoiceCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'VoiceCmd-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'VoiceCmd)))
  'VoiceCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'VoiceCmd)))
  'VoiceCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VoiceCmd)))
  "Returns string type for a service object of type '<VoiceCmd>"
  "xf_mic_asr_offline/VoiceCmd")