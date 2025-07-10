;; Auto-generated. Do not edit!


(when (boundp 'xf_mic_asr_offline::VoiceCmd)
  (if (not (find-package "XF_MIC_ASR_OFFLINE"))
    (make-package "XF_MIC_ASR_OFFLINE"))
  (shadow 'VoiceCmd (find-package "XF_MIC_ASR_OFFLINE")))
(unless (find-package "XF_MIC_ASR_OFFLINE::VOICECMD")
  (make-package "XF_MIC_ASR_OFFLINE::VOICECMD"))
(unless (find-package "XF_MIC_ASR_OFFLINE::VOICECMDREQUEST")
  (make-package "XF_MIC_ASR_OFFLINE::VOICECMDREQUEST"))
(unless (find-package "XF_MIC_ASR_OFFLINE::VOICECMDRESPONSE")
  (make-package "XF_MIC_ASR_OFFLINE::VOICECMDRESPONSE"))

(in-package "ROS")





(defclass xf_mic_asr_offline::VoiceCmdRequest
  :super ros::object
  :slots (_voice_cmd ))

(defmethod xf_mic_asr_offline::VoiceCmdRequest
  (:init
   (&key
    ((:voice_cmd __voice_cmd) "")
    )
   (send-super :init)
   (setq _voice_cmd (string __voice_cmd))
   self)
  (:voice_cmd
   (&optional __voice_cmd)
   (if __voice_cmd (setq _voice_cmd __voice_cmd)) _voice_cmd)
  (:serialization-length
   ()
   (+
    ;; string _voice_cmd
    4 (length _voice_cmd)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _voice_cmd
       (write-long (length _voice_cmd) s) (princ _voice_cmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _voice_cmd
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _voice_cmd (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xf_mic_asr_offline::VoiceCmdResponse
  :super ros::object
  :slots (_success _message ))

(defmethod xf_mic_asr_offline::VoiceCmdResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xf_mic_asr_offline::VoiceCmd
  :super ros::object
  :slots ())

(setf (get xf_mic_asr_offline::VoiceCmd :md5sum-) "db201c99548e0ca4c062a24b1bddb118")
(setf (get xf_mic_asr_offline::VoiceCmd :datatype-) "xf_mic_asr_offline/VoiceCmd")
(setf (get xf_mic_asr_offline::VoiceCmd :request) xf_mic_asr_offline::VoiceCmdRequest)
(setf (get xf_mic_asr_offline::VoiceCmd :response) xf_mic_asr_offline::VoiceCmdResponse)

(defmethod xf_mic_asr_offline::VoiceCmdRequest
  (:response () (instance xf_mic_asr_offline::VoiceCmdResponse :init)))

(setf (get xf_mic_asr_offline::VoiceCmdRequest :md5sum-) "db201c99548e0ca4c062a24b1bddb118")
(setf (get xf_mic_asr_offline::VoiceCmdRequest :datatype-) "xf_mic_asr_offline/VoiceCmdRequest")
(setf (get xf_mic_asr_offline::VoiceCmdRequest :definition-)
      "# 请求部分 - 指定语音指令
string voice_cmd
---
# 响应部分 - 返回播放结果
bool success
string message
")

(setf (get xf_mic_asr_offline::VoiceCmdResponse :md5sum-) "db201c99548e0ca4c062a24b1bddb118")
(setf (get xf_mic_asr_offline::VoiceCmdResponse :datatype-) "xf_mic_asr_offline/VoiceCmdResponse")
(setf (get xf_mic_asr_offline::VoiceCmdResponse :definition-)
      "# 请求部分 - 指定语音指令
string voice_cmd
---
# 响应部分 - 返回播放结果
bool success
string message
")



(provide :xf_mic_asr_offline/VoiceCmd "db201c99548e0ca4c062a24b1bddb118")


