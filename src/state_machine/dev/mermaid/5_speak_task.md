```mermaid
stateDiagram-v2
    %% 状态定义
    IDLE: 0-IDLE 空闲
    NAVIGATE_TO_QR1: 1-导航至QR1
    ROTATE_TO_QR2: 2-原地旋转至QR2
    NAVIGATE_TO_QR_AREA: 3-导航至二维码区QR
    WAIT_FOR_QR_RESULT: 4-等待二维码识别结果
    SPEAK_TASK_TYPE: 5-播报任务类型
    NAVIGATE_TO_PICKING_AREA: 6-导航至拣货识别区
    ERROR: 99-错误状态
    
    %% 定义初始状态
    [*] --> IDLE: 初始化
    
    %% 主要状态转换流程
    IDLE --> NAVIGATE_TO_QR1: Event.START_CMD\n(语音唤醒或服务调用)
    NAVIGATE_TO_QR1 --> ROTATE_TO_QR2: Event.NAV_DONE_SUCCESS
    ROTATE_TO_QR2 --> NAVIGATE_TO_QR_AREA: Event.NAV_DONE_SUCCESS
    NAVIGATE_TO_QR_AREA --> WAIT_FOR_QR_RESULT: Event.NAV_DONE_SUCCESS
    
    %% 错误处理转换
    NAVIGATE_TO_QR1 --> ERROR: Event.NAV_DONE_FAILURE
    ROTATE_TO_QR2 --> ERROR: Event.NAV_DONE_FAILURE
    NAVIGATE_TO_QR_AREA --> ERROR: Event.NAV_DONE_FAILURE
    WAIT_FOR_QR_RESULT --> ERROR: Event.PERCEPTION_TIMEOUT
    
    %% 二维码结果处理和语音播报
    WAIT_FOR_QR_RESULT --> SPEAK_TASK_TYPE: Event.QR_RESULT_VALID\n解析内容并准备播报
    
    %% 播报成功后进入拣货区域状态
    SPEAK_TASK_TYPE --> NAVIGATE_TO_PICKING_AREA: Event.SPEAK_DONE
    
    %% 错误状态处理
    ERROR --> [*]: stop_all_activities()
    
    
    %% 语音播报说明
    note left of SPEAK_TASK_TYPE
        语音播报逻辑:
        1. 根据任务类型发送播报命令
        2. 设置播报超时(10秒)
        3. 等待播报完成事件
        4. 播报完成后进入导航到拣货区域状态
    end note
```