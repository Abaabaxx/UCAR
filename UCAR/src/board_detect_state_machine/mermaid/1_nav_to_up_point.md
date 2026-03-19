```mermaid
stateDiagram-v2
    %% 状态定义
    IDLE: 0-IDLE 空闲
    NAVIGATE_TO_UP_POINT: 1-导航至上半平面点
    SEARCH_UP_BOARD: 2-寻找上平面板子
    ERROR: 99-错误状态

    %% 定义初始状态
    [*] --> IDLE: 节点初始化

    %% 状态转换
    IDLE --> NAVIGATE_TO_UP_POINT: Event.START_CMD\n(服务调用)

    NAVIGATE_TO_UP_POINT --> SEARCH_UP_BOARD: Event.NAV_DONE_SUCCESS\n(move_base 导航成功)
    NAVIGATE_TO_UP_POINT --> ERROR: Event.NAV_DONE_FAILURE\n(move_base 导航失败)

    %% 流程终点说明
    note right of SEARCH_UP_BOARD
        成功路径终点 (当前阶段):
        - 打印日志
        - 等待后续功能扩展
    end note
    
    note left of ERROR
        失败路径终点:
        - 打印错误日志
        - 停止所有活动
    end note
```