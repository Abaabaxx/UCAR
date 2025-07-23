```mermaid
stateDiagram-v2
    %% 模式: 常规
    %% AI模型: Gemini
    %% 目的: 展示扩展后的状态机逻辑

    %% --- 状态定义 ---
    IDLE: 0-IDLE 空闲
    NAVIGATE_TO_UP_POINT: 1-导航至上平面点
    SEARCH_UP_BOARD: 2-寻找上平面板子
    NAVIGATE_TO_DOWN_POINT: 3-导航至下平面点
    SEARCH_DOWN_BOARD: 4-寻找下平面板子
    PICK_UP_GOODS: 5-巡检并识别货物
    SPEAK_GOODS: 6-播报找到的货物
    NAV_TO_SIMULATION: 7-导航至仿真区
    DO_SIMULATION_TASKS: 8-执行仿真任务
    SPEAK_SIMULATION_GOODS: 9-仿真完成并停留
    ERROR: 99-错误状态

    %% --- 流程开始 ---
    [*] --> IDLE: 节点初始化

    %% --- 主要任务流程 ---
    IDLE --> NAVIGATE_TO_UP_POINT: Event.START_CMD
    NAVIGATE_TO_UP_POINT --> SEARCH_UP_BOARD: Event.NAV_DONE_SUCCESS
    NAVIGATE_TO_UP_POINT --> ERROR: Event.NAV_DONE_FAILURE
    SEARCH_UP_BOARD --> NAVIGATE_TO_DOWN_POINT: Event.SEARCH_DONE_SUCCESS
    SEARCH_UP_BOARD --> ERROR: Event.SEARCH_DONE_FAILURE
    NAVIGATE_TO_DOWN_POINT --> SEARCH_DOWN_BOARD: Event.NAV_DONE_SUCCESS
    NAVIGATE_TO_DOWN_POINT --> ERROR: Event.NAV_DONE_FAILURE
    SEARCH_DOWN_BOARD --> PICK_UP_GOODS: Event.SEARCH_DONE_SUCCESS
    SEARCH_DOWN_BOARD --> ERROR: Event.SEARCH_DONE_FAILURE
    PICK_UP_GOODS --> SPEAK_GOODS: Event.GOODS_FOUND
    PICK_UP_GOODS --> ERROR: Event.PATROL_SEQUENCE_COMPLETED
    SPEAK_GOODS --> NAV_TO_SIMULATION: Event.SPEAK_DONE
    SPEAK_GOODS --> ERROR: Event.SPEAK_TIMEOUT

    %% --- 核心修改：扩展最终流程 ---
    NAV_TO_SIMULATION --> DO_SIMULATION_TASKS: Event.NAV_DONE_SUCCESS
    NAV_TO_SIMULATION --> ERROR: Event.NAV_DONE_FAILURE

    %% --- 新增的仿真到停留流程 ---
    DO_SIMULATION_TASKS --> SPEAK_SIMULATION_GOODS: Event.SIMULATION_DONE

    %% --- 流程终点 ---
    ERROR --> [*]



    note right of DO_SIMULATION_TASKS
        **核心修改:**
        1. 核心动作: 启动一个3秒的定时器模拟任务。
        2. 定时器结束后，触发 Event.SIMULATION_DONE 事件。
    end note

    note right of SPEAK_SIMULATION_GOODS
        **新增的最终停留状态:**
        1. 核心动作: 无，仅打印日志。
        2. 状态机将停留在此状态，不响应任何事件。
    end note
```