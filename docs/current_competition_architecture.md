# 当前比赛程序架构与任务流程

## 1. 结论

当前真实比赛入口是 `src/competition/scripts/navigation_transport/position_correction_pick.launch`，主控制程序是 `src/competition/scripts/navigation_transport/voice_control_navigation.py`。项目是 ROS1/catkin 工作空间，不是 ROS2 项目。

三个“月球场景元素卡片识别”任务点是原比赛流程中的三个插入点，不是一个独立的三点巡检任务，也不是整体任务完成条件。第三个识别点之后，基线源码仍有后退、第二次放置、功能节点收尾以及返航坡道相关动作；不得在第三个识别函数中直接返航或播报。

本文件中的“基线”指创建本分支前的 `voice_control_navigation.py` 行为；“本分支实现”指本次增量加入的 ROS1 检测节点、任务生命周期和结果保存逻辑。当前 Windows 开发机不能运行 ROS 图，因此话题和服务均为源码静态确认，实机连接状态见 `docs/runtime_environment_report.md`。

## 2. Launch 调用关系

```mermaid
flowchart TD
    L["position_correction_pick.launch"] --> MIC["competition/launch/mic_init.launch"]
    MIC --> AWAKE["awake_node.py"]
    MIC --> ASR["asr_node.py"]
    MIC --> VC["voice_control C++ offline ASR"]
    L --> MAP["load_map.launch / map_server"]
    L --> ROBOT["robot.launch / controller / servo / lidar / camera"]
    L --> NAV["navigation_base.launch"]
    NAV --> AMCL["AMCL"]
    NAV --> MB["move_base"]
    L --> YOLO["yolov5_node.py / existing 3-class TensorRT shape model"]
    L --> KIN["kinematics_node.launch"]
    L --> ALIGN["automatic_pick.py / position_correction services"]
    L --> SHAPE["shape_recognition_down.py / grasp services"]
    L --> RAMP["ramp.py / ramp services"]
    L --> MAIN["voice_control_navigation.py"]
    L --> RVIZ["rviz_navigation.launch"]
    MAIN -->|"/move_base_simple/goal"| MB
    MB -->|"/move_base/result"| MAIN
    MAIN -->|"/controller/cmd_vel"| ROBOT
    MAIN --> ALIGN
    MAIN --> SHAPE
    MAIN --> RAMP
    MAIN --> YOLO
    MAIN --> MOON["moon_detector.py / this branch"]
    MOON -->|"result, finished, status"| MAIN
```

`position_correction_pick.launch` 的源码顺序为：加载比赛参数和麦克风，读取地图，启动机器人底层，启动三分类矿石卡片 YOLO、运动学、AMCL/move_base、夹取平台对齐、坡面处理、形状夹取、主比赛节点和 RViz。

## 3. 已确认接口

| 功能 | 源码静态确认的接口 | 说明 |
| --- | --- | --- |
| 语音结果 | `/asr_node/voice_words`，`std_msgs/String` | `voice_control_navigation.py` 订阅；离线语音链路由 `awake_node.py`、`asr_node.py` 和 `voice_control` 组成 |
| 导航目标 | `/move_base_simple/goal`，`geometry_msgs/PoseStamped` | 主程序发布 |
| 导航结果 | `/move_base/result`，`move_base_msgs/MoveBaseActionResult` | 状态 3 被基线程序视为成功，状态大于 3 视为失败 |
| 底盘速度 | `/controller/cmd_vel`，`geometry_msgs/Twist` | 主程序、对齐节点和坡面节点均可能发布；`move_base` 也重映射到该话题 |
| 里程计/雷达 | `/odom`、`/scan` | 主程序用于航向、距离和返坡激光纠偏 |
| 机械臂 | `/servo_controllers/port_id_1/multi_id_pos_dur` | 总线舵机动作；夹取还调用运动学服务 |
| 对齐/放置 | `/position_correction/start`、`pick_1`、`pick_2`、`place_3` 等 | `automatic_pick.py` 提供，状态参数为 `/position_correction/status` |
| 形状夹取 | `/shape_recognition/start`、`pick`、`stop`/`close` | `shape_recognition_down.py` 使用 Gemini RGB、深度和相机内参 |
| 坡面 | `/ramp/up`、`down`、`start`、`stop` | `ramp.py` 使用 `/gemini_camera/depth/image_raw`，状态参数为 `/ramp/status` |
| 原矿石卡片 YOLO | `/yolov5/start`、`stop`、`calibration`，结果参数 `/yolov5/shape` | Astra RGB；三类 `cube/box/cylinder`；现有 TensorRT engine 不能当作月球十分类模型 |
| 月球卡片检测 | `/moon_detector/reset`、`start`、`stop`；`/moon_detector/result`、`finished`、`status` | 本分支实现；每次识别由独立 `session_id` 隔离 |

相机的源码默认值不是单一相机：原三分类 YOLO 和本分支月球检测默认读取 `/astra_camera/rgb/image_raw`；平台对齐、形状夹取和坡面处理读取 Gemini 的 RGB/深度话题。实际设备上的相机型号、命名空间和发布频率必须上车用 `rostopic list/info/hz` 确认。

## 4. 基线完整任务顺序

根据基线主程序、2026 比赛规则第 9-11 页和实现引导第 21、26-27、37 页，当前代码对应的主体顺序为：

```mermaid
flowchart TD
    S["等待语音开始命令或 15 秒自动启动"] --> D0["驶离基地并进入导航"]
    D0 --> D1["导航至资源库/目标确认点"]
    D1 --> TGT["识别矿石卡片并设置目标形状"]
    TGT --> M1["任务点 1：月球场景元素卡片识别，仅保存"]
    M1 --> P1["导航至采集平台 1、对齐并夹取"]
    P1 --> BACK1["0.05 m/s 后退 1 秒"]
    BACK1 --> M2["任务点 2：独立识别，仅保存"]
    M2 --> PLACE1["返回资源库，第一次放置"]
    PLACE1 --> P2["导航至采集平台 2、对齐并夹取"]
    P2 --> M3["任务点 3：独立识别，仅保存"]
    M3 --> REMAIN["原有 3 秒后退"]
    REMAIN --> PLACE2["返回资源库，第二次放置"]
    PLACE2 --> CLOSE["关闭对齐/形状识别，机械臂安全收尾"]
    CLOSE --> RAMPDONE["最后正式收尾：导航坡前点并完成一次激光纠偏坡面穿越"]
    RAMPDONE --> BODYDONE["全部主体/坡面任务完成"]
    BODYDONE --> RETURN["统一返航：发送最终基地 move_base 目标"]
    RETURN --> BASE["move_base 成功后发布零速度并确认基地到达"]
    BASE --> ANN["固定顺序集中播报任务点 1、2、3"]
    ANN --> DONE["保存结果并保持 COMPLETED"]
```

基线源码中的具体调用序列是 `detect -> pick1 -> place -> pick2 -> place`，随后关闭 `/position_correction` 和 `/shape_recognition`，在 `slope_surface=true` 时导航至约 `(1.3, 0, 0)`，再执行一次激光纠偏倒车上坡。为满足本任务明确的返航门禁，本分支把这段基线坡面动作作为最后一个正式收尾任务：成功后才设置 `ramp_task_finished` 和 `all_competition_tasks_finished`，之后仅发送一次最终基地 `move_base` 目标用于停车位到达确认。不会再执行第二套 `/ramp/up` 开环上坡动作。2026 规则第 10-11 页要求往返两次采矿并最终回到基地，因此第二次放置不能被第三次场景识别跳过。

## 5. 三个识别插入点

| 任务点 | 真实插入位置 | 第一次识别后的下一项原任务 |
| --- | --- | --- |
| 1 | `control(..., "detect")` 中，原矿石形状识别完成并写入 `/shape_recognition/target_shape` 后 | 前往第一采集平台并完成第一次夹取 |
| 2 | `pick1` 夹取成功、以 0.05 m/s 后退 1 秒后 | 原后退收尾、第一次返回资源库并放置 |
| 3 | `pick2` 第二次夹取完成后、原 3 秒后退动作之前 | 原 3 秒后退、第二次放置、节点收尾和返航流程 |

第三次识别函数只能保存第三个结果并令 `three_scene_recognitions_finished=true`。它不得设置 `all_competition_tasks_finished`，也不得调用返航、播报或完成函数。

## 6. 本分支状态与职责边界

本分支将生命周期拆分为：

`WAITING_FOR_START -> STARTING -> RUNNING -> RETURNING_TO_BASE -> ANNOUNCING_RESULTS -> COMPLETED`

异常分支为 `STOPPED` 和 `ERROR`。关键状态互不替代：

- `three_scene_recognitions_finished`：三个识别会话都已有独立结果，包括明确的失败结果。
- `all_competition_tasks_finished`：只能在第二次放置、节点关闭、安全臂姿态和实际坡面穿越全部完成后，由主状态机设置。
- `return_to_base_requested`：返航门禁通过后设置一次。
- `returned_to_base`：只有返航目标真实成功且停车后设置。
- `results_announced`：三条结果按 1、2、3 播放完后设置。

返航门禁至少要求三个识别流程结束、两次夹取/放置、导航、功能收尾、坡面任务和安全臂姿态结束，无停止请求、ROS 未关闭且此前未请求返航。播报门禁还要求最终基地 move_base 成功并设置 `returned_to_base=true`。任何检测节点都无权判断整体任务结束或返航。

`moon_detector.py` 只加载模型、订阅图像、过滤检测框、多帧投票并发布单次会话结果。主程序负责生成和校验 `session_id`、保存三个槽位、写 ROS 参数和 JSON、继续原流程、返航及离线播报。

## 7. 基线缺口与实机确认项

- 基线 15 秒逻辑不是一次性 Timer，且完整任务运行在持续外层循环内，语音启动后会被超时路径再次启动；根因见 `docs/mission_duplicate_start_root_cause.md`。
- 基线在三个插入点调用过占位播报，违反“识别点只保存、不现场播报”。本分支必须移除这些调用。
- 基线只用共享列表/参数表达场景结果，没有 `task_point_index + session_id + timestamp` 的旧消息防护。
- 基线返坡动作完成后没有独立的、可静态证明的“基地目标 ID + 到达确认 + 停车 + 集中播报”闭环。基地坐标、坡顶终点是否等同停车位，以及 move_base 与坡面动作的交接必须在目标小车验证。
- `/position_correction/close` 与 `/shape_recognition/close` 同时保留历史 `colse` alias；回调现在立即注销订阅、清活动状态并归零，但保留服务进程供人工 `/competition/reset_mission` 再次启用，shape launch 不再 respawn。实际服务可用性仍必须以实机 `rosservice list/type` 为准。

## 8. 变更影响与回滚

本分支采用增量接入：不改地图、原导航点、夹取动作、放置动作、坡面算法和底层速度接口。新增识别会话和生命周期门禁只包围原流程的三个位置及任务首尾。

若现场需要回滚，可停止比赛节点和月球检测节点、恢复本分支前的主 launch/主程序，并保留原三分类 `shape_models.engine` 链路。禁止用 Moon 数据集内的文件覆盖原矿石形状识别 engine，也禁止删除原 `automatic_pick.py`、`shape_recognition_down.py` 或 `ramp.py`。
