# ROS1 月球探索比赛集成说明

## 当前结论

本分支在原 ROS1/catkin 月球探索程序中增量接入三个“月球场景元素卡片识别”任务点，并修复任务完成后被 15 秒启动逻辑再次执行的问题。主入口仍为：

```bash
roslaunch competition position_correction_pick.launch
```

三个识别点不是完整比赛，也不是返航条件。第三个识别结束后，程序继续执行原有后退、第二次放置、功能节点收尾和机械臂安全复位；只有所有主体任务完成后才返航。确认到达基地并发布零速度后，才按任务点 1、2、3 的固定顺序集中播放离线语音。

当前分支已在目标 Jetson Orin Nano 的 `/home/ubuntu/ros_ws` 完成 Moon CPU 模型原生加载、ROS 预加载服务、无运动完整 launch、140 项自动测试和 `catkin build competition -j4 -l4`。真实比赛卡片、完整导航、机械臂、坡面、返航和离线播报仍必须在比赛场地实机确认。详见 `docs/moon_competition_test_report.md`。

## 真实比赛顺序

当前源码和比赛资料还原出的执行顺序为：

```text
等待语音开始或 15 秒一次性自动启动
-> 驶离基地并导航至资源库/目标确认点
-> 原三分类矿石卡片识别
-> 月球场景任务点 1：只保存结果
-> 第一采集平台对齐与夹取
-> 原后退动作
-> 月球场景任务点 2：只保存结果
-> 返回资源库并完成第一次放置
-> 第二采集平台对齐与夹取
-> 月球场景任务点 3：只保存结果
-> 继续原 3 秒后退动作
-> 返回资源库并完成第二次放置
-> 关闭对齐/识别功能并将机械臂置于安全状态
-> 完成一次基线激光纠偏坡面穿越
-> 设置全部比赛任务完成
-> 通过统一返航门禁发送最终基地 move_base 目标
-> 确认导航成功并停车
-> 依次播报任务点 1、2、3
-> 保存最终 JSON 并保持 COMPLETED
```

任何月球识别函数都不能设置 `all_competition_tasks_finished`，也不能直接调用返航、播报或完成逻辑。

## 八分钟截止与夹取识别时序

比赛计时预算从语音、15 秒 Timer 或服务中的第一个合法启动请求被接受时开始，不从 ROS 节点启动时开始。`mission_timeout_seconds=450` 是比赛动作和普通等待的 7 分 30 秒硬截止，比正式 8 分钟上限预留 30 秒系统余量；`STARTING/RUNNING` 状态默认扣留最后 60 秒，因此主体任务实际截止为 `450 - 60 = 390` 秒。进入 `RETURNING_TO_BASE` 后仍扣留最后 20 秒给集中播报，返航阶段最多使用 40 秒，播报阶段最多使用最后 20 秒。

所有局部等待同时受“本步骤上限、所属阶段上限、当前任务状态的预留时间、450 秒硬截止”约束，最先到达的截止生效。主体任务到 390 秒仍未完成时，程序停止继续等待、发布零速度并进入统一失败清理，不会用返航/播报预留继续卡在未完成动作上，也不会把未完成任务伪装为完成后直接返航。

| 聚合阶段 | 上限 |
| --- | ---: |
| 初始驶离（含首次视觉启动和固定驶离动作） | 20 秒 |
| 资源库目标识别 | 50 秒 |
| 第一次夹取 / 第二次夹取 | 每次 85 秒 |
| 第一次放置 / 第二次放置 | 每次 45 秒 |
| 第三次场景识别后的功能收尾、机械臂安全姿态和坡面任务 | 60 秒 |
| 最终返航 | 40 秒 |
| 基地集中播报 | 20 秒 |

主体阶段上限合计正好为 390 秒，但每一阶段内部仍会被全局主体截止再次截断，前一阶段节省的时间可以留给后续阶段，任何后续等待都不能突破 t=390。路径导航不再设置独立的 20/35/40 秒倒计时，`move_base` 可使用当前任务阶段的剩余时间；夹取 85 秒、放置 45 秒、返航 40 秒等任务级上限和 450 秒全局截止仍然有效，`move_base` 服务端发现仍限制为 10 秒。其他关键内部等待为：服务发现 2 秒；原三分类 YOLO 检测/启动/卸载 8/12/5 秒；形状夹取启动/预热/`pick` 服务/状态等待/整段夹取/停止 3/0.5/4/18/25/1 秒；Moon 单会话/启动/检测/卸载 25/12/12/5 秒；平台状态等待 15 秒，放置服务 8 秒，激光坡面穿越 15 秒，可选深度坡面对齐 20 秒，单段离线音频播放 4 秒。初始化阶段另有语音初始化 20 秒、节点初始化 60 秒和旧 TensorRT engine 预热 90 秒上限，它们发生在任务启动令牌被接受之前；15 秒一次性启动窗口只在预热成功后打开。为保证超时后仍能停车，统一清理中的少量 `stop/unload` 调用不受剩余任务预算阻止，但各自仍有 1–5 秒短截止；预留的 30 秒系统余量不能用于继续比赛动作，实机必须验证包括这些安全清理在内的总墙钟时间仍小于 480 秒。

原夹取形状识别与 Moon 场景卡片识别是两条不同链路。现在每次夹取的顺序固定为：

```text
到达夹取导航点并停车
-> 停止原三分类 YOLO，调用 /shape_recognition/start 预热夹取视觉
-> 执行最后一段靠近采集平台的底盘运动
-> 再次发布零速度并确认进入夹取步骤
-> 调用 /shape_recognition/pick
-> 在夹取整段 25 秒和状态等待 18 秒双重截止内等待
-> finally 调用 /shape_recognition/stop，并按生命周期恢复原 YOLO
```

用户提供的旧运行日志并不是“放置动作完成后才为同一次放置打开形状识别”。日志顺序是 `1783931844.773 start place_3`（第一次放置），随后 `1783931853.640 Navigating to pick2 point`、`1783931879.450 GOAL Reached`，最后才在 `1783931881.680 === safe_pick start ===` 进入第二次夹取。也就是说，形状夹取属于下一段 `pick2`；旧版本只是把启动放在 `safe_pick` 内，视觉预热偏晚。本分支已将预热前移到夹取点、最终靠近动作之前，而 `/shape_recognition/pick` 仍只允许在停车后触发。日志中稍后的“第三次环境识别”是 Moon 场景元素卡片任务点 3，也不是放置用的形状识别。

前两个 Moon 场景点的底盘转向是显式动作：任务点 1 先左转 90°再识别，识别后不回转；任务点 2 先左转 90°再识别，并在退出识别会话时右转 90°恢复原朝向；任务点 3 不旋转。转向发生在 YOLO 卸载和 Moon detector 服务调用之前，所以即使视觉服务缺失或模型启动失败，前两个点也不会跳过转向。角度和速度分别由 `moon_rotation_degrees`、`moon_rotation_speed` 配置。

## 一次性启动约束

语音、15 秒超时、启动服务和人工调试入口统一进入带锁的启动请求。第一个合法请求会立即消费启动令牌、取消一次性 Timer，并把执行次数加一。运行中、返航中、播报中和 `COMPLETED` 状态都拒绝后续启动。

节点正常完成后保持：

```text
mission_state = COMPLETED
mission_started = true
mission_finished = true
start_trigger_consumed = true
mission_execution_count = 1
```

程序不会自动重新创建 15 秒 Timer，也不会自动回到等待状态。调试复位只能在 `COMPLETED`、`STOPPED` 或 `ERROR` 且没有活动任务线程、底盘已停止时人工调用；复位服务还会实际发送机械臂安全姿态，失败时拒绝 reset。

## 月球识别接口

`moon_detector.py` 是独立 ROS1 Python3 节点，仅负责模型推理和单次会话结果，不负责导航、机械臂、返航或语音。

服务：

- `/moon_detector/reset`
- `/moon_detector/preload`
- `/moon_detector/start`
- `/moon_detector/stop`
- `/moon_detector/unload`

话题：

- `/moon_detector/result`，`std_msgs/String` JSON
- `/moon_detector/finished`，`std_msgs/Bool`
- `/moon_detector/status`，`std_msgs/String`
- 可选 `/moon_detector/debug_image`

每个识别点使用独立 `session_id`。主程序校验任务点编号、会话 ID 和结果时间，结果分别写入 `/moon_task/results/1/...`、`/moon_task/results/2/...`、`/moon_task/results/3/...`，并原子更新 `runtime/moon_task_results.json`。

成功播报严格为：

```text
第一个任务点识别到[场景元素名称]
第二个任务点识别到[场景元素名称]
第三个任务点识别到[场景元素名称]
```

失败槽位不会被其他结果替代，播报为“第 N 个任务点未识别到有效场景元素”。识别点现场不播放任何结果。

## 模型与类别

默认模型：`src/competition/models/moon/best.pt`

```text
size:   14438056 bytes
SHA256: ab953a754cc6ea68742d49ccf26d8b122bb771fe65aa6bd582761bdeaaa7da34
input:  640
```

目标 Orin 默认使用 `device: cpu`，并在语音/15 秒一次性启动窗口打开前调用 `/moon_detector/preload`。实测 CPU 模型加载约 6.4-9.5 秒，640x640 推理平均约 0.84 秒/帧，四票确认估算约 3.24 秒；预加载后的 `/moon_detector/start` 约 1.52 秒。GPU 冷加载约 89.5 秒，且与原三分类 TensorRT 同时驻留时只剩约 940 MiB 可用内存并开始使用 swap，因此不作为当前默认方案。

该 checkpoint 由较新的 Windows Python 保存，序列化数据引用了 `pathlib._local.WindowsPath`。目标 Python 3.8 没有该模块，且 Linux 不能直接实例化 `WindowsPath`。检测节点在加载前把 checkpoint 中的 Windows/Posix 具体路径类映射到当前主机原生路径类，不修改模型文件，模型 SHA256 保持不变。

类别顺序不可修改：

| ID | English | 中文 |
| ---: | --- | --- |
| 0 | satellite | 卫星 |
| 1 | space_station | 空间站 |
| 2 | lunar_crater | 月坑 |
| 3 | lunar_rover | 月球车 |
| 4 | meteorite | 陨石 |
| 5 | earth | 地球 |
| 6 | lunar_soil | 月壤 |
| 7 | moon | 月球 |
| 8 | rocket | 火箭 |
| 9 | astronaut | 宇航员 |

该模型识别的是比赛用月球场景元素卡片，不应描述为任意真实月球自然环境识别。模型验证边界见 `docs/moon_model_validation.md`。

## 部署与运行

在目标 Jetson 的工作空间根目录执行：

```bash
export MOON_WORKSPACE="$(pwd)"
# 自动发现错误时显式指定真实 ROS1 underlay：
# export MOON_ROS_SETUP=/opt/ros_ws/melodic/setup.bash
bash ./setup_moon_runtime.sh
./run_moon_competition.sh
```

`setup_moon_runtime.sh` 会拒绝非 Jetson/aarch64 环境，要求 Python 3.8 或更新版本，检查 ROS1、PyTorch、OpenCV、cv_bridge、PyCUDA/TensorRT、比赛功能节点依赖、模型哈希和 GPU 状态。已有 `.catkin_tools` 工作空间沿用 `catkin build`，其他工作空间使用 `catkin_make`；它不会强制整个 Melodic 工作空间改用 Python3，也不会升级 JetPack、CUDA、TensorRT、PyTorch、torchvision、NumPy 或 OpenCV。运行脚本禁止 YOLOv5 在比赛过程中联网安装依赖。

紧急停止：

```bash
./stop_moon_competition.sh
```

比赛退出后不会自动恢复可能占用硬件的 `start_app_node.service`。确认小车已停车且比赛 launch 已退出后，人工执行：

```bash
./restore_app_service.sh --confirm
```

完整上车步骤、话题检查、回滚和可选 TensorRT 构建见 `docs/robot_deployment_guide.md`。

## 关键文件

| 文件 | 职责 |
| --- | --- |
| `src/competition/scripts/navigation_transport/voice_control_navigation.py` | 完整任务状态机、三次会话、返航、播报和安全停止 |
| `src/competition/scripts/navigation_transport/moon_detector.py` | Moon 模型加载、多帧投票和会话结果 |
| `src/competition/scripts/navigation_transport/moon_mission_core.py` | ROS 无关生命周期、投票和原子 JSON 工具 |
| `src/competition/config/moon_detector.yaml` | 推理、ROI、框面积和会话参数 |
| `src/competition/config/moon_competition.yaml` | 启动、导航、返航和比赛级参数 |
| `src/competition/scripts/navigation_transport/position_correction_pick.launch` | 真实比赛 launch |
| `src/competition/tests/test_moon_mission_core.py` | ROS 无关自动测试 |
| `runtime/moon_task_results.json` | 运行期原子结果快照，不提交 Git |

## 安全边界

- 运行前停止 `start_app_node.service`，比赛过程中不恢复。
- 导航、检测、返航、播报和异常分支都应保持统一零速度能力。
- 默认相机话题 `/astra_camera/rgb/image_raw` 只是源码确认值，必须在实机检查真实发布者和帧率。
- 基地坐标、坡道距离、地图导航点、机械臂安全姿态和声卡设备必须现场标定。
- 默认只执行基线的激光纠偏坡面方案；深度 `/ramp/up` 对齐默认关闭，只有在目标小车验证车身朝向后才能启用，且不会叠加第二次坡面穿越。
- Moon 运行时禁止自动 `pip install`；缺失依赖必须在比赛前通过部署检查解决。
- 原三分类形状识别完成后卸载其 TensorRT/PyCUDA 资源，本轮不再重载；Moon CPU 模型在启动窗口打开前预加载，三个会话复用同一模型，并在完成、停止、错误或 reset 时卸载。只有中文“系统启动完成”横幅出现后才允许语音唤醒或等待 15 秒自动启动。
- 不复制其他 Jetson 或桌面 GPU 生成的 TensorRT engine；只能在目标 Jetson 上构建并回归十类结果。
- 未完成实机检查前，不得把本分支标记为比赛验收通过。
