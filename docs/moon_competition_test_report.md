# 月球探索比赛测试报告

检查日期：2026-07-15（Asia/Shanghai）

## 1. 当前测试边界

本次同时使用 Windows 开发电脑和目标 Jetson Orin Nano。Windows 侧执行静态检查、资源验证和 ROS 无关测试；目标机 `/home/ubuntu/ros_ws` 执行 ROS 服务、Moon CPU 模型、无运动 launch、自动测试和 catkin 编译。完整导航、机械臂、坡面、返航、真实比赛卡片和声卡播报尚未跑完整比赛，因此仍单独标记为现场验收项。

状态说明：

- `开发机已验证`：已有实际命令、测试或模型验证证据。
- `目标机已验证`：已在当前 Jetson/ROS 工作空间执行并保存实际输出。
- `待执行`：需要补充测试夹具或完整集成运行。
- `需要小车实机确认`：依赖 ROS 图、Jetson、导航、执行器、相机、音频或 systemd。
- `部分验证`：开发机已有一部分证据，但尚未满足最终实机门槛。

模型的详细数值结果见 `docs/moon_model_validation.md`；运行环境边界见 `docs/runtime_environment_report.md`。

## 2. 五十项测试矩阵

| # | 测试项 | 当前证据或执行方法 | 状态 |
| ---: | --- | --- | --- |
| 1 | Python 语法 | `py_compile` 实测主程序、检测节点、核心模块和测试文件，退出码 0 | 开发机已验证 |
| 2 | launch XML | 实测解析主比赛 launch 和 Moon detector launch | 开发机已验证 |
| 3 | YAML 解析 | 临时隔离安装的 PyYAML 6.0.3 实测解析两个 Moon 配置，均为有效字典 | 开发机已验证 |
| 4 | shell `bash -n` | Git for Windows Bash 实测五个部署脚本，退出码 0 | 开发机已验证 |
| 5 | `package.xml` | .NET XML 解析实测通过；ROS 依赖解析仍由 catkin 验证 | 开发机已验证 |
| 6 | `CMakeLists.txt` | 静态规则检查通过，并由目标机 `catkin build` 实际完成配置与构建 | 目标机已验证 |
| 7 | catkin 编译 | 目标机执行 `catkin build competition -j4 -l4`，competition 及 10 个相关包全部成功 | 目标机已验证 |
| 8 | 绝对路径检查 | 实测搜索新增脚本/文档，无 Windows 用户路径或固定 Linux 用户家目录 | 开发机已验证 |
| 9 | 模型 SHA256 | 独立资源测试实测为 `ab953a...7da34`，大小 14,438,056 bytes | 开发机已验证 |
| 10 | 模型加载 | checkpoint/OpenCV DNN 加载通过；目标 Orin `DetectMultiBackend` CPU 加载和 `/moon_detector/preload` 通过 | 目标机已验证 |
| 11 | 十类映射 | checkpoint、数据 YAML、标签与规定 ID 0-9 一致 | 开发机已验证 |
| 12 | 单图推理 | 开发机 OpenCV DNN 数值前向和目标 Orin CPU 单图推理均已执行 | 目标机已验证 |
| 13 | 十类抽样 | 71 张验证图覆盖十类，主目标映射 71/71 一致 | 开发机已验证 |
| 14 | 背景误报 | 代理背景已测；真实赛场、曝光和距离背景尚缺 | 部分验证，需实机确认 |
| 15 | 多帧投票 | `MultiFrameVoter` 稳定票数测试包含在 28 项单元测试中 | 开发机已验证 |
| 16 | 票数不足 | 实测保持 pending，不强制分类 | 开发机已验证 |
| 17 | 两类冲突 | 实测同票同置信度返回 conflict，不随机选择 | 开发机已验证 |
| 18 | 超时 | fake clock 实测 timeout/no forced classification | 开发机已验证 |
| 19 | 模型缺失 | 检测节点显式返回 `model_error`，主任务记录失败并保持统一停止；ROS 服务联动仍需实机故障注入 | 部分验证，需实机确认 |
| 20 | 三个结果独立 | 生命周期防御性复制和固定槽位单元测试已通过 | 开发机已验证 |
| 21 | 第一任务点后不播报 | 控制器契约测试确认统一识别函数不调用播报/返航；声卡仍需实机 | 部分验证，需实机确认 |
| 22 | 第二任务点后不播报 | 控制器契约测试确认统一识别函数不调用播报/返航；声卡仍需实机 | 部分验证，需实机确认 |
| 23 | 第三任务点后不播报 | 契约测试确认第三点后先执行原后退和第二次放置 | 部分验证，需实机确认 |
| 24 | 第三点完成但仍有夹取时不返航 | 生命周期返航门禁单元测试；实机观察第二次夹取 | 部分验证，需实机确认 |
| 25 | 第三点完成但仍有坡面任务时不返航 | 契约测试确认激光坡面穿越成功后才置 `ramp_task_finished`，该标志又是返航门禁的一部分 | 部分验证，需实机确认 |
| 26 | 三点完成但 `all_competition_tasks_finished=false` 不返航 | `test_third_scene_result_does_not_request_return` 已通过 | 开发机已验证 |
| 27 | 所有剩余任务完成后才返航 | 第二次放置、节点关闭、安全臂姿态和坡面穿越均位于 `mark_all_tasks_finished` 前；生命周期门禁测试通过 | 部分验证，需实机确认 |
| 28 | 已发基地目标但未到达时不播报 | `begin_announcing` 在未 returned 时拒绝，单元测试已通过 | 开发机已验证 |
| 29 | 真正到达基地后才播报 | move_base 结果、坡道里程和停车位必须现场确认 | 需要小车实机确认 |
| 30 | 播报顺序 1、2、3 | snapshot 固定排序、控制器播报顺序和 16 个 PCM WAV 测试通过 | 部分验证，需实机确认 |
| 31 | 第二点失败仍播报三条 | 失败槽位保留单元测试；声卡完整播放需实机 | 部分验证，需实机确认 |
| 32 | 重复导航成功回调只播报一次 | returned/announcing one-shot 生命周期测试已通过 | 开发机已验证 |
| 33 | 第 5 秒语音只执行一次 | 语音回调统一入口与启动令牌测试通过；真实 ASR/Timer 仍需 ROS | 部分验证，需实机确认 |
| 34 | 无语音第 15 秒执行一次 | oneshot `rospy.Timer` 和 timeout 只请求统一入口的契约测试通过 | 部分验证，需实机确认 |
| 35 | 第 14.9 秒语音和 Timer 竞争只执行一次 | 16 线程并发竞争测试已通过；真实 Timer 仍需 ROS | 部分验证，需实机确认 |
| 36 | 连续多次启动语音只执行一次 | 重复 `request_start_token` 单元测试；ASR 需 ROS | 部分验证，需实机确认 |
| 37 | 运行中再次收到语音被拒绝 | RUNNING 状态单元测试已通过 | 开发机已验证 |
| 38 | 完成后再次收到语音被拒绝 | COMPLETED 状态单元测试已通过 | 开发机已验证 |
| 39 | 完成后 start service 被拒绝 | service 统一入口及 COMPLETED 拒绝测试通过；ROS 服务仍需实机 | 部分验证，需实机确认 |
| 40 | 完成后等待至少 30 秒不执行第二遍 | 实际墙钟 30 秒并发 voice/timeout/service 回归通过；完整 ROS 节点仍需实机 | 部分验证，需实机确认 |
| 41 | `mission_execution_count` 保持 1 | 完成态 snapshot 单元测试已通过 | 开发机已验证 |
| 42 | stop 后 Timer 不启动任务 | stop 状态拒绝启动测试已通过；ROS Timer 取消仍需实机 | 部分验证，需实机确认 |
| 43 | error 后 Timer 不启动任务 | error 状态拒绝启动测试已通过；ROS Timer 取消仍需实机 | 部分验证，需实机确认 |
| 44 | shutdown 取消 Timer | 契约测试确认 shutdown 进入统一清理；真实 ROS Timer、服务竞争和进程信号仍需实机 | 部分验证，需实机确认 |
| 45 | 导航失败底盘归零 | 注入 move_base 失败并监测 `/controller/cmd_vel` | 需要小车实机确认 |
| 46 | 检测失败底盘归零 | 模型/相机故障注入并监测速度话题 | 需要小车实机确认 |
| 47 | Ctrl+C 底盘归零 | `run` trap、shutdown 回调和真实控制器观测 | 需要小车实机确认 |
| 48 | 返回基地后底盘归零 | 实际基地到达后观测 Twist 全零 | 需要小车实机确认 |
| 49 | 播报异常不会重新运动 | 声卡/WAV 故障注入并观察状态和速度 | 需要小车实机确认 |
| 50 | APP 服务冲突检查 | `systemctl is-active`、运行脚本停止、比赛中不恢复 | 需要小车实机确认 |

## 3. 自动测试实测结果

完整开发机测试发现命令：

```text
python -m unittest discover -s src/competition/tests -p "test_*.py" -v
Ran 136 tests
OK
```

136 项由以下部分组成：

- 4 项资源测试：`best.pt` 大小/SHA256、十类文件顺序、vendored YOLOv5 文件和无嵌套 `.git`、16 个单声道 16-bit PCM WAV。
- 74 项控制器/检测器/辅助节点契约测试：第三点后原流程、前两点转向及第二点回转、转向先于视觉服务、第二次放置、坡面完成门禁、单次坡面穿越、最终 move_base、统一异常清理、关闭节点、播报顺序、一次性 Timer、统一启动入口、语音开关、比赛启动窗口前两个模型的预热、ROS 参数可序列化失败结果、跨平台 pathlib checkpoint 兼容、手工服务互斥、分层动作超时与阶段取消、夹取点视觉预热与停车后 `pick`、禁止运行时 pip 安装和视觉资源卸载。
- 17 项部署契约测试：launch/YAML 覆盖优先级、Moon CPU 默认设备、启动窗口前预加载、无运动诊断启动开关、Moon 转向参数、主体七阶段上限合计 390 秒、返航/播报窗口不突破 450 秒、ROS/catkin 依赖、install-space 资产、Python 3.8 门禁、ROS underlay/build tool 选择、固定依赖版本、运行时禁止联网安装和安全停止卸载两个视觉模型。
- 34 项 ROS 无关核心测试：启动竞争、终态拒绝、三个结果、返航/播报 one-shot、450/390 秒预算与阶段截断、多帧投票、冲突/超时和原子 JSON。
- 7 项可部署验证工具测试：manifest 十类映射、12 张样本资产、路径边界、严格正样本匹配及 0.70 阈值负样本误报失败。

另行实际执行 30 秒实时回归：

```text
python src/competition/tests/integration_duplicate_start_30s.py
30-second duplicate-start regression passed: COMPLETED
exit: 0
```

该测试在完成态持续并发注入 `voice`、`timeout` 和 `service` 请求，所有请求均被拒绝，`mission_state` 保持 `COMPLETED`，`mission_execution_count` 保持 `1`。它是实际墙钟等待，不是 fake clock；但仍是 ROS 无关生命周期测试，目标小车上的真实 ASR、`rospy.Timer` 和服务回调必须补测。

## 4. 八分钟截止与夹取时序回归

本次修改增加分层时间预算，配置基线为：450 秒全局硬截止、60 秒返航与播报预留、其中 20 秒播报预留。因此主体任务截止为 390 秒，返航阶段上限 40 秒，播报阶段上限 20 秒。主体阶段的局部上限为初始驶离 20 秒、资源库识别 50 秒、两次夹取各 85 秒、两次放置各 45 秒、第三次识别后的剩余任务 60 秒，合计正好 390 秒；每个阶段仍由同一个绝对主体截止再次截断，不能累加突破全局预算。

开发机测试覆盖以下预算行为：

- `MissionTimeBudget` 启动时，扣除 60 秒预留后主体可用时间精确为 390 秒；经过 360 秒后主体只剩 30 秒，而硬截止仍剩 90 秒。
- 当已运行 380 秒时，请求 35 秒等待会被主体截止压缩为 10 秒；局部父阶段只剩 4 秒时又会压缩为 4 秒。
- 到 449.9 秒时任何 10 秒请求最多只得到 0.1 秒；450 秒后得到 0 秒，截止不会被新的调用重置或延长。
- 主流程的九个阶段全部经过 `_run_timed_stage`；导航、固定速度运动、服务发现/调用、平台状态、Moon 会话、形状夹取、坡面和音频分别还有更短的内部截止。
- 服务发现默认 2 秒；超时后返回的夹取、放置、视觉或坡面启动响应会尝试通过对应 `stop` 服务中和，避免迟到响应在下一阶段重新驱动设备。
- Moon 任务点 1、2 分别验证为“左转 90°不回转”和“左转 90°后在 `finally` 右转恢复”；转向调用必须位于 YOLO 卸载和 Moon 服务之前，任务点 3 保持不旋转。

夹取时序契约测试分别检查 `pick1` 和 `pick2`：`_prepare_shape_pick()` 必须位于最后靠近动作之前，`safe_pick(prepared=True)` 必须位于靠近动作之后；`safe_pick()` 内又必须先调用 `safe_stop_robot()`，再触发 `/shape_recognition/pick`。因此 `/shape_recognition/start` 用于在夹取点预热，真正的 `pick` 只在底盘停止后发出。

用户日志的时间线复核结果为：

| 日志时间 | 事件 | 判读 |
| ---: | --- | --- |
| `1783931844.773` | `start place_3` | 第一次放置 |
| `1783931853.640` | `Navigating to pick2 point` | 已进入第二次夹取流程，不再属于放置 |
| `1783931879.450` | `GOAL Reached` | 到达第二夹取点 |
| `1783931881.680` | `=== safe_pick start ===` | 旧版本此时才启动夹取形状视觉，预热偏晚 |
| `1783931911.980` | `pick2: 执行第三次环境识别` | Moon 场景任务点 3，不是形状夹取视觉 |

所以日志并不证明“放置动作结束后才为该放置打开形状识别”；它证明第一次放置完成后开始了下一段 `pick2`，而旧代码直到 `safe_pick` 才准备形状识别。本分支已将 `/shape_recognition/start` 前移到夹取导航点之后、最后靠近之前。日志同时来自缺少 `yolov5_scene_card_node.py`、仍等待 `/yolov5_scene_card/start` 的旧部署，不代表当前分支的 Moon 节点接口；当前接口是 `/moon_detector/*`。

450 秒是比赛动作与普通等待的硬截止；为保证故障时仍能停车，统一清理中的 `stop/unload` 可在自身 1–5 秒短截止内继续，不会被剩余任务预算直接跳过。仍需在小车实机记录：启动接受时间、390 秒主体截止前的完成时间、返航开始/到达时间、450 秒前播报完成时间，以及包括安全清理在内小于 480 秒的最终零速度时间；同时用 `/controller/cmd_vel` 与服务时间戳证明 `/shape_recognition/start` 在最终靠近前、`/shape_recognition/pick` 在零速度后。开发机契约测试不能替代真实 ROS 调度、网络抖动、TensorRT 首次加载和机械臂耗时验证。

## 5. 数据集复检结果

Moon 数据集已重新实际统计，结果为：

| 项目 | 结果 |
| --- | ---: |
| 训练图片 | 283 |
| 验证图片 | 71 |
| 测试图片 | 0 |
| 总图片 | 354 |
| 尺寸 | 全部 640x480 |
| 每图标注框 | 1 |
| 空标签/缺失标签/损坏图片 | 0/0/0 |
| 非法或越界坐标 | 0 |
| 重复图片 | 0 |
| 独立负样本 | 0 |

`classes.txt`、`data.yaml` 和 checkpoint 类别顺序一致。71 张验证图的开发机 OpenCV DNN 一次性数值验证主目标类别为 71/71 一致，十类均有代表样本；代理背景在默认 0.70 阈值下未产生有效结果。仓库已保存十类各一张验证图片和 `bus.jpg`、`zidane.jpg` 两张代理负样本，manifest 固定 SHA256、尺寸、类别和真值标注。`python tools/validate_moon_assets.py` 静态资产校验实测通过，无需外部数据集。

严格原生推理命令 `--require-cpu-inference` 和 `--require-gpu-inference` 也已实际执行，但当前 Windows 默认 Python 缺少 `torch`、`cv2`、`numpy` 和 `ultralytics`，两者均以退出码 2 失败，不能写成已通过。新增 7 项验证工具测试全部通过，包含十类/manifest 一致性、路径越界防护、正样本错类失败和 0.71 负样本误报严格失败。数据集没有独立测试集和真实比赛场地负样本，所以真实赛场背景、距离、角度、光照和遮挡仍必须上车复检。完整分布、置信度和耗时见 `docs/moon_model_validation.md`。

## 6. 当前已确认事实

- 真实主 launch 为 `competition/position_correction_pick.launch`。
- 主程序、move_base、对齐和坡面共同使用 `/controller/cmd_vel`。
- Moon 默认图像话题为 `/astra_camera/rgb/image_raw`，但真实相机发布者仍需上车确认。
- 三次识别插入点分别位于原目标确认后、第一次夹取后退后、第二次夹取后。
- 第三次识别之后仍有原后退、第二次放置、节点关闭、安全臂姿态和坡面收尾任务；这些完成后才允许请求最终基地目标。
- 模型文件大小、SHA256、十类顺序和开发机数值推理已有证据。
- Git for Windows Bash 对五个部署脚本执行 `bash -n` 通过。
- 主程序、检测节点、核心模块和测试文件 `py_compile` 通过。
- 自动测试在开发机和目标机分别实际运行 136 项，全部通过。
- 30 秒实时重复启动回归通过，完成态和执行次数保持不变。
- 任务启动后的硬截止为 450 秒，主体截止为 390 秒；返航最多使用随后 40 秒，最后 20 秒留给基地播报。所有阶段和内部等待均受更短截止约束。
- 两次夹取均在夹取导航点调用 `/shape_recognition/start` 预热，最后靠近后再次停车才调用 `/shape_recognition/pick`；用户日志中的 `place_3 -> pick2 -> safe_pick` 是两个连续任务，不是放置动作启动识别。
- 主比赛 launch、Moon detector launch 和 `package.xml` XML 解析通过。
- 使用隔离在临时目录中的 PyYAML 6.0.3 实际解析 `moon_detector.yaml` 和 `moon_competition.yaml` 通过；没有修改系统 Python 或项目依赖。
- 已在目标 Orin Nano 的 `/home/ubuntu/ros_ws` 运行 `catkin build competition -j4 -l4`，10 个依赖包和 competition 全部成功；旧三分类 TensorRT engine 加载、Astra 图像订阅、服务启动/暂停和短时推理循环通过。
- Moon 十分类真实卡片推理、完整导航、机械臂全流程、坡面、基地返航和最终音频仍需比赛场地实机确认。

## 7. 2026-07-15 Orin Nano 启动故障回归

目标机为 NVIDIA Jetson Orin Nano Developer Kit、JetPack 5.1.3、L4T 35.5.0、ROS Noetic，真实工作空间为 `/home/ubuntu/ros_ws`。故障运行中，主控已输出等待单次启动触发，15 秒 Timer 接受启动后在 `initial departure` 第一行同步调用 `/yolov5/start`；配置只允许 12 秒，而旧 `shape_models.engine` 首次加载仍在初始化 cuBLAS/cuDNN，随后主控将其误判为初始驶离失败并进入 ERROR。后续视觉 stop/unload 警告是取消清理的连锁结果，不是最初根因。

修复后，主控在订阅开始语音、设置 `initialization_complete` 和创建 15 秒一次性 Timer 之前调用 `/yolov5/start`，最多等待 90 秒；加载成功后立即调用 `/yolov5/stop` 暂停推理，但保留 engine、CUDA context 和相机订阅。随后还会完成 Moon CPU 预加载；只有看到中文“系统启动完成”横幅后才允许语音唤醒，模型预热时间不进入 450 秒比赛任务预算。

两次无运动 probe 均使用：

```bash
roslaunch competition position_correction_pick.launch \
  enable_timeout_auto_start:=false enable_voice:=false
```

实测证据：

- 第一次 probe：`1784126820.948` 开始预热，`1784126825.544` 完成，随后 `1784126827.568` 进入 `WAITING_FOR_START`；`mission_execution_count=0`。
- 第二次 probe：`1784127203.598` 开始预热，`1784127208.395` 完成，随后 `1784127208.555` 进入等待；自动启动参数为 false，任务执行次数保持 0。
- RViz 1.14.25 成功使用 NVIDIA Tegra Orin OpenGL 4.6 启动，没有 OpenGL/Qt 崩溃。
- 预热后再次调用 `/yolov5/start` 返回 `success: True`，`rosnode ping /yolov5` 正常，连续 6 秒相机推理没有 `yolov5 inference failed`、CUDA exception 或 traceback；镜头无三分类卡片时 `/yolov5/shape=None` 属于正确空结果。
- `enable_voice:=false` 现在不会再错误包含 `awake_node`、`asr_node` 和 `voice_control`；该开关只用于安全诊断，正式比赛默认仍启用语音。
- 测试结束后已调用停止接口、发布零速度并清理全部 ROS 进程，`start_app_node.service` 保持 inactive。

旧三分类 engine 仍打印“跨设备使用 engine plan 不推荐”。仓库和目标机没有找到该 engine 对应的 ONNX/WTS/PT 源文件，因此不能在缺少源模型和回归数据时擅自重建。当前证据只证明它在本车成功加载并执行短时推理，不消除 TensorRT 可移植性风险。讯飞日志中的 `11212` 在项目源码中被标注为 license expired；这不会阻止一次性 15 秒自动启动或 `/competition/start_mission` 服务启动，但真实语音识别需要更新合法的离线授权资源后再验收。

## 8. 2026-07-15 Moon checkpoint 兼容与预加载回归

故障日志先出现：

```text
ModuleNotFoundError: No module named 'pathlib._local'; 'pathlib' is not a package
cannot marshal None unless allow_none is enabled
```

第一项根因是 `best.pt` 由较新的 Windows Python 保存，pickle 引用了 `pathlib._local.WindowsPath`；目标 Linux Python 3.8 的 `pathlib` 是单模块，且 Linux 不能直接实例化 `WindowsPath`。检测节点现在在反序列化前提供兼容模块，并把 Windows/Posix 具体路径类型都映射为当前主机原生路径类型。第二项根因是 Moon 失败结果把 `class_id` 写成 `None`，而 ROS 参数服务器 XML-RPC 不允许该值；失败结果现使用明确哨兵值 `-1`。

为避免第一次场景识别时冷加载，新增 `/moon_detector/preload`。主控先预热旧三分类 TensorRT，再预加载 Moon CPU 模型，最后才创建语音订阅和 15 秒一次性 Timer。无运动验证命令为：

```bash
roslaunch competition position_correction_pick.launch \
  enable_timeout_auto_start:=false enable_voice:=false
```

实际启动顺序和证据：

- 旧 YOLO 预热：`1784131817.260` 至 `1784131822.902`。
- Moon CPU 预热：`1784131822.908` 至 `1784131830.115`。
- 中文“系统启动完成”横幅：`1784131831.794`，随后状态保持 `WAITING_FOR_START`，没有执行比赛动作。
- CPU 模型加载约 6.371-9.494 秒；640x640 推理平均 0.840 秒/帧、最大 0.936 秒/帧；预加载后的 `/moon_detector/start` 约 1.523 秒。
- GPU 首次加载约 89.5 秒；与旧 TensorRT 同时驻留时约使用 6.0/7.3 GiB 内存、只剩约 940 MiB 可用并使用约 496 MiB swap，所以默认改为 CPU。
- `/moon_detector/preload/start/stop/unload` 全部返回成功；修复后日志中没有再次出现 `pathlib._local`、`cannot marshal None` 或 Moon model load failure。
- 目标机 136 项自动测试全部通过；`catkin build competition -j4 -l4` 的 10 个相关包全部成功。
- probe 由测试超时主动终止，随后确认无 ROS 残留进程，`start_app_node.service` 保持 inactive。

这次回归证明节点可以完成安全初始化和识别服务启动，不等于完整比赛验收。十类真实卡片、三个现场识别点、完整驾驶/机械臂/坡面、基地返航和离线播报仍需实车确认。

## 9. 实机测试记录模板

每次实机运行至少保存：

```text
日期/操作者：
Git commit：
Jetson/JetPack/ROS：
模型 SHA256：
地图与基地坐标版本：
启动来源：voice / timeout / service
启动请求接受时间（t=0）：
mission_execution_count：
主体任务完成时间（必须 <=390 秒）：
形状视觉 start / 最终靠近 / 零速度 / pick 时间戳：
任务点 1 session/result：
任务点 2 session/result：
任务点 3 session/result：
第二次放置完成证据：
返航目标与 move_base 状态：
返航开始/到达时间（目标阶段上限 40 秒）：
基地到达/零速度证据：
播报 1-2-3 录音或观察：
播报完成时间（必须早于 450 秒硬截止）：
最终零速度/安全清理完成时间（必须 <480 秒）：
完成后 30 秒观察：
异常与日志路径：
```

任何失败都应保留对应任务点和日志，不得用人工结果替换，也不得为了形成“通过”结论删除失败运行。
