# ROS1 月球探索比赛集成说明

## 当前结论

本分支在原 ROS1/catkin 月球探索程序中增量接入三个“月球场景元素卡片识别”任务点，并修复任务完成后被 15 秒启动逻辑再次执行的问题。主入口仍为：

```bash
roslaunch competition position_correction_pick.launch
```

三个识别点不是完整比赛，也不是返航条件。第三个识别结束后，程序继续执行原有后退、第二次放置、功能节点收尾和机械臂安全复位；只有所有主体任务完成后才返航。确认到达基地并发布零速度后，才按任务点 1、2、3 的固定顺序集中播放离线语音。

当前开发电脑是 Windows 且没有 ROS1/catkin，目标 Jetson 也无法从本机连通。因此源码静态检查、ROS 无关单元测试和模型离线验证可以在开发机执行；catkin 编译、ROS 图、相机、导航、机械臂、声卡及 systemd 服务仍必须在小车实机确认。详见 `docs/moon_competition_test_report.md`。

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
- `/moon_detector/start`
- `/moon_detector/stop`

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
- 原三分类形状识别完成后卸载其 TensorRT/PyCUDA 资源，本轮不再重载；Moon 首次识别时懒加载，并在完成、停止、错误或 reset 时卸载。实际显存余量仍须在目标 Jetson 用 `tegrastats` 验证。
- 不复制其他 Jetson 或桌面 GPU 生成的 TensorRT engine；只能在目标 Jetson 上构建并回归十类结果。
- 未完成实机检查前，不得把本分支标记为比赛验收通过。
