# 月球探索比赛小车部署指南

## 1. 适用范围

本指南用于把当前 ROS1/catkin 分支部署到实际 Jetson 小车。部署脚本不改变 JetPack、CUDA、TensorRT、系统 ROS、PyTorch、torchvision 或 OpenCV；如果这些组件缺失或 ABI 不兼容，脚本会停止并要求按小车现有镜像安装匹配版本。

开发电脑无法连接目标小车，且本机没有 ROS1，因此以下标记为“实机”的步骤尚未在本次开发环境执行。执行后应把原始输出补入 `docs/runtime_environment_report.md` 和 `docs/moon_competition_test_report.md`。

## 2. 部署前安全准备

1. 抬起或架空驱动轮，确保测试期间底盘意外速度不会造成伤害。
2. 保持急停、主电源和底盘电源可触达。
3. 确认机械臂活动范围内无人和障碍物。
4. 记录当前可回滚 Git 提交、地图、导航点、舵机参数及 systemd 服务状态。
5. 不在比赛 launch 运行期间启动厂商 APP。

在小车上进入实际工作空间，不假定它一定是 `~/ros_ws`：

```bash
cd /path/to/actual/catkin_workspace
export MOON_WORKSPACE="$(pwd)"
git status -sb
git branch --show-current
git log --oneline -5
```

必须确认当前分支是本任务独立分支，而不是 `main`、`master` 或 `develop`。

## 3. 记录实际运行环境

在 source 任何自定义工作空间前执行并保存输出：

```bash
echo "$ROS_VERSION"
echo "$ROS_DISTRO"
python --version
python3 --version
uname -a
cat /etc/os-release
cat /proc/device-tree/model 2>/dev/null || true
cat /etc/nv_tegra_release 2>/dev/null || true
which catkin_make
which catkin
which roscore
which rostopic
which rosservice
dpkg-query -W 'nvidia-jetpack' 'nvidia-l4t-core' 'tensorrt' 'libnvinfer*' 2>/dev/null || true
nvcc --version 2>/dev/null || true
systemctl is-active start_app_node.service
```

检查 ROS 节点实际使用的 Python 环境：

```bash
python3 - <<'PY'
import cv2
import torch
import torchvision
from cv_bridge import CvBridge

print('OpenCV:', cv2.__version__)
print('PyTorch:', torch.__version__)
print('torchvision:', torchvision.__version__)
print('CUDA available:', torch.cuda.is_available())
if torch.cuda.is_available():
    print('CUDA device:', torch.cuda.get_device_name(0))
CvBridge()
print('cv_bridge: OK')
PY
```

不要用桌面电脑的 `nvidia-smi` 代替 Jetson 的设备树、L4T、CUDA 和 TensorRT 检查。

## 4. 安装检查与 catkin 编译

先核对模型：

```bash
sha256sum "$MOON_WORKSPACE/src/competition/models/moon/best.pt"
stat "$MOON_WORKSPACE/src/competition/models/moon/best.pt"
```

预期 SHA256：

```text
ab953a754cc6ea68742d49ccf26d8b122bb771fe65aa6bd582761bdeaaa7da34
```

执行部署检查和编译：

```bash
cd "$MOON_WORKSPACE"
# 当自动发现的 ROS underlay 不正确时必须显式指定，例如：
# export MOON_ROS_SETUP=/opt/ros_ws/melodic/setup.bash
chmod +x setup_moon_runtime.sh run_moon_competition.sh \
  stop_moon_competition.sh restore_app_service.sh \
  build_moon_tensorrt_engine.sh
./setup_moon_runtime.sh
```

脚本执行内容：

- 确认设备树包含 Jetson 且架构为 aarch64。
- 优先使用显式 `MOON_ROS_SETUP`，否则依次检查 `/opt/ros/<distro>` 和 `/opt/ros_ws/{noetic,melodic}`，并确认 `ROS_VERSION=1`。
- 要求 Moon 运行解释器为 Python 3.8 或更新版本；这不会把整个 Melodic 工作空间强制切换到 Python3，安装规则保留各脚本原有 shebang。
- 若现有工作空间含 `.catkin_tools`，沿用 `catkin build`；否则使用 `catkin_make`。可用 `MOON_CATKIN_BUILD_TOOL` 显式覆盖。
- 对缺失的纯 Python 辅助包仅执行 `pip --user --no-deps`；带本地扩展的依赖缺失时直接失败。
- `ultralytics` 固定为 `8.4.83`；比赛运行环境设置 `PIP_NO_INDEX=1`，vendored YOLOv5 也强制 `install=False`，禁止运行时联网安装。
- 缺少 `torch`、`torchvision`、`cv2`、`numpy`、`pycuda`、`TensorRT`、`scikit-learn`、`transforms3d`、`SciPy`、`seaborn` 或 ROS Python 模块时明确失败，不用 PyPI 普通 wheel 覆盖 NVIDIA/ROS 版本。
- 校验 `best.pt` 哈希。
- 仅在选择 `catkin_make` 且 `src/CMakeLists.txt` 不存在时运行 `catkin_init_workspace`。
- 完成所选 catkin 构建，source 工作空间并实际导入 `message_filters`、`interfaces`、`sdk` 和 `kinematics`。

若编译失败，保存完整命令和首个真实错误。不要删除用户工作区、运行 `git reset --hard`，也不要通过升级整套 JetPack 掩盖依赖错误。

## 5. 实机 ROS 接口检查

编译通过后，先启动底层或使用现有 bringup，检查真实话题和服务：

```bash
source /opt/ros/"$ROS_DISTRO"/setup.bash
source "$MOON_WORKSPACE/devel/setup.bash"
rostopic list
rosservice list
rostopic info /controller/cmd_vel
rostopic info /astra_camera/rgb/image_raw
rostopic hz /astra_camera/rgb/image_raw
rostopic info /gemini_camera/rgb/image_raw
rostopic info /gemini_camera/depth/image_raw
rostopic info /scan
rostopic info /odom
rosservice type /competition/stop_mission
rosservice type /moon_detector/start
rosservice type /position_correction/start
rosservice type /shape_recognition/start
rosservice type /ramp/start
```

必须确认 `/astra_camera/rgb/image_raw` 真正朝向比赛卡片。若相机话题不同，在 launch/YAML 中覆盖参数，不要在 Python 中写死设备路径。

同时确认离线音频：

```bash
aplay -l
amixer scontrols
find "$MOON_WORKSPACE/src/competition/voice/moon" -maxdepth 1 -type f -name '*.wav' -print
```

使用比赛声卡播放单个测试 WAV，验证声卡、音量和中文片段，但不要在三个识别点触发播报。

## 6. 启动比赛

确保机器人在基地初始位、地图和导航点已现场确认，然后执行：

```bash
cd "$MOON_WORKSPACE"
./run_moon_competition.sh
```

额外的 roslaunch 参数可原样传入，例如：

```bash
./run_moon_competition.sh debug:=true
```

脚本会：

1. source ROS 和当前工作空间。
2. 检查并停止冲突的 `start_app_node.service`。
3. 启动 `roslaunch competition position_correction_pick.launch`。
4. 收到 `INT`、`TERM` 或 launch 退出时调用安全停止脚本。
5. 保持 APP 服务为停止状态，不自动恢复。

主节点等待语音开始命令；15 秒没有收到命令时只自动启动一次。第 14.9 秒语音和 Timer 同时到达时，带锁入口只接受一个请求。

## 7. 运行中观测

在另一个终端：

```bash
source /opt/ros/"$ROS_DISTRO"/setup.bash
source "$MOON_WORKSPACE/devel/setup.bash"
rostopic echo /moon_detector/status
rostopic echo /moon_detector/result
rostopic echo /moon_detector/finished
rosparam get /moon_task
rostopic echo /move_base/result
```

每个任务点应创建新 `session_id`，三个槽位独立保存。任务点 1、2、3 都不得现场播放识别结果。第三点完成后应继续第二次放置、功能节点关闭、机械臂安全复位和一次基线激光纠偏坡面穿越；这些全部成功后才允许设置整体任务完成并发送最终基地目标。

默认 `use_ramp_alignment_service=false`。只有在目标小车上单独验证 `/ramp/start -> /ramp/up -> stop` 的车身朝向与激光倒车方案兼容后才能启用；启用时它只是坡前对齐，实际坡面穿越仍只能执行一次。

到达基地前不得播报。到达后应先看到 `/controller/cmd_vel` 零速度，然后只按 1、2、3 播放三条结果。最终检查：

```bash
cat "$MOON_WORKSPACE/runtime/moon_task_results.json"
rosparam get /moon_task/mission_state
rosparam get /moon_task/mission_execution_count
```

预期最终状态为 `COMPLETED`，执行次数为 `1`。继续观察至少 30 秒，确认任务不会第二次启动。

## 8. 安全停止

普通或紧急软件停止：

```bash
cd "$MOON_WORKSPACE"
./stop_moon_competition.sh
```

脚本会尽力调用：

- `/competition/stop_mission`
- `/moon_detector/stop`
- `/ramp/stop`
- `/position_correction/stop`
- `/shape_recognition/stop`
- `/yolov5/stop`
- `/moon_detector/unload`
- `/yolov5/unload`
- `/move_base/cancel`
- `/controller/cmd_vel` 全零速度

软件停止不能替代物理急停。若 ROS master、底盘控制器或网络失效，应立即使用小车硬件急停/断开驱动电源，并在安全后保存日志和 `runtime/moon_task_results.json`。

## 9. 比赛后恢复 APP

确认以下条件全部满足后才能恢复：

- 比赛 launch 已退出。
- 底盘确实静止。
- 机械臂处于安全姿态。
- 没有维护人员处于运动范围内。
- 结果和 ROS 日志已经保存。

人工执行：

```bash
./restore_app_service.sh --confirm
systemctl is-active start_app_node.service
```

`run_moon_competition.sh` 和 `stop_moon_competition.sh` 都不会自动重启该服务。

## 10. 可选 TensorRT 构建

默认生产方案使用 `best.pt` 和本地 YOLOv5 源码。只有原生 PyTorch 十类验证已在目标 Jetson 通过、且实测性能不满足比赛时序时，才考虑 TensorRT。

在目标 Jetson 上确认 `onnx>=1.12`、`onnxscript`、TensorRT Python 包和 CUDA 可用后执行：

```bash
cd "$MOON_WORKSPACE"
./build_moon_tensorrt_engine.sh
```

脚本拒绝非 Jetson/aarch64，使用仓库内 `third_party/yolov5/export.py` 和已校验的 `best.pt` 构建 FP16 640x640 engine，并把 Jetson、L4T、CUDA、TensorRT、PyTorch、OpenCV、Git 提交和 engine SHA256 写入 `runtime/tensorrt/`。构建期间 pip 索引和缓存被禁用；依赖不满足时直接失败，不自动升级目标环境。

已有 engine 时脚本默认拒绝覆盖；明确保存旧元数据后可设置：

```bash
MOON_OVERWRITE_ENGINE=1 ./build_moon_tensorrt_engine.sh
```

构建完成不等于允许比赛使用。必须对十类验证图、真实卡片距离、背景误报、持续帧率和原三分类 YOLO 资源切换重新测试。当前 launch 仍指向 `best.pt`；切换 engine 必须作为单独审查的部署改动，不能现场临时替换且不能把 engine 复制到另一台 Jetson。

## 11. 回滚

1. 调用 `stop_moon_competition.sh` 并使用硬件方式确认停车。
2. 保存 `runtime/`、ROS 日志和 `git diff`。
3. 切回部署前已记录的稳定提交或分支；不要覆盖地图、导航点和用户现场参数。
4. 重新 `catkin_make` 并仅执行原比赛烟雾测试。
5. 确认比赛 launch 已退出后，人工恢复 APP 服务。

Moon 的 `best.pt`、engine 或配置绝不能覆盖原矿石三分类的 `shape_models.engine` 和插件 `.so`。

## 12. 实机验收门槛

以下全部有日志证据后才能标记部署完成：

- ROS1 发行版、Jetson 型号、JetPack、CUDA、TensorRT、PyTorch、OpenCV 和 cv_bridge 已记录。
- catkin 编译通过，比赛 launch 无缺包错误。
- 相机、雷达、里程计、底盘、机械臂、麦克风和声卡接口已确认。
- `best.pt` 原生加载、CPU/GPU 推理和十类映射通过。
- 三个识别点独立，现场不播报，第三点后继续剩余任务。
- 两次夹取、两次放置、坡面和机械臂收尾均完成后才返航。
- 真实到达基地并停车后，按 1、2、3 集中播报。
- 语音/Timer 竞争、重复语音、完成后等待 30 秒均只执行一次。
- 导航失败、检测失败、语音异常、Ctrl+C 和 shutdown 均保持底盘零速度。
- 比赛期间 `start_app_node.service` 不会重新占用硬件。
