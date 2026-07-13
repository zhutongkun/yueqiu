# 运行环境检查报告

检查时间：2026-07-13（Asia/Shanghai）

## 1. 结论

本次可访问的执行机是 Windows 11/AMD64 开发电脑，不是月球小车上的 Ubuntu/Jetson 主机。该机器没有可用 ROS1、catkin 或 WSL Linux 发行版；目标地址 `192.168.149.1:22` 的 TCP 和 Ping 检查均失败，因此无法登录小车执行 ROS、JetPack、CUDA、TensorRT、相机、雷达、串口和 GPU 检查。

因此，本报告将“本机实际结果”“源码静态线索”“必须上车确认”严格分开。不得用本机 RTX 4060 或 `nvidia-smi` 输出代替 Jetson 环境结论，也不得声称 catkin 或实机话题已经通过。

## 2. 当前执行机实际结果

| 项目 | 实际结果 | 判定 |
| --- | --- | --- |
| 操作系统 | Windows 11 Home China 25H2，内核版本 `10.0.26200.8655` | 非目标 Ubuntu |
| 架构 | `AMD64` | 非 Jetson ARM64 |
| PowerShell | 5.1.26100.8655 | 当前 shell |
| `ROS_VERSION` | 空 | 未安装/未 source ROS |
| `ROS_DISTRO` | 空 | 无法确认目标发行版 |
| `python --version` | Python 3.11.9 | 仅开发机 Python |
| `python3 --version` | Windows App Execution Alias 无法启动 | 不是可用 Linux `python3` |
| `catkin_make` | 未找到 | 无法本机 catkin 编译 |
| `catkin` | 未找到 | 无法本机 catkin build |
| `roscore` | 未找到 | 无 ROS master |
| `rostopic` | 未找到 | 无法查询实机话题 |
| `rosservice` | 未找到 | 无法查询实机服务 |
| WSL | `wsl.exe` 存在，但无已安装发行版，命令提示先执行安装 | 无 Ubuntu/ROS 环境 |
| 默认 Python `torch` | 未安装 | 原生 PyTorch checkpoint 加载未在本机完成 |
| 默认 Python `cv2` | 未安装 | 默认 Python 不能直接运行 OpenCV ROS 节点 |
| `nvcc` | 未找到 | 无本机 CUDA Toolkit 编译器 |
| GPU | Windows 主机可见 NVIDIA GeForce RTX 4060 Laptop GPU；驱动 581.80，驱动报告 CUDA 13.0 capability | 与目标 Jetson 无关 |
| 小车 SSH | `192.168.149.1:22` TCP 失败，Ping 超时 | 目标不可达 |
| 当前仓库 | `C:\Users\Lenovo\Documents\月球小车` | Windows 源码副本，不是已确认的 `~/ros_ws` |

Windows 上不能执行 `uname -a`、`cat /etc/os-release` 或读取 `/proc/device-tree/model`。本次没有用伪造的 Linux 输出替代这些检查。

## 3. 源码与资料静态线索

| 线索 | 结论边界 |
| --- | --- |
| `.robotrrc` source `/opt/ros/noetic/setup.{bash,sh,zsh}` | 仓库当前配置强烈指向 ROS1 Noetic，但不能替代目标机 `echo $ROS_DISTRO` |
| `package.xml`/`CMakeLists.txt` 使用 catkin、`rospy`、`roscpp` | 项目是 ROS1/catkin，不是 ROS2/colcon |
| 大量 `rospy.Service`、`ServiceProxy`、ROS1 launch XML | ROS1 接口静态确认 |
| 技术手册第 28 页 | 标准 Jetson Nano 资料为 Ubuntu 18.04/ROS Melodic；高性能 Orin Nano 为 Ubuntu 20.04/ROS Noetic，故必须识别实机型号 |
| 实现引导第 21 页 | 旧部署工作空间示例为 `~/ros_ws`；Orin Nano 启动提示使用 `bash start.sh`，但路径只能作为线索 |
| `position_correction_pick.launch` | 当前比赛 launch 静态确认 |
| `/controller/cmd_vel` | 主程序、move_base、对齐、坡面和底盘控制源码共同确认 |
| `/astra_camera/rgb/image_raw` | 原 YOLO 和本分支月球检测的默认 RGB 话题；实机仍需确认 |
| `/gemini_camera/rgb/image_raw`、`/gemini_camera/depth/image_raw` | 对齐、夹取和坡面源码使用；实机仍需确认 |

## 4. 目标小车尚未确认的项目

以下项目全部标记为“需要小车实机确认”：

- Ubuntu 版本、`uname -a` 和 ARM64 架构。
- `ROS_VERSION`、`ROS_DISTRO`、ROS Python 路径和 catkin 编译方式。
- Jetson Nano B01 还是 Orin Nano，以及 `/proc/device-tree/model`。
- JetPack/L4T 版本（`/etc/nv_tegra_release`、`dpkg-query`）。
- CUDA Toolkit/driver、cuDNN、TensorRT 版本。
- ROS 节点实际使用的 Python 版本。
- PyTorch、torchvision、OpenCV、cv_bridge 的实际版本和 ABI 兼容性。
- `torch.cuda.is_available()`、CUDA device name、显存余量和实际推理延迟。
- 工作空间真实位置、`catkin_make` 或 `catkin build` 的实际选择。
- `/astra_camera/*`、`/gemini_camera/*`、`/scan`、`/odom`、`/move_base/*`、`/controller/cmd_vel` 的真实发布者、类型和频率。
- `/position_correction/*`、`/shape_recognition/*`、`/ramp/*`、`/yolov5/*` 的服务是否与源码一致。
- `start_app_node.service` 状态及其对相机、雷达、串口、底盘、麦克风和声卡的占用。
- USB 声卡、`aplay`、`amixer` 和比赛离线 WAV 的实际播放设备。

## 5. 上车后必须执行的命令

在能 SSH 登录目标小车后，必须原样保存以下输出并更新本报告：

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
python3 - <<'PY'
import cv2
import torch
print('OpenCV:', cv2.__version__)
print('PyTorch:', torch.__version__)
print('CUDA available:', torch.cuda.is_available())
if torch.cuda.is_available():
    print('CUDA device:', torch.cuda.get_device_name(0))
PY
pwd
find .. -maxdepth 2 -name .catkin_workspace -o -name catkin_tools 2>/dev/null
systemctl is-active start_app_node.service
rostopic list
rosservice list
```

还需分别执行 `rostopic info`/`rostopic hz` 检查实际 RGB 图像、深度图、雷达、里程计和底盘话题。只有这些检查完成后，才能把对应条目改为“实机通过”。

## 6. 构建状态

本机不存在 ROS/catkin，无法执行有效的 `catkin_make`。Windows 上的 Python、XML、YAML、shell 静态检查不能替代 ROS1 构建。最终 catkin 编译和 launch 烟雾测试必须在目标 Ubuntu ROS1 工作空间完成；在此之前，任何报告都应写“需要小车实机确认”，不能写“catkin 编译通过”。
