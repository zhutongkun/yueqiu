# Moon 推理部署决策

## 1. 决策

本分支默认部署方案是：

`best.pt + 仓库内本地 YOLOv5 源码 + 独立 ROS1 Python3 moon_detector 节点 + device:auto`

不重新训练，不依赖在线 Torch Hub，不把整个主控制程序改成 ROS2，也不把其他机器生成的 TensorRT engine 复制到目标小车。

TensorRT 仅作为目标 Jetson 上验证 PyTorch 路径后可选的性能优化；若需要，必须在该台 Jetson、该版本 JetPack/CUDA/TensorRT 上由 `best.pt` 可重复构建。

## 2. 现有工程能力

源码中已有一条 TensorRT YOLOv5 链路：

- 节点：`scripts/navigation_transport/yolo5/yolov5_node.py`
- engine：`shape_models.engine`
- 插件：`shape_models_libmyplugins.so`
- 服务：`/yolov5/start`、`/yolov5/stop`、`/yolov5/unload`、`/yolov5/calibration`、`/yolov5/shutdown`
- 类别：`cube`、`box`、`cylinder`
- 默认图像：`/astra_camera/rgb/image_raw`

该链路用于矿石任务卡片/形状识别，是原比赛功能的一部分，必须保留。它不能直接复用于 Moon 十分类：类别数和语义不同，engine 与插件的构建平台也未知。

Moon ZIP 中有 YOLOv5 源码、`best.pt` 和 `last.pt`，没有 ONNX、TensorRT engine 或 Moon 专用插件 `.so`。

## 3. 为什么选择本地 PyTorch YOLOv5

- 权重原始格式就是 YOLOv5 `best.pt`，哈希和十类数值推理已经校验。
- 仓库携带匹配的本地 YOLOv5 源码，部署时无需联网下载代码或模型。
- `device:auto` 可以在目标 PyTorch CUDA 可用时选 GPU，不可用时保留 CPU 诊断路径。
- 检测节点与主任务通过 ROS1 服务/话题/参数通信，模型依赖故障不会要求升级整个导航和机械臂环境。
- 主控制和检测脚本当前均保留 Python3 shebang；vendored YOLOv5 要求 Python 3.8 或更新版本。构建安装不会把 Melodic underlay 或整个工作空间强制切换解释器，目标机必须提供能同时导入 `rospy`/`cv_bridge` 和模型依赖的 Python3 环境。

## 4. ROS1 节点边界

本分支 `moon_detector.py` 负责：模型加载、图像订阅、ROI/框面积过滤、多帧投票、单会话结果、状态和调试图。它不负责导航、夹取、坡面、返航、三结果汇总或语音。

接口为：

- `/moon_detector/reset`
- `/moon_detector/start`
- `/moon_detector/stop`
- `/moon_detector/unload`
- `/moon_detector/result` (`std_msgs/String` JSON)
- `/moon_detector/finished` (`std_msgs/Bool`)
- `/moon_detector/status` (`std_msgs/String`)
- 可选 `/moon_detector/debug_image`

每次 start 前由主任务写入当前 `task_point_index` 和 `session_id`。主任务只接受任务点正确、session 匹配且时间晚于会话开始的结果。

## 5. 路径与配置

本分支使用包相对路径，不写 Windows 用户目录或 `/home/ubuntu`：

- 模型：`src/competition/models/moon/best.pt`
- YOLOv5：`src/competition/third_party/yolov5`
- 配置：`src/competition/config/moon_detector.yaml`
- launch：`src/competition/launch/moon_detector.launch`

默认图像话题是源码已经使用的 `/astra_camera/rgb/image_raw`。`moon_detector.yaml` 是默认参数源；`moon_detector.launch` 的覆盖参数默认为空，只有显式传入 `moon_model_path`、`moon_image_topic` 或调试参数时才覆盖 YAML。实机必须确认 Astra 是否为面向场景卡片的相机；Gemini 目前主要服务机械臂、夹取和坡面。

## 6. 依赖策略

目标机应优先使用 JetPack/ROS 镜像已经兼容的系统包和 NVIDIA PyTorch wheel，避免全局升级：

- ROS1 `rospy`、`sensor_msgs`、`std_msgs`、`std_srvs`、`cv_bridge`
- 与 JetPack 匹配的 Python3 PyTorch/torchvision
- 系统 OpenCV 或与 `cv_bridge` ABI 相容的 OpenCV
- NumPy、PyYAML、PyCUDA、TensorRT Python bindings、scikit-learn、transforms3d、SciPy、seaborn 及本地 YOLOv5 的最小运行依赖

部署脚本只能安装确认缺失的安全依赖，不能升级 JetPack、CUDA、TensorRT 或替换 ROS Python。不能用普通 PyPI CUDA wheel 覆盖 Jetson 的 NVIDIA 构建。`ultralytics` 固定为 `8.4.83`；vendored YOLOv5 的 requirements 检查强制 `install=False`，运行脚本同时设置 `PIP_NO_INDEX=1` 和 `YOLOv5_AUTOINSTALL=false`。检测节点会先实际导入并校验依赖，失败时 fail closed，绝不在比赛运行时联网改环境。

## 7. TensorRT 决策门禁

只有同时完成以下检查才考虑 TensorRT：

1. 确认 Jetson 型号、JetPack/L4T、CUDA 和 TensorRT 版本。
2. `best.pt` 在目标机 PyTorch/CPU 或 CUDA 原生加载和十类推理通过。
3. PyTorch 延迟或显存实测无法满足比赛时序。
4. 在目标机导出 ONNX，并在目标机生成 engine。
5. 用十类验证图和真实相机回归类别映射、置信度、NMS 和多帧投票。

严禁复制现有 `shape_models.engine`、其他 Jetson 的 engine 或 Windows GPU 生成的 engine。TensorRT engine 通常绑定 GPU 架构、TensorRT/CUDA 版本、插件 ABI 和构建参数。

若后续生成 Moon engine，必须继续保留 `best.pt` 为源模型，并记录构建命令、输入尺寸、精度（FP32/FP16）、类别顺序、TensorRT 版本和 SHA256。

## 8. 与原 YOLO 的资源协调

原矿石三分类 YOLO 和 Moon 十分类节点可能竞争 Astra 图像、GPU 显存和 CUDA context。本分支在语音/15 秒一次性启动窗口打开前预热原 TensorRT 三分类模型，预热完成后暂停推理但保留已加载资源；矿物卡片形状一旦锁定，就调用 `/yolov5/unload`，注销图像订阅、释放 TensorRT device buffers 并 detach 该节点显式创建的 PyCUDA context，本轮比赛不再恢复旧模型。这样避免把 engine 冷启动时间算进正式任务，也避免在 Moon PyTorch 模型加载时同时持有两个 GPU 模型。

Moon 节点启动时不加载模型，第一次 `/moon_detector/start` 才懒加载 `best.pt`。三个独立会话之间只停止推理会话以避免重复加载；正常完成、stop、error、shutdown 或人工 reset 时调用 `/moon_detector/unload`，取消会话、注销订阅、删除模型并清理 PyTorch 可释放的 CUDA allocator 缓存。PyTorch 进程底层 driver context 未必能在不退出进程时完全消失，因此 Nano 4GB/Orin 的真实回收量必须用 `tegrastats` 实测，不能只凭代码路径宣称显存足够。

任何加载、卸载或服务超时都必须保持底盘零速度，不能让检测异常触发返航或重跑比赛。reset 会先卸载 Moon，再允许下一轮任务重新加载原三分类 YOLO。

## 9. 当前验证边界

已完成：权重大小/哈希、checkpoint 结构和张量检查、OpenCV DNN 数值前向、71 张验证图映射、代理背景测试；目标 Orin Nano 上旧三分类 TensorRT engine 预热、Astra 订阅、服务启动/暂停和 6 秒短时推理循环。

旧三分类 engine 会警告其 plan 可能来自不同设备型号，且当前没有找到对应 ONNX/WTS/PT 源文件，所以不能擅自重建或宣称可移植性问题已解决。Moon 十分类仍未完成真实卡片原生 PyTorch 推理、三会话显存稳定性、场地背景误报、曝光/距离和终态卸载验证；完整 Jetson 比赛部署仍需实车全流程验收。
