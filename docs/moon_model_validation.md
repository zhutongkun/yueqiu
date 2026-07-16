# Moon 月球场景元素卡片模型验证报告

## 1. 结论

`runs/moon/weights/best.pt` 的文件大小和 SHA256 与用户给定值完全一致；checkpoint 的归档结构、张量、类别元数据和数值推理已检查。对 71 张验证图进行 OpenCV DNN 数值验证时，71/71 的主目标类别与标签一致。

目标 Jetson Orin Nano 已使用仓库内 YOLOv5 的 `DetectMultiBackend` 在 CPU 原生加载该权重，确认 10 个类别名称及顺序完全一致，并完成单图推理和 ROS 服务链路验证。GPU 冷加载与资源占用已测，但没有把 GPU 十类严格推理写成通过；真实比赛卡片和场地背景仍需现场验证。

为使验证不再依赖外部完整数据集，仓库现在保存十类各一张验证图片及 `bus.jpg`、`zidane.jpg` 两张代理负样本，位于 `src/competition/models/moon/validation_samples/`。`manifest.json` 固定类别 ID、中英文名称、原验证集文件名、YOLO 真值框、图片尺寸和 SHA256；2026-07-13 已对 12 张图片重新执行静态资产校验并通过。该小型样本集只用于部署冒烟验证，不替代 354 张原始训练/验证数据集。

## 2. 数据集实际统计

数据来源：`Moon(2).zip`。

| 项目 | 实测结果 |
| --- | ---: |
| 训练图片 | 283 |
| 验证图片 | 71 |
| 测试图片 | 0 |
| 总图片 | 354 |
| 图片尺寸 | 全部 640x480 |
| 每图标注框 | 1 |
| 空标签 | 0 |
| 缺失标签 | 0 |
| 损坏图片 | 0 |
| 越界/非法 YOLO 坐标 | 0 |
| 重复图片 | 0 |
| 独立负样本 | 0 |

类别分布：

| ID | English | 中文 | train | val |
| ---: | --- | --- | ---: | ---: |
| 0 | satellite | 卫星 | 29 | 7 |
| 1 | space_station | 空间站 | 30 | 8 |
| 2 | lunar_crater | 月坑 | 43 | 11 |
| 3 | lunar_rover | 月球车 | 29 | 7 |
| 4 | meteorite | 陨石 | 32 | 8 |
| 5 | earth | 地球 | 24 | 6 |
| 6 | lunar_soil | 月壤 | 24 | 6 |
| 7 | moon | 月球 | 24 | 6 |
| 8 | rocket | 火箭 | 24 | 6 |
| 9 | astronaut | 宇航员 | 24 | 6 |

`classes.txt`、`data.yaml` 和 checkpoint 类别顺序一致，必须保持上述 ID，不得重排。`data.yaml` 的 `path` 是训练电脑遗留的 `C:/Users/Tika/Desktop/Moon/dataset`，部署时不能直接使用该绝对路径；训练/验证相对目录本身有效。

数据集用途应描述为“月球场景元素卡片识别”，不能扩展宣称为任意真实月表自然环境识别。

## 3. 权重完整性与训练元数据

| 项目 | 实测值 |
| --- | --- |
| ZIP 内路径 | `runs/moon/weights/best.pt` |
| 分支部署路径 | `src/competition/models/moon/best.pt` |
| 大小 | 14,438,056 bytes |
| SHA256 | `ab953a754cc6ea68742d49ccf26d8b122bb771fe65aa6bd582761bdeaaa7da34` |
| 基础模型 | YOLOv5s |
| 输入尺寸 | 640 |
| 训练轮数 | 100 |
| batch size | 8 |
| 类别数 | 10 |

Moon ZIP 中没有 ONNX 或 TensorRT engine；包含 `best.pt`、`last.pt`、`yolov5s.pt` 和本地 YOLOv5 源码。现有比赛工程里的 `shape_models.engine` 是三分类矿石卡片模型，不属于 Moon 数据集。

## 4. 模型结构和加载验证方法

开发机的初始离线验证环境缺少 PyTorch，因此先采用以下只读方法，没有生成 ONNX 文件：

1. 校验 `best.pt` 文件大小与 SHA256。
2. 解析 PyTorch checkpoint 归档，读取模型元数据和状态张量。
3. 按归档中的 YOLOv5 模型图在内存构建 OpenCV DNN 网络，并融合 BatchNorm。
4. 使用同一组张量权重完成数值前向、YOLO 解码和 NMS。
5. 对全部 71 张验证图运行推理并与 YOLO 标签对照。

由此可确认 checkpoint 不是空文件/截断文件，十类输出结构与权重数值可用。随后已在目标 Jetson 的实际 PyTorch/YOLOv5 环境完成原生 CPU 加载和推理验证。

上述 OpenCV DNN 路径是本次开发机分析阶段的一次数值验证，不是目标部署运行时。仓库内可重复入口为 `tools/validate_moon_assets.py`：它默认无需外部数据集即可重新检查模型大小/SHA256、manifest、十类代表样本和两张负样本；指定严格推理参数时才加载 YOLOv5/PyTorch。vendored YOLOv5 的运行时自动 `pip install` 已移除，依赖损坏时该工具和 ROS 检测节点都会明确失败。

## 5. 十类代表样本

以下各抽取一张验证图；真值均等于预测。时间为该样本数值前向测量，单位 ms。

| 图片 | 真值/预测 | 置信度 | forward ms |
| --- | --- | ---: | ---: |
| `val_0007.jpg` | satellite | 0.949701 | 112.536 |
| `val_0010.jpg` | space_station | 0.943244 | 112.498 |
| `val_0001.jpg` | lunar_crater | 0.948358 | 114.736 |
| `val_0004.jpg` | lunar_rover | 0.934781 | 114.073 |
| `val_0022.jpg` | meteorite | 0.969181 | 112.805 |
| `val_0042.jpg` | earth | 0.963422 | 113.575 |
| `val_0044.jpg` | lunar_soil | 0.949758 | 113.976 |
| `val_0046.jpg` | moon | 0.932208 | 114.934 |
| `val_0048.jpg` | rocket | 0.929501 | 119.361 |
| `val_0049.jpg` | astronaut | 0.925909 | 120.459 |

全部验证集结果：

- 71/71 图片的主目标类别与标签一致。
- 十类代表样本置信度范围为 0.925909-0.969181。
- 全验证集平均 forward 时间为 116.108 ms/图。
- 含预处理、解码和 NMS 的平均端到端时间为 126.136 ms/图。

这些时间来自当前开发机的 OpenCV DNN 数值验证，不能当作 Jetson Nano/Orin Nano 的性能数据。

## 6. 背景和误报代理测试

数据集没有真实负样本，因此使用 YOLOv5 自带非卡片照片和合成背景作为代理：

| 背景 | 最高错误候选 | 结论 |
| --- | --- | --- |
| `bus.jpg` | astronaut 0.01405 | 远低于 0.70，默认阈值过滤 |
| `zidane.jpg` | earth 0.41271 | 在 0.25 阈值会成为误报，在 0.70 阈值被过滤 |
| 黑、白、灰和随机噪声图 | 无大于等于 0.70 的候选 | 默认阈值下无结果 |
| 训练输出图表等非卡片图像 | 无大于等于 0.70 的候选 | 仅能作为弱负样本 |

结论是 0.70 阈值可抑制已测试的代理误报，但这不能替代真实比赛场地背景。上车后必须采集至少空场地、资源库、矿石、坡道、机械臂、人员和屏幕反光背景进行复测。

## 7. 可重复严格验证入口

静态资产校验不加载 PyTorch，也不需要外部 Moon 数据集：

```bash
python tools/validate_moon_assets.py
```

2026-07-13 实测退出码为 0，校验了：`best.pt` 大小和 SHA256、十类顺序、vendored YOLOv5 必需文件、10 张正样本、2 张负样本、manifest 路径边界、图片 SHA256/JPEG 尺寸和 YOLO 标注。完整数据集仍可通过 `--dataset <dataset目录>` 额外复检。

CPU 与 GPU 原生推理分别使用：

```bash
python tools/validate_moon_assets.py --require-cpu-inference
python tools/validate_moon_assets.py --require-gpu-inference
```

两条命令都无需外部数据集，并严格要求十类逐类预测正确；`bus.jpg` 或 `zidane.jpg` 在置信度阈值 0.70 下出现任意检测都会令命令失败并返回非零退出码。还可使用 `--require-inference --device auto|cpu|0` 选择单一设备，并用 `--output-json` 保存机器可读报告。

当前 Windows 默认 Python 3.11 环境实际探测结果为 `torch`、`cv2`、`numpy`、`ultralytics` 均不可导入。CPU 和 GPU 严格命令均已实际运行并以退出码 2 失败，首个明确原因是 `ModuleNotFoundError: No module named 'cv2'`；因此仅 Windows 开发机上的原生 PyTorch 推理不标记为通过。相关 7 项工具单元测试通过，其中包含 0.71 负样本误报故障注入，证明严格失败分支有效，但模拟测试不替代下述目标机实测。

目标 Orin Nano 的补充实测结果：

- checkpoint SHA256 仍为 `ab953a754cc6ea68742d49ccf26d8b122bb771fe65aa6bd582761bdeaaa7da34`。
- `DetectMultiBackend` CPU 加载成功，类别严格为 `satellite, space_station, lunar_crater, lunar_rover, meteorite, earth, lunar_soil, moon, rocket, astronaut`。
- 独立 CPU 模型加载约 9.494 秒；ROS 节点内 `/moon_detector/preload` 约 6.371 秒。
- 640x640 CPU 推理平均约 0.840 秒/帧、最大约 0.936 秒/帧；取得四票的估算时间约 3.238 秒。
- 预加载后的 `/moon_detector/start` 约 1.523 秒，随后 `/stop` 和 `/unload` 均成功。
- GPU 首次加载约 89.5 秒；旧 TensorRT 与 Moon GPU 同时驻留时约使用 6.0/7.3 GiB 内存，只剩约 940 MiB 可用并使用约 496 MiB swap，因此生产配置固定为 CPU。

首次 ROS 加载曾失败于 `ModuleNotFoundError: No module named 'pathlib._local'; 'pathlib' is not a package`。checkpoint 是由较新的 Windows Python 保存的，pickle 引用了 `pathlib._local.WindowsPath`；目标 Linux Python 3.8 既没有该子模块，也不能实例化 WindowsPath。检测节点现在把 checkpoint 中的 Windows/Posix 具体路径类兼容映射到当前系统原生路径类，模型文件本身未被修改。

## 8. 风险控制参数

由于没有独立测试集和真实负样本，本分支默认采用：

- `confidence_threshold: 0.70`
- `iou_threshold: 0.45`
- `vote_window: 7`
- `minimum_votes: 4`
- 多帧最高可信候选投票，不允许单帧确认
- `minimum_box_area`/`maximum_box_area`
- 可选 ROI
- `minimum_consecutive_frames`
- `maximum_result_age`
- 超时返回 `timeout/no_detection`，不强制分类

`zidane.jpg` 的 0.41271 误报证明不应随意把比赛阈值降到 0.25。最终阈值、ROI 和框面积需要按实机卡片距离标定。

## 9. 是否重训

当前没有发现文件损坏、哈希不一致、类别映射错误或验证集数值推理失败，因此不重训。若 Jetson 原生加载失败，先排查 PyTorch/YOLOv5/JetPack 兼容性；只有确认 checkpoint 本身或比赛类别变化后，才记录原因并向用户申请重训。

## 10. 必须上车补测

- 使用十类真实比赛卡片完成目标 Orin CPU 逐类抽样和持续帧率验证。
- 如未来重新评估 GPU，再执行 CUDA 十类严格推理；当前比赛默认 CPU，不依赖 GPU Moon 推理。
- Astra 实际 RGB 话题编码、分辨率和 `cv_bridge` 转换。
- 真实场地负样本和不同光照、距离、角度、遮挡。
- 三个真实任务点的 7 帧投票稳定性与 15 秒超时。
- 原三分类 TensorRT `/yolov5/unload` 后的显存回收、Moon 三会话常驻量、终态 `/moon_detector/unload` 回收量和相机占用；用 `tegrastats` 留存证据。
