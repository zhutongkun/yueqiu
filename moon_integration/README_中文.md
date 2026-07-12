# 月球环境识别集成

本分支加入月球环境识别、训练模型、数据集归档和任务结束语音播报集成文件。

## 安装

```bash
cd moon_integration
chmod +x install.sh
./install.sh /你的/yueqiu/仓库根目录
```

模型会安装到：

```text
src/agripicking/models/moon_yolov5s_state.pt
```

完整训练数据集归档：

```text
moon_integration/assets/moon_dataset.zip
```

识别类别：卫星、空间站、月坑、月球车、陨石、地球、月壤、月球、火箭、宇航员。
