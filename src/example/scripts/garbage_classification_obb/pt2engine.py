from ultralytics import YOLO
model = YOLO("best.pt")   #修改为自己放在同目录下的pt文件名称
model.export(format="engine")  # 生成 'yolov8n.engine'名称的文件
# Run inference
model = YOLO("yolov8n.engine")
results = model("/home/ubuntu/data/image.jpg") #修改自己待检测的图片