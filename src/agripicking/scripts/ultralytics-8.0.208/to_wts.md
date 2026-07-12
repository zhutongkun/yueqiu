### How to Run, yolov8n as example
// download https://github.com/ultralytics/assets/releases/yolov8n.pt
// download https://github.com/lindsayshuo/yolov8-p2/releases/download/VisDrone_train_yolov8x_p2_bs1_epochs_100_imgsz_1280_last.pt (only for 10 cls p2 model)
cp {tensorrtx}/yolov8/gen_wts.py {ultralytics}/ultralytics
cd {ultralytics}/ultralytics
python gen_wts.py -w yolov8n.pt -o yolov8n.wts -t detect
// a file 'yolov8n.wts' will be generated.


// For p2 model
// download https://github.com/lindsayshuo/yolov8_p2_tensorrtx/releases/download/VisDrone_train_yolov8x_p2_bs1_epochs_100_imgsz_1280_last/VisDrone_train_yolov8x_p2_bs1_epochs_100_imgsz_1280_last.pt (only for 10 cls p2 model)
python gen_wts.py -w VisDrone_train_yolov8x_p2_bs1_epochs_100_imgsz_1280_last.pt -o VisDrone_train_yolov8x_p2_bs1_epochs_100_imgsz_1280_last.wts -t detect (only for  10 cls p2 model)
// a file 'VisDrone_train_yolov8x_p2_bs1_epochs_100_imgsz_1280_last.wts' will be generated.