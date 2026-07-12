#!/usr/bin/python3
#coding=utf8
# YOLOv8 OBB识别节点

import rospy
import cv2
import math
import time
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

class YOLOv8Node:
    def __init__(self):
        rospy.init_node('yolov8_node', anonymous=True)
        self.tensorrt_model = YOLO("/home/ubuntu/ros_ws/src/example/scripts/garbage_classification_obb/best.engine", task='obb')
        self.pub = rospy.Publisher('/yolov8/obb_data', Float32MultiArray, queue_size=10)
        self.bridge = CvBridge()

        # 初始化帧率计算变量
        self.start_time = time.time()
        self.frame_count = 0
        self.fps = 0
        
        # 当前帧
        self.current_frame = None
        
        # 订阅相机话题
        self.image_sub = rospy.Subscriber('/gemini_camera/rgb/image_raw', Image, self.image_callback)
        
        # 等待第一帧到达
        rospy.loginfo("等待相机画面...")
        while self.current_frame is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rospy.loginfo("相机画面已接收，开始处理...")
        self.run()

    def image_callback(self, data):
        """处理订阅到的图像消息"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"转换图像失败: {e}")

    def run(self):
        while not rospy.is_shutdown():
            if self.current_frame is None:
                rospy.sleep(0.01)  # 短暂休眠以避免CPU占用过高
                continue
                
            # 复制当前帧以避免处理过程中被回调函数更新
            frame = self.current_frame.copy()
            
            # 更新帧率信息
            self.frame_count += 1
            if time.time() - self.start_time >= 1.0:
                self.fps = self.frame_count
                self.frame_count = 0
                self.start_time = time.time()

            results = self.tensorrt_model(frame)
            for result in results:
                obb = result.obb
                if obb is not None:
                    xywhr = obb.xywhr
                    conf = obb.conf
                    cls = obb.cls
                    for i in range(len(xywhr)):
                        x, y, w, h, r = xywhr[i]
                        angle_in_degrees = math.degrees(r)
                        depth = 300.0  # 假设固定深度值

                        msg = Float32MultiArray()
                        msg.data = [x, y, depth, angle_in_degrees, cls[i], conf[i]]
                        
                        # 打印将要发布的数据
                        # rospy.loginfo(f"Publishing data: X={x}, Y={y}, Depth={depth}, Angle={angle_in_degrees}, Class={cls[i]}, Confidence={conf[i]}")
                        
                        self.pub.publish(msg)

            # 绘制检测框和帧率信息在图像上并显示
            annotated_frame = result.plot()
            cv2.putText(annotated_frame, f"FPS: {self.fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow('YOLOv8 OBB Real-time Detection', annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    YOLOv8Node()