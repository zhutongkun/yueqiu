#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/12/19

import cv2
import math
import time
import rospy    
import queue
import signal
import numpy as np
from sdk import common
import message_filters
from std_srvs.srv import SetBool
from kinematics import kinematics_control
from servo_msgs.msg import MultiRawIdPosDur
from sensor_msgs.msg import Image, CameraInfo
from servo_controllers import bus_servo_control
from std_srvs.srv import Trigger, TriggerResponse


class ColorDebug():
    hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.125],
    [-1.0, 0.0, 0.0, 0.011],
    [0.0, -1.0, 0.0, 0.065],
    [0.0, 0.0, 0.0, 1.0]
    ]
    def __init__(self,name):
        rospy.init_node(name, anonymous=False, log_level=rospy.INFO)
        signal.signal(signal.SIGINT, self.shutdown)

        self.image_queue = queue.Queue(maxsize=1)

        self.servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(0.2)

        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
        self.display = rospy.get_param('~display',False)
        self.running = True
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 开始执行功能
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法

        self.start_function()
    def shutdown(self, signum, frame):
        self.running = False
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 550)))
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    def start_srv_callback(self,msg):
        self.start_function()
        return TriggerResponse(success = True)
    
    def start_function(self):
        # 初始化机械臂的姿态
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))

        self.running = True

        self.camera_name = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
        rospy.ServiceProxy('/%s/set_ldp'%self.camera_name, SetBool)(False)        #   获取相机信息
        self.rgb_sub = message_filters.Subscriber('/%s/rgb/image_raw'%self.camera_name, Image, queue_size=1)
        self.depth_sub = message_filters.Subscriber('/%s/depth/image_raw'%self.camera_name, Image, queue_size=1)
        self.info_sub = message_filters.Subscriber('/%s/depth/camera_info'%self.camera_name, CameraInfo, queue_size=1)

        # 同步时间戳, 时间允许有误差在0.03s
        sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub], 3, 0.02)
        sync.registerCallback(self.multi_callback) #执行反馈函数
        rospy.sleep(1)

        self.color_detect()

    def stop_srv_callback(self, msg):

        self.running = False

        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 600)))
        self.tree_pick_time = 0
    
        self.rgb_sub.unregister() #关闭话题
        self.depth_sub.unregister()
        self.info_sub.unregister()

        # 发送消息的返回值
        return TriggerResponse(success=True)


    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def img_pre_process(self):    
        ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True)
        rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        h, w = depth_image.shape[:2]
        rgb_h, rgb_w = rgb_image.shape[:2]
        rgb_image = rgb_image[int((rgb_h - h)/2):h+int((rgb_h - h)/2), :]
        return h, w, rgb_image, depth_image
  

    def contour_detect(self,color_name):
        _, _, rgb_image, _ = self.img_pre_process()
        h, w = rgb_image.shape[:2]
        img = cv2.resize(rgb_image, (int(w/2), int(h/2)))
        img_h, img_w = img.shape[:2]
        # img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊  去噪声

        img_blur = cv2.GaussianBlur(rgb_image, (3, 3), 3) # 高斯模糊  去噪声
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
        color = self.lab_data['lab']['gemini_camera'][color_name]
        mask_red = cv2.inRange(img_lab, tuple(color['min']), tuple(color['max'])) # 二值化
        eroded = cv2.erode(mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)))
        if self.display:
            cv2.imshow(color_name,dilated)
            key = cv2.waitKey(1)
            if key == 'q' :
                rospy.signal_shutdown('shutdown1')
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        return contours, img_h, img_w

    def color_detect(self):
       while not rospy.is_shutdown() and self.running:
            # print('>>>>>>>>>>>>>>>>>>>>>>>> Color_Detect <<<<<<<<<<<<<<<<<<<<<<<<<<')
            red_contours,_,_ = self.contour_detect('red')
            green_contours,_,_ = self.contour_detect('green')



if __name__ == '__main__':
    ColorDebug('color_debug')
    try:
        rospy.spin()
    except exception as e:
        print('enter except')
        rospy.logerr(str(e))
