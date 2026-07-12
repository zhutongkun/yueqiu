#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os
import cv2
import queue
import rospy
import signal
import threading
import numpy as np
import sdk.fps as fps
from sdk import common
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from interfaces.msg import ObjectInfo, ObjectsInfo
import yolov8_det_trt

MODE_PATH = os.path.split(os.path.realpath(__file__))[0]

class Yolov8Node:
    def __init__(self, name):
        rospy.init_node(name)   

        self.start = False
        self.running = True
        self.image_queue = queue.Queue(maxsize=1)
        signal.signal(signal.SIGINT, self.shutdown)
        
        self.fps = fps.FPS()  
        engine = rospy.get_param('~engine')
        lib = rospy.get_param('~lib')
        conf_thresh = rospy.get_param('~conf_thresh', 0.8)
        self.classes = rospy.get_param('~classes')
        categories = ["red_fruit","green_fruit","red_tag","green_tag"]
        self.target_fruit = rospy.get_param('~target_fruit','None')
        self.last_target_fruit = None
        self.count_ = 0
        self.last_cmera = 'astra_camera'
        self.yolov8 = yolov8_det_trt.YoLov8TRT(os.path.join(MODE_PATH, engine),os.path.join(MODE_PATH, lib),categories)
        if rospy.get_param('~use_astra_camera', False):
            camera = rospy.get_param('/astra_camera/camera_name', 'astra_camera')
        else:
            camera = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
        self.image_sub = rospy.Subscriber('/agricultural_picking/rgb/image_raw', Image, self.image_callback, queue_size=1)
        self.object_pub = rospy.Publisher('~object_detect', ObjectsInfo, queue_size=10)
        self.result_image_pub = rospy.Publisher('~object_image', Image, queue_size=10)

        rospy.Service('/yolov8/start', Trigger, self.start_srv_callback) 
        rospy.Service('/yolov8/stop', Trigger, self.stop_srv_callback)  

        rospy.set_param('~init_finish', True)
        self.image_proc()


    def start_srv_callback(self, msg):
        rospy.loginfo("start yolov8 detect")

        camera = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
        if camera != self.last_cmera:
            self.image_sub.unregister()
        self.last_cmera = camera
        self.image_sub = rospy.Subscriber('/agricultural_picking/rgb/image_raw', Image, self.image_callback, queue_size=1)
        # self.image_sub = rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, self.image_callback, queue_size=1)
        self.running = True
        self.start = True

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop yolov8 detect')
        self.running = False
        self.start = False

        return TriggerResponse(success=True)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(bgr_image)
   
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def image_proc(self):
        while self.running:
            image = self.image_queue.get(block=True)
            try:
                if self.start:
                    # print('yolov8 running')
                    objects_info = []
                    h, w = image.shape[:2]
                    boxes, scores, classid = self.yolov8.infer_video(image)
                    for box, cls_conf, cls_id in zip(boxes, scores, classid):
                        color = yolov8_det_trt.colors(cls_id, True)
                        object_info = ObjectInfo()
                        object_info.class_name = self.classes[cls_id]
                        if object_info.class_name == 'red_tag' or object_info.class_name == 'green_tag':
                            self.target_fruit = object_info.class_name
                        object_info.box = box.astype(int)
                        object_info.score = cls_conf
                        object_info.width = abs(object_info.box[2] - object_info.box[0])
                        object_info.height = abs(object_info.box[3] - object_info.box[1])
                        objects_info.append(object_info)

                        yolov8_det_trt.plot_one_box(
                        box,
                        image,
                        color=color,
                        label="{} {:.2f}".format(
                            self.classes[cls_id], cls_conf
                        ),
                    )

                    if self.last_target_fruit == self.target_fruit:
                        self.count_ += 1
                        if self.count_ > 3:
                            rospy.set_param('~target_fruit',self.target_fruit)
                            self.count_ = 0
                    self.last_target_fruit = self.target_fruit
                    object_msg = ObjectsInfo()
                    object_msg.objects = objects_info
                    self.object_pub.publish(object_msg)
                else:
                    rospy.sleep(0.01)
            except BaseException as e:
                print(e)

            self.fps.update()
            result_image = self.fps.show_fps(image)
            self.result_image_pub.publish(common.cv2_image2ros(result_image, frame_id='yolov8'))
        self.yolov8.destroy() 
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    node = Yolov8Node('yolov8')
