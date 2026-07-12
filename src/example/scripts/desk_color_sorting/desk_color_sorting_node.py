#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 颜色分拣
import os
import sys
import cv2
import math
import queue
import rospy
import colors
import signal
import threading
import numpy as np
import sdk.common as common
import message_filters
import transforms3d as tfs
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool,Trigger,TriggerResponse
from interfaces.msg import ColorsInfo, ColorDetect, ROI
from interfaces.srv import SetColorDetectParam, SetCircleROI
from interfaces.srv import GetRobotPose
from servo_controllers.bus_servo_control import set_servos
from servo_msgs.msg import MultiRawIdPosDur
from kinematics import kinematics_control

def xyz_quat_to_mat(xyz, quat):
    mat = tfs.quaternions.quat2mat(np.asarray(quat))
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

def xyz_euler_to_mat(xyz, euler, degrees=True):
    if degrees:
        mat = tfs.euler.euler2mat(math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]))
    else:
        mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

def mat_to_xyz_euler(mat, degrees=True):
    t, r, _, _ = tfs.affines.decompose(mat)
    if degrees:
        euler = np.degrees(tfs.euler.mat2euler(r))
    else:
        euler = tfs.euler.mat2euler(r)
    return t, euler

def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])

class ColorSortingNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.center = None
        self.start_pick = False
        self.pick_state = False
        self.place_state = False
        self.target_color = None
        self.endpoint = None
        self.queue = queue.Queue(maxsize=1) #图像队列
        self.hand2cam_tf_matrix_old = [[0.0,0.0,1.0,-0.105],
                                   [-1.0,0.0,0.0,0.0],
                                   [0.0,-1.0,0.0,0.044],
                                   [0.0,0.0,0.0,1.0]]
        self.hand2cam_tf_matrix = [[0.0, 0.0, 1.0, -0.101],
                                  [-1.0, 0.0, 0.0, 0.011],
                                  [0.0, -1.0, 0.0, 0.045],
                                  [0.0, 0.0, 0.0, 1.0]]
        self.offset_x = 0 
        self.offset_y_t = 0
        self.offset_y_f = 0
        self.offset_z = 0 
        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
        signal.signal(signal.SIGINT, self.shutdown)
        self.servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        

        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/servo_manager/init_finish') and rospy.get_param(
                        '/joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        rospy.sleep(1)
        set_servos(self.servos_pub, 2, ((1, 500), (2, 500), (3, 150), (4, 100), (5, 500), (10, 200)))
        rospy.sleep(2)

        # rospy.Subscriber('gemini_camera/rgb/image_raw', Image, self.image_callback)
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        rospy.wait_for_service('/gemini_camera/set_ldp') #等待相机ldp服务
        rospy.wait_for_service('/kinematics/get_current_pose') #等待相机ldp服务

        rospy.ServiceProxy('/gemini_camera/set_ldp', SetBool)(False) #关闭相机ldp服务器，近距离也能进行识别

        self.rgb_sub = message_filters.Subscriber('/gemini_camera/rgb/image_raw', RosImage, queue_size=1) # rgb话题
        self.depth_sub = message_filters.Subscriber('/gemini_camera/depth/image_raw', RosImage, queue_size=1) # 深度话题
        self.info_sub = message_filters.Subscriber('/gemini_camera/depth/camera_info', CameraInfo, queue_size=1) # 相机内参话题
        # 同步时间戳, 时间允许有误差在0.03s
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub], 3, 0.03)
        self.sync.registerCallback(self.multi_callback) #执行反馈函数
        
        # threading.Thread(target=self.pick, daemon=True).start()

        threading.Thread(target=self.goto_default, args=()).start() #得到相机目前的末端位置
        rospy.set_param('~init_finish', True)
        print("ok")
        # try:
            # rospy.spin()
        # except exception as e:
            # rospy.logerr(str(e))
        self.color_sorting() 
    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image, depth_camera_info))

    # 得到机械臂末端坐标
    def goto_default(self):
        while not rospy.is_shutdown():
            endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)()
            # print(endpoint)
            pose_t = endpoint.pose.position
            pose_r = endpoint.pose.orientation
            self.endpoint = xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def start_srv_callback(self, msg):
        rospy.loginfo("start color sorting")
        
        self.start = True
         
        return TriggerResponse(success=True)
    
    def stop_srv_callback(self, msg):
        rospy.loginfo('stop color sorting')

        return TriggerResponse(success=True)   

    def get_area_max_contour(self,contours, threshold=100):
        """
        获取轮廓中面积最重大的一个, 过滤掉面积过小的情况
        :param contours: 轮廓列表
        :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤
        :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None
        """
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None

    def color_detect(self,img,depth_image,target_color_list):
        h, w = img.shape[:2]
        result_image = img.copy()
        result_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB)
        result_depth_image = cv2.applyColorMap(depth_image.astype(np.uint8), cv2.COLORMAP_HOT)
        img = cv2.resize(img, (int(w/2), int(h/2)))
        center_list = []
        for color in target_color_list:
            img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊
            img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
            mask = cv2.inRange(img_lab, tuple(self.lab_data['lab']['gemini_camera'][color]['min']),tuple(self.lab_data['lab']['gemini_camera'][color]['max']))  # 对原图像和掩模进行位运算
            mask[:,0:90] = np.array([[0,]*90]* 240)
            mask[:,274:319] = np.array([[0,]*45]* 240)
            mask[0:30,:] = np.array([[0,]*320]* 30)
            mask[100:239,:] = np.array([[0,]*320]* 139)
            # 平滑边缘，去除小块，合并靠近的块
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

            # 找出最大轮廓
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            max_contour_area = self.get_area_max_contour(contours, 10)

            # 如果有符合要求的轮廓
            if max_contour_area is not None:
                (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour_area[0]) # 最小外接圆
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形
                angle = rect[2]

                # 圈出识别的的要追踪的色块
                circle_color = colors.bgr[color] if color in colors.bgr else (0x55, 0x55, 0x55)
                cv2.circle(result_image, (int(center_x * 2), int(center_y * 2)), int(radius * 2), circle_color, 2)
                cv2.circle(result_depth_image, (int(center_x * 2)+20,int(center_y * 2)-50), 5, circle_color, -1)
                center_list.append([color,int(center_x * 2), int(center_y * 2),int(depth_image[int(center_y * 2)-50][int(center_x * 2)+20])/1000, int(angle)])
                # center_list.append([color,int(center_x * 2)+20, int(center_y * 2)-50,int(depth_image[int(center_y * 2)-50][int(center_x * 2)+20])/1000, int(radius)])
                # print([color,int(center_x * 2)+20, int(center_y * 2)-50,depth_image[int(center_y * 2)-50][int(center_x * 2)+20]/1000, int(radius * 2)],end="\n")
                center_x,center_y ,radius = 0 , 0 , 0


        cv2.rectangle(result_image, (180, 60), (550, 200), (0, 255, 0), 2)
        return result_image, result_depth_image,center_list

    def get_pose(self,info,center_list):
        K = info.K 
        position = depth_pixel_to_camera((center_list[1], center_list[2]), center_list[3], (K[0], K[4], K[2], K[5]))
        # position[0] += 0.0171
        pose_end = np.matmul(self.hand2cam_tf_matrix, xyz_euler_to_mat(position, (0, 0, 0)))
        # print(pose_end)
        # print(self.endpoint)

        world_pose = np.matmul(self.endpoint, pose_end)
        pose_t, pose_r = mat_to_xyz_euler(world_pose)
        print(pose_t)
        self.offset_x = rospy.get_param('/offset_x')  
        self.offset_y_t = rospy.get_param('/offset_+y')  
        self.offset_y_f = rospy.get_param('/offset_-y')  
        self.offset_z = rospy.get_param('/offset_z')  

        pose_t[0] += self.offset_x 
        if pose_t[1] > 0:
            pose_t[1] += self.offset_y_t 
        else:
            pose_t[1] += self.offset_y_f
        # pose_t[1] += self.offset_y + 0.015 
        pose_t[2] += self.offset_z
        # pose_t[2] += self.offset_z + 0.005

        return pose_t

    def pick(self,pose_t, angle):
        rospy.sleep(0.5)
        pose_t[2] += 0.02
        ret1 = kinematics_control.set_pose_target(pose_t, 85) # 根据逆运动学得到舵机脉宽
        if len(ret1[1]) > 0:
            set_servos(self.servos_pub, 1.5, ((1, ret1[1][0]), (2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]),(5, ret1[1][4])))
            rospy.sleep(1.5)
        pose_t[2] -= 0.05
        # print(pose_t)
        ret2 = kinematics_control.set_pose_target(pose_t, 85)
        # print(ret2)
        if angle != 0 and len(ret2[1]) > 0:
            angle = angle % 180
            angle = angle - 180 if angle > 90 else (angle + 180 if angle < -90 else angle)
            angle = 500 + int(1000 * (angle + ret2[3][-1]) / 240)
        else:
            angle = 500
        if len(ret2[1]) > 0:

            print(angle)
            set_servos(self.servos_pub, 0.5, ((5, angle),))
            rospy.sleep(0.5)
            set_servos(self.servos_pub, 1, ((1, ret2[1][0]), (2, ret2[1][1]), (3, ret2[1][2]), (4, ret2[1][3]),(5, angle)))
            rospy.sleep(1)
            set_servos(self.servos_pub, 0.6, ((10, 750),))
            rospy.sleep(0.6)
        if len(ret1[1]) > 0:
            set_servos(self.servos_pub, 1, ((1, ret1[1][0]), (2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]),(5, angle)))
            rospy.sleep(1)
        set_servos(self.servos_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
        rospy.sleep(2)
        self.place_state = True

    def place(self, target_color):
        print(target_color)
        if target_color == "red":
            set_servos(self.servos_pub, 1, ((1, 685),))
            rospy.sleep(1)
            set_servos(self.servos_pub, 2, ((1, 685), (2, 230), (3, 315), (4, 335), (5, 500), (10, 650)))
            rospy.sleep(2)
            set_servos(self.servos_pub, 0.6, ((10, 200),))
            rospy.sleep(0.6)
            set_servos(self.servos_pub, 2, ((1, 685), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
            rospy.sleep(2)
            set_servos(self.servos_pub, 1, ((1, 500),))
            rospy.sleep(1)
        elif target_color == "green":
            set_servos(self.servos_pub, 1, ((1, 750),))
            rospy.sleep(1)
            set_servos(self.servos_pub, 2, ((1, 750), (2, 265), (3, 250), (4, 335), (5, 500), (10, 650)))
            rospy.sleep(2)
            set_servos(self.servos_pub, 0.6, ((10, 200),))
            rospy.sleep(0.6)
            set_servos(self.servos_pub, 2, ((1, 750), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
            rospy.sleep(2)
            set_servos(self.servos_pub, 1, ((1, 500),))
            rospy.sleep(1)
        elif target_color == "blue":
            set_servos(self.servos_pub, 1, ((1, 840),))
            rospy.sleep(1)
            set_servos(self.servos_pub, 2, ((1, 840), (2, 265), (3, 250), (4, 280), (5, 500), (10, 650)))
            rospy.sleep(2)
            set_servos(self.servos_pub, 0.6, ((10, 200),))
            rospy.sleep(0.6)
            set_servos(self.servos_pub, 2, ((1, 840), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
            rospy.sleep(2)
            set_servos(self.servos_pub, 1, ((1, 500),))
            rospy.sleep(1)
        elif target_color == "yellow":
            set_servos(self.servos_pub, 1, ((1, 910),))
            rospy.sleep(1)
            set_servos(self.servos_pub, 2, ((1, 910), (2, 265), (3, 250), (4, 280), (5, 500), (10, 650)))
            rospy.sleep(2)
            set_servos(self.servos_pub, 0.6, ((10, 200),))
            rospy.sleep(0.6)
            set_servos(self.servos_pub, 2, ((1, 910), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
            rospy.sleep(2)
            set_servos(self.servos_pub, 1, ((1, 500),))
            rospy.sleep(1)
        set_servos(self.servos_pub, 2, ((1, 500), (2, 500), (3, 150), (4, 100), (5, 500), (10, 200)))
        rospy.sleep(3)
        self.pick_state = True

    def color_sorting(self):
        self.pick_state = True
        count = 0
        tamp_color = None
        while self.running:
            ros_rgb_image, ros_depth_image, depth_camera_info = self.queue.get(block=True)
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)

            depth_image = depth_image.copy()
            if self.pick_state : 
                # result_image, result_depth_image,center_list = self.color_detect(rgb_image,depth_image,["blue"])
                result_image, result_depth_image,center_list = self.color_detect(rgb_image,depth_image,["blue","red","green","yellow"])
            else:
                result_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
                result_depth_image = cv2.applyColorMap(depth_image.astype(np.uint8), cv2.COLORMAP_HOT)
            # print(self.pick_state,center_list)
            if center_list != [] :
                if center_list[0][0] == tamp_color:
                    count += 1
                else:
                    tamp_color = center_list[0][0] 
            else:
                count = 0
                
            if count > 10  and self.pick_state:
                pose = self.get_pose(depth_camera_info,center_list[0])
                self.pick_state = False
                self.target_color = str(center_list[0][0])
                print(pose)
                print(self.target_color)
                angle = center_list[0][4]
                if angle < 45:
                    angle += 180
                else:
                    angle -= 180
                threading.Thread(target=self.pick, args=(pose, angle)).start()
                rospy.sleep(1)
                center_list = []
                pose = []
                count = 0
            elif self.place_state and self.target_color != None:
                # self.place(self.target_color)
                threading.Thread(target=self.place, args=([self.target_color])).start()
                self.target_color = None
                self.place_state = False
            else:
                rospy.sleep(0.01)

            # cv2.rectangle(result_image, (180, 60), (550, 200), (0, 255, 0), 2)
            cv2.imshow("color_detect", result_image)
            # cv2.imshow("color_detect_depth", result_depth_image)
            key = cv2.waitKey(1)
            if key != -1:
                rospy.signal_shutdown('shutdown1')


if __name__ == "__main__":
    ColorSortingNode('desk_color_sorting')
