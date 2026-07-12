#!/usr/bin/env python3
# encoding: utf-8

import cv2
import math
import time
import rospy    
import queue
import signal
import threading
import numpy as np
from enum import Enum
import message_filters

from std_srvs.srv import SetBool
from sdk import pid, common, fps, misc
from interfaces.srv import SetString
from interfaces.srv import GetRobotPose
from interfaces.msg import ObjectInfo, ObjectsInfo

from kinematics import kinematics_control
from servo_msgs.msg import MultiRawIdPosDur
from sensor_msgs.msg import Image, CameraInfo
from servo_controllers import bus_servo_control
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Twist


class TrackAndGrabNode():
    hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.101],
    [-1.0, 0.0, 0.0, 0.011],
    [0.0, -1.0, 0.0, 0.045],
    [0.0, 0.0, 0.0, 1.0]
    ]

    def __init__(self,name):
        rospy.init_node(name, anonymous=False, log_level=rospy.INFO)
        signal.signal(signal.SIGINT, self.shutdown)

        # 机械臂移动的标志
        self.arm_move_flag = 0

        self.mode = rospy.get_param('~mode','get_target_fruit')
        self.display = rospy.get_param('~display', 'false')
        self.target_color = rospy.get_param('~target_color', 'red_fruit')
        self.next_pick_point = rospy.get_param('~next_pick_point', 'pick_point_1')
        self.target_area_judge = rospy.get_param('~target_area_judge', 1800)
        offset = rospy.get_param('/offset') #夹取补偿
        self.move_dist = rospy.get_param('/move_dist') #向左平移距离
        rospy.set_param('~detect_fruit_color',False)
        self.pose_x_offset = offset[0]  #夹取x轴补偿
        self.pose_y_offset = offset[1]  #夹取y轴补偿
        self.pose_z_offset = offset[2]  #夹取z轴补偿

        # 舵机控制
        self.servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        # 麦轮运动控制节点
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        # 发布给yolo11 rgb 图像
        self.yolo11_rgb_pub = rospy.Publisher('~rgb/image_raw', Image, queue_size=1)

        self.roi = [0,0,0,0]
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 开始执行功能
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        rospy.Service('~place', Trigger, self.place_function_callback)  # 机械臂放置
        
        self.endpoint = None
        self.running = False
        self.image_queue = queue.Queue(maxsize=1)
        self.draw_queue = queue.Queue(maxsize=1)


        self.last_pitch_yaw = (0, 0)
        self.last_position = (0, 0, 0)
        self.objects_info = []
        self.tree_pick_time = 0
        self.target_fruit = None
        self.draw_finsh = False


        self.detect_target_fruit_count = 0

        self.green_color = (0,255,0)
        self.green_center_list = []
        self.red_color = (0,0,255)
        self.red_center_list = []
        self.blue_color = (255,0,0)
        self.target_center_ratio = 1
        
        self.inter_cali_one_flag = False
        self.inter_cali_one_count = 0


        self.go_close_one_count = 0
        self.go_close_one_flag = False
        self.go_close_one_distance = 1.75

        self.picking_status = False
        self.begin_to_check = False
        # 在校准模式2下的参数
        # 1 水平第二次 2 前进后退
        self.calibration_mode = 1
        self.horizontal_threshold = 0.05
        self.vertical_threshold = 0.05
        self.robot_move_count = 0


        self.ratio_count = 0
        self.last_recent_dist = 0.0
        self.last_recent_area = 0
        self.recent_dist_count = 0  # 用于判断当前果实距离的稳定次数
        

        # 
        self.pid_yaw = pid.PID(20.5, 1.0, 1.2)
        self.pid_pitch = pid.PID(20.5, 1.0, 1.2)
        self.yaw = 500
        self.pitch = 300


        self.rgb_img = None
        self.camera_name = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
        rospy.ServiceProxy('/%s/set_ldp'%self.camera_name, SetBool)(False) #   获取相机信息
        self.rgb_sub = message_filters.Subscriber('/%s/rgb/image_raw'%self.camera_name, Image, queue_size=10)
        self.depth_sub = message_filters.Subscriber('/%s/depth/image_raw'%self.camera_name, Image, queue_size=10)
        self.info_sub = message_filters.Subscriber('/%s/depth/camera_info'%self.camera_name, CameraInfo, queue_size=10)

        # 接收返回的yolo11 图像
        self.draw_img_sub = rospy.Subscriber('/yolo11/object_image', Image, self.result_image_callback, queue_size=1)
        self.draw_img = None
        
        '''
            green                 red     
        red       red       green      green
            green                 red

            [tree_one]            [tree_two]
        '''
        self.tree_type = None
        self.judege_tree_type = False
        self.tree_left_fruit_color = None

        # 同步时间戳, 时间允许有误差在0.03s
        sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub], 3, 0.02)
        sync.registerCallback(self.multi_callback) #执行反馈函数

        rospy.sleep(1)
        rospy.spin()


    def shutdown(self, signum, frame):
        self.running = False
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 550)))
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    def place_function_callback(self,msg):
        self.place_functon_single()
        return TriggerResponse(success=True)

    def start_srv_callback(self,msg):
        self.start_function()
        # 发送消息的返回值
        return TriggerResponse(success=True)

    def start_function(self):
        # 初始化机械臂的姿态
        mode = rospy.get_param('~detect_mode','get_tree_style')
        rospy.set_param('~pick_fuit_finish','false')
        # detect_target_fruit_color 检查目标果实颜色
        if mode != 'detect_target_fruit_color':
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))
        else:
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 300), (5, 500), (10, 550)))
        if mode == 'get_tree_style':
            self.tree_pick_time = 2
            self.arm_move_flag = rospy.get_param('~arm_move_flag',0)
        # 设置需要夹取的策略
        self.mode = mode
        self.horizontal_threshold = 0.08
        
        rospy.set_param('/yolo11/use_astra_camera',False)
        rospy.sleep(2)
        rospy.ServiceProxy('/yolo11/start', Trigger)()
        rospy.sleep(1)
        # 开始获取yolo11检测得到的结果
        rospy.Subscriber('/yolo11/object_detect', ObjectsInfo, self.get_object_callback)

        self.running = True
        self.start = True

    # 程序在目标检测话题的回调函数执行 get_object_callback
    def get_object_callback(self, msg):
        # 每次循环之后，清空数据
        self.green_center_list.clear()
        self.red_center_list.clear()
        # 识别结果
        self.objects_info = msg.objects
        ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True)
        rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        h, w = depth_image.shape[:2]
        # print('img_pre_process---->h',h)
        # print('img_pre_process---->w',w)
        rgb_h, rgb_w = rgb_image.shape[:2]
        rgb_image = rgb_image[int((rgb_h - h)/2):h+int((rgb_h - h)/2), :]
        # 因为深度相机的回传华和目标检测的回传画面做标点不匹配，所以需要对图像的长和宽进行比例缩缩放
        # rgb_h：回传画面的高度 h：深度画面的高度
        # rgb_w：回传画面的高度 w：深度画面的高度
        new_h_ratio = h / rgb_h
        new_w_ratio = w / rgb_w
        # h, w, rgb_img_raw, depth_img_raw,depth_camera_info = self.img_pre_process()
        # print('rgb_h---->h',rgb_h)
        # print('rgb_w---->w',rgb_w)
        # print('new_h_ratio---->h',new_h_ratio)
        # print('new_w_ratio---->w',new_w_ratio)
        rgb_img = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR) # 转换到 BGR 空间
        self.rgb_img = rgb_img
        if self.objects_info == []:  # 没有识别到时重置变量
            print('Detect None')
            self.roi = [0,0,0,0]
            self.mecanum_pub.publish(Twist())
        else:
            print('current_mode',self.mode)
            if self.mode == 'end_pick':
                print('Last Pict Finsh,Ready For the Secon begin')
                stop_service = rospy.ServiceProxy('~stop', Trigger)
                rospy.sleep(2)
                response = stop_service()
                rospy.loginfo("stop_service response: %s", response.message)
                rospy.set_param('~pick_fuit_finish','true')
                return
            for i in self.objects_info:
                class_name = i.class_name
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                box_w,box_h = int (i.box[2]),int(i.box[3]),
                if self.mode == 'detect_target_fruit_color':
                    if class_name == 'red_tag' or class_name == 'green_tag':
                        self.detect_target_fruit_count += 1
                        if class_name == 'red_tag':
                            self.target_fruit = 'red_fruit'
                        else:
                            self.target_fruit = 'gren_fruit'
                        # 稳定检测
                        if self.detect_target_fruit_count > 5:
                            self.detect_target_fruit_count = 0
                            self.mode = 'detect_fruit_color_finish'

                if class_name == 'green_fruit':
                    dist = self.depth_distance(center,depth_image,h,w,new_h_ratio,new_w_ratio)
                    if np.isnan(dist):
                        dist = 0.0
                    print('green_dist',dist)
                    self.green_center_list.append((center,dist,box_w,box_h))
                if class_name == 'red_fruit':
                    dist = self.depth_distance(center,depth_image,h,w,new_h_ratio,new_w_ratio)
                    print('red_dist',dist)
                    self.red_center_list.append((center,dist,box_w,box_h))
                    if np.isnan(dist):
                        dist = 0.0
            
            if self.mode == 'pick_finish':
                print('----[[[[[[[[Pick Finish]]]]]]]]---')
                near_depth = 100000
                near_area = 0
                center_x = w
                caculate_furit_list = self.red_center_list if self.target_color == 'red_fruit' else self.green_center_list
                if len(caculate_furit_list) <= 0:
                    self.mode = 'end_pick'
                    self.running = False
                else:
                    for i in caculate_furit_list:
                        if i[1] < near_depth:
                            center_x = i[0][0]
                            near_depth = i[1]
                            near_area = i[2] * i[3]
                    # 如果上一个夹取的果实相比于下一个夹取的果实在一定范围不满足，说明夹取成功
                    if abs(self.last_recent_dist - near_depth) > 0.1 or self.last_recent_area / near_area < 0.5:
                            self.mode = 'end_pick'
                            self.running = False
                    elif abs(center_x / w - 1) > 0.9:
                            self.mode = 'end_pick'
                            self.running = False

                    else:
                        self.mode = 'judge_pick'

            if self.mode == 'get_tree_style':
                min_red_x  = 10000
                min_green_x  = 10000
                for i in self.red_center_list:
                    if i[0][0] < min_red_x:
                        min_red_x = i[0][0]
                for i in self.green_center_list:
                    if i[0][0] < min_green_x:
                        min_green_x = i[0][0]
                if min_red_x <= 1000 and min_green_x <= 1000:
                    if min_red_x < min_green_x:
                        self.tree_type = 'tree_one'
                        self.tree_left_fruit_color = 'red_fruit'
                    else:
                        self.tree_left_fruit_color = 'green_fruit'
                        self.tree_type = 'tree_two'
                    print('tree_type is',self.tree_type)
                    self.judege_tree_type = True
                    self.mode = 'inter_cali_two'
                else:
                    self.mode = 'get_tree_style'

            elif self.mode == 'inter_cali_two':
                print('!!!!!!!!!!!!! inter_cali_two !!!!!!!!!!!!!!!!!!!!!')
                cv2.line(self.draw_img,
                    (int((w/self.target_center_ratio)/new_w_ratio * 0.5),1),
                    (int((w/self.target_center_ratio)/new_w_ratio * 0.5),int(h/new_h_ratio)),
                    self.blue_color,
                    2)
                largest_distance = 10000
                largest_center = [10000,10000]
                horizontal = 0 # 水平移动

                direction_flag = 1
                twist = Twist()

                caculate_furit_list = self.red_center_list if self.target_color == 'red_fruit' else self.green_center_list
                for i in caculate_furit_list:
                    if self.tree_left_fruit_color == self.target_color:
                        if i[0][0] < largest_center[0]:  # 使用最左边的点进行判断
                            largest_center[0],largest_center[1] = i[0][0],i[0][1]  # 中心点位置（x,y）
                            horizontal = largest_center[0] - (int(w/self.target_center_ratio * 0.5))  # 到中间水平方向的距离
                            largest_distance = i[1]  # 深度信息
                    else:
                        if i[1] < largest_distance:  # 使用最近的距离作为判断
                            largest_center[0],largest_center[1] = i[0][0],i[0][1]
                            horizontal = largest_center[0] - (int(w/self.target_center_ratio * 0.5))  # 到中间水平方向的距离
                            largest_distance = i[1]  # 深度信息
                
                direction_flag = 1 if horizontal < 0 else -1  # 平移方向

                #  1 水平
                if self.calibration_mode == 1:
                    if self.robot_move_count >= 5:
                        self.robot_move_count = 0
                        horizontal = 0
                        self.calibration_mode = 2
                        twist = Twist()
                        self.mecanum_pub.publish(twist)
                    if abs(horizontal) < 80:
                        self.horizontal_threshold = 0.02
                        twist.linear.y = self.horizontal_threshold * direction_flag
                        if abs(horizontal) < 20:
                            self.robot_move_count += 1
                    else:
                        self.robot_move_count = 0
                        self.horizontal_threshold = 0.08
                        twist.linear.y = self.horizontal_threshold * direction_flag
                    self.mecanum_pub.publish(twist)

                # 2 前进后退
                elif self.calibration_mode == 2:
                    if largest_distance < 0.4: 
                        self.vertical_threshold = 0.05
                        twist.linear.x = self.vertical_threshold
                        if largest_distance < 0.30: # 在 0.28 - 0.32 的距离内进行夹取
                            self.robot_move_count += 1
                    else:
                        self.robot_move_count = 0
                        self.vertical_threshold = 0.08
                        twist.linear.x = self.vertical_threshold

                    if self.robot_move_count >= 2:
                        largest_distance = 0
                        self.mode = 'judge_pick'
                        self.arm_move(self.arm_move_flag)
                        self.arm_move_flag = 2
                        self.mecanum_pub.publish(Twist())
                    else:
                        self.mecanum_pub.publish(twist)

                print('horizontal',horizontal)
                print('largest_distance',largest_distance,' m')
                print('twist.linear.x',twist.linear.x)
            
            elif self.mode == 'judge_pick':
                print('!!!!!!!!!!!!! judge_pick !!!!!!!!!!!!!!!!!!!!!')
                # to do 绘制方框示意图
                target_center = [10000,1000]
                area = 0
                dist = 10000
                furit_list = self.red_center_list if self.target_color == 'red_fruit' else self.green_center_list
                for i in furit_list:
                    # 设置夹取策略：不要过于靠边的果实或者不要距离过于远的果实
                    if abs(int(rgb_w * 0.5) - int(i[0][0])) / rgb_w > 0.8 or i[1] > 0.8:
                        continue
                    if int(i[0][0]) < target_center[0]:  # 使用使用最左边的进行夹取筛选
                        target_center = [int(i[0][0]),int(i[0][1])]
                        temp_dist  = self.depth_distance(target_center,depth_image,h,w,new_h_ratio,new_w_ratio)
                        print('temp_dist->>>>>>>>>>>>>',temp_dist)
                        if temp_dist < dist:
                            dist = temp_dist
                        if area < i[2] * i[3]:
                            area = i[2] * i[3]

                print('----- dist ---')
                if dist == 10000:
                    return
                recent_dist = dist
                if recent_dist - self.last_recent_dist < 0.05:  # 在一定范围内稳定
                    self.recent_dist_count += 1
                else:
                    self.recent_dist_count = 0
                self.last_recent_area = area
                self.last_recent_dist = recent_dist

                print('>>>>>> fruit_distance <<<<',recent_dist)
                # recent_dist += 0.015 # 物体半径补偿
                # recent_dist += 0.015 # 误差补偿
                cv2.putText(self.draw_img,str(recent_dist),(100,100),cv2.FONT_HERSHEY_PLAIN,3,(0,0,255),3)
                cv2.circle(self.draw_img, (int(target_center[0]),int(target_center[1])), 10, (0,255,0), 3)

                center_x = target_center[0]
                center_x_1 = center_x / (w/new_w_ratio)

                if abs(center_x_1 - 0.5) > 0.1: # 相差范围小于一定值就不用再动了
                    self.pid_yaw.SetPoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置
                    self.pid_yaw.update(center_x_1)
                    print('self.pid_yaw.output',self.pid_yaw.output)
                    self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
                else:
                    self.pid_yaw.clear() # 如果已经到达中心了就复位一下 pid 控制器

                center_y = target_center[1]
                center_y_1 = (center_y / (h/new_h_ratio))
                print('center_y_1',center_y_1)
                if abs(center_y_1 - 0.5) > 0.1:
                    self.pid_pitch.SetPoint = 0.5
                    self.pid_pitch.update(center_y_1)
                    print('self.pid_pitch.output',self.pid_pitch.output)
                    self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 720)
                else:
                    self.pid_pitch.clear()
                # 控制机械臂，让目标的果实在视野的中央
                if self.pitch is not None or self.yaw is not None:
                    print('self.pitch',self.pitch)
                    print('self.yaw',self.yaw)
                    bus_servo_control.set_servos(self.servos_pub, 0.02, ((1, self.yaw), (4, self.pitch)))

                try:
                    if self.recent_dist_count >= 20:  # 检测稳定计数 20 次后再进行夹取
                        K = depth_camera_info.K
                        self.get_endpoint()
                        print('----------------center x y:', target_center[0], target_center[1])
                        position = self.depth_pixel_to_camera((target_center[0], target_center[1]), recent_dist, (K[0], K[4], K[2], K[5]))
                        position[0] -= 0.01  # rgb相机和深度相机tf有1cm偏移
                        pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标
                        world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标
                        print('--------------world pose', world_pose)
                        pose_t, pose_R = common.mat_to_xyz_euler(world_pose)
                        print('--------------pose_t', pose_t)
                        print('--------------pose_R', pose_R)
                        #   夹取补偿 pose_t [x, y, z](机械臂坐标系)
                        pose_t[0] += self.pose_x_offset   #x轴坐标增加 xx m
                        pose_t[1] += self.pose_y_offset   #y轴坐标增加 xx m
                        pose_t[2] += self.pose_z_offset   #Z轴坐标增加 xx m

                        self.recent_dist_count = 0  # 开始夹取后清零
                        print('judge_pick: pose_t',pose_t)
                        if not self.picking_status:
                            self.picking_status = True  # 进入夹取的状态不进入这一段函数的计算
                            threading.Thread(target=self.pick, args=(pose_t,)).start()
                            self.mode = 'pick_check'
                    else:
                        print('!!!!! Wait !!!!!')
                        print('self.recent_dist_count',self.recent_dist_count)
                except BaseException as e:
                    print(e)
                    txt = "DISTANCE ERROR !!!"
                self.last_pitch_yaw = (self.pitch,self.yaw)


            elif self.mode == 'pick_check':
                # 等待机械臂到达位置的时候 begin_to_check 再开启
                if self.begin_to_check:
                    target_center = [1000,1000]
                    area = 0
                    check_furit_list = self.red_center_list if self.target_color == 'red_fruit' else self.green_center_list
                    for i in check_furit_list:
                        if int(i[0][0]) < target_center[0]:
                            target_center = [int(i[0][0]),int(i[0][1])]
                        area += (i[2] * new_w_ratio) * (i[3] * new_h_ratio)  # 面积
                    area_ratio = area / (h * w)
                    print('pick_check---->area_ratio',area_ratio)
                    if area_ratio < 1.5:
                        self.ratio_count += 1
                    else:
                        self.ratio_count = 0
                    print('pick_check->>>>>>>>>self.ratio_count',self.ratio_count)
                else:
                    print('waiting to check')

            elif self.mode == 'picking':
                print('>>>>THE ARM IS TRACKING<<<<<<')

        # cv2.line(self.rgb_img,(int(w/self.target_cener_ratio),0),(int(w/self.target_center_ratio),int(h)),self.blue_color,2)

        if self.draw_img is not None:
            cv2.imshow('Detect_img',self.draw_img)
            # roi_img = rgb_img_raw[self.roi[0]:self.roi[1],self.roi[2]:self.roi[3]]
            # cv2.imshow('roi_img',roi_img)
            cv2.waitKey(1)

    def pick(self, position):
        if position[2] < 0.2:
            yaw = 80
        else:
            yaw = 30
        print('function_position',position)
        ret = kinematics_control.set_pose_target(position, yaw)
        #   末端到达抓取点
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret[1][0]), ))
            rospy.sleep(1)
            bus_servo_control.set_servos(self.servos_pub, 2, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
            rospy.sleep(2)
        self.begin_to_check = True
        #   等待检测结果
        while not rospy.is_shutdown():
            rospy.sleep(0.5)
            print('pick->>>>>>>>>self.ratio_count',self.ratio_count)
            if self.ratio_count >= 5:
                print('---------pick check ok--------------')
                self.mode = 'picking'
                self.ratio_count = 0
                break
            else:
                print('---------pick check fail--------------')
                #self.mode = 'pick_check_fail'
                position[2] += 0.006  # 单位 cm
                self.ratio_count = 0
                self.mode = 'pick_check'
                self.servo_control(position)
        self.begin_to_check = False
        rospy.sleep(1)
        #   夹住小球
        bus_servo_control.set_servos(self.servos_pub, 0.5, ((10, 600),))
        rospy.sleep(1)
        #   向下拉，摘下小球
        position[2] -= 0.05
        ret = kinematics_control.set_pose_target(position, yaw)
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
            rospy.sleep(1)

        # 调用果实放置的函数 
        self.place_functon_single()
        self.picking_status = False
        self.yaw = 500
        self.pitch = 300
        self.pid_yaw.clear()
        self.pid_pitch.clear()
        self.mode = 'pick_finish'
        
        
    def depth_distance(self,center,depth_img_raw,h,w,h_ratio = 1,w_ratio = 1):
        center_x,center_y = center[0] * w_ratio, center[1] * h_ratio
        roi = [int(center_y) - 5, int(center_y) + 5, int(center_x) - 5, int(center_x) + 5]
        if roi[0] < 0:
            roi[0] = 0
        if roi[1] > h:
            roi[1] = h
        if roi[2] < 0:
            roi[2] = 0
        if roi[3] > w:
            roi[3] = w
        if self.mode == 'judge_pick':
            self.roi = [roi[0],roi[1],roi[2],roi[3]]
        else:
            self.roi = [0,0,0,0]
        roi_distance = depth_img_raw[roi[0]:roi[1], roi[2]:roi[3]]

        try:
            dist = round(float(np.mean(roi_distance[np.logical_and(roi_distance>0, roi_distance<10000)])/1000.0), 3)
            return dist
        except BaseException as e:
            print(e)
            txt = "DISTANCE ERROR !!!"
            return 0.0
        if np.isnan(dist):
            txt = "DISTANCE ERROR !!!"
            return 0.0


    def depth_pixel_to_camera(self,pixel_coords, depth, intrinsics):
        fx, fy, cx, cy = intrinsics
        px, py = pixel_coords
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth
        return np.array([x, y, z])


    def img_pre_process(self):    
        ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True)
        rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        h, w = depth_image.shape[:2]
        print('img_pre_process---->h',h)
        print('img_pre_process---->w',w)
        rgb_h, rgb_w = rgb_image.shape[:2]
        rgb_image = rgb_image[int((rgb_h - h)/2):h+int((rgb_h - h)/2), :]

        return rgb_h, rgb_w, rgb_image, depth_image,depth_camera_info


    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.yolo11_rgb_pub.publish(ros_rgb_image)
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))


    # 放置果实函数
    def place_functon_single(self):
        rospy.sleep(0.5)
        bus_servo_control.set_servos(self.servos_pub, 1.5, ((1, 500), (2, 550), (3, 145), (4, 160), (5, 500), (10, 550)))
        # 10 550 夹取，300 放置
        rospy.sleep(1.5)
        bus_servo_control.set_servos(self.servos_pub, 2, ((1, 0), (2, 770), (3, 12), (4, 155), (5, 500), (10, 550)))
        rospy.sleep(2)
        bus_servo_control.set_servos(self.servos_pub, 2, ((1, 0), (2, 490), (3, 145), (4, 160), (5, 500), (10, 550)))
        rospy.sleep(2)
        bus_servo_control.set_servos(self.servos_pub, 2, ((1, 0), (2, 490), (3, 145), (4, 160), (5, 500), (10, 300)))
        rospy.sleep(2)
        bus_servo_control.set_servos(self.servos_pub, 1.5, ((1, 0), (2, 550), (3, 145), (4, 160), (5, 500), (10, 550)))
        rospy.sleep(1.5)
        self.arm_move(self.arm_move_flag)
        rospy.sleep(2)

    def arm_move(self,arm_move_flag):
        if arm_move_flag == 0:
            bus_servo_control.set_servos(self.servos_pub, 2, ((1, 600), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))
        elif arm_move_flag == 1:
            bus_servo_control.set_servos(self.servos_pub, 2, ((1, 400), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))
        else:
            bus_servo_control.set_servos(self.servos_pub, 2, ((1, 500), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))

    def servo_control(self, position):
        if position[2] < 0.2:
            yaw = 80
        else:
            yaw = 30
        ret = kinematics_control.set_pose_target(position, yaw)
        # print(ret, position, yaw)

        #   末端到达抓取点
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret[1][0]), ))
            rospy.sleep(1)
            bus_servo_control.set_servos(self.servos_pub, 1.5, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
            print('----------servo control finish')
            rospy.sleep(1.5)

    def get_endpoint(self):
        endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose
        self.endpoint = common.xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
        return self.endpoint
        # if self.endpoint is not None:
        #     return self.endpoint
        # else:
        #     return None   

    def result_image_callback(self,ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
        if self.draw_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.draw_queue.get()
            # 将图像放入队列
        self.draw_queue.put(rgb_image)
        self.draw_img = self.draw_queue.get(block=True)

    def param_clear(self):
        self.start = False
        self.moving = False
        self.running = False
        self.last_pitch_yaw = (0, 0)
        self.last_position = (0, 0, 0)
        self.tree_pick_time = 0

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop')
        # rospy.ServiceProxy('/yolo11/stop', Trigger)()
        # rospy.sleep(1)
        self.param_clear()
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 600)))
        rospy.sleep(1)
        self.mecanum_pub.publish(Twist())

        self.rgb_sub.unregister() #关闭话题
        self.depth_sub.unregister()
        self.info_sub.unregister()
        # 发送消息的返回值
        return TriggerResponse(success=True)

if __name__ == '__main__':
    TrackAndGrabNode('agricultural_picking')
    try:
        rospy.spin()
    except exception as e:
        print('enter except')
        rospy.logerr(str(e))


