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
import grab_tool_function


class EchoColor(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3
    YELLOW = 4

def echo_str(str,font_color):
    if font_color == EchoColor.BLUE:
        # blue
        print(f"\033[34m\033[47m{str}\033[0m".format(str))
    elif font_color == EchoColor.RED:
        # red
        print(f"\033[31m\033[47m{str}\033[0m".format(str))
    elif font_color == EchoColor.GREEN:
        # green
        print(f"\033[32m\033[47m{str}\033[0m".format(str))
    elif font_color == EchoColor.YELLOW:
        # yellow
        print(f"\033[33m\033[47m{str}\033[0m".format(str))



class ColorTracker:
    def __init__(self, target_color):
        self.target_color = target_color
        self.pid_yaw = pid.PID(20.5, 1.0, 1.2)
        self.pid_pitch = pid.PID(20.5, 1.0, 1.2)
        self.yaw = 500
        self.pitch = 150

    def proc(self, source_image, result_image, color_ranges):
        h, w = source_image.shape[:2]
        color = color_ranges['lab']['gemini_camera'][self.target_color]

        img = cv2.resize(source_image, (int(w/2), int(h/2)))
        img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊  去噪声
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
        mask = cv2.inRange(img_lab, tuple(color['min']), tuple(color['max'])) # 二值化

        # 平滑边缘，去除小块，合并靠近的块
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)))
        # 找出最大轮廓
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        min_c = None
        if contours:
            min_c = max(contours, key=cv2.contourArea)
            circle_judge = cv2.minAreaRect(min_c) 
            # print(circle_judge)
            # 通过半径筛选不规则的图形,减少干扰 (x, y),(w,h),angle
            if  1 - circle_judge[1][0] / circle_judge[1][1] > 0.2:
                return (result_image, None, None, 0)
        
        # for c in contours:
        #     print('ColorTracker.proc.area:',cv2.contourArea(c)/(h*w))
        #     if math.fabs(cv2.contourArea(c))/(h*w) < 50/(640*480):
        #         continue
        #     (center_x, center_y), radius = cv2.minEnclosingCircle(c) # 最小外接圆
        #     if min_c is None:
        #         min_c = (c, center_x)
        #     elif center_x < min_c[1]:
        #         if center_x < min_c[1]:
        #             min_c = (c, center_x)
        

        # 如果有符合要求的轮廓
        if min_c is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(min_c) # 最小外接圆

            # 圈出识别的的要追踪的色块
            circle_color = common.range_rgb[self.target_color] if self.target_color in common.range_rgb else (0x55, 0x55, 0x55)
            cv2.circle(result_image, (int(center_x * 2), int(center_y * 2)), int(radius * 2), circle_color, 2)
            center_x = center_x * 2
            center_x_1 = center_x / w
            if abs(center_x_1 - 0.5) > 0.2: # 相差范围小于一定值就不用再动了
                self.pid_yaw.SetPoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置
                self.pid_yaw.update(center_x_1)
                self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
            else:
                self.pid_yaw.clear() # 如果已经到达中心了就复位一下 pid 控制器

            center_y = center_y * 2
            center_y_1 = center_y / h
            if abs(center_y_1 - 0.5) > 0.2:
                self.pid_pitch.SetPoint = 0.5
                self.pid_pitch.update(center_y_1)
                self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 720)
            else:
                self.pid_pitch.clear()
            # rospy.loginfo("x:{:.2f}\ty:{:.2f}".format(self.x , self.y))
            print('radius *2>>>>>>>>>>>>>>',radius * 2)
            return (result_image, (self.pitch, self.yaw), (center_x, center_y), radius * 2)
        else:
            print('min_c is None!!!!!')
            return (result_image, None, None, 0)


class TrackAndGrabNode():
    hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.125],
    [-1.0, 0.0, 0.0, 0.011],
    [0.0, -1.0, 0.0, 0.065],
    [0.0, 0.0, 0.0, 1.0]
    ]
    def __init__(self,name):
        rospy.init_node(name, anonymous=False, log_level=rospy.INFO)
        signal.signal(signal.SIGINT, self.shutdown)
        self.mode = rospy.get_param('~mode','detect_tree_position')
        self.display = rospy.get_param('~display', 'false')
        self.target_color = rospy.get_param('~target_color', 'red')
        self.next_pick_point = rospy.get_param('~next_pick_point', 'pick_point_1')
        self.target_area_judge = rospy.get_param('~target_area_judge', 1800)
        self.judge_status = 'pick_adjust_x_1'
        self.pick_finish = False
        self.running = False
        self.image_queue = queue.Queue(maxsize=1)
        self.tree_list = []
        self.next_picking_strategy = ''
        self.tree_pick_time = 0  # 树夹取的次数
        self.first_tree_finish = False
        self.timeout_count = 0
        self.arm_move_flag = 0
        self.last_fruit_distance = 0

        self.arm_move_x = 0
        self.arm_move_y = 0

        self.area_pid = pid.PID(1.0, 0.0, 0.0)
        self.servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        # 麦轮运动控制节点
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.2)

        rospy.Service('~set_color', SetString, self.set_color_srv_callback)

        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
        self.red_list = []
        self.green_list = []
        
        self.stamp = time.time()
        self.last_position = (0, 0, 0)
        self.enable_disp = False
        self.moving = False
        self.position = None
        self.endpoint = None
        self.ttt = time.time() + 3
        self.start = rospy.get_param('~start',False)
        self.last_pitch_yaw = (0, 0)
        self.last_position = (0, 0, 0)
        self.go_to_move = False

        self.ratio_count = 0    #   连续检测小球占画面整体比例的次数
        self.turn_up_down = 1   # 检查是否需要上下调整机械臂
        self.ratio_x_count = 0
        self.ratio_y_count = 0

        offset = rospy.get_param('/offset') #夹取补偿
        self.move_dist = rospy.get_param('/move_dist') #向左平移距离
        self.pose_x_offset = offset[0]  #夹取x轴补偿
        self.pose_y_offset = offset[1]  #夹取y轴补偿
        self.pose_z_offset = offset[2]  #夹取z轴补偿

        self.pose_t = None
        self.radius = 0
        
        # yolov8 
        self.objects_info = []
        self.yolov8_rgb_pub = rospy.Publisher('~rgb/image_raw', Image, queue_size=1)
        self.red_min_x = 100000
        self.green_min_x = 100000
        self.yolov8_detect_tree = False
        self.tree_detect_finish = False
        self.tree_detect_last = ''
        self.tree_detect_count = 0
        self.tree_left_fruit_color = None
        self.detect_w = 320
        self.detect_h = 480
        self.detect_center = [int(self.detect_w * 0.5), int(self.detect_h * 0.5)]
        self.detect_move_x_sub = 0
        self.detect_move_finish = False

        self.red_fruit_list = []
        self.green_fruit_list = []

        rospy.Service('~start', Trigger, self.start_srv_callback)  # 开始执行功能
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        rospy.Service('~place', Trigger, self.place_function_callback)  # 机械臂放置


    def shutdown(self, signum, frame):
        self.running = False
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 550)))
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    def place_function_callback(self,msg):
        self.place_function()
        return TriggerResponse(success=True)

    def start_srv_callback(self,msg):
        self.start_function()
        # 发送消息的返回值
        return TriggerResponse(success=True)

    def start_function(self):
        rospy.set_param('/yolov8/use_astra_camera',False)
        rospy.sleep(2)
        rospy.ServiceProxy('/yolov8/start', Trigger)()
        rospy.sleep(1)

        # 初始化机械臂的姿态
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))
        self.running = True
        # 设置需要夹取的策略
        mode = rospy.get_param('~detect_mode','detect_tree_style')
        if mode == 'detect_tree_style':
            self.tree_pick_time = 0
            self.arm_move_flag = rospy.get_param('~self.arm_move_flag',0)

        self.mode = mode
        self.last_pitch_yaw = (0, 0)
        self.last_position = (0, 0, 0)

        self.camera_name = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
        rospy.ServiceProxy('/%s/set_ldp'%self.camera_name, SetBool)(False)        #   获取相机信息
        self.rgb_sub = message_filters.Subscriber('/%s/rgb/image_raw'%self.camera_name, Image, queue_size=10)
        self.depth_sub = message_filters.Subscriber('/%s/depth/image_raw'%self.camera_name, Image, queue_size=10)
        self.info_sub = message_filters.Subscriber('/%s/depth/camera_info'%self.camera_name, CameraInfo, queue_size=10)


        # 同步时间戳, 时间允许有误差在0.03s
        sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub], 3, 0.02)
        sync.registerCallback(self.multi_callback) #执行反馈函数
        rospy.sleep(1)

        self.tracker = ColorTracker(self.target_color)
        # 开始获取yolov8检测得到的结果
        self.object_sub = rospy.Subscriber('/yolov8/object_detect', ObjectsInfo, self.get_object_callback)

        self.start = True
        self.run()

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop')
        rospy.ServiceProxy('/yolov8/stop', Trigger)()
        rospy.sleep(1)

        self.start = False
        self.moving = False
        self.running = False
        self.last_pitch_yaw = (0, 0)
        self.last_position = (0, 0, 0)

        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 600)))
        self.tree_pick_time = 0
    
        self.rgb_sub.unregister() #关闭话题
        self.depth_sub.unregister()
        self.info_sub.unregister()

        # 发送消息的返回值
        return TriggerResponse(success=True)

    def set_color_srv_callback(self, msg):
        self.target_color = msg.data
        rospy.loginfo("set_color")
        self.tracker = ColorTracker(self.target_color)
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))
        rospy.sleep(1)
        self.start = True
        return [True, 'set_color']


    #   速度发布
    def pub_vel(self, dist_x, dist_y):
        twist = Twist()
        if dist_x == 0:
            orientation = dist_y / abs(dist_y)
            twist.linear.y = 0.1 * orientation
            run_time = dist_y / twist.linear.y
        elif dist_y == 0:
            twist.linear.x = 0.1
            run_time = dist_x / twist.linear.x
        
        self.mecanum_pub.publish(twist)


        print('------- move linear_y:', twist.linear.y)
        rospy.sleep(run_time)

        self.mecanum_pub.publish(Twist())
        print('--------- move finish')


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


    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.yolov8_rgb_pub.publish(ros_rgb_image)
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
        img_h, img_w = rgb_image.shape[:2]
        img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊  去噪声
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
        color = self.lab_data['lab']['gemini_camera'][color_name]
        mask_red = cv2.inRange(img_lab, tuple(color['min']), tuple(color['max'])) # 二值化
        eroded = cv2.erode(mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)))
        if self.display:
            cv2.imshow(color_name,dilated)
            key = cv2.waitKey(1)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        return contours, img_h, img_w

    '''
          green                 red     
      red       red       green      green
          green                 red

        [tree_one]            [tree_two]
    '''
    def get_fruit_tree_feature(self,detect_mode):
        print('>>>>>>>>>>>>>>>>>>>>>>>> get fruit position <<<<<<<<<<<<<<<<<<<<<<<<<<')
        red_contours,_,_ = self.contour_detect('red')
        green_contours,_,_ = self.contour_detect('green')
        # tree_style 检测树模式 、 max_area 最大面积模式

        if detect_mode == 'tree_style':
            MAX_AREA = 100000
            '''
            >>>>>>>>>>>>>>>>>>>>>>>> red <<<<<<<<<<<<<<<<<<<<<<<<<<
            '''
            red_x = 0
            red_y = 0
            red_min_x = MAX_AREA
            if red_contours is not None:
                # red_controus_count = len(red_contours)
                # print('red_contours_count:', red_controus_count)
                for contour in red_contours:
                    (red_x, red_y), red_radius = cv2.minEnclosingCircle(contour)  # 最小外接圆
                    if red_x < red_min_x and red_radius > 5:
                        red_min_x = red_x
                    echo_str(((red_x, red_y),red_radius),EchoColor.RED)
                echo_str(red_min_x,EchoColor.RED)
            else:
                print('No red contours found.')
            '''
            >>>>>>>>>>>>>>>>>>>>>>>> green <<<<<<<<<<<<<<<<<<<<<<<<<<
            '''
            green_x = 0
            green_y = 0
            green_min_x = MAX_AREA
            if green_contours is not None:
                # green_controus_count = len(green_contours)
                # print('green_contours_count:', len(green_controus_count))
                for contour in green_contours:
                    (green_x, green_y), red_radius = cv2.minEnclosingCircle(contour)  # 最小外接圆
                    if green_x < green_min_x and red_radius > 5:
                        green_min_x = green_x
                    echo_str(((green_x, green_y),red_radius),EchoColor.GREEN)
                echo_str(green_min_x,EchoColor.GREEN)
            else:
                print('No green contours found.')
            '''
            >>>>>>>>>>>>>>>>>>>>>>>> output <<<<<<<<<<<<<<<<<<<<<<<<<<
            '''
            if green_min_x != MAX_AREA and red_min_x != MAX_AREA:
                if red_min_x < green_min_x:
                    return 'tree_one'
                else:
                    return 'tree_two'
            else:
                return None

        if detect_mode == 'max_area':
            red_radius = 0
            green_radius = 0
            '''
            >>>>>>>>>>>>>>>>>>>>>>>> red <<<<<<<<<<<<<<<<<<<<<<<<<<
            '''
            if red_contours:
                max_contour = max(red_contours, key=cv2.contourArea)
                (_, _), red_radius = cv2.minEnclosingCircle(max_contour)  # 获取最小外接圆的圆心坐标和半径
                # 继续处理找到的最大轮廓
            else:
                print("No red contours found.")

            green_radius = 0
            if green_contours:
                max_contour = max(green_contours, key=cv2.contourArea)
                (_, _), green_radius = cv2.minEnclosingCircle(max_contour)  # 获取最小外接圆的圆心坐标和半径
                # 继续处理找到的最大轮廓
            else:
                print("No green contours found.")
            
            largest_color = None
            if red_radius is not None and green_radius is not None:
                if red_radius > green_radius:
                    largest_color = 'red'
                else:
                    largest_color = 'green'
            else:
                print('could not find 2 kinds of color')
            return largest_color
        
        

    
    # 检测果实并且夹取的函数
    def pick_function(self):
        while not self.pick_finish:
            result = self.get_world_pose()
            if result is not None:
                pose_t, radius = result
            else:
                # 处理返回值为 None 的情况
                # 可以选择抛出异常、返回默认值或者执行其他逻辑
                print('world pose is None, try again')
                rospy.sleep(1)
            if pose_t is not None and radius > 10:
                pose_y = pose_t[1]  #单位是M
                print('---------- try adjust robot positon_y')
                self.pub_vel(0, pose_y)
                print('-----------finish adjust , try to pick fruit')
            else:
                print('-----------could not find the target---------')
            
            judge_flag = False
            while not judge_flag:
                self.judge_one()
            rospy.sleep(1)
        
            
    def get_endpoint(self):
        endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose
        self.endpoint = common.xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
        if self.endpoint is not None:
            return self.endpoint
        else:
            return None


    def get_world_pose(self):
        ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True)
        rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        h, w = depth_image.shape[:2]
        rgb_h, rgb_w = rgb_image.shape[:2]
        rgb_image = rgb_image[int((rgb_h - h)/2):h+int((rgb_h - h)/2), :]
        result_image = np.copy(rgb_image)
        result_image, p_y, center, r = self.tracker.proc(rgb_image, result_image, self.lab_data)
        print('--------------------- radius =',r)
        if p_y is not None:
            center_x, center_y = center
            if center_x > w:
                center_x = w
            if center_y > h:
                center_y = h
            roi = [int(center_y) - 5, int(center_y) + 5, int(center_x) - 5, int(center_x) + 5]
            if roi[0] < 0:
                roi[0] = 0
            if roi[1] > h:
                roi[1] = h
            if roi[2] < 0:
                roi[2] = 0
            if roi[3] > w:
                roi[3] = w
            roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]] 
            try:
                dist = round(float(np.mean(roi_distance[np.logical_and(roi_distance>0, roi_distance<10000)])/1000.0), 3)
            except BaseException as e:
                print(e)
                txt = "DISTANCE ERROR !!!"
                return  None, None
            if np.isnan(dist):
                txt = "DISTANCE ERROR !!!"
                return  None, None
            dist += 0.015 # 物体半径补偿
            dist += 0.015 # 误差补偿

            K = depth_camera_info.K
            if self.get_endpoint() is not None:
                print('----------------center x y:', center_x, center_y)
                position = grab_tool_function.depth_pixel_to_camera((center_x, center_y), dist, (K[0], K[4], K[2], K[5]))
                position[0] -= 0.01  # rgb相机和深度相机tf有1cm偏移
                pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标
                world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标
                #print('--------------world pose', world_pose)
                pose_t, pose_R = common.mat_to_xyz_euler(world_pose)
                print('--------------pose_x, pose_y, pose_z', pose_t)  
                return pose_t, r
            else:
                return None, None
        else:
            print('---------------------None')   
            return None, None


    def distance_jude_area(self):
        contours,h,w = self.contour_detect(self.target_color)
        area = 0
        print('contours',len(contours))
        if contours:
            if len(contours) < 2:
                self.mecanum_pub.publish(Twist())
                rospy.sleep(1)
                return True
            else:
                for i in contours:
                    area += cv2.contourArea(i)
                print('area >>>>>>>>>',area)
                area_ratio = area/(w)
                print('area_ratio:area >>>>>>>>>',area/(w))
                self.area_pid.SetPoint = self.target_area_judge
                if abs(area - self.target_area_judge) < 10:
                    area = self.target_area_judge
                    self.mecanum_pub.publish(Twist())
                    return True
                self.area_pid.update(area)
                print('area_output',str(self.area_pid.output))

                twist = Twist()
                twist.linear.x = misc.set_range(self.area_pid.output, 0, 0.05)
                print('twist.linear.x',str(twist.linear.x))
                self.mecanum_pub.publish(twist)

                if area == self.target_area_judge or twist.linear.x == 0:
                    rospy.sleep(0.2)
                    print('go to next mode')
                    return True
                elif area_ratio > 3.8:
                    rospy.sleep(0.2)
                    print('go to next mode')
                    return True
                # rospy.sleep(twist.linear.y)
            return False
        else:
            self.mecanum_pub.publish(Twist())
            rospy.sleep(1)
            return False

    def distance_judge_pick(self,color_name,detect_mode):
        _, _, rgb_image, _ = self.img_pre_process()
        h, w = rgb_image.shape[:2]
        img = cv2.resize(rgb_image, (int(w/2), int(h/2)))
        img_h, img_w = img.shape[:2]
        if detect_mode == 'yolov8':
            center_img = [int(img_w),int(img_h)]
            self.detect_h, self.detect_w = img.shape[:2]
            self.detect_center = center_img
            if self.detect_move_x_sub_flag == True:
                subtract = self.detect_move_x_sub
                print("subtract: ",subtract)
                # if abs(subtract) < 5:
                #     self.mecanum_pub.publish(Twist())
                #     return True
                # else:
                #     direct = 1 if subtract > 0 else -1
                #     speed = (abs(subtract) - 5) * 10
                #     car_move_speed = direct*speed * 0.001
                #     if abs(car_move_speed) > 0.1:
                #         car_move_speed = direct * 0.05
                #         # car_move_speed = car_move_speed * 0.5
                #     elif abs(car_move_speed) <= 0.002:
                #         car_move_speed = car_move_speed * 2

                #     print('speed------>',car_move_speed)
                #     twist = Twist()
                #     twist.linear.y = car_move_speed
                #     self.mecanum_pub.publish(twist)
                #     rospy.sleep(car_move_speed)
                #     print('move car finish')
                # self.detect_move_x_sub_flag = False
            else:
                return False
            return False
            
        else:
            center_img = [int(img_w/2),int(img_h/2)]
            img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊  去噪声
            img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
            color = self.lab_data['lab']['gemini_camera'][color_name]
            mask_red = cv2.inRange(img_lab, tuple(color['min']), tuple(color['max'])) # 二值化
            eroded = cv2.erode(mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)))
            if self.display:
                cv2.imshow(color_name,dilated)
                key = cv2.waitKey(1)
            contours_dist = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            x_add = 0
            x_cacualte = 0
            y_cacualte = 0
            y_add = 0
            x = 0
            y = 0
            print(len(contours_dist))
            for contour in contours_dist:
                circle_judge = cv2.minAreaRect(contour) 
                # print(circle_judge)
                # 通过半径筛选不规则的图形,减少干扰 (x, y),(w,h),angle
                if  1 - circle_judge[1][0] / circle_judge[1][1] > 0.2:
                    continue
                (x,y),r = cv2.minEnclosingCircle(contour)
                
                x_cacualte = center_img[0]- x
                print('x_caculate',x_cacualte)
                y_cacualte = center_img[1]- y
                x_add += x_cacualte
                y_add += y_cacualte
                # x_add += x
                # y_add += y
            subtract = x_add
            # print("center_img_x: ",center_img[0])
            # print("center_img_y: ",center_img[1])
            print("subtract: ",subtract)
            if abs(subtract) < 5:
                self.mecanum_pub.publish(Twist())
                return True
            else:
                direct = 1 if subtract > 0 else -1
                speed = (abs(subtract) - 5) * 10
                car_move_speed = direct*speed * 0.001
                if abs(car_move_speed) > 0.1:
                    car_move_speed = direct * 0.05
                    # car_move_speed = car_move_speed * 0.5
                elif abs(car_move_speed) <= 0.002:
                    car_move_speed = car_move_speed * 2

                print('speed------>',car_move_speed)
                twist = Twist()
                twist.linear.y = car_move_speed
                self.mecanum_pub.publish(twist)
                rospy.sleep(car_move_speed)
                print('move car finish')
            # print("x_add: ",x_add * 0.5)
            # print("y_add: ",y_add * 0.5)
            return False

    def arm_move(self,arm_move_flag):
        if arm_move_flag == 0:
            bus_servo_control.set_servos(self.servos_pub, 2, ((1, 600), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))
        elif arm_move_flag == 1:
            bus_servo_control.set_servos(self.servos_pub, 2, ((1, 400), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))
        else:
            bus_servo_control.set_servos(self.servos_pub, 2, ((1, 500), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))


    def run(self):
        while not rospy.is_shutdown() and self.running:
            if self.running:
                echo_str('-----------------RUNNING------------------',EchoColor.BLUE)
                if self.mode == 'get_pick_result_ok':
                    # 如果还是同一棵树，则还需要进行夹取
                    if self.next_picking_strategy == 'same_point':
                        self.tree_pick_time += 1
                        self.arm_move_flag = self.tree_pick_time
                        echo_str(('tree_pick_time',self.tree_pick_time),EchoColor.YELLOW)
                        echo_str(('arm_move_flag',self.arm_move_flag),EchoColor.YELLOW)
                        # print('tree_pick_time',self.tree_pick_time)
                        # print('arm_move_flag',self.arm_move_flag)

                        # 当夹取成功两次之后，就可以退出，反之则继续进入夹取状态
                        if self.tree_pick_time >= 2:
                            rospy.set_param('~pick_fuit_finish','true')
                            self.mode = 'wait_message'
                            print('[][][][][][][[][][][][][][][]')
                            stop_service = rospy.ServiceProxy('~stop', Trigger)
                            response = stop_service()
                            rospy.loginfo("stop_service response: %s", response.message)
                            rospy.set_param('~pick_fuit_finish','true')
                            continue
                        else:
                            self.arm_move(self.arm_move_flag)
                            rospy.sleep(2)
                            self.mode = 'get_target_positon'
                            rospy.loginfo("continue to pick next fruit")
                            continue
                    else:
                        self.mode = 'wait_message'
                        rospy.set_param('~pick_fuit_finish','true')
                        print('[][][][][][][[][][][][][][][]')
                        stop_service = rospy.ServiceProxy('~stop', Trigger)
                        response = stop_service()
                        rospy.loginfo("stop_service response: %s", response.message)
                        continue

                if self.mode == 'pick_finish':
                    # 检查是否已经完成夹取，如果没有的话，就需要再次检测并且夹取。    
                    if self.next_picking_strategy == 'same_point':
                        result = self.get_pick_result_same()
                    elif self.next_picking_strategy == 'different_point':
                        result = self.get_pick_result_diff()
                    else:
                        result = self.get_pick_result_same()
                    print(result)
                    if result == 'get_pick_result_ok':
                        self.mode = 'get_pick_result_ok'
                    continue

                if self.mode == 'picking':  # 检测并且夹取
                    print('picking')


                if self.mode == 'detect_tree_style':  # 检测果树类型
                    self.yolov8_detect_tree = True
                    if self.tree_detect_finish:
                        self.yolov8_detect_tree = False
                        print('----- tree_left_fruit_color -----',self.tree_left_fruit_color)
                        if self.tree_left_fruit_color == self.target_color:
                            self.mode = 'inter_calibation'
                            self.next_picking_strategy = 'same_point'
                            rospy.set_param('/agricultural_picking/next_picking_strategy',self.next_picking_strategy)
                            # self.pub_vel(0, 0.05)
                            rospy.set_param('/agricultural_picking/next_tree', 'true')  # 接下来是需要去到下一颗树的夹取点 Point3
                        else:
                            
                            self.mode = 'pick_mode_two'
                            self.next_picking_strategy = 'different_point'
                            rospy.set_param('/agricultural_picking/next_picking_strategy',self.next_picking_strategy)
                            rospy.set_param('/agricultural_picking/next_tree', 'false')  # 不需要去下一棵树，去到Point2或者Point4
                        print(self.next_picking_strategy)
                        continue
                    else:
                        print('Detect Tree Style Again!!!')
                        continue
                        # rospy.set_param('/track_and_garb/next_pick_point', 'pick_point_2')  # 若不是，则是2
                '''
                ------------------------------ 根据树的进行不同策略的夹取 ------------------------
                '''
                '''
                ------------------------------ same point ------------------------
                '''
                if self.mode == 'pick_mode_one':
                    print('---------- pick_mode_one -----------')
                    if self.next_picking_strategy == 'same_point':
                        if self.distance_judge_pick(self.target_color,'color') == False:
                            self.mode = 'pick_mode_one'
                            self.arm_move_flag = 0
                            continue
                        else:
                            echo_str('distance_judge_pick finish!!!!!',EchoColor.YELLOW)
                            self.mode = 'distance_jude_x'
                            continue

                if self.mode == 'distance_jude_x':
                    if self.distance_jude_area():
                        self.arm_move(self.arm_move_flag)
                        self.mode = 'get_target_positon'
                        continue
                    else:
                        echo_str('distance_jude_x finish!!!!!',EchoColor.YELLOW)
                        rospy.sleep(0.2)
                        continue
                
                if self.mode == 'inter_calibation':
                    # 如果没有检测到的话就继续检测
                    if self.distance_judge_pick(self.target_color,'yolov8') == False:
                        self.mode = 'inter_calibation'
                        self.arm_move_flag = 0
                        continue
                    else:
                        pass

                '''
                ------------------------------ different pont ------------------------
                '''
                if self.mode == 'pick_mode_two':  # 正常的识别夹取
                    print('---------- pick_mode_two -----------')
                    self.go_to_move = True
                    self.arm_move_flag = 2
                    result = self.get_world_pose()
                    if result is not None:
                        self.pose_t, self.radius = result
                    else:
                        # 处理返回值为 None 的情况
                        # 可以选择抛出异常、返回默认值或者执行其他逻辑
                        print('world pose is None, try again')
                        rospy.sleep(1)
                        self.mode = 'pick_mode_two'
                        continue

                    self.mode = 'pick_adjust_x_1'
                    continue

                if self.mode == 'pick_adjust_x_1':
                    #print('------------pick start, get world_pose of target')
                    result = self.get_world_pose()
                    if result is not None:
                        pose_t, radius = result
                    else:
                        # 处理返回值为 None 的情况
                        # 可以选择抛出异常、返回默认值或者执行其他逻辑
                        print('world pose is None, try again')
                        rospy.sleep(1)
                        continue
                    if pose_t is not None and radius > 10:
                        pose_x = pose_t[0]  #单位是M
                        if pose_x > 0.4:
                            print('---------- try adjust robot positon_x_1')
                            self.pub_vel(pose_x - 0.4, 0)
                            print('----------- finish adjust , try to pick fruit')
                    else:
                        print('----------- [pick_adjust_x_1] could not find the target---------')
                        self.mode = 'pick_adjust_x_1'
                        continue

                    self.mode = 'pick_adjust_y_2'
                    rospy.sleep(1)

                #   进行第二次y方向位姿调整
                elif self.mode == 'pick_adjust_y_2':
                    result = self.get_world_pose()
                    if result is not None:
                        pose_t, radius = result
                    else:
                        # 处理返回值为 None 的情况
                        # 可以选择抛出异常、返回默认值或者执行其他逻辑
                        print('world pose is None, try again')
                        rospy.sleep(1)
                        continue
                    if pose_t is not None and radius > 10:
                        pose_y = pose_t[1]  #单位是M
                        print('---------- try adjust robot positon_y_2')
                        self.pub_vel(0, pose_y)
                        print('-----------finish adjust , try to pick fruit')
                    else:
                        self.mode = 'pick_adjust_y_2'
                        print('----------- [pick_adjust_y_2] could not find the target---------')
                        continue
                    self.mode = 'pick_adjust_x_2'
                    rospy.sleep(1)

                #   进行第二次x方向位姿调整
                elif self.mode == 'pick_adjust_x_2':
                    #print('------------pick start, get world_pose of target')
                    result = self.get_world_pose()
                    if result is not None:
                        pose_t, radius = result
                    else:
                        # 处理返回值为 None 的情况
                        # 可以选择抛出异常、返回默认值或者执行其他逻辑
                        print('world pose is None, try again')
                        rospy.sleep(1)
                        continue
                    if pose_t is not None:  
                        pose_x = pose_t[0]  #单位是M
                        if pose_x > 0.35:
                            print('---------- try adjust robot positon_x')
                            self.pub_vel(pose_x - 0.35, 0)
                            print('-----------finish adjust , try to pick fruit')
                    else:
                        self.mode = 'pick_adjust_x_2'
                        print('-----------[pick_adjust_x_2] could not find the target---------')
                        continue
                    self.arm_move(self.arm_move_flag)
                    self.mode = 'get_target_positon'

                if self.mode == 'get_target_positon':
                    #print('---------- image proc --------------')
                    self.image_proc()
                
                if self.mode == 'pick_check':
                    white_ratio,(x_ratio,y_ratio) = self.pick_check(self.image_queue,self.target_color)
                    print('white_ratio:',white_ratio)
                    print('pick_check (x,),y',x_ratio,y_ratio)
                    if x_ratio is not None or y_ratio is not None:
                        self.arm_move_x = x_ratio
                        self.arm_move_y = y_ratio
                    # if white_ratio == 0:
                    #     self.mode == 'get_pick_result_ok'
                    # 当面积小于 5.0 的时候，就可以进行夹取
                    target_white_ratio = 5.0
                    if white_ratio is not None:
                        if white_ratio < target_white_ratio:  # 在一定范围里面进行计数
                            self.ratio_count += 1
                    print("turn_up_down: ", self.turn_up_down)
                    print("self.ratio_count: ", self.ratio_count)
                rospy.sleep(0.2)
            else:
                print('go to check again')

    def pick(self, position):
        if position[2] < 0.2:
            yaw = 80
        else:
            yaw = 30
        
        ret = kinematics_control.set_pose_target(position, yaw)
        #   末端到达抓取点
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret[1][0]), ))
            rospy.sleep(1)
            bus_servo_control.set_servos(self.servos_pub, 2, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
            rospy.sleep(2)
        #   到达准确性检测
        self.moving = False
        self.mode = 'pick_check'
        #   等待检测结果
        while not rospy.is_shutdown():
            #   如果两秒内检测到目标物在镜头里面五次都是不超过5%的画面占比，就认为此时可以夹住
            rospy.sleep(1)  
            if self.ratio_count >= 5:
                # print('---------pick check ok--------------')
                # self.mode = 'picking'
                self.mode = 'picking'
                self.ratio_count = 0
                break
            else:
                # print('---------pick check fail--------------')
                #self.mode = 'pick_check_fail'
                position[2] += 0.005
                self.ratio_count = 0
                self.mode = 'pick_check'
                self.servo_control(position)
                
        #   夹住小球
        bus_servo_control.set_servos(self.servos_pub, 0.5, ((10, 600),))
        rospy.sleep(1)
        #   向下拉，摘下小球
        position[2] -= 0.05
        ret = kinematics_control.set_pose_target(position, yaw)
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
            rospy.sleep(1)


        if self.arm_move_flag == 0:
            position[0] -= 0.1
            # position[1] += 0.1
            print('position{},position{},position{}'.format(position[0],position[1],position[2]))
            ret = kinematics_control.set_pose_target(position, yaw)
            if len(ret[1]) > 0:
                bus_servo_control.set_servos(self.servos_pub,1, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
                rospy.sleep(1)

            position[0] -= 0.1
            # position[1] += 0.1
            print('position{},position{},position{}'.format(position[0],position[1],position[2]))
            ret = kinematics_control.set_pose_target(position, yaw)
            if len(ret[1]) > 0:
                bus_servo_control.set_servos(self.servos_pub,1, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
                rospy.sleep(1)

        # 调用果实放置的函数 
        self.place_function()
        self.mode = 'pick_finish'
        print('-------------------pick_finish-------------------------')
        self.tracker.yaw = 500
        self.tracker.pitch = 150
        self.tracker.pid_yaw.clear()
        self.tracker.pid_pitch.clear()
        self.stamp = time.time()
        self.first_time = time.time() + 2
        self.moving = False
    
    
    def get_pick_result_diff(self):
        print('-------------------get_pick_result_diff-------------------------')
        ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True)
        try:
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
            h, w = depth_image.shape[:2]
            rgb_h, rgb_w = rgb_image.shape[:2]
            rgb_image = rgb_image[int((rgb_h - h)/2):h+int((rgb_h - h)/2), :]
            result_image = np.copy(rgb_image)
            h, w = depth_image.shape[:2]
            depth = np.copy(depth_image).reshape((-1, ))
            depth[depth<=0] = 55555
            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)
            sim_depth_image = sim_depth_image / 2000.0 * 255.0
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
            result_image, p_y, center, r = self.tracker.proc(rgb_image, result_image, self.lab_data)
            dist = 0.0
            if p_y is not None:
                #bus_servo_control.set_servos(self.servos_pub, 0.02, ((1, p_y[1]), (4, p_y[0])))
                center_x, center_y = center
                if center_x > w:
                    center_x = w
                if center_y > h:
                    center_y = h
                    roi = [int(center_y) - 5, int(center_y) + 5, int(center_x) - 5, int(center_x) + 5]
                    if roi[0] < 0:
                        roi[0] = 0
                    if roi[1] > h:
                        roi[1] = h
                    if roi[2] < 0:
                        roi[2] = 0
                    if roi[3] > w:
                        roi[3] = w
                    roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]] 
                    try:
                        dist = round(float(np.mean(roi_distance[np.logical_and(roi_distance>0, roi_distance<10000)])/1000.0), 3)
                    except BaseException as e:
                        print(e)
                        txt = "DISTANCE ERROR !!!"
                        return 'get_pick_result_ok'
                    if np.isnan(dist) :
                        txt = "DISTANCE ERROR !!!"
                        return 'get_pick_result_ok'

                # 如果检测距离大于10cm的话，那么直接进入下一个点进行夹取
                print('abs(dist - self.last_fruit_distance):::::::::>>>>',abs(dist - self.last_fruit_distance))
                
                if abs(dist - self.last_fruit_distance) > 0.10:
                    return 'get_pick_result_ok'
                else:
                    self.mode = 'get_target_positon'
                    return 'get_target_positon'
            else:
                return 'get_pick_result_ok'

        except Exception as e:
            rospy.logerr('callback error:', str(e))
            self.mode = 'get_target_positon'
            return 'get_target_positon'

    
    def get_pick_result_same(self):
        _, _, rgb_image, _ = self.img_pre_process()
        result_image = np.copy(rgb_image)
        result_image, p_y, center, r = self.tracker.proc(rgb_image, result_image, self.lab_data)
        print('current radius:',r)
        if p_y is None or r < 20:
            print('----------------go to next Point--------------------')
            print('in here : get_pick_result')
            if not self.go_to_move:
                self.mode = 'get_pick_result_ok'
                self.go_to_move = True
            return 'get_pick_result_ok'
        else:
            print('-----------------------pick_fail, try again------------------')
            self.mode = 'get_target_positon'
            # 夹取失败，调整z坐标。
            self.pose_x_offset += 0.001
            self.pose_x_offset -= 0.001
            self.pose_z_offset -= 0.001
            print('--------------pose z offset :', self.pose_z_offset)
            # stop_service = rospy.ServiceProxy('~stop', Trigger)
            # response = stop_service()
            # rospy.loginfo("stop_service response: %s", response.message)
            # start_service = rospy.ServiceProxy('~start', Trigger)
            # response = start_service()
            # rospy.loginfo("start_service response: %s", response.message)                  
            return 'get_target_positon'

    # 放置果实函数
    def place_function(self):
        rospy.sleep(0.5)
        # to test 放置到盒子当中
        move_center_time = 2
        if self.arm_move_flag != 1:
            bus_servo_control.set_servos(self.servos_pub, 2, ((1, 500), (2, 765), (3, 12), (4, 147), (5, 500), (10, 505)))
            rospy.sleep(2)
        else:
            move_center_time = 3
        bus_servo_control.set_servos(self.servos_pub, move_center_time, ((1, 0), (2, 770), (3, 12), (4, 155), (5, 500), (10, 550)))
        rospy.sleep(move_center_time)
        bus_servo_control.set_servos(self.servos_pub, 2, ((1, 0), (2, 490), (3, 145), (4, 160), (5, 500), (10, 550)))
        rospy.sleep(2)
        bus_servo_control.set_servos(self.servos_pub, 2, ((1, 0), (2, 490), (3, 145), (4, 160), (5, 500), (10, 300)))
        rospy.sleep(2)
        bus_servo_control.set_servos(self.servos_pub, 1.5, ((1, 0), (2, 490), (3, 145), (4, 160), (5, 500), (10, 500)))
        rospy.sleep(1.5)
        #  回到机械臂init位置(left,right,center)
        self.arm_move(self.arm_move_flag)
        # bus_servo_control.(self.servos_pub, 2, ((1, 500), (2, 763), (3, 10), (4, 300), (5, 500), (10, 200)))
        rospy.sleep(2)

    def fruit_depth(self,center):
        ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True)
        try:
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
            h, w = depth_image.shape[:2]
            rgb_h, rgb_w = rgb_image.shape[:2]
            rgb_image = rgb_image[int((rgb_h - h)/2):h+int((rgb_h - h)/2), :]
            result_image = np.copy(rgb_image)
            h, w = depth_image.shape[:2]
            depth = np.copy(depth_image).reshape((-1, ))
            depth[depth<=0] = 55555
            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)
            sim_depth_image = sim_depth_image / 2000.0 * 255.0
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
            center_x, center_y = center[0]*2, center[1]*2
            roi = [int(center_y) - 5, int(center_y) + 5, int(center_x) - 5, int(center_x) + 5]
            if roi[0] < 0:
                roi[0] = 0
            if roi[1] > h:
                roi[1] = h
            if roi[2] < 0:
                roi[2] = 0
            if roi[3] > w:
                roi[3] = w
            roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]]
            try:
                dist = round(float(np.mean(roi_distance[np.logical_and(roi_distance>0, roi_distance<10000)])/1000.0), 3)
                print('fruit dist:',dist)
                return dist
            except BaseException as e:
                print(e)
                txt = "DISTANCE ERROR !!!"
                return 0
                if np.isnan(dist) :
                    txt = "DISTANCE ERROR !!!"
                    return 0
        except Exception as e:
            rospy.logerr('callback error:', str(e))


    # 获取目标检测结果
    def get_object_callback(self, msg):
        self.objects_info = msg.objects
        if self.objects_info == []:  # 没有识别到时重置变量
            print('Detect None')
        else:
            min_distance = 0
            for i in self.objects_info:
                class_name = i.class_name
                center = (int((i.box[0] + i.box[2])/2) , int((i.box[1] + i.box[3])/2))
                # 检测树的类型
                if self.yolov8_detect_tree:
                    if class_name == 'red_fruit':
                        if self.red_min_x > center[0]:
                            self.red_min_x = center[0]
                        # self.red_fruit_list.append(center)
                    elif class_name == 'green_fruit':
                        if self.green_min_x > center[0]:
                            self.green_min_x = center[0]
                        # self.green_fruit_list.append(center)

                elif self.mode == 'inter_calibation':
                    if class_name == 'red_fruit' and self.target_color == 'red':
                        self.detect_move_x_sub += (self.detect_center[0] - center[0])
                    elif class_name == 'green_fruit' and self.target_color == 'green':
                        self.detect_move_x_sub += (self.detect_center[0] - center[0])
                    else:
                        continue
                    print('self.detect_center[0]',self.detect_center[0])
                    print('center_x',center[0])



            #判断树的类型
            if self.yolov8_detect_tree:
                if self.red_min_x < self.green_min_x:
                    self.tree_style = 'tree_one'
                    self.tree_left_fruit_color = 'red'
                else:
                    self.tree_style = 'tree_two'
                    self.tree_left_fruit_color = 'green'

                print('self.red_min_x',self.red_min_x)
                print('self.green_min_x',self.green_min_x)
                print('self.tree_left_fruit_color',self.tree_left_fruit_color)
                print('self.tree_detect_last',self.tree_detect_last)
                print('self.tree_detect_count',self.tree_detect_count)

                if self.tree_detect_last == self.tree_left_fruit_color:
                    self.tree_detect_count += 1
                    if self.tree_detect_count > 5:
                        self.tree_detect_finish = True
                self.tree_detect_last = self.tree_left_fruit_color

            if self.mode == 'inter_calibation':
                if self.detect_move_finish == False:

    
            

                    
            
    #   抓取检测
    def pick_check(self,image_queue_input,target_color_input):
        ros_rgb_image, ros_depth_image, depth_camera_info = image_queue_input.get(block=True)
        #   在NumPy中，可以通过buffer参数来指定数组的缓存区，如上一个例子中的buffer=ros_rgb_image.data，这样就可以直接使用ROS图像消息中的数据作为NumPy数组的缓存区，而不需要额外的复制操作，提高了效率并节省了内存。
        rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
        h, w= rgb_image.shape[:2]   #取元组的前两个元素
        #result_image = np.copy(rgb_image)
        color = lab_data['lab']['gemini_camera'][target_color_input]

        #   把图像缩小，需要处理的数据少了四分之三
        img = cv2.resize(rgb_image, (int(w/2), int(h/2)))
        img_h, img_w= img.shape[:2]   #取元组的前两个元素
        img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊  去噪声
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
        mask = cv2.inRange(img_lab, tuple(color['min']), tuple(color['max'])) # 二值化

        # 平滑边缘，去除小块，合并靠近的块
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))     #cv2.getStructuringElement()函数创建一个形态学操作的结构元素（structuring element）。  这意味着生成的结构元素是一个3行3列的矩阵，用于形态学操作，比如腐蚀（erosion）和膨胀（dilation）。
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, MORPH_RECT_CORE))
        if self.display:
            cv2.imshow(target_color_input,dilated)
            key = cv2.waitKey(1)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        center = [0,0]
        center_x_ratio = None
        center_y_ratio = None
        if len(contours) > 1:
            max_contour = max(contours, key=cv2.contourArea)
            (center[0], center[1]), radius = cv2.minEnclosingCircle(max_contour) # 最小外接圆
            center_x_ratio = center[0]/img_w
            center_y_ratio = center[1]/img_h
        #   获取小球占画面的比例
        white_pixels = np.sum(dilated == 255)
        total_pixels = dilated.shape[0] * dilated.shape[1]
        white_ratio = white_pixels / total_pixels
        # print('white_ratio =', white_ratio * 100, '%')
        #rospy.sleep(2)
        return white_ratio * 100,(center_x_ratio, center_y_ratio)

    def image_proc(self):
        ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True)
        try:
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
            h, w = depth_image.shape[:2]
            rgb_h, rgb_w = rgb_image.shape[:2]
            rgb_image = rgb_image[int((rgb_h - h)/2):h+int((rgb_h - h)/2), :]
            result_image = np.copy(rgb_image)
            h, w = depth_image.shape[:2]
            depth = np.copy(depth_image).reshape((-1, ))
            depth[depth<=0] = 55555
            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)
            sim_depth_image = sim_depth_image / 2000.0 * 255.0
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

            if self.tracker is not None and self.moving is False and time.time() > self.ttt and self.start:
                result_image, p_y, center, r = self.tracker.proc(rgb_image, result_image, self.lab_data)
                if p_y is not None:
                    #bus_servo_control.set_servos(self.servos_pub, 0.02, ((1, p_y[1]), (4, p_y[0])))
                    center_x, center_y = center
                    if center_x > w:
                        center_x = w
                    if center_y > h:
                        center_y = h
                    if abs(self.last_pitch_yaw[0] - p_y[0]) < 5 and abs(self.last_pitch_yaw[1] - p_y[1]) < 5:
                        if time.time() - self.stamp > 2:
                            self.stamp = time.time()
                            roi = [int(center_y) - 5, int(center_y) + 5, int(center_x) - 5, int(center_x) + 5]
                            if roi[0] < 0:
                                roi[0] = 0
                            if roi[1] > h:
                                roi[1] = h
                            if roi[2] < 0:
                                roi[2] = 0
                            if roi[3] > w:
                                roi[3] = w
                            roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]] 
                            try:
                                dist = round(float(np.mean(roi_distance[np.logical_and(roi_distance>0, roi_distance<10000)])/1000.0), 3)
                                self.last_fruit_distance = dist
                            except BaseException as e:
                                print(e)
                                txt = "DISTANCE ERROR !!!"
                                return
                            if np.isnan(dist) :
                                txt = "DISTANCE ERROR !!!"
                                return
                            dist += 0.015 # 物体半径补偿
                            dist += 0.015 # 误差补偿
                            K = depth_camera_info.K
                            self.get_endpoint()
                            print('----------------center x y:', center_x, center_y)
                            position = grab_tool_function.depth_pixel_to_camera((center_x, center_y), dist, (K[0], K[4], K[2], K[5]))
                            position[0] -= 0.01  # rgb相机和深度相机tf有1cm偏移
                            pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标
                            world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标
                            print('--------------world pose', world_pose)
                            pose_t, pose_R = common.mat_to_xyz_euler(world_pose)
                            #   夹取补偿 pose_t [x, y, z](机械臂坐标系)
                            pose_t[2] += self.pose_z_offset  #Z轴坐标增加 xx m
                            pose_t[0] += self.pose_x_offset   #x轴坐标增加 xx m
                            pose_t[1] += self.pose_y_offset   #y轴坐标增加 xx m

                            self.stamp = time.time()
                            if self.mode != 'pick_check':
                                self.moving = True
                                self.position = pose_t
                                threading.Thread(target=self.pick, args=(pose_t,)).start()
                    else:
                        self.stamp = time.time()
                    dist = depth_image[int(center_y),int(center_x)]                           

                    if dist < 100:
                        txt = "TOO CLOSE !!!"
                    else:
                        txt = "Dist: {}mm".format(dist)
                    if self.display:
                        cv2.circle(result_image, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                        cv2.circle(depth_color_map, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                        cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 10, cv2.LINE_AA)
                        cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)
                    self.last_pitch_yaw = p_y
                    
                else:
                    self.stamp = time.time()
            if self.enable_disp:
                result_image = np.concatenate([cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), depth_color_map, ], axis=1)
                cv2.imshow("depth", result_image)
                key = cv2.waitKey(1)
                if key != -1:
                    rospy.signal_shutdown('shutdown1')

        except Exception as e:
            rospy.logerr('callback error:', str(e))



if __name__ == '__main__':
    TrackAndGrabNode('agricultural_picking')
    try:
        rospy.spin()
    except exception as e:
        print('enter except')
        rospy.logerr(str(e))
