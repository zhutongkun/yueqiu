#!/usr/bin/env python2
# encoding: utf-8
# 果实采摘
import os
import json
import rospy
import signal
import subprocess   #运行命令行
import math
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Twist, PoseStamped, Pose
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from ros_robot_controller.msg import BuzzerState
from servo_controllers import bus_servo_control
from servo_msgs.msg import MultiRawIdPosDur

#将rpy转换成qua
def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

class VoiceControlNavNode:
    def __init__(self, name):
        rospy.init_node(name)
        
        self.words = None #语音识别到的内容
        self.running = False # 循环检测任务开关 
        self.move_base_status = 1 # 导航状态 1 是还在运动中，3是导航完毕
        self.pick_location_time = rospy.get_param('/pick_location_time', 3) # map节点名
        self.up_ramp_time = rospy.get_param('/up_ramp_time', 3.5) # map节点名
        self.pick_point = None
        self.next_pick_point = None
        pick_point_1 = rospy.get_param('/pick_point_1') #夹取点1
        pick_point_2 = rospy.get_param('/pick_point_2') #夹取点2
        pick_point_3 = rospy.get_param('/pick_point_3') #夹取点3
        pick_point_4 = rospy.get_param('/pick_point_4') #夹取点4
        back_point = rospy.get_param('/back_point') #终点处（相当于起点）
        self.pick_point_1 = (pick_point_1[0], pick_point_1[1], pick_point_1[2])
        self.pick_point_2 = (pick_point_2[0], pick_point_2[1], pick_point_2[2])
        self.pick_point_3 = (pick_point_3[0], pick_point_3[1], pick_point_3[2])
        self.pick_point_4 = (pick_point_4[0], pick_point_4[1], pick_point_4[2])
        self.back_point = (back_point[0], back_point[1], back_point[2])

        rospy.Service('~start', Trigger, self.start_callback) # 开始任务
        rospy.Service('~stop', Trigger, self.stop_callback) # 结束任务
        
        rospy.set_param('~target_shape', 'None') # 设置目标形状
        rospy.set_param('~status', 'start') # 设置状态
        
        self.costmap = '/move_base/local_costmap/costmap' # costmap节点名

        self.navigation_pick_point = None

        self.map_frame = rospy.get_param('~map_frame', '/map') # map节点名
        self.base_foot_print_frame = rospy.get_param('~base_foot_print','/base_foot_print')
        # 底盘运动控制节点
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        # 舵机控制
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/servo_manager/init_finish') and rospy.get_param(
                        '/joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        # 初始状态 
        bus_servo_control.set_servos(self.joints_pub, 2, ((1, 500), (2, 760), (3, 15), (4, 150), (5, 500), (10, 200)))
        rospy.sleep(2)
        # 导航点发布
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        # 订阅路径规划返回话题
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_callback)
        # 等待语音识别节点启动
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/voice_control/init_finish'):
                    break
            except:
                rospy.sleep(0.1)

        # 等待导航启动
        rospy.wait_for_message(self.costmap, OccupancyGrid)

        #开始任务
        signal.signal(signal.SIGINT, self.shutdown)
        
        rospy.spin()
    def stop_callback(self,msg):
        self.running = False
        rospy.ServiceProxy('/agricultural_picking/stop', Trigger)() # 停止果实摘取
        rospy.set_param('/agricultural_picking/pick_fuit_finish','false')
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')
        return TriggerResponse(success=True)


    def start_callback(self,msg):
        self.control(0,0,1,"detect") #导航并进行检测
        self.running = True
        self.run()
        return TriggerResponse(success=True)


    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

        
    #进行导航状态检测
    def move_callback(self, msg):
        print(msg)
        try:
            if msg.status.status == 3:
                self.move_base_status = msg.status.status
            else : 
                self.move_base_status = 1
        except:
            self.move_base_status = 1
    #设置的导航点
    def nav_position(self,x,y,w):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = rpy2qua(math.radians(0),math.radians(0),math.radians(w))
        self.goal_pub.publish(pose)

    def nav_position_base(self,x,y,w):
        pose = PoseStamped()
        pose.header.frame_id = self.base_foot_print_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = rpy2qua(math.radians(0),math.radians(0),math.radians(w))
        self.goal_pub.publish(pose)

    #等到导航状态
    def wait_nav_status(self):
        while True:
            # print(self.move_base_status)
            if self.move_base_status == 3 :
                self.move_base_status = 1
                break
            else:
                rospy.sleep(2)
    
    #等到夹取状态（果实摘取）,当夹取的状态被设置成为 pick_ok的时候，退出循环，进入到导航状态
    def wait_pick_status(self):
        while True:
            pick_status = rospy.get_param('/agricultural_picking/pick_fuit_finish','false')
            print('grab_status:',pick_status)
            if pick_status == "true":
                rospy.ServiceProxy('/agricultural_picking/stop', Trigger)() # 停止果实摘取
                rospy.set_param('/agricultural_picking/pick_fuit_finish','false')
                print('pick the fruit finish')
                break
            else:
                rospy.sleep(2)

    # 等待 YOLOv11 的识别对应果实颜色的结果
    def wait_detect_status(self):
        while True:
            pick_status = rospy.get_param('/agricultural_picking/detect_fruit_color_finish','false')
            print('grab_status:',pick_status)
            if pick_status == "true":
                rospy.ServiceProxy('/agricultural_picking/stop', Trigger)()
                print('detect Fruit Finish')
                break
            else:
                rospy.sleep(2)


    # set_status 为控制小车任务 pick、place、detect、back
    def control(self,x,y,w,set_status):
        self.nav_position(x,y,w) #导航
        rospy.sleep(5)
        self.wait_nav_status() #等待导航
        if set_status == 'pick' : #夹取
            rospy.ServiceProxy('/agricultural_picking/start', Trigger)() # 启动果实摘取
            print('pick the fruit')
            self.wait_pick_status()

        # 设置程序进行检测
        elif set_status == "detect" : 
            rospy.set_param('/agricultural_picking/mode','detect_target_fruit_color')
            rospy.sleep(4)
            rospy.ServiceProxy('/agricultural_picking/start', Trigger)() # 启动程序检查目标颜色
            print('Detect the fruit finish')
            self.wait_detect_status()

        elif set_status == "back" : #回出发点
            rospy.set_param('~status', 'stop')
            self.mecanum_pub.publish(Twist())

    def pick_point(self):
        self.control(self.pick_point_green[0], self.pick_point_green[1], self.pick_point_green[2], "pick")
    
    # 移动的模式，速度，以及时间
    def move_function(self,move_mode,move_speed = 0.0,time = 0):
        if move_mode == 'stop' :
            self.mecanum_pub.publish(Twist())
            rospy.sleep(time) 
            return
        else:
            twist = Twist()
            twist.angular.z = move_speed if move_mode == 'turn_back' else 0
            if move_mode == 'go':
                twist.linear.x = move_speed
            elif move_mode == 'back':
                twist.linear.x = -move_speed
            else:
                twist.linear.x = 0
            if move_mode =='left':
                twist.linear.y = move_speed
            elif move_mode == 'right':
                twist.linear.y = -move_speed
            else:
                twist.linear.y = 0
            self.mecanum_pub.publish(twist)
            rospy.sleep(time) 

    def run(self):
        while not rospy.is_shutdown() and self.running: 
            # print(self.words)
            print('>>>>>>>>>>>>>>>>>>>> 开始进行任务<<<<<<<<<<<<<<<<<') 
            print("go")
            self.move_base_status = 1
            # 逻辑顺序 yolov11检测 -> 导航到夹取点  -> 回到起点
            '''
            ------------------------------- 目标果实的颜色 ------------------------
            '''
            self.pick_point = self.target_color_point
            rospy.set_param('/agricultural_picking/mode','detect_target_fruit_color')
            self.control(self.pick_point[0],  self.pick_point[1],   self.pick_point[2],  "detect") # 进行检测
            '''
            ------------------------------ 第一颗树开始 ------------------------
            '''
            # 设置目标点1，并且导航到目标点进行夹取的动作
            self.pick_point = self.pick_point_1
            rospy.set_param('/agricultural_picking/mode', 'inter_cali_two')
            self.control(self.pick_point[0], self.pick_point[1], self.pick_point[2], "pick")  # 去点1夹取
            self.move_function('back',0.1,1)
            self.move_function('turn_back',-0.5,3)

            # 设置目标点2，并且导航到目标点进行夹取的动作
            self.pick_point = self.pick_point_2
            self.control(self.pick_point[0], self.pick_point[1], self.pick_point[2], "pick")  # 去下一个点进行识别夹取
            self.move_function('back',0.1,1)
            self.move_function('turn_back',-0.5,3)

            '''
            ------------------------------ 第二颗树开始 ------------------------
            '''
            # 设置目标点3，并且导航到目标点进行夹取的动作
            self.pick_point = self.pick_point_3
            self.control(self.pick_point[0], self.pick_point[1], self.pick_point[2], "pick")  # 去下一个点进行识别夹取
            self.move_function('back',0.1,1)
            self.move_function('turn_back',-0.5,3)

            # 设置目标点4，并且导航到目标点进行夹取的动作
            self.pick_point = self.pick_point_4
            self.control(self.pick_point[0], self.pick_point[1], self.pick_point[2], "pick")  # 去下一个点进行识别夹取
            self.move_function('back',0.1,1)
            self.move_function('turn_back',0.5,3)

            '''
            ------------------------------ 回到起点 ------------------------
            '''
             # 因为没有下一棵树了，所以回到起点位置
            self.control(self.back_point[0], self.back_point[1], self.back_point[2], "back")  # 回到起点的位置
            print('mission Finish!!!')
            self.mecanum_pub.publish(Twist())


if __name__ == "__main__":
    VoiceControlNavNode('voice_control_nav')
