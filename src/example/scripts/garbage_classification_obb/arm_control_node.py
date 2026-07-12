#!/usr/bin/python3
#coding=utf8
# 机械臂控制节点
import sys
import rospy
import numpy as np
import threading
import math
from std_msgs.msg import Float32MultiArray
from sdk import common
from kinematics import kinematics_control
from servo_msgs.msg import MultiRawIdPosDur
from servo_controllers import bus_servo_control
from ros_robot_controller.msg import BuzzerState
sys.path.append('/home/ubuntu/software/arm_pc')
from action_group_controller import ActionGroupController


WASTE_CLASSES = {
    'food_waste': ['BananaPeel', 'BrokenBones', 'Ketchup'],
    'hazardous_waste': ['Marker', 'OralLiquidBottle', 'StorageBattery'],
    'recyclable_waste': ['PlasticBottle', 'Toothbrush', 'umbrella'],
    'residual_waste': ['Plate', 'CigaretteEnd', 'DisposableChopsticks'],
}

def depth_pixel_to_camera(pixel_coords, intrinsic_matrix):
    fx, fy, cx, cy = intrinsic_matrix[0], intrinsic_matrix[4], intrinsic_matrix[2], intrinsic_matrix[5]
    px, py, pz = pixel_coords
    x = (px - cx) * pz / fx
    y = (py - cy) * pz / fy
    z = pz
    return np.array([x, y, z])

class ArmControlNode:
    hand2cam_tf_matrix = [
        [0.0, 0.0, 1.0, -0.101],
        [-1.0, 0.0, 0.0, 0.011],
        [0.0, -1.0, 0.0, 0.045],
        [0.0, 0.0, 0.0, 1.0]
    ]
    pick_offset = [0.03, 0.03, 0.015, -0.005, 0.02]  # x1, x2, y1, y2, z
    '''
                x1
        y1    center    y2
                x2

                arm
                car
    ''' 

    def __init__(self):
        rospy.init_node('arm_control_node', anonymous=True)
        self.moving = False
        self.count = 0
        self.last_detected_object = None
        self.endpoint = None
        self.current_class = None
        self.servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.controller = ActionGroupController(use_ros=True)
        self.lock = threading.Lock()

        rospy.Subscriber('/yolov8/obb_data', Float32MultiArray, self.control_callback)

        rospy.wait_for_service('/kinematics/set_joint_value_target')
        rospy.sleep(0.2)
        self.goto_default()

        rospy.spin()

    def goto_default(self):
        endpoint = kinematics_control.set_joint_value_target([500, 470, 220, 70, 500])  
        pose_t = endpoint.pose.position
        pose_r = endpoint.pose.orientation
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 470), (3, 220), (4, 70), (5, 500), (10, 200)))
        self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])
        rospy.loginfo(f"self.endpoint initialized: {self.endpoint}")

    def cal_position(self, x, y, depth, intrinsic_matrix):
        position = depth_pixel_to_camera([x, y, depth / 1000], intrinsic_matrix)   #像素坐标转换为相机坐标
        position[0] -= 0.01
        pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))   #相机坐标转换为机械臂基坐标
        rospy.loginfo(f"pose_end shape: {pose_end.shape}")
        rospy.loginfo(f"self.endpoint shape: {self.endpoint.shape}")
        if pose_end.shape != (4, 4) or self.endpoint.shape != (4, 4):
            rospy.logerr("Matrix dimensions are incorrect.")
        world_pose = np.matmul(self.endpoint, pose_end)
        pose_t, pose_r = common.mat_to_xyz_euler(world_pose)    #获取位置与旋转矩阵
        return pose_t

    def is_similar(self, obj1, obj2, tolerance=1):
        """
        判断两个检测对象是否相似，允许在 x, y, angle 上有正负1的误差。
        
        :param obj1: 第一个对象 (x, y, angle, cls)
        :param obj2: 第二个对象 (x, y, angle, cls)
        :param tolerance: 允许的误差范围，默认为1
        :return: 如果相似返回 True，否则返回 False
        """
        return (abs(obj1[0] - obj2[0]) <= tolerance and
                abs(obj1[1] - obj2[1]) <= tolerance and
                abs(obj1[2] - obj2[2]) <= tolerance and
                obj1[3] == obj2[3])  # 确保分类一致
    def control_callback(self, msg):
        # 类别对应的名称列表
        cls_list = ['BananaPeel', 'BrokenBones', 'CigaretteEnd', 'DisposableChopsticks', 'Ketchup', 'Marker', 'OralLiquidBottle', 'PlasticBottle', 'Plate', 'StorageBattery', 'Toothbrush', 'umbrella']
        # 从消息中获取数据
        x, y, depth, angle, cls, conf = msg.data

        intrinsic_matrix = [475.8579406738281, 0.0, 322.50006103515625, 0.0, 475.8579406738281, 202.42349243164062, 0.0, 0.0, 1.0]
        position = self.cal_position(x, y, depth, intrinsic_matrix)

        detected_object = (int(x), int(y), int(angle), int(cls))

        # 打印 detected_object 和置信度的值
        rospy.loginfo(f"Detected Object - X: {detected_object[0]}, Y: {detected_object[1]}, Angle: {detected_object[2]}, Class: {self.current_class}")

        # 检查置信度是否大于或等于 0.85
        if conf >= 0.80:
            # 使用 is_similar 函数进行相似性判断
            if self.last_detected_object is not None and self.is_similar(self.last_detected_object, detected_object):
                self.count += 1
            else:
                self.count = 0
                self.last_detected_object = detected_object

            if self.count >= 5 and not self.moving:
                self.count = 0
                self.moving = True

                # 在这里设置 self.current_class
                cls_list = ['BananaPeel', 'BrokenBones', 'CigaretteEnd', 'DisposableChopsticks', 'Ketchup', 'Marker', 'OralLiquidBottle', 'PlasticBottle', 'Plate', 'StorageBattery', 'Toothbrush', 'umbrella']
                self.current_class = cls_list[int(cls)]

                # 打印5次都判定好的数据
                rospy.loginfo(f"最终识别到的物体信息 - X: {detected_object[0]}, Y: {detected_object[1]}, Angle: {detected_object[2]}, Class: {self.current_class}")
                # 使用线程来运行机械臂动作
                threading.Thread(target=self.move, args=(position, angle)).start()

    def move(self, position, angle):
        with self.lock:
            # 位置补偿
            offset_z = 0.01 + self.pick_offset[-1]
            if position[0] > 0.21:
                offset_x = self.pick_offset[0]
            else:
                offset_x = self.pick_offset[1]
            if position[1] > 0:
                offset_y = self.pick_offset[2]
            else:
                offset_y = self.pick_offset[3]
            position[0] += offset_x
            position[1] += offset_y
            position[2] += offset_z
            # 设置目标位置
            ret1 = kinematics_control.set_pose_target(position, 85)
            # 移动机械臂到目标位置
            if len(ret1[1]) > 0:
                # bus_servo_control.set_servos(self.servos_pub, 1.5, ((1, ret1[1][0]), (2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]), (5, ret1[1][4])))
                # rospy.sleep(1.5)
                # 先执行舵机1的动作
                print(f"Servo 1 target position: {ret1[1][0]}")
                bus_servo_control.set_servos(self.servos_pub, 1.0, ((1, ret1[1][0]),))
                rospy.sleep(1)  # 等待1.5秒
                #然后再执行剩余的动作
                bus_servo_control.set_servos(self.servos_pub, 1.0, ((2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]), (5, ret1[1][4])))
                rospy.sleep(1)  # 再等待1.5秒
            # 向下移动以抓取物体
            position[2] -= 0.05
            ret2 = kinematics_control.set_pose_target(position, 85)

            # 调整机械臂的抓取角度
            value = ret1[1][0]
            # 取整后减去 500，再乘以 (240/1000) 得到度数
            degree = (round(value) - 500) * (240 / 1000)   # 机械臂云台偏移的角度
            angle = angle + degree  # 将偏移角度加到原始角度上
            
            # 如果 angle 大于 180 度，重置为 0 度并加上剩余的角度
            if angle > 180:
                angle = 0 + (angle - 180)

            # 如果 angle 小于 0 度，重置为 180 度并加上剩余的负数角度
            elif angle < 0:
                angle = 180 + angle  # 这里 angle 是负数，加上它相当于减去

            if 0 <= angle <= 90:
                angle = 100 + int(angle * 4.44)
            elif 90 < angle <= 180:
                angle = 500 + int((angle - 90) * 4.44)   # 0.44 = 400/90


            # 执行抓取动作
            if len(ret2[1]) > 0:
                bus_servo_control.set_servos(self.servos_pub, 1.0, ((5, angle),))
                rospy.sleep(1)
                bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret2[1][0]), (2, ret2[1][1]), (3, ret2[1][2]), (4, ret2[1][3]), (5, angle)))
                rospy.sleep(1)

                bus_servo_control.set_servos(self.servos_pub, 0.6, ((10, 550),))
                rospy.sleep(0.6)
            if len(ret1[1]) > 0:
                bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret1[1][0]), (2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]), (5, angle)))
                rospy.sleep(1)
            #放置垃圾到指定位置，机械臂复位
            # bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 640), (3, 150), (4, 130), (5, 500), (10, 650)))
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 520), (3, 50), (4, 280), (5, 500), (10, 650)))
            rospy.sleep(1)

            # 根据物体的分类来执行不同的动作组
            if self.current_class in WASTE_CLASSES['food_waste']:
                self.controller.runAction("place_food_waste1")
            elif self.current_class in WASTE_CLASSES['hazardous_waste']:
                self.controller.runAction("place_hazardous_waste1")
            elif self.current_class in WASTE_CLASSES['recyclable_waste']:
                self.controller.runAction("place_recyclable_waste1")
            elif self.current_class in WASTE_CLASSES['residual_waste']:
                self.controller.runAction("place_residual_waste")

            self.goto_default()
            rospy.sleep(2)
            self.moving = False

            # 重置检测相关变量
            self.last_detected_object = None
            self.count = 0

if __name__ == "__main__":
    ArmControlNode()
