#!/usr/bin/env python3
# encoding: utf-8
# 动作调用测试程序

import cv2
import math
import time
import rospy 
from servo_controllers import bus_servo_control
from servo_msgs.msg import MultiRawIdPosDur

class ArmSet():
    def __init__(self, name):
        rospy.init_node(name, anonymous=False, log_level=rospy.INFO)
        self.servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.set_arm()

    def set_arm(self):
        bus_servo_control.set_servos(self.servos_pub, 2, ((1, 500), (2, 765), (3, 12), (4, 147), (5, 500), (10, 505)))
        rospy.sleep(2)
        bus_servo_control.set_servos(self.servos_pub, 2, ((1, 1), (2, 770), (3, 12), (4, 151), (5, 500), (10, 600)))
        rospy.sleep(2)
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 1), (2, 520), (3, 200), (4, 151), (5, 500), (10, 600)))
        rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 1), (2, 520), (3, 200), (4, 151), (5, 500), (10, 450)))
        rospy.sleep(2)
        #  回到机械臂init位置
        bus_servo_control.set_servos(self.servos_pub, 2, ((1, 500), (2, 765), (3, 12), (4, 147), (5, 500), (10, 505)))


if __name__ == '__main__':
    ArmSet('arm_set')