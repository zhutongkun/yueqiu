#!/usr/bin/python3
# coding=utf8
# 全向轮底盘控制(Omni wheel chassis control)
import math
from ros_robot_controller.msg import MotorState

class OmniChassis:
    # wheelbase = 0.220   # 前后轴距
    # track_width = 0.194 # 左右轴距
    # wheel_diameter = 0.099  # 轮子直径
    def __init__(self, wheelbase=0.220, track_width=0.194, wheel_diameter=0.099):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_covert(self, speed):
        """
        covert speed m/s to rps/s
        :param speed:
        :return:
        """
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_x, angular_rate):
        # vp = angular_rate * (self.wheelbase + self.track_width) / 2
        vp = angular_rate * (self.wheelbase) 

        vx = linear_x
        v1,v2,v3,v4 = 0,0,0,0

        # if vx != 0:
            # v1 = vx 
            # v3 = -vx
        # print(angular_rate)
        if angular_rate > 0:
            v1 = vx - vp
            v2 = vx - vp
            v3 = -vx - vp
            v4 = -vx - vp
        elif angular_rate < 0:
            v1 = vx - vp
            v2 = vx - vp
            v3 = -vx - vp
            v4 = -vx - vp
        elif angular_rate == 0:
            v1 = vx 
            v2 = vx
            v3 = -vx
            v4 = -vx

        v_s = [self.speed_covert(v) for v in [v1, v2, v3, v4]]
        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = v_s[i]
            data.append(msg)  
        # print("motor data",data)
        return data
