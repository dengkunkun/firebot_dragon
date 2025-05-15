#!/usr/bin/python3
# coding=utf8
# 麦克纳姆轮底盘运动学(Mecanum wheel chassis kinematic)
import math
from ros_robot_controller_msgs.msg import MotorState, MotorsState

class MecanumChassis:
    # wheelbase = 0.216   # 前后轴距
    # track_width = 0.195 # 左右轴距
    # wheel_diameter = 0.097  # 轮子直径
    
# self.wheel_diameter_mecanum = 0.2
# self.mecanum = mecanum.MecanumChassis(wheelbase=0, track_width=0.5932, wheel_diameter=self.wheel_diameter_mecanum)
    def __init__(self, wheelbase=0.216, track_width=0.195, wheel_diameter=0.097):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_covert(self, speed):
        """
        covert speed m/s to rps/s
        :param speed:
        :return:
        """
        # distance / circumference = rotations per second
        return speed / (math.pi * self.wheel_diameter)
# speeds = self.mecanum.set_velocity(self.linear_x*(self.wheel_diameter_mecanum), 0.0, self.angular_z)
    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        Use polar coordinates to control moving
                    x
        v1 motor1|  ↑  |motor3 v3
          +  y - |     |
        v2 motor2|     |motor4 v4
        :param speed: m/s
        :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
        :param angular_rate:  The speed at which the chassis rotates rad/sec
        :param fake:
        :return:
        """
        # vx = speed * math.sin(direction)
        # vy = speed * math.cos(direction)
        # vp = angular_rate * (self.wheelbase + self.track_width) / 2
        # v1 = vx - vy - vp
        # v2 = vx + vy - vp
        # v3 = vx + vy + vp
        # v4 = vx - vy + vp
        # v_s = [self.speed_covert(v) for v in [v1, v2, -v3, -v4]]
        motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        v_s = [self.speed_covert(v) for v in [motor1, motor2, -motor3, -motor4]]
        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v_s[i])
            data.append(msg)
        
        msg = MotorsState()
        msg.data = data
        return msg

'''
cmd_vel转轮上速度 差速驱动模型
参数有：轮距b，转弯半径l
线速度转角速度：v=w*r
vl=w*(l+b/2) vr=w*(l-b/2) 差速驱动，l可以为0
def cmd_vel_callback(self, msg):
        # 提取线速度和角速度
        v = msg.linear.x  # m/s
        w = msg.angular.z  # rad/s

        # 计算左右轮线速度
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        # 转换为角速度 (rad/s)
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # 发布轮子速度
        left_speed = Float32()
        right_speed = Float32()
        left_speed.data = omega_left
        right_speed.data = omega_right
'''