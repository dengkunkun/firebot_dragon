#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
import time
import rclpy
import signal
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from ros_robot_controller_msgs.msg import MotorsState
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseWithCovarianceStamped, TransformStamped
from rclpy.parameter_event_handler import ParameterEventHandler

ODOM_POSE_COVARIANCE = list(map(float, 
                        [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]))

ODOM_POSE_COVARIANCE_STOP = list(map(float, 
                            [1e-9, 0, 0, 0, 0, 0, 
                             0, 1e-3, 1e-9, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e-9]))

ODOM_TWIST_COVARIANCE = list(map(float, 
                        [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]))

ODOM_TWIST_COVARIANCE_STOP = list(map(float, 
                            [1e-9, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0,
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0,
                              0, 0, 0, 0, 1e6, 0,
                              0, 0, 0, 0, 0, 1e-9]))

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

def qua2rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
  
    return roll, pitch, yaw


from ros_robot_controller_msgs.msg import MotorState, MotorsState

class MecanumChassis:
    # wheelbase = 0.216   # 前后轴距--实际应该是转弯半径
    # track_width = 0.195 # 左右轴距
    # wheel_diameter = 0.097  # 轮子直径
    
    def __init__(self, wheelbase: int | float, track_width: int | float, wheel_diameter: int | float):
        if not isinstance(wheelbase, (int, float)) or wheelbase < 0:
            raise ValueError("Wheelbase must be a positive number")
        if not isinstance(track_width, (int, float)) or track_width <= 0:
            raise ValueError("Track width must be a positive number")
        if not isinstance(wheel_diameter, (int, float)) or wheel_diameter <= 0:
            raise ValueError("Wheel diameter must be a positive number")
        
        self.wheelbase = float(wheelbase)  # 转换为 float 类型确保一致性
        self.track_width = float(track_width)
        self.wheel_diameter = float(wheel_diameter)

    def speed_covert(self, speed):
        """
        covert speed m/s to rps/s
        :param speed:
        :return:
        """
        # distance / circumference = rotations per second
        return speed / (math.pi * self.wheel_diameter)
    def rps_convert(self, rps):
        """
        covert rps/s to m/s
        :param rps:
        :return:
        """
        # rotations per second * circumference = distance
        return rps * (math.pi * self.wheel_diameter)
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
    def get_velocity(self,left_rps, right_rps):
        """
        :param left_rps:
        :param right_rps:
        :return:
        """
        left_v=self.rps_convert(left_rps)
        right_v=self.rps_convert(right_rps)
        # 计算线速度和角速度
        linear_x = (left_v + right_v) / 2.0
        angular_z = (right_v - left_v) / self.track_width
        return linear_x, angular_z
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


class Controller(Node):
    
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.x = 0.0
        self.y = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.pose_yaw = 0
        self.last_time = None
        self.current_time = None
        signal.signal(signal.SIGINT, self.shutdown)
        
        # 声明参数
        self.declare_parameter('pub_odom_topic', True)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('linear_correction_factor', 1.00)
        self.declare_parameter('angular_correction_factor', 1.00)
        # self.declare_parameter('machine_type', os.environ['MACHINE_TYPE'])
        self.declare_parameter('odom_pub_rate', 50)
        self.declare_parameter('linear_max_x', 1.0)
        self.declare_parameter('linear_max_y', 0.0)
        self.declare_parameter('angular_max_z', 0.3)
        
        self.declare_parameter('wheelbase', 0.0)
        self.declare_parameter('track_width',0.0)
        self.declare_parameter('wheel_diameter', 0.0)
        
        self.declare_parameter('open_loop', False)
        self.declare_parameter('stop', False)
        
        
        
        self.pub_odom_topic = self.get_parameter('pub_odom_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        
        # self.machine_type = os.environ['MACHINE_TYPE']
        self.linear_factor = self.get_parameter('linear_correction_factor').value
        self.angular_factor = self.get_parameter('angular_correction_factor').value
        self.odom_pub_rate = self.get_parameter('odom_pub_rate').value
        self.linear_max_x = self.get_parameter('linear_max_x').value
        self.linear_max_y = self.get_parameter('linear_max_y').value
        self.angular_max_z = self.get_parameter('angular_max_z').value
        
        self.handler = ParameterEventHandler(self)

        self.callback_handle = self.handler.add_parameter_callback(
            parameter_name="angular_max_z",
            node_name="motion_jetson",
            callback=self.angular_max_z_change_callback,
        )
        self.stop_callback_handle = self.handler.add_parameter_callback(
            parameter_name="stop",
            node_name="motion_jetson",
            callback=self.stop_change_callback,
        )
        
        self.open_loop = self.get_parameter('open_loop').value
        self.get_logger().info(f'\033[1;32m{self.open_loop}\033[0m')
        self.stop = self.get_parameter('stop').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.mecanum = MecanumChassis(self.wheelbase, self.track_width ,self.wheel_diameter)

        self.clock = self.get_clock() 
        if self.pub_odom_topic:
            # self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)  # 定义TF变换广播者
            # self.odom_trans = TransformStamped()
            # self.odom_trans.header.frame_id = self.odom_frame_id
            # self.odom_trans.child_frame_id = self.base_frame_id
            
            self.odom = Odometry()
            self.odom.header.frame_id = self.odom_frame_id
            self.odom.child_frame_id = self.base_frame_id
            
            self.odom.pose.covariance = ODOM_POSE_COVARIANCE
            self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
            
            self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 1)
            self.dt = 1.0/self.odom_pub_rate
            if self.open_loop:
                threading.Thread(target=self.odom_from_cmd_vel, daemon=True).start()
                self.get_logger().info('\033[1;32m%s\033[0m' % 'open loop')
            else:
                self.motor_pub = self.create_subscription(MotorsState, 'motors_rsp', self.odom_from_motor,1)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'close loop')
        self.motor_pub = self.create_publisher(MotorsState, 'motors_set', 1)
        self.create_subscription(Pose2D, 'set_odom', self.set_odom, 1)
        self.create_subscription(Twist, 'controller/cmd_vel', self.controller_cmd_vel_callback, 1)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def angular_max_z_change_callback(self, p: rclpy.parameter.Parameter) -> None:
        self.get_logger().warn(f"Received an update to parameter: {p.name}: {rclpy.parameter.parameter_value_to_python(p.value)}")
        # self.get_logger().warn(f'p:{p}') # p:rcl_interfaces.msg.Parameter
        self.angular_max_z=self.get_parameter('angular_max_z').value
        self.get_logger().warn(f"Updated angular_max_z: {self.angular_max_z}")
    
    def stop_change_callback(self, p: rclpy.parameter.Parameter) -> None:
        self.get_logger().warn(f"Received an update to parameter: {p.name}: {rclpy.parameter.parameter_value_to_python(p.value)}")
        # self.get_logger().warn(f'p:{p}') # p:rcl_interfaces.msg.Parameter
        self.stop=self.get_parameter('stop').value
        if self.stop:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'stop')
            speed=self.mecanum.set_velocity(0.0, 0.0, 0.0)
            self.motor_pub.publish(speed)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'shutdown')
        rclpy.shutdown()

    def set_odom(self, msg):
        self.odom = Odometry()
        self.odom.header.frame_id = self.odom_frame_id
        self.odom.child_frame_id = self.base_frame_id
        
        self.odom.pose.covariance = ODOM_POSE_COVARIANCE
        self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom.pose.pose.position.x = msg.x
        self.odom.pose.pose.position.y = msg.y
        self.pose_yaw = msg.theta
        self.odom.pose.pose.orientation = rpy2qua(0, 0, self.pose_yaw)
        
        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0
        
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.odom_frame_id
        pose.header.stamp = self.clock().now().to_msg()
        pose.pose.pose = self.odom.pose.pose
        pose.pose.covariance = ODOM_POSE_COVARIANCE
        self.pose_pub.publish(pose)

    def cmd_vel_callback(self, msg):
        if self.stop:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('\033[1;32m%s\033[0m' % 'stop')
            return
        if msg.linear.x > self.linear_max_x:
            msg.linear.x = self.linear_max_x
        if msg.linear.x < -self.linear_max_x:
            msg.linear.x = -self.linear_max_x
        if msg.linear.y > self.linear_max_y:
            msg.linear.y = self.linear_max_y
        if msg.linear.y < -self.linear_max_y:
            msg.linear.y = -self.linear_max_y
        if msg.angular.z > self.angular_max_z:
            msg.angular.z = self.angular_max_z
        if msg.angular.z < -self.angular_max_z:
            msg.angular.z = -self.angular_max_z
        self.controller_cmd_vel_callback(msg)

    def controller_cmd_vel_callback(self, msg):
        # msg.linear.x *= self.linear_factor
        # msg.linear.y *= self.linear_factor
        # msg.angular.z *= self.angular_factor
        self.get_logger().debug(f'\033[1;32m m/s {msg.linear.x} {msg.linear.y} {msg.angular.z}\033[0m')
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        if abs(msg.linear.y) > 1e-8:  #如果转弯太急，则不允许前进，只做转弯
            self.linear_x = 0.0
        else:
            self.linear_x = msg.linear.x
        self.linear_y = 0.0

        self.angular_z = msg.angular.z
        speeds = self.mecanum.set_velocity(self.linear_x, 0.0, self.angular_z)
        self.get_logger().debug(f'\033[1;32m rps {speeds}\033[0m')
        self.motor_pub.publish(speeds)


    def odom_from_cmd_vel(self):
        while True:
            self.current_time = time.time()
            if self.last_time is None:
                self.dt = 0.0
            else:
                # 计算时间间隔
                self.dt = self.current_time - self.last_time
            self.odom.header.stamp = self.clock.now().to_msg()
            
            self.x += math.cos(self.pose_yaw)*self.linear_x*self.dt - math.sin(self.pose_yaw)*self.linear_y*self.dt
            self.y += math.sin(self.pose_yaw)*self.linear_x*self.dt + math.cos(self.pose_yaw)*self.linear_y*self.dt
            self.pose_yaw += self.angular_factor*self.angular_z*self.dt

            self.odom.pose.pose.position.x = self.linear_factor*self.x
            self.odom.pose.pose.position.y = self.linear_factor*self.y
            self.odom.pose.pose.position.z = 0.0

            self.odom.pose.pose.orientation = rpy2qua(0.0, 0.0, self.pose_yaw)
            self.odom.twist.twist.linear.x = self.linear_x
            self.odom.twist.twist.linear.y = self.linear_y
            self.odom.twist.twist.angular.z = self.angular_z

            # self.odom_trans.header.stamp = self.clock.now().to_msg()
            # self.odom_trans.transform.translation.x = self.odom.pose.pose.position.x
            # self.odom_trans.transform.translation.y = self.odom.pose.pose.position.y
            # self.odom_trans.transform.translation.z = 0.0
            # self.odom_trans.transform.rotation = self.odom.pose.pose.orientation

            # 如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
            # 如果velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
            if self.linear_x == 0 and self.linear_y == 0 and self.angular_z == 0:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE_STOP
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE_STOP
            else:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
            self.get_logger().debug(f'\033[1;32m{self.odom.pose.pose.position.x} {self.odom.pose.pose.position.y} {self.odom.twist.twist.linear.x} {self.odom.twist.twist.linear.y}\033[0m')
            # self.odom_broadcaster.sendTransform(self.odom_trans)
            self.odom_pub.publish(self.odom)
            self.last_time = self.current_time
            time.sleep(1/self.odom_pub_rate)
    def odom_from_motor(self, msg):
        self.current_time = time.time()
        if self.last_time is None:
            self.last_time=time.time()
            return
        else:
            # 计算时间间隔
            self.dt = self.current_time - self.last_time
        self.odom.header.stamp = self.clock.now().to_msg()
        self.linear_x, self.angular_z = self.mecanum.get_velocity(msg.data[0].rps, msg.data[1].rps)
        if self.linear_x != 0:
            self.get_logger().debug(f'\033[1;32m rps {msg.data[0].rps} {msg.data[1].rps} \033[0m')
            self.get_logger().debug(f'\033[1;32m m/s {self.linear_x} {self.angular_z} dt:{self.dt} \033[0m')
        
        self.x += math.cos(self.pose_yaw)*self.linear_x*self.dt - math.sin(self.pose_yaw)*self.linear_y*self.dt
        self.y += math.sin(self.pose_yaw)*self.linear_x*self.dt + math.cos(self.pose_yaw)*self.linear_y*self.dt
        self.pose_yaw += self.angular_factor*self.angular_z*self.dt

        self.odom.pose.pose.position.x = self.linear_factor*self.x
        self.odom.pose.pose.position.y = self.linear_factor*self.y
        self.odom.pose.pose.position.z = 0.0

        self.odom.pose.pose.orientation = rpy2qua(0.0, 0.0, self.pose_yaw)
        self.odom.twist.twist.linear.x = self.linear_x
        self.odom.twist.twist.linear.y = self.linear_y
        self.odom.twist.twist.angular.z = self.angular_z
        if self.linear_x != 0:
            self.get_logger().debug(f'\033[1;32m{self.odom.pose.pose.position.x} {self.odom.pose.pose.position.y} {self.odom.twist.twist.linear.x} {self.odom.twist.twist.linear.y}\033[0m')

        # self.odom_trans.header.stamp = self.clock.now().to_msg()
        # self.odom_trans.transform.translation.x = self.odom.pose.pose.position.x
        # self.odom_trans.transform.translation.y = self.odom.pose.pose.position.y
        # self.odom_trans.transform.translation.z = 0.0
        # self.odom_trans.transform.rotation = self.odom.pose.pose.orientation

        # 如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
        # 如果velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
        if self.linear_x == 0 and self.linear_y == 0 and self.angular_z == 0:
            self.odom.pose.covariance = ODOM_POSE_COVARIANCE_STOP
            self.odom.twist.covariance = ODOM_TWIST_COVARIANCE_STOP
        else:
            self.odom.pose.covariance = ODOM_POSE_COVARIANCE
            self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
        # self.odom_broadcaster.sendTransform(self.odom_trans)
        self.odom_pub.publish(self.odom)
        self.last_time = self.current_time
        
def main():
    node = Controller('motion_jetson')
    try:
        rclpy.spin(node)  # 循环等待ROS2退出
    except KeyboardInterrupt:
        node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')
    
    
if __name__ == "__main__":
    main()
